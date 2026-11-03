#!/usr/bin/env python3
"""自主建图（随每次录制由 record.sh 启动）：录制期间一直建 2D 栅格；进入自主建图后机器人自己走遍封闭场地。

    FAST-LIVO2 位姿 /aft_mapped_to_init + Mid-360 /livox/lidar → explore_grid.Grid（2D 占据栅格）
    → 每秒找一次边界、选目标、规划路径 → 10 Hz 跟随路径 → 速度发给 7011 遥控服务 /api/cmd

底盘只由 7011（web/teleop_server.py）控制：避障（Mid-360 + 超声波）、加减速限制、心跳停车、急停都在那里。

状态：
    idle      录制中、未进入自主建图：只建栅格，不发任何速度
    spin      刚进入（本次录制第一次）：原地转一圈看清四周
    explore   选最近、最大的边界（已知可通行与未知的交界）前往
    home      没有可去的边界或超过时长上限：返回录制起点
    countdown 已回到起点、退出自主建图：STOP_COUNTDOWN 秒内没人取消就停止录制
    error     位姿跳变（定位跑飞）等，已退出自主建图，可检查后重新进入

进入 / 退出 / 取消停止：POST /enable、/disable、/cancel（web/server.py 转发自 7010 页面）。
7011 上有人按方向键（人工接管）立即退出自主建图；7011 急停期间暂停，解除后继续。
卡住（一段时间几乎没动或一直被避障挡住）：后退一点、把目标拉黑、换下一个。位姿中断则暂停。

状态在 http://127.0.0.1:7013/state（JSON）与 /grid.png（栅格图）。
"""
import json
import math
import os
import subprocess
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS"):
    os.environ[_v] = "1"     # numpy 单线程，不与 FAST-LIVO2 抢核

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

PROJECT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_DIR / "web"))
import explore_grid as eg      # noqa: E402
from safety import Safety      # noqa: E402

TELEOP = "http://127.0.0.1:7011/api/cmd"
PORT = 7013
V_MAX = float(os.environ.get("EXPLORE_V", "0.15"))     # m/s，实测合适
W_MAX = float(os.environ.get("EXPLORE_W", "0.4"))      # rad/s
TIME_LIMIT = float(os.environ.get("EXPLORE_MINUTES", "30")) * 60    # 每次进入自主建图的时长上限
STOP_COUNTDOWN = 120        # 完成后这么久没人取消就停止录制 (s)
LOOKAHEAD = 0.6             # 纯追踪前视距离 (m)
TURN_IN_PLACE = math.radians(50)    # 前视点偏离车头超过此角度先原地转
ARRIVE = 0.3                # 到达目标的距离 (m)
REPLAN = 1.0                # 重新规划间隔 (s)
STUCK_TIME, STUCK_DIST = 8.0, 0.05  # 这么久内移动不到这么远算卡住
BLOCKED_TIME = 4.0          # 一直被避障挡住这么久算卡住
BACKOFF_TIME, BACKOFF_V = 1.5, 0.1  # 卡住后反向退开
POSE_STALE, POSE_JUMP = 0.5, 0.3    # 位姿超过 0.5 s 没更新暂停；相邻两帧跳变超过 0.3 m 视为定位跑飞
TAKEOVER = 0.5              # 7011 这么久内收到过网页的指令，视为人工接管 (s)
SCAN_EVERY = 2              # 每 2 帧点云更新一次栅格（5 Hz）
DRY_RUN = os.environ.get("EXPLORE_DRY_RUN") == "1"   # 只算不动：不向 7011 发速度，用于检查流程


def quat_R(qx, qy, qz, qw):
    return np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                     [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                     [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Explorer:
    def __init__(self):
        p = rospy.get_param("galbot/imu_in_base")
        self.T_b_i = np.eye(4)
        self.T_b_i[:3, :3] = quat_R(*p[3:])
        self.T_b_i[:3, 3] = p[:3]
        self.T_i_b = np.linalg.inv(self.T_b_i)
        self.t_i_l = np.array(rospy.get_param("extrin_calib/extrinsic_T"))   # 雷达在 IMU 系（旋转为单位阵）
        self.T_b_l = self.T_b_i.copy()
        self.T_b_l[:3, 3] = self.T_b_i[:3, :3] @ self.t_i_l + self.T_b_i[:3, 3]
        self.session = rospy.get_param("~session", "")
        self.auto_start = rospy.get_param("~auto_start", False)   # record.sh start --explore：一就绪就进入
        self.safety = Safety()      # 只用它的车身轮廓与自标定结果过滤机器人自身的点

        self.lock = threading.Lock()
        self.grid = None
        self.T_w_i = None           # 最新 IMU 位姿
        self.pose_t = 0.0
        self.pose = None            # 底盘 (x, y, yaw)
        self.home = None            # 录制起点
        self.scan_n = 0

        self.state = "idle"
        self.msg = "等待 FAST-LIVO2 位姿"
        self.enabled = False        # 是否处于自主建图（由本程序发速度）
        self.spun = False           # 本次录制是否已环视过
        self.t_enter = 0.0
        self.stop_at = None         # 倒计时结束时间
        self.goal = None            # 世界坐标 (x, y)
        self.path = []              # 世界坐标 [(x, y)]
        self.frontiers = []
        self.blacklist = []         # 世界坐标
        self.goals_done = 0
        self.going_home = False
        self.grid_png = b""
        self.grid_meta = None
        self.feedback = {}          # 7011 最近一次返回
        self.cmd = (0.0, 0.0, 0.0)

    # ------------------------------------------------------------ 输入
    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        T = np.eye(4)
        T[:3, :3] = quat_R(q.x, q.y, q.z, q.w)
        T[:3, 3] = (p.x, p.y, p.z)
        T_w_b = T @ self.T_i_b
        pose = (T_w_b[0, 3], T_w_b[1, 3], math.atan2(T_w_b[1, 0], T_w_b[0, 0]))
        with self.lock:
            jump = math.hypot(pose[0] - self.pose[0], pose[1] - self.pose[1]) if self.pose else 0.0
            if jump > POSE_JUMP and self.enabled:
                self.fail(f"FAST-LIVO2 位姿跳变 {jump:.2f} m，疑似定位跑飞，已退出自主建图（录制继续）")
            self.T_w_i, self.pose, self.pose_t = T, pose, time.time()
            if self.grid is None:
                self.grid = eg.Grid(floor_z=T_w_b[2, 3])        # base_link 原点在地面
                self.home = pose[:2]
                self.msg = "录制中，可进入自主建图"

    def on_cloud(self, msg):
        self.scan_n += 1
        with self.lock:
            T, grid = self.T_w_i, self.grid
        if T is None or self.scan_n % SCAN_EVERY:
            return
        off = {f.name: f.offset for f in msg.fields}
        a = np.frombuffer(msg.data, dtype=np.dtype({
            "names": ["x", "y", "z"], "formats": ["<f4"] * 3,
            "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step}), count=msg.width * msg.height)
        xyz = np.stack([a["x"], a["y"], a["z"]], axis=1).astype(np.float64)
        xyz = xyz[np.einsum("ij,ij->i", xyz, xyz) > 0.04][::3]     # 去掉无效点，3 抽 1
        base = xyz @ self.T_b_l[:3, :3].T + self.T_b_l[:3, 3]
        xyz = xyz[~self.safety.is_self(base[:, :2])]
        pl = xyz + self.t_i_l                                       # 雷达系 → IMU 系
        grid.integrate(T[:3, :3] @ self.t_i_l + T[:3, 3], pl @ T[:3, :3].T + T[:3, 3])
        T_w_b = T @ self.T_i_b
        grid.mark_free_disk(T_w_b[:2, 3])

    # ------------------------------------------------------------ 进入 / 退出 / 取消停止（网页调用）
    def enable(self):
        with self.lock:
            if self.pose is None:
                return False, "FAST-LIVO2 还没输出位姿，请稍候"
            if self.enabled:
                return False, "已在自主建图中"
            self.enabled, self.stop_at, self.going_home = True, None, False
            self.goal, self.path, self.blacklist = None, [], []
            self.t_enter = time.time()
            self.state = "explore" if self.spun else "spin"
            self.msg = "开始探索" if self.spun else "原地转一圈，先看清四周"
        rospy.loginfo("进入自主建图")
        return True, "已进入自主建图"

    def disable(self, why="已退出自主建图", stop_robot=True):
        with self.lock:
            if not self.enabled:
                return False, "未在自主建图中"
            was = self.enabled
            self.enabled = False
            self.goal, self.path = None, []
            if self.state != "error":
                self.state, self.msg = "idle", why
        if was and stop_robot:
            self.send(0, 0, 0)
        rospy.loginfo(why)
        return True, why

    def cancel_stop(self):
        with self.lock:
            if self.stop_at is None:
                return False, "没有在倒计时"
            self.stop_at, self.state, self.msg = None, "idle", "已取消自动停止，录制继续"
        return True, self.msg

    def fail(self, why):
        """调用方需持有 self.lock。"""
        self.enabled, self.goal, self.path = False, None, []
        self.state, self.msg = "error", why
        rospy.logerr(why)

    # ------------------------------------------------------------ 规划（后台线程）
    def plan_loop(self):
        while not rospy.is_shutdown():
            t0 = time.time()
            try:
                self.replan()
            except Exception as e:      # noqa: BLE001
                rospy.logerr("规划异常：%s", e)
            time.sleep(max(0.0, REPLAN - (time.time() - t0)))

    def replan(self):
        with self.lock:
            grid, pose, state, enabled = self.grid, self.pose, self.state, self.enabled
        if grid is None:
            return
        st, (i0, j0) = grid.state()
        if st is None:
            return
        self.grid_png = eg.render(st)
        h, w = st.shape
        self.grid_meta = {"x0": (i0 - grid.n / 2) * eg.RES, "y0": (j0 - grid.n / 2) * eg.RES,
                          "w": h * eg.RES, "h": w * eg.RES}
        local = lambda xy: tuple(grid.cell(xy) - (i0, j0))
        to_world = lambda ij: tuple(np.round(grid.center(np.asarray(ij) + (i0, j0)), 3).tolist())
        bl = [local(b) for b in self.blacklist]
        res = eg.plan(st, local(pose[:2]), bl, local(self.home) if self.going_home else None)
        with self.lock:
            self.frontiers = [(to_world(c), n) for c, n in res.get("frontiers", [])]
            if not enabled or state not in ("explore", "home") or not self.enabled:
                return
            if "path" in res:
                self.path = [to_world(c) for c in res["path"]]
                self.goal = to_world(res["goal"])
                self.msg = (f"返回起点，剩余 {res['length']:.1f} m" if self.going_home
                            else f"前往第 {self.goals_done + 1} 个边界，路程 {res['length']:.1f} m，"
                                 f"剩余边界 {len(self.frontiers)} 处")
            elif self.going_home:
                self.fail(f"无法规划回到起点：{res.get('reason')}")
            else:
                self.going_home, self.path, self.goal = True, [], None
                self.state, self.msg = "home", f"{res.get('reason')}，返回起点"

    # ------------------------------------------------------------ 控制（主线程 10 Hz）
    def send(self, vx, vy, wz):
        self.cmd = (round(vx, 3), round(vy, 3), round(wz, 3))
        if DRY_RUN:
            self.feedback = {"ok": True}
            return True
        body = json.dumps({"vx": vx, "vy": vy, "wz": wz, "safe": True, "source": "explore"}).encode()
        req = urllib.request.Request(TELEOP, data=body, method="POST", headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=0.3) as r:
                self.feedback = json.loads(r.read())
        except urllib.error.HTTPError as e:
            self.feedback = json.loads(e.read() or b"{}")
        except OSError as e:
            self.feedback = {"ok": False, "msg": f"7011 遥控服务无响应：{e}"}
        return self.feedback.get("ok", False)

    def follow(self, pose):
        """纯追踪：返回 (vx, vy, wz) 或 None（已到达）。全向底盘，但尽量车头朝前进方向。"""
        x, y, yaw = pose
        if self.goal is None:
            return 0.0, 0.0, 0.0
        # 先判到达：已在目标处时路径只有一个点
        if math.hypot(self.goal[0] - x, self.goal[1] - y) < ARRIVE:
            return None
        if len(self.path) < 2:
            return 0.0, 0.0, 0.0
        pts = np.array(self.path)
        d = np.hypot(pts[:, 0] - x, pts[:, 1] - y)
        i = int(np.argmin(d))
        ahead = np.cumsum(np.r_[0, np.hypot(*np.diff(pts[i:], axis=0).T)])
        k = i + int(np.searchsorted(ahead, LOOKAHEAD))
        tx, ty = pts[min(k, len(pts) - 1)]
        dx, dy = tx - x, ty - y
        bx = math.cos(yaw) * dx + math.sin(yaw) * dy                # 前视点在底盘系
        by = -math.sin(yaw) * dx + math.cos(yaw) * dy
        alpha = math.atan2(by, bx)
        wz = max(-W_MAX, min(W_MAX, 1.2 * alpha))
        if abs(alpha) > TURN_IN_PLACE:
            return 0.0, 0.0, wz
        dist = math.hypot(self.goal[0] - x, self.goal[1] - y)
        v = V_MAX * max(0.4, min(1.0, dist / 0.8)) * math.cos(alpha)
        n = math.hypot(bx, by)
        return v * bx / n, v * by / n, wz

    def run(self):
        rate = rospy.Rate(10)
        spun, last_yaw = 0.0, None
        progress = []               # [(时间, x, y)]
        blocked_since = None
        backoff_until, backoff_v = 0.0, (0.0, 0.0)
        while not rospy.is_shutdown():
            rate.sleep()
            now = time.time()
            if self.auto_start and self.pose is not None:
                self.auto_start = False
                self.enable()
            with self.lock:
                pose, pose_t, state, enabled, stop_at = self.pose, self.pose_t, self.state, self.enabled, self.stop_at
            if stop_at is not None and now >= stop_at:
                self.stop_recording()
                continue
            if not enabled:
                spun, last_yaw = 0.0, None
                progress.clear()
                blocked_since, backoff_until = None, 0.0
                continue
            if now - pose_t > POSE_STALE:
                self.msg = "FAST-LIVO2 位姿中断，暂停"
                continue                # 不再发指令，7011 心跳超时自行减速停车
            if (self.feedback.get("others") or {}).get("web", math.inf) < TAKEOVER:
                self.disable("人工接管（7011 有人在遥控），已退出自主建图", stop_robot=False)
                continue
            if self.feedback.get("estop"):
                self.msg = "急停中：解除后继续自主建图"
                self.send(0, 0, 0)
                continue
            if now - self.t_enter > TIME_LIMIT and not self.going_home:
                with self.lock:
                    self.going_home, self.state, self.goal, self.path = True, "home", None, []
                    self.msg = f"达到时长上限 {TIME_LIMIT / 60:.0f} 分钟，返回起点"

            if state == "spin":
                if last_yaw is not None:
                    spun += abs(wrap(pose[2] - last_yaw))
                last_yaw = pose[2]
                if spun >= 2 * math.pi or now - self.t_enter > 2 * math.pi / W_MAX * 2.5:
                    self.send(0, 0, 0)
                    with self.lock:
                        self.spun = True
                        if self.enabled:
                            self.state, self.msg = "explore", "开始探索"
                else:
                    self.send(0, 0, W_MAX)
                continue

            if now < backoff_until:
                self.send(*backoff_v, 0.0)
                continue

            cmd = self.follow(pose)
            if cmd is None:             # 到达
                progress.clear()
                blocked_since = None
                self.send(0, 0, 0)
                if self.going_home:
                    self.finish()
                    continue
                with self.lock:
                    self.goals_done += 1
                    self.goal, self.path = None, []
                continue
            self.send(*cmd)

            # 卡住检测：被避障挡住，或长时间几乎没动
            moving_cmd = math.hypot(cmd[0], cmd[1]) > 0.02
            lim = self.feedback.get("limit") or {}
            if moving_cmd and lim.get("lin_scale", 1) == 0:
                blocked_since = blocked_since or now
            else:
                blocked_since = None
            progress.append((now, pose[0], pose[1]))
            while progress and now - progress[0][0] > STUCK_TIME:
                progress.pop(0)
            stalled = (moving_cmd and progress and now - progress[0][0] > STUCK_TIME - 0.5
                       and math.hypot(pose[0] - progress[0][1], pose[1] - progress[0][2]) < STUCK_DIST)
            if (blocked_since and now - blocked_since > BLOCKED_TIME) or stalled:
                why = "一直被避障挡住" if blocked_since else f"{STUCK_TIME:.0f} s 内几乎没动"
                n = math.hypot(cmd[0], cmd[1]) or 1.0
                backoff_v = (-BACKOFF_V * cmd[0] / n, -BACKOFF_V * cmd[1] / n)
                backoff_until = now + BACKOFF_TIME
                with self.lock:
                    if self.goal is not None and not self.going_home:
                        self.blacklist.append(self.goal)
                    self.goal, self.path = None, []
                    self.msg = f"卡住（{why}），后退并换目标"
                rospy.logwarn(self.msg)
                progress.clear()
                blocked_since = None

    def finish(self):
        """回到起点：退出自主建图，开始停止录制的倒计时。"""
        with self.lock:
            self.enabled, self.going_home = False, False
            self.state = "countdown"
            self.stop_at = time.time() + STOP_COUNTDOWN
            self.msg = f"自主建图完成，已回到起点；{STOP_COUNTDOWN // 60} 分钟内不取消将自动停止录制"
        rospy.loginfo(self.msg)
        self.save_result()

    def stop_recording(self):
        with self.lock:
            self.stop_at, self.state, self.msg = None, "idle", "倒计时结束，正在停止录制"
        self.save_result()
        # record.sh stop 会停掉本进程，所以放到独立会话里跑
        subprocess.Popen([str(PROJECT_DIR / "record.sh"), "stop"], start_new_session=True,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL)

    def save_result(self):
        if not self.session:
            return
        with open(os.path.join(self.session, "explore.json"), "w") as f:
            json.dump(self.snapshot(full=True), f, ensure_ascii=False)
        if self.grid_png:
            with open(os.path.join(self.session, "explore_grid.png"), "wb") as f:
                f.write(self.grid_png)

    def snapshot(self, full=False):
        with self.lock:
            return {
                "state": self.state, "msg": self.msg, "enabled": self.enabled,
                "elapsed_s": round(time.time() - self.t_enter) if self.enabled else None,
                "stop_in": max(0, round(self.stop_at - time.time())) if self.stop_at else None,
                "goals_done": self.goals_done, "pose": self.pose and [round(v, 3) for v in self.pose],
                "home": self.home and [round(v, 3) for v in self.home], "goal": self.goal,
                "path": self.path if full else self.path[::2], "frontiers": self.frontiers,
                "blacklist": self.blacklist, "grid": self.grid_meta, "cmd": self.cmd,
                "applied": self.feedback.get("applied"),
                "limit_reason": (self.feedback.get("limit") or {}).get("reason") if self.enabled else "",
                "teleop_msg": self.feedback.get("msg") if self.enabled and not self.feedback.get("ok", True) else "",
                "v_max": V_MAX, "time_limit_min": TIME_LIMIT / 60, "dry_run": DRY_RUN,
            }


def serve(ex):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, body, ctype="application/json; charset=utf-8", code=200):
            if isinstance(body, dict):
                body = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.startswith("/state"):
                self.reply(ex.snapshot())
            elif self.path.startswith("/grid.png") and ex.grid_png:
                self.reply(ex.grid_png, "image/png")
            else:
                self.send_error(404)

        def do_POST(self):
            action = {"/enable": ex.enable, "/disable": ex.disable, "/cancel": ex.cancel_stop}.get(self.path)
            if action is None:
                return self.send_error(404)
            ok, msg = action()
            self.reply({"ok": ok, "msg": msg}, code=200 if ok else 409)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()


def main():
    rospy.init_node("explorer")
    ex = Explorer()
    rospy.Subscriber("/aft_mapped_to_init", Odometry, ex.on_odom, queue_size=10)
    rospy.Subscriber("/livox/lidar", PointCloud2, ex.on_cloud, queue_size=1, buff_size=2 ** 24)
    serve(ex)
    threading.Thread(target=ex.plan_loop, daemon=True).start()
    rospy.loginfo("explorer 启动：限速 %.2f m/s %.2f rad/s，每次时长上限 %.0f 分钟%s", V_MAX, W_MAX, TIME_LIMIT / 60,
                  "（只算不动）" if DRY_RUN else "")
    try:
        ex.run()
    finally:
        if ex.enabled:
            ex.send(0, 0, 0)
        ex.save_result()


if __name__ == "__main__":
    main()
