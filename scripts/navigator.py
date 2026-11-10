#!/usr/bin/env python3
"""成熟地图内导航（由 nav.sh 启动，不录制）：重定位 → 在地图上点选目标 → 规划并开过去。

    FAST-LIVO2 里程计 /aft_mapped_to_init（本次的 odom 系）+ Mid-360 /livox/lidar
    → nav_loc：自动重定位（失败或有歧义时网页点选大概位置再定位）→ 每秒跟踪一次，得到 T_map_odom
    → 位姿换到地图系，规划与跟随复用自主建图（explorer.Explorer：避障膨胀、走中间、拐点放在能转身处、
      纯追踪、卡住后退、急停暂停、7011 有人遥控即退出）→ 速度经 7011 遥控服务下发

地图：录制生成的成熟地图 <录制目录>/nav/（scripts/nav_map.py）；载入的栅格会继续用实时雷达更新
（新放的东西会变成障碍、挪走的会被射线洗掉），只影响本次运行，不写回地图文件。

状态（state）：
    idle      已定位，等待目标        home   前往目标（沿用 Explorer 的状态名）
    arrived   已到达                  error  规划失败、定位跑飞等，已停车
定位（loc.state）：unlocalized / relocalizing / localized / lost（连续跟踪失败，正在导航则停车）

HTTP（127.0.0.1:7014，web/server.py 转发自 7010 /api/nav/...）：
    GET  /state  /grid.png
    POST /goto {"x", "y"}     /cancel     /reloc {"mode": "auto"} 或 {"mode": "manual", "x", "y", "yaw"}
    POST /confirm             人核对过自动重定位的结果（自动重定位后必须核对才能选目标；手动点选的不用）
NAV_DRY_RUN=1：只算不动（不向 7011 发速度）；EXPLORE_V / EXPLORE_W 限速（nav.sh 默认 0.3 m/s）。
"""
import json
import math
import os
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS"):
    os.environ[_v] = "1"

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2

sys.path.insert(0, str(Path(__file__).resolve().parent))
import explore_grid as eg      # noqa: E402
import explorer                # noqa: E402
import nav_loc as nl           # noqa: E402

explorer.DRY_RUN = os.environ.get("NAV_DRY_RUN") == "1"
PORT = 7014
LOC_WINDOW = 1.0            # 重定位用最近这么久的点云 (s)
TRACK_WINDOW = 0.5          # 跟踪用最近这么久的点云 (s)
TRACK_PERIOD = 1.0          # 跟踪间隔 (s)
AUTO_RELOC_AFTER = 3.0      # FAST-LIVO2 出位姿这么久后自动重定位一次 (s)
LOC_VOXEL = 0.08            # 定位用点云先体素降采样 (m)


class Navigator(explorer.Explorer):
    HOME_LABEL = "目标"
    COLLECT_CLOUD = False

    def __init__(self, nav_dir):
        super().__init__()
        self.nav_dir = nav_dir
        self.nav = nl.NavMap(nav_dir)
        with open(os.path.join(nav_dir, "meta.json")) as f:
            self.meta = json.load(f)
        g = np.load(os.path.join(nav_dir, "grid.npz"))
        self.grid = eg.Grid(floor_z=float(g["floor_z"]))
        i0, j0 = int(g["i0"]), int(g["j0"])
        h, w = g["hits"].shape
        self.grid.hits[i0:i0 + h, j0:j0 + w] = g["hits"]
        self.grid.free[i0:i0 + h, j0:j0 + w] = g["free"]
        self.grid.bbox = [i0, i0 + h, j0, j0 + w]
        self.spun = True                # 不环视
        self.state, self.msg = "idle", "等待定位"
        self.T_mo = None                # 地图 ← 本次里程计
        self.odom_T = deque(maxlen=100)     # [(数据时间, T_odom_imu)]
        self.odom_pose = None           # 本次里程计下的底盘 (x, y, yaw)，跳变检查用
        self.first_odom_t = None
        self.loc_frames = deque(maxlen=20)  # [(数据时间, 雷达系点)]，定位用
        self.loc = {"state": "unlocalized", "msg": "等待 FAST-LIVO2 位姿"}
        self.loc_req = None             # 网页的重定位请求
        self.confirmed = False          # 定位结果是否经人确认（自动重定位可能落在重复结构的相似位置上）
        self.auto_tried = False
        self.loc_fail = 0
        self.loc_ms = None
        self.target = None

    # ------------------------------------------------------------ 输入
    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        T = np.eye(4)
        T[:3, :3] = explorer.quat_R(q.x, q.y, q.z, q.w)
        T[:3, 3] = (p.x, p.y, p.z)
        stamp = msg.header.stamp.to_sec()
        Tb = T @ self.T_i_b
        op = (Tb[0, 3], Tb[1, 3], math.atan2(Tb[1, 0], Tb[0, 0]))
        with self.lock:
            # 跳变检查放在里程计系：地图系位姿还含定位修正（每秒最多 0.5 m），不能拿来判 FAST-LIVO2 跑飞
            if self.odom_pose and self.enabled and math.hypot(op[0] - self.odom_pose[0],
                                                              op[1] - self.odom_pose[1]) > explorer.POSE_JUMP:
                self.fail("FAST-LIVO2 位姿跳变，疑似定位跑飞，已停车")
            self.odom_pose = op
            self.odom_T.append((stamp, T))
            if self.first_odom_t is None:
                self.first_odom_t = time.time()
            T_mo = self.T_mo
            if T_mo is not None:
                Tm = T_mo @ T
                Tmb = Tm @ self.T_i_b
                self.T_w_i, self.pose_t, self.pose_stamp = Tm, time.time(), stamp
                self.pose = (Tmb[0, 3], Tmb[1, 3], math.atan2(Tmb[1, 0], Tmb[0, 0]))
                self.odom_hist.append((stamp, Tm))      # 栅格更新用地图系位姿
        if T_mo is not None:
            self.integrate_pending()

    def on_cloud(self, msg):
        off = {f.name: f.offset for f in msg.fields}
        a = np.frombuffer(msg.data, dtype=np.dtype({
            "names": ["x", "y", "z"], "formats": ["<f4"] * 3,
            "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step}), count=msg.width * msg.height)
        xyz = np.stack([a["x"], a["y"], a["z"]], axis=1).astype(np.float64)
        xyz = xyz[np.einsum("ij,ij->i", xyz, xyz) > 0.04]
        base = xyz @ self.T_b_l[:3, :3].T + self.T_b_l[:3, 3]
        xyz = xyz[~self.safety.is_self(base[:, :2])]
        ts = msg.header.stamp.to_sec()
        with self.lock:
            self.loc_frames.append((ts, nl.voxel(xyz, LOC_VOXEL)))
        self.scan_n += 1
        if self.scan_n % explorer.SCAN_EVERY == 0 and self.T_mo is not None:
            with self.integ_lock:
                self.cloud_q.append((ts, xyz[::3]))
            self.integrate_pending()

    # ------------------------------------------------------------ 定位（后台线程）
    def level_scan(self, window):
        """最近 window 秒的点云（与同一帧的里程计配对）→ 当前水平底盘系点，以及 T_odom_level。"""
        with self.lock:
            frames, odom = list(self.loc_frames), list(self.odom_T)
        if not odom:
            return None, None
        t_last = odom[-1][0]
        ot = np.array([o[0] for o in odom])
        pts = []
        for ts, xyz in frames:
            if ts + 0.1 < t_last - window:
                continue
            k = int(np.argmin(np.abs(ot - (ts + 0.1))))
            if abs(ot[k] - (ts + 0.1)) > 0.06:
                continue
            T = odom[k][1]
            pts.append((xyz + self.t_i_l) @ T[:3, :3].T + T[:3, 3])
        if len(pts) < max(2, int(window * 10 * 0.5)):
            return None, None
        T_ob = odom[-1][1] @ self.T_i_b
        yaw, _ = nl.level_frame(T_ob)
        T_ol = np.eye(4)
        T_ol[:3, :3], T_ol[:3, 3] = nl.Rz(yaw), T_ob[:3, 3]
        w = nl.voxel(np.vstack(pts), 0.05)
        return (w - T_ol[:3, 3]) @ T_ol[:3, :3], T_ol

    def set_loc(self, **kw):
        with self.lock:
            self.loc = {**kw, "t": round(time.time(), 1)}
        rospy.loginfo("定位：%s", json.dumps(kw, ensure_ascii=False))

    def loc_loop(self):
        last_track = 0.0
        while not rospy.is_shutdown():
            time.sleep(0.1)
            try:
                with self.lock:
                    req, self.loc_req = self.loc_req, None
                    first = self.first_odom_t
                if req is None and not self.auto_tried and first and time.time() - first > AUTO_RELOC_AFTER:
                    req, self.auto_tried = {"mode": "auto"}, True
                if req is not None:
                    self.relocalize(req)
                elif self.T_mo is not None and time.time() - last_track > TRACK_PERIOD:
                    last_track = time.time()
                    self.track()
            except Exception as e:      # noqa: BLE001
                rospy.logerr("定位异常：%s", e)

    def relocalize(self, req):
        manual = req.get("mode") == "manual"
        if self.enabled:
            self.disable("重新定位，已停止导航")
        self.set_loc(state="relocalizing", msg="手动重定位中…" if manual else "自动重定位中…")
        lev, T_ol = self.level_scan(LOC_WINDOW)
        if lev is None:
            self.set_loc(state="unlocalized", msg="雷达或 FAST-LIVO2 数据不足，稍后重试")
            return
        hint = (float(req["x"]), float(req["y"]), float(req["yaw"])) if manual else None
        t0 = time.time()
        r = nl.relocalize(self.nav, lev, hint)
        info = {k: round(r[k], 3) for k in ("fitness", "rmse") if k in r}
        info["time_s"] = round(time.time() - t0, 1)
        if not r["ok"]:
            # 原来定位着的话保留原结果（例如误点了手动定位）
            self.set_loc(state="localized" if self.T_mo is not None else "unlocalized",
                         msg=r["msg"] + ("（保留原定位）" if self.T_mo is not None else ""), **info,
                         candidates=r.get("candidates", [])[:5])
            return
        with self.lock:
            self.T_mo = r["T"] @ np.linalg.inv(T_ol)
            self.loc_fail = 0
            self.confirmed = manual
            self.state, self.msg = "idle", ("已定位，可在地图上点选目标" if manual
                                            else "已自动定位，请核对地图上机器人的位置与朝向")
        Tm = r["T"]
        self.set_loc(state="localized", msg="定位成功" if manual else "自动定位成功，请核对", **info,
                     pose=[round(Tm[0, 3], 2), round(Tm[1, 3], 2), round(math.atan2(Tm[1, 0], Tm[0, 0]), 3)])

    def track(self):
        lev, T_ol = self.level_scan(TRACK_WINDOW)
        if lev is None:
            return
        with self.lock:
            T_mo = self.T_mo
        t0 = time.time()
        r = nl.track(self.nav, lev, T_mo @ T_ol)
        self.loc_ms = round((time.time() - t0) * 1000)
        if r["ok"]:
            with self.lock:
                self.T_mo = r["T"] @ np.linalg.inv(T_ol)
                self.loc_fail = 0
                if self.loc.get("state") != "localized":
                    self.loc = {"state": "localized", "msg": "定位恢复", "t": round(time.time(), 1)}
                self.loc.update(fitness=round(r["fitness"], 3), rmse=round(r["rmse"], 3),
                                correction_cm=round(r["jump"] * 100, 1))
            return
        with self.lock:
            self.loc_fail += 1
            lost = self.loc_fail >= nl.LOST_AFTER and self.loc.get("state") == "localized"
            self.loc.update(fitness=round(r["fitness"], 3), rmse=round(r["rmse"], 3), fails=self.loc_fail)
        if lost:
            if self.enabled:
                self.disable("定位丢失，已停车；请重新定位")
            self.set_loc(state="lost", msg="定位丢失（与地图对不上），请重新定位",
                         fitness=round(r["fitness"], 3), rmse=round(r["rmse"], 3))

    # ------------------------------------------------------------ 目标（网页调用）
    def goto(self, x, y):
        with self.lock:
            if self.T_mo is None or self.loc.get("state") != "localized":
                return False, "还没定位，先定位再选目标"
            if not self.confirmed:
                return False, "请先核对自动定位的结果（地图上的机器人位置与朝向对不对）"
            pose = self.pose
            grid = self.grid
        if pose is None:
            return False, "还没有地图系位姿"
        st, (i0, j0) = grid.state()
        local = lambda xy: tuple(grid.cell(xy) - (i0, j0))
        res = eg.plan(st, local(pose[:2]), home=local((x, y)))
        if "path" not in res:
            return False, f"去不了这里：{res.get('reason')}（目标要在白色的可通行区域、离障碍 0.4 m 以上）"
        with self.lock:
            self.home = (float(x), float(y))
            self.target = self.home
            self.going_home, self.enabled, self.stop_at = True, True, None
            self.goal, self.path, self.corners, self.corner_idx = None, [], [], []
            self.blacklist, self.visited = [], []
            self.feedback = {}
            self.t_enter = time.time()
            self.state, self.msg = "home", f"前往目标，路程约 {res['length']:.1f} m"
        rospy.loginfo("前往目标 (%.2f, %.2f)，路程约 %.1f m", x, y, res["length"])
        return True, self.msg

    def confirm(self):
        with self.lock:
            if self.T_mo is None or self.loc.get("state") != "localized":
                return False, "还没定位"
            self.confirmed = True
            self.msg = "定位已确认，可在地图上点选目标"
        return True, self.msg

    def cancel(self):
        ok, msg = self.disable("已取消导航")
        return ok, msg

    def request_reloc(self, body):
        if body.get("mode") == "manual" and not all(k in body for k in ("x", "y", "yaw")):
            return False, "手动定位需要 x、y、yaw"
        with self.lock:
            if self.loc.get("state") == "relocalizing":
                return False, "正在重定位，请稍候"
            self.loc_req = body
        return True, "开始重定位"

    def finish(self):
        """到达目标（Explorer.run 在 going_home 到达时调用）。"""
        with self.lock:
            self.enabled, self.going_home = False, False
            self.goal, self.path, self.corners = None, [], []
            self.state, self.msg = "arrived", "已到达目标"
        rospy.loginfo("已到达目标")

    def replan(self):
        if self.pose is None:          # 未定位：只更新栅格图
            st, _ = self.grid.state()
            if st is not None:
                self.grid_png = eg.render(st)
                i0, _, j0, _ = self.grid.bbox
                self.grid_meta = {"x0": (max(0, i0 - 2) - self.grid.n / 2) * eg.RES,
                                  "y0": (max(0, j0 - 2) - self.grid.n / 2) * eg.RES,
                                  "w": st.shape[0] * eg.RES, "h": st.shape[1] * eg.RES}
            return
        super().replan()

    def snapshot(self, full=False):
        s = super().snapshot(full)
        with self.lock:
            s.update(loc=dict(self.loc), target=self.target, map=self.meta, loc_ms=self.loc_ms, confirmed=self.confirmed,
                     odom_pose=self.odom_pose and [round(v, 3) for v in self.odom_pose])
        s["dry_run"] = explorer.DRY_RUN
        return s


def serve(nv):
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
                self.reply(nv.snapshot())
            elif self.path.startswith("/grid.png") and nv.grid_png:
                self.reply(nv.grid_png, "image/png")
            else:
                self.send_error(404)

        def do_POST(self):
            try:
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"{}") if n else {}
            except ValueError:
                body = {}
            if not isinstance(body, dict):
                body = {}
            try:
                if self.path == "/goto":
                    ok, msg = nv.goto(float(body["x"]), float(body["y"]))
                elif self.path == "/cancel":
                    ok, msg = nv.cancel()
                elif self.path == "/confirm":
                    ok, msg = nv.confirm()
                elif self.path == "/reloc":
                    ok, msg = nv.request_reloc(body)
                else:
                    return self.send_error(404)
            except (KeyError, TypeError, ValueError) as e:
                ok, msg = False, f"参数错误：{e}"
            self.reply({"ok": ok, "msg": msg}, code=200 if ok else 409)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()


def main():
    rospy.init_node("navigator")
    nav_dir = rospy.get_param("~nav_dir")
    nv = Navigator(nav_dir)
    rospy.Subscriber("/aft_mapped_to_init", Odometry, nv.on_odom, queue_size=10)
    rospy.Subscriber("/livox/lidar", PointCloud2, nv.on_cloud, queue_size=1, buff_size=2 ** 24)
    rospy.Subscriber("/livox/imu", Imu, nv.on_imu, queue_size=400)
    serve(nv)
    threading.Thread(target=nv.plan_loop, daemon=True).start()
    threading.Thread(target=nv.loc_loop, daemon=True).start()
    rospy.loginfo("navigator 启动：地图 %s（%s），限速 %.2f m/s%s", nv.meta.get("name"), nav_dir, explorer.V_MAX,
                  "（只算不动）" if explorer.DRY_RUN else "")
    try:
        nv.run()
    finally:
        if nv.enabled:
            nv.send(0, 0, 0)


if __name__ == "__main__":
    main()
