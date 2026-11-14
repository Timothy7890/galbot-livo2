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
    GET  /state  /grid.png    /stream（SSE，每 0.2 s 一个 state 事件，网页用它，不再每秒轮询）
    POST /goto {"x", "y"[, "yaw"]}   目标离障碍太近时挪到 GOAL_SNAP_R 内最近的可通行处；给了 yaw 则到点后原地转到该朝向
    POST /cancel     /reloc {"mode": "auto"} 或 {"mode": "manual", "x", "y", "yaw"}（重定位进行中、排队中的重复请求一律拒绝）
    POST /confirm             人核对过自动重定位的结果（自动重定位后必须核对才能选目标；手动点选的不用；
                              启动时在上次停止的位置附近定位成功的，视为已核对）
    POST /speed {"v"}         线速度上限（0.1–0.5 m/s）
    POST /heartbeat           网页每秒调一次；收到过心跳后中断超过 HEARTBEAT_TIMEOUT，正在导航则停车（页面关了、手机息屏）
上次位姿：定位且已核对时每 3 s 把地图系位姿写进 <地图>/last_pose.json；下次启动先在它附近小范围搜（快、不会认到远处的相似位置），
对上了就直接可用，对不上再全图搜并要求人核对。
NAV_DRY_RUN=1：只算不动（不向 7011 发速度）；EXPLORE_V / EXPLORE_W 限速（nav.sh 默认 0.3 m/s）。
"""
import base64
import json
import math
import os
import sys
import zlib
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
from scipy.ndimage import distance_transform_edt
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
AUTO_RELOC_AFTER = 1.2      # FAST-LIVO2 出位姿这么久后自动重定位一次 (s)：只要攒够 LOC_WINDOW 的点云
AUTO_RETRIES = 10           # 启动时点云还不够就隔 0.5 s 再试，最多这么多次
LOC_VOXEL = 0.08            # 定位用点云先体素降采样 (m)
LAST_POSE_R, LAST_POSE_YAW = 1.0, math.radians(25)   # 启动定位结果离上次位姿在此之内，视为已核对
LAST_POSE_PERIOD = 3.0      # 上次位姿的保存间隔 (s)
GOAL_SNAP_R = 1.0           # 目标不可通行时，在这个半径内找最近的可通行格 (m)
FINAL_YAW_TOL = math.radians(7)     # 到点后转向指定朝向的容差
FINAL_TURN_TIMEOUT = 15.0   # 转不到指定朝向（被避障挡住等）这么久就算到达 (s)
HEARTBEAT_TIMEOUT = 5.0     # 网页心跳中断这么久，正在导航则停车 (s)
V_RANGE = (0.1, 0.5)        # 网页可调的线速度上限范围 (m/s)；7011 的上限是 0.5
SCAN_EVERY_SHOW, SCAN_MAX, SCAN_UNIT = 5, 1200, 0.02    # 网页显示的实时障碍点：每 5 帧一次、最多这么多点、2 cm 为单位的 int16
SCAN_BAND = (0.1, 1.6)      # 显示地面以上这个高度带内的点 (m)


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
        self.target = None              # [x, y] 或 [x, y, yaw]
        self.target_yaw = None          # 到点后要转到的朝向（没指定为 None）
        self.final_turn_t0 = None
        self.auto_retries = 0
        self.hb_t = None                # 最近一次网页心跳；None = 没有网页在发（命令行 / 仿真），不检查
        self.scan_b64 = ""              # 网页显示的实时障碍点（地图系，int16 小端，SCAN_UNIT 为单位）
        self.grid_v, self._grid_crc = 0, None       # 栅格图版本：变了网页才重新取图
        self.last_pose_file = os.path.join(nav_dir, "last_pose.json")
        self.last_pose = None           # 上次运行最后的地图系位姿 [x, y, yaw]
        try:
            with open(self.last_pose_file) as f:
                lp = json.load(f)
            self.last_pose = [float(lp["x"]), float(lp["y"]), float(lp["yaw"])]
        except (OSError, ValueError, KeyError, TypeError):
            pass

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
        keep = ~self.safety.is_self(base[:, :2])
        xyz, base = xyz[keep], base[keep]
        ts = msg.header.stamp.to_sec()
        with self.lock:
            self.loc_frames.append((ts, nl.voxel(xyz, LOC_VOXEL)))
            pose = self.pose
        self.scan_n += 1
        if self.scan_n % SCAN_EVERY_SHOW == 0 and pose is not None:
            # 底盘系（原点在地面）高度带内的点按当前位姿放到地图上，给网页看"机器人现在看到的障碍"
            b = base[(base[:, 2] > SCAN_BAND[0]) & (base[:, 2] < SCAN_BAND[1])]
            if len(b) > SCAN_MAX:
                b = b[:: len(b) // SCAN_MAX + 1]
            c, s_ = math.cos(pose[2]), math.sin(pose[2])
            mx, my = pose[0] + c * b[:, 0] - s_ * b[:, 1], pose[1] + s_ * b[:, 0] + c * b[:, 1]
            q = np.clip(np.round(np.column_stack([mx, my]) / SCAN_UNIT), -32000, 32000).astype("<i2")
            self.scan_b64 = base64.b64encode(q.tobytes()).decode()
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
                    req, self.auto_tried = {"mode": "auto", "startup": True}, True
                if req is not None:
                    self.relocalize(req)
                elif self.T_mo is not None and time.time() - last_track > TRACK_PERIOD:
                    last_track = time.time()
                    self.track()
            except Exception as e:      # noqa: BLE001
                rospy.logerr("定位异常：%s", e)

    def relocalize(self, req):
        manual = req.get("mode") == "manual"
        startup = bool(req.get("startup"))
        if self.enabled:
            self.disable("重新定位，已停止导航")
        self.set_loc(state="relocalizing", msg="手动重定位中…" if manual else "自动重定位中…")
        lev, T_ol = self.level_scan(LOC_WINDOW)
        if lev is None:
            if startup and self.auto_retries < AUTO_RETRIES:
                # 刚启动、点云还没攒够：过 0.5 s 再试，不用人去点
                self.auto_retries += 1
                with self.lock:
                    self.first_odom_t = time.time() - AUTO_RELOC_AFTER + 0.5
                    self.auto_tried = False
                self.set_loc(state="unlocalized", msg="等待雷达与 FAST-LIVO2 数据…")
                return
            self.set_loc(state="localized" if self.T_mo is not None else "unlocalized",
                         msg="雷达或 FAST-LIVO2 数据不足，稍后重试")
            return
        t0 = time.time()
        r, trusted = None, False
        if startup and self.last_pose is not None:
            # 先在上次停止的位置附近小范围搜：机器人多半没被挪过，这样快，也不会认到远处的相似位置
            lp = self.last_pose
            r1 = nl.relocalize(self.nav, lev, tuple(lp))
            if r1["ok"] and r1["fitness"] >= nl.FIT_ACCEPT_AUTO:
                T = r1["T"]
                d = math.hypot(T[0, 3] - lp[0], T[1, 3] - lp[1])
                dy = abs(math.remainder(math.atan2(T[1, 0], T[0, 0]) - lp[2], 2 * math.pi))
                if d <= LAST_POSE_R and dy <= LAST_POSE_YAW:
                    r, trusted = r1, True
        if r is None:
            hint = (float(req["x"]), float(req["y"]), float(req["yaw"])) if manual else None
            r = nl.relocalize(self.nav, lev, hint)
        info = {k: round(r[k], 3) for k in ("fitness", "rmse") if k in r}
        info["time_s"] = round(time.time() - t0, 1)
        if not r["ok"]:
            # 原来定位着的话保留原结果（例如误点了手动定位）
            self.set_loc(state="localized" if self.T_mo is not None else "unlocalized",
                         msg=r["msg"] + ("（保留原定位）" if self.T_mo is not None else ""), **info,
                         candidates=r.get("candidates", [])[:5])
            return
        ok_now = manual or trusted
        with self.lock:
            self.T_mo = r["T"] @ np.linalg.inv(T_ol)
            self.loc_fail = 0
            self.confirmed = ok_now
            self.state, self.msg = "idle", ("已定位，可在地图上选目标" if ok_now
                                            else "已自动定位，请核对地图上机器人的位置与朝向")
        Tm = r["T"]
        self.set_loc(state="localized", **info,
                     msg="定位成功" if manual else "与上次停止时的位置一致，已自动确认" if trusted else "自动定位成功，请核对",
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
    def snap_goal(self, st, xy, local, to_world):
        """目标格不可通行（障碍、未知、离障碍不到膨胀半径）时，GOAL_SNAP_R 内的可通行格由近到远排好返回（世界坐标）。"""
        clear = distance_transform_edt(st != eg.OCC) * eg.RES
        passable = (st == eg.FREE) & (clear > eg.INFLATE + 0.05)
        ci, cj = local(xy)
        k = int(GOAL_SNAP_R / eg.RES)
        h, w = st.shape
        a0, b0 = max(0, ci - k), max(0, cj - k)
        win = passable[a0:max(0, ci + k + 1), b0:max(0, cj + k + 1)]
        ij = np.argwhere(win) + (a0, b0)
        if not len(ij) or not (-k <= ci < h + k and -k <= cj < w + k):
            return []
        d = np.hypot(ij[:, 0] - ci, ij[:, 1] - cj)
        order = np.argsort(d)
        order = order[d[order] <= k]
        # 最近的一个，以及最近的"能转身"的一个（离障碍 TURN_CLEAR 以上），各试一次
        out = [ij[order[0]]] if len(order) else []
        roomy = [o for o in order if clear[ij[o][0], ij[o][1]] >= eg.TURN_CLEAR]
        if roomy and not (ij[roomy[0]] == out[0]).all():
            out.append(ij[roomy[0]])
        return [to_world(c) for c in out]

    def goto(self, x, y, yaw=None):
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
        local = lambda xy: tuple(int(v) for v in grid.cell(xy) - (i0, j0))
        to_world = lambda ij: tuple(np.round(grid.center(np.asarray(ij) + (i0, j0)), 3).tolist())
        start = local(pose[:2])
        res = eg.plan(st, start, home=local((x, y)))
        note = ""
        if "path" not in res and res.get("reason") != "起点不在可通行区域":
            for alt in self.snap_goal(st, (x, y), local, to_world):
                r2 = eg.plan(st, start, home=local(alt))
                if "path" in r2:
                    note = f"（目标离障碍太近，已挪 {math.hypot(alt[0] - x, alt[1] - y):.2f} m 到最近的可通行处）"
                    res, (x, y) = r2, alt
                    break
        if "path" not in res:
            return False, f"去不了这里：{res.get('reason')}（附近 {GOAL_SNAP_R:.0f} m 内没有能到的白色可通行区域）"
        corners = [to_world(c) for c in res["path"]]
        with self.lock:
            self.home = (float(x), float(y))
            self.target = [round(float(x), 3), round(float(y), 3)] + ([round(float(yaw), 3)] if yaw is not None else [])
            self.target_yaw, self.final_turn_t0 = (float(yaw) if yaw is not None else None), None
            self.going_home, self.enabled, self.stop_at = True, True, None
            # 路线直接用上面规划的，不等规划线程下一轮（最多 1 s）
            self.goal, self.corners = to_world(res["goal"]), corners
            self.path, self.corner_idx = explorer.densify(corners) if len(corners) > 1 else ([], [])
            self.blacklist, self.visited = [], []
            self.feedback = {}
            self.t_enter = time.time()
            self.state, self.msg = "home", f"前往目标，路程约 {res['length']:.1f} m{note}"
        rospy.loginfo("前往目标 (%.2f, %.2f)，路程约 %.1f m%s", x, y, res["length"], note)
        return True, self.msg

    def follow(self, pose):
        """到点后若指定了车头朝向，先原地转过去再算到达。"""
        cmd = super().follow(pose)
        if cmd is not None or self.target_yaw is None:
            return cmd
        err = explorer.wrap(self.target_yaw - pose[2])
        if abs(err) <= FINAL_YAW_TOL:
            return None
        now = time.time()
        if self.final_turn_t0 is None:
            self.final_turn_t0 = now
        if now - self.final_turn_t0 > FINAL_TURN_TIMEOUT:
            rospy.logwarn("到点后 %.0f s 内没能转到指定朝向（还差 %.0f°），按到达处理", FINAL_TURN_TIMEOUT, math.degrees(err))
            return None
        self.msg = f"已到目标点，转向指定朝向（还差 {abs(math.degrees(err)):.0f}°）"
        return 0.0, 0.0, math.copysign(min(explorer.W_MAX, max(explorer.MIN_TURN_W, 1.5 * abs(err))), err)

    def confirm(self):
        with self.lock:
            if self.T_mo is None or self.loc.get("state") != "localized":
                return False, "还没定位"
            self.confirmed = True
            self.loc["msg"] = "定位已确认"
            self.msg = "定位已确认，可在地图上选目标"
        return True, self.msg

    def cancel(self):
        ok, msg = self.disable("已取消导航")
        return ok, msg

    def request_reloc(self, body):
        if body.get("mode") == "manual" and not all(k in body for k in ("x", "y", "yaw")):
            return False, "手动定位需要 x、y、yaw"
        with self.lock:
            if self.loc.get("state") == "relocalizing" or self.loc_req is not None:
                return False, "正在重定位，请稍候"
            if not self.auto_tried:
                if body.get("mode") != "manual":
                    return False, "启动后的自动重定位马上开始，请稍候"
                self.auto_tried = True      # 人已经手动点选了，不再自动来一遍
            self.loc_req = dict(body, startup=False)
        return True, "开始重定位"

    def set_speed(self, v):
        v = max(V_RANGE[0], min(V_RANGE[1], float(v)))
        explorer.V_MAX = v
        explorer.LOOKAHEAD = max(0.9, 3.0 * v)
        return True, f"限速 {v:.2f} m/s"

    def heartbeat(self):
        self.hb_t = time.time()
        return True, ""

    def watch_loop(self):
        """心跳中断 / 急停时取消导航；定期保存上次位姿。"""
        last_save = 0.0
        while not rospy.is_shutdown():
            time.sleep(0.5)
            try:
                now = time.time()
                if self.enabled and self.hb_t is not None and now - self.hb_t > HEARTBEAT_TIMEOUT:
                    self.disable(f"网页心跳中断 {HEARTBEAT_TIMEOUT:.0f} s（页面关闭或手机息屏），已停车并取消本次导航")
                if self.enabled and self.feedback.get("estop"):
                    self.disable("急停，已取消本次导航；解除急停后请重新选目标")
                if now - last_save > LAST_POSE_PERIOD:
                    last_save = now
                    self.save_last_pose()
            except Exception as e:      # noqa: BLE001
                rospy.logerr("监控线程异常：%s", e)

    def save_last_pose(self):
        with self.lock:
            ok = self.confirmed and self.loc.get("state") == "localized" and self.pose is not None
            pose = self.pose
        if not ok:
            return
        tmp = self.last_pose_file + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"x": round(pose[0], 3), "y": round(pose[1], 3), "yaw": round(pose[2], 4),
                       "t": time.strftime("%Y-%m-%d %H:%M:%S")}, f)
        os.replace(tmp, self.last_pose_file)

    def finish(self):
        """到达目标（Explorer.run 在 going_home 到达时调用）。"""
        with self.lock:
            self.enabled, self.going_home = False, False
            self.goal, self.path, self.corners = None, [], []
            self.target_yaw, self.final_turn_t0 = None, None
            self.state, self.msg = "arrived", "已到达目标"
        rospy.loginfo("已到达目标")

    def replan(self):
        with self.lock:
            grid = self.grid
            busy = self.pose is not None and self.enabled
        if busy:
            super().replan()
        elif grid is not None:
            # 没在导航（未定位、待命）：只更新栅格图，不做规划（Explorer.replan 每秒一次全图最短路 + 找边界，白占一个核）
            st, (i0, j0) = grid.state()
            if st is not None:
                self.grid_png = eg.render(st)
                self.grid_meta = {"x0": (i0 - grid.n / 2) * eg.RES, "y0": (j0 - grid.n / 2) * eg.RES,
                                  "w": st.shape[0] * eg.RES, "h": st.shape[1] * eg.RES}
        crc = zlib.crc32(self.grid_png) if self.grid_png else None
        if crc != self._grid_crc:
            self._grid_crc, self.grid_v = crc, self.grid_v + 1

    def snapshot(self, full=False):
        s = super().snapshot(full)
        for k in ("frontiers", "strategies", "strategy", "blacklist", "visited", "region", "region_switches",
                  "goals_done", "stop_in", "home", "time_limit_min"):     # 自主建图的东西，导航用不着
            s.pop(k, None)
        with self.lock:
            s.update(loc=dict(self.loc), target=self.target, map=self.meta, loc_ms=self.loc_ms, confirmed=self.confirmed,
                     odom_pose=self.odom_pose and [round(v, 3) for v in self.odom_pose],
                     reloc_pending=self.loc_req is not None or not self.auto_tried,
                     last_pose=self.last_pose)
        s.update(dry_run=explorer.DRY_RUN, scan=self.scan_b64, scan_unit=SCAN_UNIT, grid_v=self.grid_v,
                 v_range=list(V_RANGE), robot_r=eg.ROBOT_R,
                 heartbeat=self.hb_t is not None and time.time() - self.hb_t < HEARTBEAT_TIMEOUT)
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
            elif self.path.startswith("/stream"):
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    while not rospy.is_shutdown():
                        data = json.dumps(nv.snapshot(), ensure_ascii=False, separators=(",", ":"))
                        self.wfile.write(f"event: state\ndata: {data}\n\n".encode())
                        self.wfile.flush()
                        time.sleep(0.2)
                except OSError:
                    pass
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
                    yaw = body.get("yaw")
                    ok, msg = nv.goto(float(body["x"]), float(body["y"]), None if yaw is None else float(yaw))
                elif self.path == "/speed":
                    ok, msg = nv.set_speed(body["v"])
                elif self.path == "/heartbeat":
                    ok, msg = nv.heartbeat()
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
    threading.Thread(target=nv.watch_loop, daemon=True).start()
    rospy.loginfo("navigator 启动：地图 %s（%s），限速 %.2f m/s%s", nv.meta.get("name"), nav_dir, explorer.V_MAX,
                  "（只算不动）" if explorer.DRY_RUN else "")
    try:
        nv.run()
    finally:
        if nv.enabled:
            nv.send(0, 0, 0)
        nv.save_last_pose()


if __name__ == "__main__":
    main()
