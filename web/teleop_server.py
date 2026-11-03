#!/usr/bin/env python3
"""Galbot G1 底盘遥控网页（端口 7011）。经 Galbot SDK 控制底盘，需在 sdk_env.sh 下运行（web/web.sh 已处理）。

    GET  /                  控制页面（web/dist/teleop.html）
    GET  /api/status        SDK 状态、里程计、最近一次指令结果
    GET  /api/scan          避障用的障碍点、8 个超声波读数、限速明细（网页俯视图）
    GET  /api/camera/<名称>.jpg  环视相机最新画面（默认 320x240 预览，?full=1 原图）
    POST /api/cmd           {"vx", "vy", "wz", "safe", "source"} 速度指令，网页按住期间、自主探索（explorer.py）运行期间
                            每 0.1 s 发一次；返回上一周期实际下发的速度、避障原因和其他来源最近一次指令距今多久
                            （others，自主探索据此判断人工接管）。底盘只由本服务控制，急停对两者都有效
    POST /api/stop          急停（锁定，之后的速度指令一律忽略）
    POST /api/release       解除急停
    POST /api/calibrate     车身自标定：静止采 CAL_FRAMES 帧 Mid-360，记录机器人自身（手臂等），见 safety.py

安全：
- 发给底盘的每条指令只持续 CMD_DURATION 秒，超时底盘自己停；
- 本服务 HEARTBEAT 秒内没收到新指令（松手、网页断线、网络卡顿）按 DEC 减速到 0，超过 HALT_AFTER 立即 stop_base；
- 输出按 ACC / DEC 限制加减速，起停平顺；避障要求停车时立即停；
- safe=true 经 safety.py（Mid-360 + 超声波）按运动方向限速/停车后再下发，false 直接下发（无避障）。
  Galbot 自带的 navigate_with_velocity 需要先有 Galbot 地图并完成定位，否则返回 WAIT_INITIALIZED，首次建图用不了。
"""
import math
import os
import re
import sys
import threading
import time
from http.server import ThreadingHTTPServer

import cv2
import numpy as np

from safety import ULTRASONIC, Safety
from webcommon import BaseHandler

PORT = int(os.environ.get("TELEOP_WEB_PORT", "7011"))
MAX_V = 0.5          # m/s
MAX_W = 1.0          # rad/s
CMD_DURATION = 0.5   # 每条指令在底盘侧的持续时间 (s)
HEARTBEAT = 0.5      # 超过这么久没收到新指令就减速停车 (s)；网页经热点遥控时包间隔抖动大，太短会一顿一顿
HALT_AFTER = 1.0     # 超过这么久没收到新指令立即 stop_base (s)
ACC_V, DEC_V = 0.4, 0.8     # 线加速度、减速度 (m/s²)
ACC_W, DEC_W = 1.0, 2.0     # 角加速度、减速度 (rad/s²)
RATE = 10            # 下发频率 (Hz)
CAL_FRAMES = 20      # 车身自标定采集帧数（约 2 s）
CAMERAS = ("left_front", "right_front", "left_rear", "right_rear")
CAMERA_PATH = re.compile(r"^/api/camera/(\w+)\.jpg$")


class Base:
    """持有 SDK，按最新指令以 RATE 下发；指令过期即停车。"""

    def __init__(self):
        # 状态和底盘调用共用一把锁：急停一旦返回，之后不会再有已经取出的旧指令被下发
        self.lock = threading.Lock()
        self.init_lock = threading.Lock()
        self.robot = self.nav = self.g1 = None
        self.sdk_error = ""
        self.target = None          # (vx, vy, wz, safe, 收到时间, 来源)
        self.moving = False
        self.estop = False
        self.last_result = ""
        self.thumbs = {}            # 名称 -> (原图 JPEG, 预览 JPEG)
        self.safety = Safety()
        self.lidar_tf = None        # Mid-360 在 base_link 下的 (R, t)
        self.lidar_stamp = 0
        self.applied = (0.0, 0.0, 0.0)  # 限速后实际下发的速度
        self.calib = None           # 车身自标定进行中时为已采集的帧列表
        self.last_by_source = {}    # 来源 -> 最近一次非零指令的时间
        self.calib_msg = ""

    def ensure_sdk(self):
        """开机自启时 Galbot 服务可能还没就绪，SDK 第一次用到时再初始化，失败下次重试。"""
        if self.robot is not None:
            return True
        with self.init_lock:
            if self.robot is not None:
                return True
            return self._init_sdk()

    def _init_sdk(self):
        try:
            from galbot_sdk import g1
            robot = g1.GalbotRobot()
            sensors = {getattr(g1.SensorType, f"{c.upper()}_SURROUND_CAMERA") for c in CAMERAS}
            sensors |= {g1.SensorType.BASE_LIDAR, g1.SensorType.BASE_ULTRASONIC}
            if not robot.init(sensors):
                raise RuntimeError("GalbotRobot.init 失败")
            nav = g1.GalbotNavigation()
            if not nav.init():
                raise RuntimeError("GalbotNavigation.init 失败")
            self.g1, self.robot, self.nav = g1, robot, nav
            self.sdk_error = ""
            return True
        except Exception as e:      # noqa: BLE001  SDK 抛出的异常类型不固定
            self.sdk_error = f"SDK 初始化失败：{e}"
            return False

    # ------------------------------------------------------------ 指令
    def command(self, vx, vy, wz, safe, source="web"):
        clamp = lambda v, m: max(-m, min(m, float(v)))
        with self.lock:
            if self.estop:
                return False, "急停中，请先解除"
            if self.calib is not None:
                return False, "车身标定中，请稍候"
            now = time.time()
            self.target = (clamp(vx, MAX_V), clamp(vy, MAX_V), clamp(wz, MAX_W), bool(safe), now, str(source)[:16])
            if any(self.target[:3]):
                self.last_by_source[self.target[5]] = now
        return True, ""

    def others(self, source):
        now = time.time()
        return {k: round(now - t, 2) for k, t in self.last_by_source.items() if k != source}

    def stop(self, latch=False):
        with self.lock:
            self.target = None
            if latch:
                self.estop = True
            self._halt("急停" if latch else "停止")

    def release(self):
        with self.lock:
            self.estop = False

    def start_calibration(self):
        with self.lock:
            if self.moving or self.calib is not None:
                return False, "机器人在运动或已在标定"
            self.target = None
            self.calib = []
            self.calib_msg = "标定中…"
        return True, "开始标定，约 2 s，期间请勿靠近机器人"

    def _halt(self, why):
        if self.robot is None:
            return
        try:
            self.robot.stop_base()
            self.nav.stop_navigation()
        except Exception as e:      # noqa: BLE001
            why += f"（停车调用异常：{e}）"
        self.moving = False
        self.applied = (0.0, 0.0, 0.0)
        self.last_result = why

    def _read_sensors(self):
        """每个周期读一次 Mid-360 和超声波，喂给避障。"""
        if self.lidar_tf is None:
            tf, _ = self.robot.get_sensor_extrinsic(self.g1.SensorType.BASE_LIDAR)
            if not tf:
                return
            x, y, z, qx, qy, qz, qw = tf
            R = np.array([
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
            self.lidar_tf = (R, np.array([x, y, z]))
        d = self.robot.get_lidar_data(self.g1.SensorType.BASE_LIDAR)
        stamp = d["header"]["timestamp_ns"] if d else 0
        if stamp and stamp != self.lidar_stamp:
            self.lidar_stamp = stamp
            f = {x["name"]: x["offset"] for x in d["fields"]}
            pts = np.frombuffer(d["data"], dtype=np.dtype({
                "names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                "offsets": [f["x"], f["y"], f["z"]], "itemsize": d["point_step"]}))
            xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64)
            xyz = xyz[np.einsum("ij,ij->i", xyz, xyz) > 0.01]       # 无效点为 (0,0,0)
            R, t = self.lidar_tf
            xyz = xyz @ R.T + t
            if self.calib is not None:
                self.calib.append(xyz)
                if len(self.calib) >= CAL_FRAMES:
                    self.calib_msg = self.safety.calibrate(self.calib)
                    self.calib = None
            self.safety.update_lidar(xyz)
        readings = {}
        for name in ULTRASONIC:
            u = self.robot.get_ultrasonic_data(getattr(self.g1.UltrasonicType, name))
            readings[name] = u.get("distance") if u else None
        self.safety.update_ultrasonic(readings)

    def _send(self, vx, vy, wz, safe):
        if safe:
            lim = self.safety.limit(vx, vy, wz)
            reason = lim[3]
            # 避障要求停下的分量立即停，不走减速斜坡
            hard = [lim[i] == 0 and (vx, vy, wz)[i] != 0 for i in range(3)]
            vx, vy, wz = lim[:3]
        else:
            reason, hard = "避障已关闭", [False] * 3
        dt = 1 / RATE
        out = []
        for i, (cur, tgt) in enumerate(zip(self.applied, (vx, vy, wz))):
            if hard[i]:
                out.append(0.0)
                continue
            acc, dec = (ACC_V, DEC_V) if i < 2 else (ACC_W, DEC_W)
            step = (acc if abs(tgt) > abs(cur) and tgt * cur >= 0 else dec) * dt
            out.append(cur + max(-step, min(step, tgt - cur)))
        self.applied = tuple(round(v, 3) for v in out)
        if not any(self.applied):
            self.robot.stop_base()
            return reason or "已停车"
        status = self.robot.set_base_velocity([out[0], out[1], 0.0], [0.0, 0.0, out[2]], CMD_DURATION)
        return reason or str(status)

    def loop(self):
        while True:
            t0 = time.time()
            with self.lock:
                target = self.target
                if self.robot is not None:
                    try:
                        self._read_sensors()
                    except Exception as e:      # noqa: BLE001
                        self.last_result = f"读取传感器异常：{e}"
                    age = t0 - target[4] if target else math.inf
                    if self.moving and age > HALT_AFTER:
                        self._halt(f"超过 {HALT_AFTER} s 没收到指令，已停车")
                    elif self.moving or (age < HEARTBEAT and any(target[:3])):
                        # 指令过期或为 0 时目标速度为 0，按减速度平顺停下
                        cmd = target[:3] if age < HEARTBEAT else (0.0, 0.0, 0.0)
                        try:
                            self.last_result = self._send(*cmd, target[3] if target else True)
                            self.moving = any(self.applied)
                            if age >= HEARTBEAT and not self.moving:
                                self.last_result = f"超过 {HEARTBEAT} s 没收到指令，已停车"
                        except Exception as e:      # noqa: BLE001
                            self.last_result = f"指令异常：{e}"
            time.sleep(max(0.0, 1 / RATE - (time.time() - t0)))

    # ------------------------------------------------------------ 状态与相机
    def status(self):
        st = {"sdk_ok": self.robot is not None, "sdk_error": self.sdk_error, "estop": self.estop,
              "moving": self.moving, "last_result": self.last_result, "max_v": MAX_V, "max_w": MAX_W,
              "applied": self.applied, "calibrating": self.calib is not None, "calib_msg": self.calib_msg,
              "source": self.target[5] if self.target else ""}
        if self.robot is None:
            return st
        odom = self.robot.get_odom()
        if odom:
            qx, qy, qz, qw = odom["orientation"]
            st["odom"] = {
                "x": round(odom["position"][0], 3), "y": round(odom["position"][1], 3),
                "yaw": round(math.degrees(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))), 1),
                "vx": round(odom["linear_velocity"][0], 3), "vy": round(odom["linear_velocity"][1], 3),
                "wz": round(odom["angular_velocity"][2], 3),
            }
        try:
            st["nav_status"] = str(self.nav.get_navigation_status())
        except Exception as e:      # noqa: BLE001
            st["nav_status"] = f"读取失败：{e}"
        return st

    def camera(self, name, full):
        if self.robot is None or name not in CAMERAS:
            return None
        d = self.robot.get_rgb_data(getattr(self.g1.SensorType, f"{name.upper()}_SURROUND_CAMERA"))
        jpg = d.get("data") if d else None
        if not jpg or full:
            return jpg
        cached = self.thumbs.get(name)
        if cached and cached[0] == jpg:
            return cached[1]
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_REDUCED_COLOR_4)
        thumb = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()
        self.thumbs[name] = (jpg, thumb)
        return thumb


base = Base()


class Handler(BaseHandler):
    index_page = "teleop.html"

    def do_GET(self):
        path, _, query = self.path.partition("?")
        cam = CAMERA_PATH.match(path)
        if path == "/api/status":
            base.ensure_sdk()
            self.send(200, base.status())
        elif path == "/api/scan":
            self.send(200, base.safety.scan())
        elif cam:
            jpg = base.camera(cam.group(1), "full=1" in query.split("&"))
            if jpg:
                self.send(200, jpg, "image/jpeg")
            else:
                self.send(503, {"error": "暂无画面"})
        else:
            self.static(path)

    def do_POST(self):
        if not base.ensure_sdk():
            return self.send(503, {"ok": False, "msg": base.sdk_error})
        if self.path == "/api/cmd":
            b = self.body_json()
            try:
                ok, msg = base.command(b.get("vx", 0), b.get("vy", 0), b.get("wz", 0), b.get("safe", True),
                                       b.get("source", "web"))
            except (TypeError, ValueError):
                ok, msg = False, "参数错误"
            self.send(200 if ok else 409, {"ok": ok, "msg": msg, "estop": base.estop, "applied": base.applied,
                                           "result": base.last_result, "limit": base.safety.last,
                                           "others": base.others(str(b.get("source", "web"))[:16])})
        elif self.path == "/api/stop":
            base.stop(latch=True)
            self.send(200, {"ok": True})
        elif self.path == "/api/release":
            base.release()
            self.send(200, {"ok": True})
        elif self.path == "/api/calibrate":
            ok, msg = base.start_calibration()
            self.send(200 if ok else 409, {"ok": ok, "msg": msg})
        else:
            self.send(404, {"error": "not found"})

    def log_message(self, fmt, *args):
        line = str(args[0] if args else "")
        if any(p in line for p in ("/api/stop", "/api/release")):
            super().log_message(fmt, *args)


def main():
    threading.Thread(target=base.loop, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"Galbot G1 底盘遥控: http://0.0.0.0:{PORT}", flush=True)
    base.ensure_sdk()
    # SDK 初始化后会在 C++ 层接管 SIGINT/SIGTERM，只能看它的运行状态来退出
    try:
        while base.robot is None or base.robot.is_running():
            time.sleep(0.3)
    except KeyboardInterrupt:
        pass
    base.stop()
    if base.robot is not None:
        base.robot.request_shutdown()
        base.robot.wait_for_shutdown()
        base.robot.destroy()
    sys.exit(0)


if __name__ == "__main__":
    main()
