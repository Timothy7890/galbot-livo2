#!/usr/bin/env python3
"""Galbot G1 底盘遥控网页（端口 7011）。经 Galbot SDK 控制底盘，需在 sdk_env.sh 下运行（web/web.sh 已处理）。

    GET  /                  控制页面（web/dist/teleop.html）
    GET  /api/status        SDK 状态、里程计、最近一次指令结果
    GET  /api/camera/<名称>.jpg  环视相机最新画面（默认 320x240 预览，?full=1 原图）
    POST /api/cmd           {"vx", "vy", "wz", "safe"} 速度指令，按住期间网页每 0.1 s 发一次
    POST /api/stop          急停（锁定，之后的速度指令一律忽略）
    POST /api/release       解除急停

安全：
- 发给底盘的每条指令只持续 CMD_DURATION 秒，超时底盘自己停；
- 本服务 HEARTBEAT 秒内没收到新指令（松手、网页断线、网络卡顿）立即 stop_base；
- safe=true 走 navigate_with_velocity（Galbot 融合全部传感器的碰撞检测），false 走 set_base_velocity（无避障）。
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

from webcommon import BaseHandler

PORT = int(os.environ.get("TELEOP_WEB_PORT", "7011"))
MAX_V = 0.5          # m/s
MAX_W = 1.0          # rad/s
CMD_DURATION = 0.5   # 每条指令在底盘侧的持续时间 (s)
HEARTBEAT = 0.35     # 超过这么久没收到网页指令就停车 (s)
RATE = 10            # 下发频率 (Hz)
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
        self.target = None          # (vx, vy, wz, safe, 收到时间)
        self.moving = False
        self.estop = False
        self.last_result = ""
        self.thumbs = {}            # 名称 -> (原图 JPEG, 预览 JPEG)

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
    def command(self, vx, vy, wz, safe):
        clamp = lambda v, m: max(-m, min(m, float(v)))
        with self.lock:
            if self.estop:
                return False, "急停中，请先解除"
            self.target = (clamp(vx, MAX_V), clamp(vy, MAX_V), clamp(wz, MAX_W), bool(safe), time.time())
        return True, ""

    def stop(self, latch=False):
        with self.lock:
            self.target = None
            if latch:
                self.estop = True
            self._halt("急停" if latch else "停止")

    def release(self):
        with self.lock:
            self.estop = False

    def _halt(self, why):
        if self.robot is None:
            return
        try:
            self.robot.stop_base()
            self.nav.stop_navigation()
        except Exception as e:      # noqa: BLE001
            why += f"（停车调用异常：{e}）"
        self.moving = False
        self.last_result = why

    def _send(self, vx, vy, wz, safe):
        if safe:
            ok, msg = self.nav.navigate_with_velocity(vx, vy, wz, CMD_DURATION, True)
            return f"{'OK' if ok else '被拒绝'} {msg}"
        status = self.robot.set_base_velocity([vx, vy, 0.0], [0.0, 0.0, wz], CMD_DURATION)
        return str(status)

    def loop(self):
        while True:
            t0 = time.time()
            with self.lock:
                target = self.target
                if self.robot is not None:
                    fresh = target and t0 - target[4] < HEARTBEAT
                    still = target and not any(target[:3])
                    if fresh and not still:
                        try:
                            self.last_result = self._send(*target[:4])
                            self.moving = True
                        except Exception as e:      # noqa: BLE001
                            self.last_result = f"指令异常：{e}"
                    elif self.moving:
                        self._halt("已停车" if still else "超过 0.35 s 没收到指令，已停车")
            time.sleep(max(0.0, 1 / RATE - (time.time() - t0)))

    # ------------------------------------------------------------ 状态与相机
    def status(self):
        st = {"sdk_ok": self.robot is not None, "sdk_error": self.sdk_error, "estop": self.estop,
              "moving": self.moving, "last_result": self.last_result, "max_v": MAX_V, "max_w": MAX_W}
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
                ok, msg = base.command(b.get("vx", 0), b.get("vy", 0), b.get("wz", 0), b.get("safe", True))
            except (TypeError, ValueError):
                ok, msg = False, "参数错误"
            self.send(200 if ok else 409, {"ok": ok, "msg": msg})
        elif self.path == "/api/stop":
            base.stop(latch=True)
            self.send(200, {"ok": True})
        elif self.path == "/api/release":
            base.release()
            self.send(200, {"ok": True})
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
