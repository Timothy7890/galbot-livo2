#!/usr/bin/env python3
"""Galbot G1 录制控制网页（端口 7010）。录制操作都通过 ../record.sh 完成。

    GET  /                控制页面（web/frontend 构建产物 web/dist/index.html）
    GET  /api/status      当前状态 + 历史录制列表
    GET  /api/live        实时地图 SSE 流（转发录制中的 scripts/live_map.py）
    GET  /api/camera/<名称>.jpg  环视相机最新画面（录制中；默认 320x240 预览，?full=1 原图）
    GET  /api/explore     自主建图状态（转发录制中的 scripts/explorer.py）；/api/explore/grid.png 2D 栅格图
    POST /api/explore/enable | disable | cancel   进入（可带 {"strategy"}）/ 退出自主建图，取消完成后的自动停止录制
    POST /api/start       开始录制
    POST /api/stop        停止录制（后台执行，状态变为 stopping）
    POST /api/estop       急停：转发给 7011 遥控服务（底盘与躯干只由它控制），锁定后底盘、自主建图、躯干升降都停
    POST /api/estop/release  解除急停（同样转发给 7011）

导航（成熟地图，见 ../nav.sh、scripts/nav_map.py、scripts/navigator.py；页面 /nav.html）：
    GET  /api/nav             导航运行状态（启动中带 step 进度）+ 可用地图（有 nav/ 的录制）+ 最近一次启动失败
    GET  /api/nav/stream      运行中 navigator 的实时状态 SSE（每 0.2 s，转发 7014 /stream）
    GET  /api/nav/grid.png    运行中：实时栅格；否则 ?session=<录制> 取该录制的地图图片
    GET  /api/nav/traj?session=<录制>   建图轨迹 [[x, y], ...]
    POST /api/nav/build       {"session", "name"} 由录制生成导航地图（后台，进度在录制列表的 nav_progress）
    POST /api/nav/start       {"session", "vision", "dry_run"} 启动导航（后台，立即返回，进度看 /api/nav）；/api/nav/stop 停止（后台）
    POST /api/nav/goto {"x","y"[,"yaw"]} | cancel | reloc {"mode", "x","y","yaw"} | confirm | speed {"v"} | heartbeat
                              转发给 navigator（7014）
状态里的 stale：本服务启动后 server.py / webcommon.py 又被更新而没重启，页面据此提示。
"""
import json
import os
import re
import subprocess
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

from webcommon import DATA_ROOT, PROJECT_DIR, BaseHandler, CodeWatch, LastError, last_line, parse_kv, run_script

PORT = int(os.environ.get("RECORD_WEB_PORT", "7010"))
RECORD_SH = PROJECT_DIR / "record.sh"
LIVE_URL = "http://127.0.0.1:7012/stream"
CAMERA_URL = "http://127.0.0.1:7012/camera/"
EXPLORE_URL = "http://127.0.0.1:7013/"
NAV_URL = "http://127.0.0.1:7014/"
NAV_SH = PROJECT_DIR / "nav.sh"
NAV_MAP = PROJECT_DIR / "scripts/nav_map.py"
SESSION_NAME = re.compile(r"^\d{8}_\d{6}$")
TELEOP_URL = "http://127.0.0.1:7011/api/"
CAMERA_PATH = re.compile(r"^/api/camera/(\w+)\.jpg$")
CALIB = PROJECT_DIR / "catkin_ws/src/galbot_livo2/config/g1_calib.yaml"


def main_camera():
    """FAST-LIVO2 用的主相机名（g1_calib.yaml 的 galbot/main_camera）。"""
    m = re.search(r"^\s*main_camera:\s*(\w+)", CALIB.read_text(), re.M) if CALIB.exists() else None
    return m.group(1) if m else "left_front"


busy = threading.Lock()   # 同一时间只允许一个 start/stop
last_error = LastError()
code_watch = CodeWatch(__file__, PROJECT_DIR / "web/webcommon.py")


def list_sessions(limit=30):
    if not DATA_ROOT.is_dir():
        return []
    sessions = []
    for d in sorted(DATA_ROOT.iterdir(), reverse=True)[:limit]:
        if not d.is_dir():
            continue
        info_file = d / "info.txt"
        info = parse_kv(info_file.read_text(errors="replace")) if info_file.exists() else {}
        sessions.append({
            "name": d.name,
            "status": info.get("status", "recording"),
            "duration_s": info.get("duration_s"),
            "bag_size": info.get("bag_size"),
            "map_points": info.get("map_points"),
            "error": info.get("error"),
            "nav": nav_meta(d),
            "nav_progress": nav_progress(d),
        })
    return sessions


def nav_meta(d):
    f = d / "nav" / "meta.json"
    try:
        return json.loads(f.read_text()) if f.exists() else None
    except ValueError:
        return None


def nav_progress(d):
    """生成导航地图的进度；进程已不在（断电等）而进度仍是 running 的，标为中断。"""
    f = d / "nav_progress.json"
    try:
        p = json.loads(f.read_text()) if f.exists() else None
    except ValueError:
        return None
    if p and p.get("state") == "running" and d.name not in builders:
        p = {**p, "state": "failed", "msg": "生成中断（服务重启或断电），请重新生成"}
    return p


builders = {}       # 录制名 -> 正在生成导航地图的 Popen


def build_nav(name, title):
    d = DATA_ROOT / name
    if not SESSION_NAME.match(name) or not d.is_dir():
        return False, "录制不存在"
    for k, p in list(builders.items()):
        if p.poll() is not None:
            builders.pop(k)
    if name in builders:
        return False, "这次录制的导航地图正在生成"
    if builders:
        return False, "已有一个导航地图在生成，请等它完成"
    st = parse_kv(run_script(RECORD_SH, "status", timeout=10)[1])
    if st.get("session", "").rstrip("/").endswith(name):
        return False, "这次录制还没结束"
    if not (d / "trajectory.csv").exists():
        return False, "这次录制没有轨迹 trajectory.csv"
    title = re.sub(r"[\x00-\x1f]", "", str(title or name))[:40]
    cmd = f'source /opt/ros/noetic/setup.bash; exec nice -n 10 python3 "{NAV_MAP}" "{d}" "$1"'
    log = open(d / "nav_build.log", "a")
    builders[name] = subprocess.Popen(["bash", "-c", cmd, "nav_map", title], stdout=log, stderr=subprocess.STDOUT,
                                      stdin=subprocess.DEVNULL, start_new_session=True)
    return True, "开始生成导航地图"


def nav_status():
    _, out = run_script(NAV_SH, "status", timeout=10)
    return parse_kv(out)


_maps_cache = {"t": 0.0, "maps": []}


def nav_maps():
    """可用的导航地图。每张要读 meta.json，导航页 0.5–2 s 轮询一次，缓存 3 s。"""
    if time.time() - _maps_cache["t"] > 3:
        _maps_cache.update(t=time.time(),
                           maps=[{"session": s["name"], **s["nav"]} for s in list_sessions(100) if s.get("nav")])
    return _maps_cache["maps"]


def nav_payload():
    st = nav_status()
    status = st.get("status", "idle")
    if nav_busy.locked() and status == "idle":
        status = nav_op["op"] or "starting"         # 脚本刚起、还没写状态文件
    out = {"status": status, "map": st.get("map"), "error": nav_error.get(), "maps": nav_maps(),
           "vision": st.get("vision") != "false", "dry_run": st.get("dry_run") == "true",
           "elapsed_s": st.get("elapsed_s"), "step": st.get("step"), "step_elapsed_s": st.get("step_elapsed_s"),
           "start_fail": nav_op["fail"], "estop": teleop_estop(), "stale": code_watch.stale(),
           "recording": recording_now()}
    if nav_busy.locked() and nav_op["op"] == "stopping":
        out["status"] = "stopping"
    return out


def recording_now():
    """是否在录制（含启动中、停止中）。直接看 record.sh 的状态文件，不为这一个标志再跑一次脚本。"""
    try:
        return parse_kv((PROJECT_DIR / "run/state").read_text()).get("status", "idle") != "idle"
    except OSError:
        return False


def nav_traj(name, max_pts=800):
    import numpy as np
    t = np.load(DATA_ROOT / name / "nav" / "traj.npz")["traj"]
    step = max(1, len(t) // max_pts)
    return np.round(t[::step, 1:3], 3).tolist()


nav_busy = threading.Lock()
nav_error = LastError()
nav_op = {"op": "", "fail": None}       # 进行中的操作（starting / stopping）；最近一次启动失败 {msg, step}，下次启动时清掉


def nav_run(op, *args):
    """后台执行 nav.sh start / stop（调用方已拿到 nav_busy）。"""
    try:
        last_step = {"step": ""}
        if op == "starting":
            # 启动中记下走到哪一步：失败后状态文件就没了，页面要标出停在哪
            def watch():
                while nav_busy.locked():
                    last_step["step"] = nav_status().get("step") or last_step["step"]
                    time.sleep(0.5)
            threading.Thread(target=watch, daemon=True).start()
        try:
            code, out = run_script(NAV_SH, *args, timeout=150)
        except subprocess.TimeoutExpired:
            code, out = 1, "操作超时（150 s）"
        nav_error.set("" if code == 0 else last_line(out, "操作失败"))
        if op == "starting" and code != 0:
            line = next((l for l in reversed(out.splitlines()) if "启动失败" in l), last_line(out, "启动失败"))
            nav_op["fail"] = {"msg": re.sub(r"^\[[\d:]+\]\s*(启动失败[:：]\s*)?", "", line), "step": last_step["step"]}
    finally:
        nav_op["op"] = ""
        nav_busy.release()


def status_payload():
    _, out = run_script(RECORD_SH, "status", timeout=10)
    st = parse_kv(out)
    if busy.locked() and st.get("status") in (None, "idle"):
        st["status"] = "starting"
    st["error"] = last_error.get()
    st["data_root"] = str(DATA_ROOT)
    st["main_camera"] = main_camera()
    st["sessions"] = list_sessions()
    st["estop"] = teleop_estop()
    st["nav"] = nav_status().get("status", "idle")
    st["stale"] = code_watch.stale()
    return st


def teleop_estop():
    """7011 的急停状态；7011 未运行或 SDK 未就绪时为 None（急停按钮不可用）。"""
    try:
        with urllib.request.urlopen(TELEOP_URL + "status", timeout=2) as r:
            d = json.loads(r.read())
        return bool(d.get("estop")) if d.get("sdk_ok") else None
    except (OSError, ValueError):
        return None


def teleop_post(action):
    req = urllib.request.Request(TELEOP_URL + action, data=b"", method="POST")
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


def do_stop():
    with busy:
        code, out = run_script(RECORD_SH, "stop", timeout=600)
        last_error.set("" if code == 0 else last_line(out, "停止失败"))


class Handler(BaseHandler):
    index_page = "index.html"

    def do_GET(self):
        path, _, query = self.path.partition("?")
        cam = CAMERA_PATH.match(path)
        if path == "/api/status":
            self.send(200, status_payload())
        elif path == "/api/live":
            self.proxy_sse(LIVE_URL, "未在录制")
        elif path in ("/api/explore", "/api/explore/grid.png"):
            url = EXPLORE_URL + ("grid.png" if path.endswith(".png") else "state")
            try:
                with urllib.request.urlopen(url, timeout=3) as r:
                    self.send(200, r.read(), r.headers.get("Content-Type"))
            except OSError:
                self.send(503, {"error": "未在录制"})
        elif path == "/api/nav":
            self.send(200, nav_payload())
        elif path == "/api/nav/stream":
            self.proxy_sse(NAV_URL + "stream", "导航未运行")
        elif path == "/api/nav/traj":
            q = dict(kv.split("=", 1) for kv in query.split("&") if "=" in kv)
            name = q.get("session", "")
            if not SESSION_NAME.match(name) or not (DATA_ROOT / name / "nav" / "traj.npz").exists():
                return self.send(404, {"error": "没有这张地图"})
            try:
                self.send(200, nav_traj(name))
            except Exception as e:      # noqa: BLE001  numpy 没装、文件坏了都只是不显示轨迹
                self.send(500, {"error": str(e)})
        elif path == "/api/nav/grid.png":
            q = dict(kv.split("=", 1) for kv in query.split("&") if "=" in kv)
            name = q.get("session", "")
            if name:
                f = DATA_ROOT / name / "nav" / "grid.png"
                if SESSION_NAME.match(name) and f.exists():
                    return self.send(200, f.read_bytes(), "image/png")
                return self.send(404, {"error": "没有这张地图"})
            try:
                with urllib.request.urlopen(NAV_URL + "grid.png", timeout=3) as r:
                    self.send(200, r.read(), "image/png")
            except OSError:
                self.send(503, {"error": "导航未运行"})
        elif cam:
            full = "?full=1" if "full=1" in query.split("&") else ""
            try:
                with urllib.request.urlopen(f"{CAMERA_URL}{cam.group(1)}.jpg{full}", timeout=3) as r:
                    self.send(200, r.read(), "image/jpeg")
            except OSError:
                self.send(503, {"error": "未在录制"})
        else:
            self.static(path)

    def do_POST(self):
        if self.path == "/api/start":
            if not busy.acquire(blocking=False):
                return self.send(409, {"ok": False, "msg": "正在执行其他操作"})
            try:
                code, out = run_script(RECORD_SH, "start", timeout=150)
            finally:
                busy.release()
            last_error.set("" if code == 0 else last_line(out, "启动失败"))
            self.send(200 if code == 0 else 500, {"ok": code == 0, "msg": out})
        elif self.path in ("/api/explore/enable", "/api/explore/disable", "/api/explore/cancel"):
            # enable 可带 {"strategy": "balanced|nearest|region"}，原样转发
            body = json.dumps(self.body_json()).encode() if self.path.endswith("/enable") else b""
            req = urllib.request.Request(EXPLORE_URL + self.path.rsplit("/", 1)[1], data=body, method="POST",
                                         headers={"Content-Type": "application/json"})
            try:
                with urllib.request.urlopen(req, timeout=5) as r:
                    self.send(r.status, r.read())
            except urllib.error.HTTPError as e:
                self.send(e.code, e.read())
            except OSError:
                self.send(503, {"ok": False, "msg": "未在录制"})
        elif self.path in ("/api/estop", "/api/estop/release"):
            try:
                r = teleop_post("stop" if self.path == "/api/estop" else "release")
                self.send(200, {"ok": bool(r.get("ok")), "msg": r.get("msg", "")})
            except (OSError, ValueError) as e:
                self.send(503, {"ok": False, "msg": f"7011 遥控服务无响应，急停未生效：{e}"})
        elif self.path == "/api/nav/build":
            b = self.body_json()
            ok, msg = build_nav(str(b.get("session", "")), b.get("name"))
            _maps_cache["t"] = 0.0
            self.send(200 if ok else 409, {"ok": ok, "msg": msg})
        elif self.path in ("/api/nav/start", "/api/nav/stop"):
            starting = self.path.endswith("start")
            args = ["stop"]
            if starting:
                b = self.body_json()
                name = str(b.get("session", ""))
                if not SESSION_NAME.match(name) or not (DATA_ROOT / name / "nav" / "meta.json").exists():
                    return self.send(404, {"ok": False, "msg": "没有这张导航地图"})
                args = ["start", str(DATA_ROOT / name / "nav")]
                if b.get("vision") is False:
                    args.append("--no-vision")
                if b.get("dry_run"):
                    args.append("--dry-run")
            if not nav_busy.acquire(blocking=False):
                return self.send(409, {"ok": False, "msg": "正在启动或停止导航"})
            nav_op["op"] = "starting" if starting else "stopping"
            if starting:
                nav_op["fail"] = None
            nav_error.set("")
            threading.Thread(target=nav_run, args=(nav_op["op"], *args), daemon=True).start()
            self.send(202, {"ok": True, "msg": "正在启动导航" if starting else "正在停止导航"})
        elif self.path in ("/api/nav/goto", "/api/nav/cancel", "/api/nav/reloc", "/api/nav/confirm",
                           "/api/nav/speed", "/api/nav/heartbeat"):
            self.proxy_post(NAV_URL + self.path.rsplit("/", 1)[1], self.body_json(), "导航未运行")
        elif self.path == "/api/stop":
            if busy.locked():
                return self.send(409, {"ok": False, "msg": "正在执行其他操作"})
            threading.Thread(target=do_stop, daemon=True).start()
            self.send(202, {"ok": True, "msg": "正在停止并保存"})
        else:
            self.send(404, {"error": "not found"})


if __name__ == "__main__":
    (PROJECT_DIR / "run").mkdir(exist_ok=True)
    print(f"Galbot G1 录制控制: http://0.0.0.0:{PORT}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
