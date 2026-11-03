#!/usr/bin/env python3
"""Galbot G1 录制控制网页（端口 7010）。录制操作都通过 ../record.sh 完成。

    GET  /                控制页面（web/frontend 构建产物 web/dist/index.html）
    GET  /api/status      当前状态 + 历史录制列表
    GET  /api/live        实时地图 SSE 流（转发录制中的 scripts/live_map.py）
    GET  /api/camera.jpg  主相机最新画面（录制中）
    POST /api/start       开始录制
    POST /api/stop        停止录制（后台执行，状态变为 stopping）
"""
import os
import threading
import urllib.request
from http.server import ThreadingHTTPServer

from webcommon import DATA_ROOT, PROJECT_DIR, BaseHandler, LastError, last_line, parse_kv, run_script

PORT = int(os.environ.get("RECORD_WEB_PORT", "7010"))
RECORD_SH = PROJECT_DIR / "record.sh"
LIVE_URL = "http://127.0.0.1:7012/stream"
CAMERA_URL = "http://127.0.0.1:7012/camera.jpg"

busy = threading.Lock()   # 同一时间只允许一个 start/stop
last_error = LastError()


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
        })
    return sessions


def status_payload():
    _, out = run_script(RECORD_SH, "status", timeout=10)
    st = parse_kv(out)
    if busy.locked() and st.get("status") in (None, "idle"):
        st["status"] = "starting"
    st["error"] = last_error.get()
    st["data_root"] = str(DATA_ROOT)
    st["sessions"] = list_sessions()
    return st


def do_stop():
    with busy:
        code, out = run_script(RECORD_SH, "stop", timeout=600)
        last_error.set("" if code == 0 else last_line(out, "停止失败"))


class Handler(BaseHandler):
    index_page = "index.html"

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/api/status":
            self.send(200, status_payload())
        elif path == "/api/live":
            self.proxy_sse(LIVE_URL, "未在录制")
        elif path == "/api/camera.jpg":
            try:
                with urllib.request.urlopen(CAMERA_URL, timeout=3) as r:
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
