"""web/server.py（7010 录制）用的 HTTP 工具。"""
import json
import mimetypes
import os
import subprocess
import time
import urllib.request
from http.server import BaseHTTPRequestHandler
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent.parent
DATA_ROOT = Path(os.environ.get("GALBOT_DATA_ROOT", Path.home() / "yx/Data/galbot_livo2"))
DIST_DIR = Path(__file__).resolve().parent / "dist"


class LastError:
    """最近一次启动/停止失败的提示，只在一段时间内显示，避免状态恢复后旧报错一直挂在页面上。"""

    def __init__(self, ttl=60):
        self.ttl, self.msg, self.t = ttl, "", 0.0

    def set(self, msg):
        self.msg, self.t = msg, time.time()

    def get(self):
        return self.msg if time.time() - self.t < self.ttl else ""


def run_script(script, *args, timeout=None):
    r = subprocess.run([str(script), *args], capture_output=True, text=True, timeout=timeout)
    return r.returncode, (r.stdout + r.stderr).strip()


def last_line(text, default):
    return text.splitlines()[-1] if text else default


def parse_kv(text):
    out = {}
    for line in text.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            out[k.strip()] = v.strip()
    return out


class BaseHandler(BaseHTTPRequestHandler):
    index_page = "index.html"

    def send(self, code, body, ctype="application/json; charset=utf-8"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def body_json(self):
        n = int(self.headers.get("Content-Length") or 0)
        try:
            return json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return {}

    def send_file(self, f):
        ctype = mimetypes.guess_type(f.name)[0] or "application/octet-stream"
        if ctype.startswith("text/") or ctype.endswith("javascript"):
            ctype += "; charset=utf-8"
        self.send(200, f.read_bytes(), ctype)

    def static(self, path):
        f = (DIST_DIR / path.lstrip("/")).resolve()
        if path == "/" or not f.is_file():
            f = DIST_DIR / self.index_page    # 单页应用：未知路径都返回入口页
        if DIST_DIR.resolve() not in f.resolve().parents or not f.is_file():
            return self.send(503, "前端未构建，请运行 web/frontend 下的 npm run build".encode(),
                             "text/plain; charset=utf-8")
        self.send_file(f)

    def proxy_sse(self, url, unavailable_msg):
        try:
            upstream = urllib.request.urlopen(url, timeout=30)
        except OSError:
            return self.send(503, {"error": unavailable_msg})
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        try:
            while True:
                line = upstream.readline()
                if not line:
                    break
                self.wfile.write(line)
                if line == b"\n":
                    self.wfile.flush()
        except OSError:
            pass
        finally:
            upstream.close()

    def log_message(self, fmt, *args):
        line = str(args[0] if args else "")
        if "/api/" in line and not any(s in line for s in ("/status", "/camera/")):
            super().log_message(fmt, *args)
