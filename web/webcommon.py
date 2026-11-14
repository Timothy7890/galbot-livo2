"""web/server.py（7010 录制）用的 HTTP 工具。"""
import json
import mimetypes
import os
import subprocess
import time
import urllib.error
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


class CodeWatch:
    """服务进程启动后源码又被更新（git pull / 部署）而没重启时，页面是新的、接口还是旧的，表现为各种莫名其妙的失败。
    记下启动时这些文件的修改时间，之后变了就在状态里带上 stale，页面提示重启 web/web.sh restart。"""

    def __init__(self, *files):
        self.files = {Path(f): self._mtime(Path(f)) for f in files}

    @staticmethod
    def _mtime(f):
        try:
            return f.stat().st_mtime
        except OSError:
            return None

    def stale(self):
        return [f.name for f, t in self.files.items() if self._mtime(f) != t]


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
        if path.startswith("/api/"):        # 未知接口不能回落到入口页（200 + HTML），调用方会当成功
            return self.send(404, {"ok": False, "error": "not found", "msg": f"服务端没有接口 {path}（代码更新后没重启？web/web.sh restart）"})
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

    def proxy_post(self, url, body, unavailable_msg):
        req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                self.send(r.status, r.read())
        except urllib.error.HTTPError as e:
            self.send(e.code, e.read())
        except OSError:
            self.send(503, {"ok": False, "msg": unavailable_msg})

    def log_message(self, fmt, *args):
        # 只记操作（POST）与少数 GET；轮询、心跳、取图不记，否则日志一天几十万行
        line = str(args[0] if args else "")
        if "/api/" in line and line.startswith("POST") and "/heartbeat" not in line:
            super().log_message(fmt, *args)
