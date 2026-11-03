#!/usr/bin/env python3
"""实时俯视地图推送（由 record.sh 随录制启动，录制结束即退出）。

订阅雷达原始点云 /livox/lidar 和 FAST-LIVO2 的 /aft_mapped_to_init（IMU 位姿），用最新位姿和
IMU-雷达外参把原始点云变换到世界坐标系（FAST-LIVO2 的 /cloud_registered 在视觉模式下只含相机视野内的点），
累积到 5 cm 的二维栅格（每格保存最高点高度；高于 Z_CAP 的天花板等不计入，否则俯视图全被天花板盖住），
通过 http://127.0.0.1:7012/stream 以 SSE 推送给 web/server.py：

    event: snapshot  {"cell", "cells": b64(int16 ix,iy,zcm)*, "traj": [[x,y],...], "pose"}
    event: frame     {"d": b64(新增/升高的格子 int16 ix,iy,zcm), "s": b64(当前帧格子 int16 ix,iy), "p": [x,y,yaw]}

另提供 http://127.0.0.1:7012/camera/<名称>.jpg：4 路环视相机最新一帧。默认是缩小到 1/4（320x240）的
预览图，只在有人请求时才解码缩放，每帧只做一次；加 ?full=1 返回原图。
"""
import os

# numpy 默认的多线程 BLAS 会占满所有核心，与 FAST-LIVO2 抢 CPU；单线程即可满足 10 Hz
for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_v] = "1"

import base64
import re
import json
import math
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, PointCloud2

LIDAR_TOPIC = "/livox/lidar"
ODOM_TOPIC = "/aft_mapped_to_init"

CELL = 0.05                 # 栅格边长 (m)
TILE = 256                  # 每个 tile 256x256 格
EMPTY = -32768              # 空格子
Z_STEP_CM = 5               # 高度至少升高 5 cm 才推送更新
SCAN_MAX = 6000             # 每帧推送的当前扫描格子上限
BLIND = 0.5                 # 与 FAST-LIVO2 blind 一致，去掉机器人身上的点 (m)
Z_CAP = 1.6                 # 世界系高度上限 (m)，原点为起始时 IMU（约在地面以上 0.19 m）
PORT = 7012

lock = threading.Lock()
tiles = {}                  # (tx, ty) -> int16[TILE, TILE]，高度 cm
traj = []                   # [[x, y], ...]
pose = None                 # [x, y, yaw]
body_to_world = None        # (R 3x3, t 3)，来自最新里程计
lidar_in_imu = np.zeros(3)  # 雷达在 IMU 系下的位置（extrin_calib/extrinsic_T，旋转为单位阵）
CAMERAS = ("left_front", "right_front", "left_rear", "right_rear")
camera_jpg = {}             # 名称 -> 最新原图 JPEG
camera_thumb = {}           # 名称 -> (原图 JPEG, 预览 JPEG)，按需生成
CAMERA_PATH = re.compile(r"^/camera/(\w+)\.jpg$")
clients = []                # 每个连接一个 queue


def b64(arr):
    return base64.b64encode(np.ascontiguousarray(arr, dtype="<i2").tobytes()).decode()


def broadcast(event, payload):
    msg = f"event: {event}\ndata: {json.dumps(payload, separators=(',', ':'))}\n\n".encode()
    for q in list(clients):
        try:
            q.put_nowait(msg)
        except queue.Full:     # 客户端太慢：丢弃积压，重新发送全量
            with q.mutex:
                q.queue.clear()
            q.put_nowait(b"resync")


def xyz_from_cloud(msg):
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                   "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
    pts = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]


def on_cloud(msg):
    tf = body_to_world
    if tf is None:          # FAST-LIVO2 尚未初始化
        return
    xyz = xyz_from_cloud(msg)
    xyz = xyz[np.einsum("ij,ij->i", xyz, xyz) > BLIND * BLIND]
    if len(xyz) == 0:
        return
    R, t = tf
    xyz = (xyz + lidar_in_imu) @ R.T + t
    xyz = xyz[xyz[:, 2] < Z_CAP]
    ix = np.floor(xyz[:, 0] / CELL).astype(np.int64)
    iy = np.floor(xyz[:, 1] / CELL).astype(np.int64)
    zc = np.clip(np.round(xyz[:, 2] * 100), -32000, 32000).astype(np.int32)
    ok = (np.abs(ix) < 32000) & (np.abs(iy) < 32000)
    ix, iy, zc = ix[ok], iy[ok], zc[ok]

    # 同一格子只保留本帧最高点
    key = (ix << 32) | (iy & 0xFFFFFFFF)
    order = np.lexsort((zc, key))
    k = key[order]
    last = np.r_[k[1:] != k[:-1], True]
    sel = order[last]
    ix, iy, zc = ix[sel], iy[sel], zc[sel]

    delta = []
    with lock:
        tx, ty = ix >> 8, iy >> 8
        tkey = (tx << 32) | (ty & 0xFFFFFFFF)
        for t in np.unique(tkey):
            m = tkey == t
            t_xy = (int(tx[m][0]), int(ty[m][0]))
            arr = tiles.get(t_xy)
            if arr is None:
                arr = tiles[t_xy] = np.full((TILE, TILE), EMPTY, dtype=np.int16)
            lx, ly, z = ix[m] & 255, iy[m] & 255, zc[m]
            old = arr[ly, lx].astype(np.int32)
            upd = (old == EMPTY) | (z >= old + Z_STEP_CM)
            if upd.any():
                arr[ly[upd], lx[upd]] = z[upd]
                delta.append(np.stack([ix[m][upd], iy[m][upd], z[upd]], axis=1))
        p = pose

    d = np.concatenate(delta) if delta else np.empty((0, 3))
    if len(ix) > SCAN_MAX:
        pick = np.random.choice(len(ix), SCAN_MAX, replace=False)
        sx, sy = ix[pick], iy[pick]
    else:
        sx, sy = ix, iy
    broadcast("frame", {"d": b64(d), "s": b64(np.stack([sx, sy], axis=1)), "p": p})


def on_odom(msg):
    global pose, body_to_world
    pp, q = msg.pose.pose.position, msg.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    w, x, y, z = q.w, q.x, q.y, q.z
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    body_to_world = (R, np.array([pp.x, pp.y, pp.z]))
    with lock:
        pose = [round(pp.x, 3), round(pp.y, 3), round(yaw, 4)]
        if not traj or math.hypot(pp.x - traj[-1][0], pp.y - traj[-1][1]) > 0.05:
            traj.append([round(pp.x, 3), round(pp.y, 3)])


def thumbnail(name):
    jpg = camera_jpg.get(name)
    if jpg is None:
        return None
    cached = camera_thumb.get(name)
    if cached and cached[0] is jpg:
        return cached[1]
    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_REDUCED_COLOR_4)  # 解码时直接缩小，比先解码再缩快得多
    thumb = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()
    camera_thumb[name] = (jpg, thumb)
    return thumb


def snapshot():
    with lock:
        parts = []
        for (tx, ty), arr in tiles.items():
            ly, lx = np.nonzero(arr != EMPTY)
            parts.append(np.stack([tx * TILE + lx, ty * TILE + ly, arr[ly, lx]], axis=1))
        cells = np.concatenate(parts) if parts else np.empty((0, 3))
        payload = {"cell": CELL, "cells": b64(cells), "traj": list(traj), "pose": pose}
    return f"event: snapshot\ndata: {json.dumps(payload, separators=(',', ':'))}\n\n".encode()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path, _, query = self.path.partition("?")
        m = CAMERA_PATH.match(path)
        if m:
            return self.send_camera(m.group(1), "full=1" in query.split("&"))
        if self.path != "/stream":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        q = queue.Queue(maxsize=50)
        clients.append(q)
        try:
            self.wfile.write(snapshot())
            self.wfile.flush()
            while not rospy.is_shutdown():
                try:
                    msg = q.get(timeout=10)
                except queue.Empty:
                    msg = b": ping\n\n"
                self.wfile.write(snapshot() if msg == b"resync" else msg)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            clients.remove(q)

    def send_camera(self, name, full):
        jpg = camera_jpg.get(name) if full else thumbnail(name)
        if jpg is None:
            self.send_error(503)
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpg)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(jpg)

    def log_message(self, *args):
        pass


def main():
    rospy.init_node("live_map", anonymous=False, disable_signals=False)
    rospy.Subscriber(LIDAR_TOPIC, PointCloud2, on_cloud, queue_size=1, buff_size=2 ** 24)
    global lidar_in_imu
    lidar_in_imu = np.array(rospy.get_param("extrin_calib/extrinsic_T", [0.0, 0.0, 0.0]))
    rospy.Subscriber(ODOM_TOPIC, Odometry, on_odom, queue_size=10)
    for name in CAMERAS:
        rospy.Subscriber(f"/galbot/{name}/image/compressed", CompressedImage,
                         lambda msg, n=name: camera_jpg.__setitem__(n, bytes(msg.data)),
                         queue_size=1, buff_size=2 ** 22)
    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    rospy.loginfo("live map stream on 127.0.0.1:%d/stream", PORT)
    rospy.spin()
    server.shutdown()


if __name__ == "__main__":
    main()
