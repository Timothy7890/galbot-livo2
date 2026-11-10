"""导航（成熟地图）共用：读录制的标定、轨迹与 raw.bag 点云，位姿插值，坐标换算。只依赖 numpy / scipy，可离线测试。

坐标系：
    map    成熟地图 = 建图那次录制的 FAST-LIVO2 世界系 camera_init（z 对齐重力）
    odom   本次运行的 FAST-LIVO2 世界系（每次启动都不同）
    imu    FAST-LIVO2 输出的位姿就是 IMU 在世界系下的位姿
    base   底盘 base_link（原点在地面，x 向前），imu_in_base 换算
    lidar  Mid-360；与 IMU 只差平移 extrinsic_T（旋转为单位阵）
"""
import csv
import math
import os
import re

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def as_matrix(rot):
    """Rotation → 3x3（或 Nx3x3）。scipy < 1.4（Ubuntu 20.04 自带 1.3）里叫 as_dcm。"""
    return rot.as_matrix() if hasattr(rot, "as_matrix") else rot.as_dcm()


def from_matrix(R):
    return Rotation.from_matrix(R) if hasattr(Rotation, "from_matrix") else Rotation.from_dcm(R)


def quat_R(qx, qy, qz, qw):
    return as_matrix(Rotation.from_quat([qx, qy, qz, qw]))


def T_from(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def yaw_of(T):
    return math.atan2(T[1, 0], T[0, 0])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_calib(path):
    """g1_calib.yaml（gen_config.py 生成）里的 imu_in_base 与 extrinsic_T。返回 (T_base_imu, t_imu_lidar)。
    只用正则读两行，不依赖 PyYAML。"""
    txt = open(path).read()

    def vec(key):
        m = re.search(rf"{key}:\s*\[([^\]]+)\]", txt)
        if not m:
            raise ValueError(f"{path} 缺少 {key}")
        return [float(v) for v in m.group(1).split(",")]

    p = vec("imu_in_base")
    T_b_i = T_from(quat_R(*p[3:]), p[:3])
    return T_b_i, np.array(vec("extrinsic_T"))


def load_trajectory(path):
    """trajectory.csv（record_odom.py：FAST-LIVO2 的 IMU 位姿，数据时间）→ (t, 位置 Nx3, Rotation)。"""
    t, p, q = [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row["timestamp"]))
            p.append([float(row["x"]), float(row["y"]), float(row["z"])])
            q.append([float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])])
    t = np.array(t)
    order = np.argsort(t)
    t, p, q = t[order], np.array(p)[order], np.array(q)[order]
    keep = np.r_[True, np.diff(t) > 1e-6]
    return t[keep], p[keep], Rotation.from_quat(q[keep])


class PoseInterp:
    """按数据时间插值 IMU 位姿（位置线性、姿态球面线性）。"""

    def __init__(self, t, p, rot):
        self.t, self.p = t, p
        self.slerp = Slerp(t, rot)

    def covers(self, ts):
        return self.t[0] <= ts <= self.t[-1]

    def at(self, ts):
        """ts 为标量或数组，返回 (R 数组 Kx3x3 或 3x3, 位置)。"""
        ts = np.clip(ts, self.t[0], self.t[-1])
        R = as_matrix(self.slerp(ts))
        pos = np.stack([np.interp(ts, self.t, self.p[:, i]) for i in range(3)], axis=-1)
        return R, pos


def cloud_fields(msg_fields, data, point_step, n):
    """PointCloud2 → (xyz Nx3 float64, 每点相对帧头的时间 s 或 None)。"""
    off = {f.name: f.offset for f in msg_fields}
    names, offs, fmts = ["x", "y", "z"], [off["x"], off["y"], off["z"]], ["<f4"] * 3
    if "offset_nsec" in off:
        names.append("t")
        offs.append(off["offset_nsec"])
        fmts.append("<u4")
    a = np.frombuffer(data, dtype=np.dtype({"names": names, "formats": fmts, "offsets": offs,
                                            "itemsize": point_step}), count=n)
    xyz = np.stack([a["x"], a["y"], a["z"]], axis=1).astype(np.float64)
    dt = a["t"].astype(np.float64) * 1e-9 if "t" in names else None
    return xyz, dt


def iter_lidar(bag_path, topic="/livox/lidar", start=None, stop=None):
    """逐帧读 raw.bag 的点云：yield (帧头数据时间 s, xyz Nx3, 每点时间偏移 s 或 None)。
    start / stop：只读这段录制时间（s，按 bag 记录时间，借助索引跳过其余部分）。
    机器人上用 ROS 的 rosbag；离线没有 ROS 时用 pip 的 rosbags。"""
    try:
        import rosbag
        import rospy
    except ImportError:
        rosbag = None
    if rosbag is not None:
        kw = {}
        if start is not None:
            kw["start_time"] = rospy.Time.from_sec(start)
        if stop is not None:
            kw["end_time"] = rospy.Time.from_sec(stop)
        for _, m, _ in rosbag.Bag(bag_path).read_messages(topics=[topic], **kw):
            xyz, dt = cloud_fields(m.fields, m.data, m.point_step, m.width * m.height)
            yield m.header.stamp.to_sec(), xyz, dt
        return
    from rosbags.highlevel import AnyReader
    from pathlib import Path
    with AnyReader([Path(bag_path)]) as r:
        conns = [c for c in r.connections if c.topic == topic]
        kw = {}
        if start is not None:
            kw["start"] = int(start * 1e9)
        if stop is not None:
            kw["stop"] = int(stop * 1e9)
        for c, _, raw in r.messages(connections=conns, **kw):
            m = r.deserialize(raw, c.msgtype)
            xyz, dt = cloud_fields(m.fields, bytes(m.data), m.point_step, m.width * m.height)
            yield m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, xyz, dt


def session_paths(session):
    conf = os.path.join(session, "config", "g1_calib.yaml")
    if not os.path.exists(conf):     # 旧录制没有 config/，用包内的
        conf = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "catkin_ws", "src", "galbot_livo2",
                            "config", "g1_calib.yaml")
    return {"calib": conf, "traj": os.path.join(session, "trajectory.csv"), "bag": os.path.join(session, "raw.bag"),
            "nav": os.path.join(session, "nav")}
