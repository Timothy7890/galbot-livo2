"""Galbot SDK 传感器读取与坐标变换的公用部分。"""
import time

import numpy as np
from galbot_sdk import g1

S = g1.SensorType
LIDAR, IMU = S.BASE_LIDAR, S.LIDAR_IMU
CAMERAS = {
    "left_front": S.LEFT_FRONT_SURROUND_CAMERA,
    "right_front": S.RIGHT_FRONT_SURROUND_CAMERA,
    "left_rear": S.LEFT_REAR_SURROUND_CAMERA,
    "right_rear": S.RIGHT_REAR_SURROUND_CAMERA,
}


def open_robot(sensors):
    robot = g1.GalbotRobot()
    if not robot.init(set(sensors)):
        raise RuntimeError("Galbot SDK 初始化失败")
    return robot


def close_robot(robot):
    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()


def wait_for(get, timeout=5.0):
    """SDK 刚初始化时接口返回空字典，等到有数据为止。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        d = get()
        if d:
            return d
        time.sleep(0.05)
    raise RuntimeError(f"等待传感器数据超时：{get}")


def pose_in_base(robot, sensor):
    """传感器在 base_link 下的位姿，4x4。SDK 返回 [x, y, z, qx, qy, qz, qw]。"""
    tf = wait_for(lambda: robot.get_sensor_extrinsic(sensor)[0])
    x, y, z, qx, qy, qz, qw = tf
    T = np.eye(4)
    T[:3, :3] = [
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ]
    T[:3, 3] = x, y, z
    return T


def lidar_xyz(d):
    """SDK 点云字典 -> Nx3 float 数组（雷达坐标系）。"""
    names = [f["name"] for f in d["fields"]]
    offs = [f["offset"] for f in d["fields"]]
    xyz = np.frombuffer(d["data"], dtype=np.dtype({
        "names": names[:3], "formats": ["<f4"] * 3, "offsets": offs[:3], "itemsize": d["point_step"]}))
    return np.stack([xyz["x"], xyz["y"], xyz["z"]], axis=1).astype(np.float64)
