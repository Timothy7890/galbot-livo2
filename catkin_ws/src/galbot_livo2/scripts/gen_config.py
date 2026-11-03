#!/usr/bin/env python3
"""从 Galbot SDK 读外参和主相机内参，生成 FAST-LIVO2 的标定配置。

  sdk_env.sh python3 gen_config.py [--camera left_front] [--overlay out.jpg]

生成 config/g1_calib.yaml（IMU-雷达、雷达-相机外参，主相机名，IMU 在底盘上的位姿）和 config/camera_g1.yaml（相机模型）。
--overlay 把一帧点云按生成的外参投影到主相机图像上，用来目检外参是否正确。
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
import numpy as np

from galbot_io import CAMERAS, IMU, LIDAR, close_robot, lidar_xyz, open_robot, pose_in_base, wait_for

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")
IMAGE_SCALE = 0.5  # FAST-LIVO2 内部把图像缩到 640x480 处理，Orin 上省算力


def fmt(v):
    return "[" + ", ".join(f"{x:.6f}" for x in np.ravel(v)) + "]"


def pose7(T):
    """4x4 -> [x, y, z, qx, qy, qz, qw]（旋转为绕 z 轴等常见情形，取 w 最大的分支即可）。"""
    R = T[:3, :3]
    w = np.sqrt(max(0.0, 1 + np.trace(R))) / 2
    if w > 1e-6:
        q = [(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(max(0.0, 1 + R[i, i] - R[j, j] - R[k, k])) * 2
        q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
        q[i], q[j], q[k] = s / 4, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
    return list(T[:3, 3]) + q


def project(xyz_cam, K):
    """等距鱼眼（Kannala-Brandt，畸变系数全 0）投影：像平面半径 = f·θ。"""
    x, y, z = xyz_cam.T
    r = np.hypot(x, y)
    theta = np.arctan2(r, z)
    s = np.where(r > 1e-9, theta / np.maximum(r, 1e-9), 0)
    return np.stack([K[0] * x * s + K[2], K[4] * y * s + K[5]], axis=1), theta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="left_front", choices=list(CAMERAS))
    ap.add_argument("--overlay", help="保存点云投影图的路径")
    args = ap.parse_args()
    cam = CAMERAS[args.camera]

    robot = open_robot({LIDAR, IMU, cam})
    try:
        T_b_l, T_b_i, T_b_c = (pose_in_base(robot, s) for s in (LIDAR, IMU, cam))
        intr = robot.get_camera_intrinsic(cam)
        lidar = wait_for(lambda: robot.get_lidar_data(LIDAR)) if args.overlay else None
        jpg = wait_for(lambda: robot.get_rgb_data(cam)) if args.overlay else None
    finally:
        close_robot(robot)

    if intr["distortion_model"] != "kannala_brandt" or any(abs(d) > 1e-9 for d in intr["D"]):
        sys.exit(f"相机模型是 {intr['distortion_model']} D={intr['D']}，本脚本只处理畸变系数全 0 的等距模型")
    K = intr["K"]

    # FAST-LIVO2 约定：extrinsic_R/T 是雷达在 IMU 系下的位姿；Rcl/Pcl 把雷达系的点变到相机系
    T_i_l = np.linalg.inv(T_b_i) @ T_b_l
    T_c_l = np.linalg.inv(T_b_c) @ T_b_l

    with open(os.path.join(CONFIG_DIR, "g1_calib.yaml"), "w") as f:
        f.write(f"""# 由 scripts/gen_config.py 从 Galbot SDK 生成，勿手改
galbot:
  main_camera: {args.camera}
  imu_in_base: {fmt(pose7(T_b_i))}   # [x y z qx qy qz qw]，FAST-LIVO2 位姿（IMU）换算到底盘用

extrin_calib:
  extrinsic_T: {fmt(T_i_l[:3, 3])}
  extrinsic_R: {fmt(T_i_l[:3, :3])}
  Rcl: {fmt(T_c_l[:3, :3])}
  Pcl: {fmt(T_c_l[:3, 3])}
""")
    with open(os.path.join(CONFIG_DIR, "camera_g1.yaml"), "w") as f:
        f.write(f"""# 由 scripts/gen_config.py 从 Galbot SDK 生成，勿手改。{args.camera}，SDK 给出的等距鱼眼模型
cam_model: EquidistantCamera
cam_width: {intr['width']}
cam_height: {intr['height']}
scale: {IMAGE_SCALE}
cam_fx: {K[0]:.6f}
cam_fy: {K[4]:.6f}
cam_cx: {K[2]:.6f}
cam_cy: {K[5]:.6f}
k1: 0.0
k2: 0.0
k3: 0.0
k4: 0.0
""")
    print("extrinsic_T", fmt(T_i_l[:3, 3]))
    print("extrinsic_R", fmt(T_i_l[:3, :3]))
    print("Rcl", fmt(T_c_l[:3, :3]))
    print("Pcl", fmt(T_c_l[:3, 3]))
    print("已写入", CONFIG_DIR)

    if args.overlay:
        xyz = lidar_xyz(lidar)
        rng = np.linalg.norm(xyz, axis=1)
        for r in (0.3, 0.5, 0.8, 1.2):
            print(f"距雷达 {r} m 内的点：{np.sum(rng < r)} / {len(rng)}")
        pc = xyz @ T_c_l[:3, :3].T + T_c_l[:3, 3]
        uv, theta = project(pc, K)
        img = cv2.imdecode(np.frombuffer(jpg["data"], np.uint8), cv2.IMREAD_COLOR)
        h, w = img.shape[:2]
        ok = (theta < np.radians(95)) & (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
        dist = np.linalg.norm(pc[ok], axis=1)
        colors = cv2.applyColorMap(np.clip(dist / 8 * 255, 0, 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_JET)
        for (u, v), c in zip(uv[ok].astype(int), colors[:, 0]):
            cv2.circle(img, (u, v), 2, tuple(int(x) for x in c), -1)
        cv2.imwrite(args.overlay, img)
        dt = (jpg["header"]["timestamp_ns"] - lidar["header"]["timestamp_ns"]) / 1e6
        print(f"投影 {ok.sum()} 点到 {args.overlay}（图像与点云时间差 {dt:.0f} ms，静止时不影响）")


if __name__ == "__main__":
    main()
