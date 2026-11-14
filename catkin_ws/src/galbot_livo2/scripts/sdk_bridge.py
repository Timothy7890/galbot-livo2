#!/usr/bin/env python3
"""Galbot SDK -> ROS：Mid-360 点云和 IMU、环视与头部相机图像、躯干关节角，发布成 FAST-LIVO2 / 录包用的话题。

SDK 只有"取最新一帧"的接口，这里按时间戳变化轮询：IMU 一个线程（200 Hz），点云一个线程（10 Hz），
环视相机一个线程（10 Hz），头部相机一个线程（15 Hz），关节角一个线程（2 Hz）。分开是为了相机多了、
JPEG 大了也不拖慢点云和主相机（FAST-LIVO2 的输入）。时间戳全部沿用 SDK 给的采集时间。

话题：
  /livox/lidar               PointCloud2，字段原样（x y z reflectivity tag time offset_nsec）
  /livox/imu                 Imu，加速度 m/s²
  /galbot/camera/image       主相机 bgr8（FAST-LIVO2 用）
  /galbot/<name>/image/compressed   环视 4 路（~publish_all_cameras）与头部 2 路（~publish_head_cameras）JPEG 原样，
                                    录包后给 3DGS 用；name 为 left_front ... right_rear、head_left、head_right
  /galbot/joint_states       JointState，leg 与 head 关节角（2 Hz）。相机在躯干上、雷达在底盘上，
                             关节一动相机-雷达外参就变：录制中关节变化超过 JOINT_TOL 会记警告
"""
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image, Imu, JointState, PointCloud2, PointField

from galbot_io import CAMERAS, HEAD_CAMERAS, IMU, LIDAR, POSE_JOINT_GROUPS, close_robot, open_robot

IMU_GAP_NS = 7_500_000      # 200 Hz 间隔 5 ms，超过 7.5 ms 记为丢了一帧
FRAME_GAP_NS = 150_000_000  # 10 Hz 间隔 100 ms
HEAD_GAP_NS = 100_000_000   # 15 Hz 间隔 67 ms
JOINT_RATE = 2.0            # 关节角发布频率 (Hz)
JOINT_TOL = 0.02            # 关节角相对录制开始时变化超过此值 (rad) 即警告
MOVED_WARNING = "躯干/头部关节在录制中移动"     # record.sh 停止时在日志里搜这句，写进 info.txt


def stamp(ns):
    return rospy.Time(ns // 1_000_000_000, ns % 1_000_000_000)


class Bridge:
    def __init__(self):
        # ~main_camera：nav.sh 与读标定同时启动本节点，那时参数服务器上还没有 galbot/main_camera
        self.main_cam = rospy.get_param("~main_camera", "") or rospy.get_param("galbot/main_camera", "left_front")
        self.all_cams = rospy.get_param("~publish_all_cameras", True)
        self.head_cams = rospy.get_param("~publish_head_cameras", True)
        # ~publish_main_camera:=false：导航不开视觉时一路相机都不取（FAST-LIVO2 只用雷达 + IMU）
        main_on = rospy.get_param("~publish_main_camera", True)
        surround = list(CAMERAS) if self.all_cams else ([self.main_cam] if main_on else [])
        self.groups = {"surround": {name: CAMERAS[name] for name in surround}}
        if self.head_cams:
            self.groups["head"] = dict(HEAD_CAMERAS)
        sensors = [s for g in self.groups.values() for s in g.values()]

        self.robot = open_robot({LIDAR, IMU, *sensors})
        # ~calib_out:=目录：顺手把标定（g1_calib.yaml / camera_g1.yaml）写到那里，成功后建 .calib_ok、失败建 .calib_fail。
        # nav.sh 开视觉时用它，不必另起 gen_config.py 再初始化一遍 SDK（两个进程同时初始化反而互相拖慢）
        out = rospy.get_param("~calib_out", "")
        if out:
            try:
                import gen_config
                g = gen_config.generate(self.robot, self.main_cam, out, with_all=False)
                rospy.loginfo("标定已写入 %s，主相机离地（底盘系 z）%.3f m", out, g["T_b_c"][2, 3])
                open(os.path.join(out, ".calib_ok"), "w").close()
            except Exception as e:      # noqa: BLE001  SDK 抛出的异常类型不固定
                rospy.logerr("生成标定失败：%s", e)
                with open(os.path.join(out, ".calib_fail"), "w") as f:
                    f.write(str(e))
        self.pub_lidar = rospy.Publisher("/livox/lidar", PointCloud2, queue_size=10)
        self.pub_imu = rospy.Publisher("/livox/imu", Imu, queue_size=400)
        self.pub_img = rospy.Publisher("/galbot/camera/image", Image, queue_size=5)
        self.pub_jpg = {}
        for gname, cams in self.groups.items():
            for name in cams:
                if gname == "head" or self.all_cams:
                    self.pub_jpg[name] = rospy.Publisher(f"/galbot/{name}/image/compressed", CompressedImage,
                                                         queue_size=10)
        self.pub_joints = rospy.Publisher("/galbot/joint_states", JointState, queue_size=10)

        keys = ["imu", "lidar", *[n for g in self.groups.values() for n in g]]
        self.count = dict.fromkeys(keys, 0)
        self.gaps = dict.fromkeys(keys, 0)
        self.lock = threading.Lock()
        self.joint_ref = None       # 启动时的关节角，之后与之比较
        self.joint_moved = 0.0      # 录制以来最大偏离 (rad)

    def tally(self, key, ts, last, gap_ns):
        with self.lock:
            self.count[key] += 1
            if last and ts - last > gap_ns:
                self.gaps[key] += int((ts - last) / (gap_ns / 1.5)) - 1

    def imu_loop(self):
        last = 0
        msg = Imu()
        msg.header.frame_id = "livox_imu"
        msg.orientation_covariance[0] = -1
        while not rospy.is_shutdown():
            d = self.robot.get_imu_data(IMU)
            ts = d.get("timestamp_ns", 0)
            if ts and ts != last:
                msg.header.stamp = stamp(ts)
                a, g = d["accel"], d["gyro"]
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a["x"], a["y"], a["z"]
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = g["x"], g["y"], g["z"]
                self.pub_imu.publish(msg)
                self.tally("imu", ts, last, IMU_GAP_NS)
                last = ts
            time.sleep(0.0005)

    def lidar_msg(self, d):
        msg = PointCloud2()
        msg.header.stamp = stamp(d["header"]["timestamp_ns"])
        msg.header.frame_id = "livox_frame"
        msg.height, msg.width = d["height"], d["width"]
        msg.fields = [PointField(f["name"], f["offset"], f["datatype"], f["count"]) for f in d["fields"]]
        msg.is_bigendian, msg.point_step, msg.row_step = d["is_bigendian"], d["point_step"], d["row_step"]
        msg.is_dense = d["is_dense"]
        msg.data = d["data"]
        return msg

    def image_msg(self, d):
        img = cv2.imdecode(np.frombuffer(d["data"], np.uint8), cv2.IMREAD_COLOR)
        msg = Image()
        msg.header.stamp = stamp(d["header"]["timestamp_ns"])
        msg.header.frame_id = d["header"]["frame_id"]
        msg.height, msg.width = img.shape[:2]
        msg.encoding, msg.step = "bgr8", img.shape[1] * 3
        msg.data = img.tobytes()
        return msg

    def lidar_loop(self):
        last = 0
        while not rospy.is_shutdown():
            d = self.robot.get_lidar_data(LIDAR)
            ts = d.get("header", {}).get("timestamp_ns", 0) if d else 0
            if ts and ts != last:
                self.pub_lidar.publish(self.lidar_msg(d))
                self.tally("lidar", ts, last, FRAME_GAP_NS)
                last = ts
            time.sleep(0.002)

    def camera_loop(self, cams, gap_ns):
        last = dict.fromkeys(cams, 0)
        while not rospy.is_shutdown():
            for name, sensor in cams.items():
                d = self.robot.get_rgb_data(sensor)
                ts = d.get("header", {}).get("timestamp_ns", 0) if d else 0
                if not ts or ts == last[name]:
                    continue
                if name in self.pub_jpg:
                    jpg = CompressedImage(format="jpeg", data=d["data"])
                    jpg.header.stamp, jpg.header.frame_id = stamp(ts), d["header"]["frame_id"]
                    self.pub_jpg[name].publish(jpg)
                if name == self.main_cam:
                    self.pub_img.publish(self.image_msg(d))
                self.tally(name, ts, last[name], gap_ns)
                last[name] = ts
            time.sleep(0.002)

    def joint_loop(self):
        msg = JointState()
        while not rospy.is_shutdown():
            t0 = time.time()
            try:
                names, pos, ready = [], [], True
                for g in POSE_JOINT_GROUPS:
                    q = list(self.robot.get_joint_positions([g]))
                    ready &= bool(q)   # 刚初始化时可能只有部分关节组有数据，等齐了再发，参考值才完整
                    names += [f"{g}_joint{i + 1}" for i in range(len(q))]
                    pos += q
                if ready:
                    msg.header.stamp = rospy.Time.now()
                    msg.name, msg.position = names, pos
                    self.pub_joints.publish(msg)
                    if self.joint_ref is None:
                        self.joint_ref = pos
                        rospy.loginfo("sdk_bridge 录制开始时关节角 %s",
                                      "  ".join(f"{n}={p:.3f}" for n, p in zip(names, pos)))
                    else:
                        dev = max(abs(a - b) for a, b in zip(pos, self.joint_ref))
                        if dev > JOINT_TOL and dev > self.joint_moved + JOINT_TOL:
                            rospy.logwarn("%s：最大偏离 %.3f rad，此后相机-雷达外参与 config/ 不符 %s", MOVED_WARNING, dev,
                                          "  ".join(f"{n}={p:.3f}" for n, p in zip(names, pos)))
                        self.joint_moved = max(self.joint_moved, dev)
            except Exception as e:      # noqa: BLE001  SDK 抛出的异常类型不固定
                rospy.logwarn_throttle(30, f"读取关节角失败：{e}")
            time.sleep(max(0.0, 1 / JOINT_RATE - (time.time() - t0)))

    def report(self, _event):
        with self.lock:
            cams = "  ".join(f"{n} {self.count[n]}（丢 {self.gaps[n]}）"
                             for g in self.groups.values() for n in g)
            rospy.loginfo("sdk_bridge 10 s：IMU %d（累计丢 %d）  点云 %d（累计丢 %d）  相机 %s",
                          self.count["imu"], self.gaps["imu"], self.count["lidar"], self.gaps["lidar"], cams)
            self.count = dict.fromkeys(self.count, 0)

    def run(self):
        threads = [threading.Thread(target=self.imu_loop, daemon=True),
                   threading.Thread(target=self.lidar_loop, daemon=True),
                   threading.Thread(target=self.joint_loop, daemon=True)]
        if self.groups["surround"]:
            threads.append(threading.Thread(target=self.camera_loop, args=(self.groups["surround"], FRAME_GAP_NS),
                                            daemon=True))
        if "head" in self.groups:
            threads.append(threading.Thread(target=self.camera_loop, args=(self.groups["head"], HEAD_GAP_NS),
                                            daemon=True))
        for t in threads:
            t.start()
        rospy.Timer(rospy.Duration(10), self.report)
        rospy.loginfo("sdk_bridge 已启动，主相机 %s，相机 %s", self.main_cam,
                      " ".join(n for g in self.groups.values() for n in g) or "不开")
        # SDK 在 C++ 层接管了 SIGINT/SIGTERM，rospy 收不到，只能看 SDK 的运行状态来退出
        while not rospy.is_shutdown() and self.robot.is_running():
            time.sleep(0.2)
        rospy.signal_shutdown("Galbot SDK 收到退出信号")
        for t in threads:
            t.join(timeout=1)
        close_robot(self.robot)


if __name__ == "__main__":
    rospy.init_node("sdk_bridge")
    Bridge().run()
