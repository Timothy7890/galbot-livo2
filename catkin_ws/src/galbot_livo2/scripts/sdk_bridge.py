#!/usr/bin/env python3
"""Galbot SDK -> ROS：Mid-360 点云和 IMU、环视相机图像，发布成 FAST-LIVO2 用的话题。

SDK 只有"取最新一帧"的接口，这里按时间戳变化轮询：IMU 单独一个线程（200 Hz），
点云和图像一个线程（10 Hz）。时间戳全部沿用 SDK 给的采集时间。

话题：
  /livox/lidar               PointCloud2，字段原样（x y z reflectivity tag time offset_nsec）
  /livox/imu                 Imu，加速度 m/s²
  /galbot/camera/image       主相机 bgr8（FAST-LIVO2 用）
  /galbot/<name>/image/compressed   4 路环视 JPEG 原样（~publish_all_cameras，录包后给 3DGS 用）
"""
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image, Imu, PointCloud2, PointField

from galbot_io import CAMERAS, IMU, LIDAR, close_robot, open_robot

IMU_GAP_NS = 7_500_000      # 200 Hz 间隔 5 ms，超过 7.5 ms 记为丢了一帧
FRAME_GAP_NS = 150_000_000  # 10 Hz 间隔 100 ms


def stamp(ns):
    return rospy.Time(ns // 1_000_000_000, ns % 1_000_000_000)


class Bridge:
    def __init__(self):
        self.main_cam = rospy.get_param("galbot/main_camera", "left_front")
        self.all_cams = rospy.get_param("~publish_all_cameras", True)
        cams = list(CAMERAS) if self.all_cams else [self.main_cam]
        self.cams = {name: CAMERAS[name] for name in cams}

        self.robot = open_robot({LIDAR, IMU, *self.cams.values()})
        self.pub_lidar = rospy.Publisher("/livox/lidar", PointCloud2, queue_size=10)
        self.pub_imu = rospy.Publisher("/livox/imu", Imu, queue_size=400)
        self.pub_img = rospy.Publisher("/galbot/camera/image", Image, queue_size=5)
        self.pub_jpg = {name: rospy.Publisher(f"/galbot/{name}/image/compressed", CompressedImage, queue_size=5)
                        for name in self.cams} if self.all_cams else {}

        self.count = {"imu": 0, "lidar": 0, "image": 0}
        self.gaps = {"imu": 0, "lidar": 0, "image": 0}
        self.lock = threading.Lock()

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

    def frame_loop(self):
        last_lidar = 0
        last_cam = {name: 0 for name in self.cams}
        while not rospy.is_shutdown():
            d = self.robot.get_lidar_data(LIDAR)
            ts = d.get("header", {}).get("timestamp_ns", 0) if d else 0
            if ts and ts != last_lidar:
                self.pub_lidar.publish(self.lidar_msg(d))
                self.tally("lidar", ts, last_lidar, FRAME_GAP_NS)
                last_lidar = ts
            for name, sensor in self.cams.items():
                d = self.robot.get_rgb_data(sensor)
                ts = d.get("header", {}).get("timestamp_ns", 0) if d else 0
                if not ts or ts == last_cam[name]:
                    continue
                if name in self.pub_jpg:
                    jpg = CompressedImage(format="jpeg", data=d["data"])
                    jpg.header.stamp, jpg.header.frame_id = stamp(ts), d["header"]["frame_id"]
                    self.pub_jpg[name].publish(jpg)
                if name == self.main_cam:
                    self.pub_img.publish(self.image_msg(d))
                    self.tally("image", ts, last_cam[name], FRAME_GAP_NS)
                last_cam[name] = ts
            time.sleep(0.002)

    def report(self, _event):
        with self.lock:
            rospy.loginfo("sdk_bridge 10 s：IMU %d（累计丢 %d）  点云 %d（累计丢 %d）  主相机 %d（累计丢 %d）",
                          self.count["imu"], self.gaps["imu"], self.count["lidar"], self.gaps["lidar"],
                          self.count["image"], self.gaps["image"])
            self.count = dict.fromkeys(self.count, 0)

    def run(self):
        threads = [threading.Thread(target=self.imu_loop, daemon=True),
                   threading.Thread(target=self.frame_loop, daemon=True)]
        for t in threads:
            t.start()
        rospy.Timer(rospy.Duration(10), self.report)
        rospy.loginfo("sdk_bridge 已启动，主相机 %s，共 %d 路相机", self.main_cam, len(self.cams))
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
