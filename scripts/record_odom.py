#!/usr/bin/env python3
"""把 FAST-LIVO2 的里程计 /aft_mapped_to_init（IMU 在世界系下的位姿）逐帧记录到 CSV。"""
import csv

import rospy
from nav_msgs.msg import Odometry


def main():
    rospy.init_node("trajectory_recorder", anonymous=False)
    out = rospy.get_param("~output_file", "trajectory.csv")
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])

        def on_odom(msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            w.writerow([f"{msg.header.stamp.to_sec():.6f}", p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            f.flush()

        rospy.Subscriber("/aft_mapped_to_init", Odometry, on_odom, queue_size=100)
        rospy.loginfo("recording /aft_mapped_to_init to %s", out)
        rospy.spin()


if __name__ == "__main__":
    main()
