#!/usr/bin/env python3
"""底盘看门狗（web/web.sh 随 7011 一起启动，需在 sdk_env.sh 下运行）。

teleop_server.py 以 duration=0 下发速度（带 duration 的调用会阻塞整个进程），底盘不会自己停。
它每次下发后写心跳文件 HEARTBEAT_FILE："时间戳 是否在动"。心跳显示在动、却超过 STALE 秒没更新
（teleop_server 崩溃、卡死或被杀），本进程就调 stop_base 停车，直到心跳恢复或显示已停。
"""
import os
import sys
import time

HEARTBEAT_FILE = "/dev/shm/galbot_teleop_heartbeat"
STALE = 0.6         # s
CHECK = 0.1         # s


def read_beat():
    try:
        t, moving = open(HEARTBEAT_FILE).read().split()
        return float(t), moving == "1"
    except (OSError, ValueError):
        return None, False


def main():
    from galbot_sdk import g1
    robot = g1.GalbotRobot()
    if not robot.init(set()):
        sys.exit("Galbot SDK 初始化失败")
    print(f"底盘看门狗启动：心跳 {HEARTBEAT_FILE}，超过 {STALE} s 未更新且在动则停车", flush=True)
    stopped_for = None
    while robot.is_running():
        t, moving = read_beat()
        if t is not None and moving and time.time() - t > STALE:
            robot.stop_base()
            if stopped_for != t:
                stopped_for = t
                print(f"{time.strftime('%H:%M:%S')} 7011 心跳中断 {time.time() - t:.1f} s，已停车", flush=True)
        time.sleep(CHECK)
    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()


if __name__ == "__main__":
    main()
