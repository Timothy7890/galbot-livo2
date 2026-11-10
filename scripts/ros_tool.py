#!/usr/bin/env python3
"""record.sh / nav.sh 用的 ROS 小工具：一个进程做完原来要多次 rosparam / rostopic 的事（每起一次 Python 在 Orin 上要 1 秒多）。

    ros_tool.py load <yaml>[:命名空间] ...         等 roscore 就绪（最多 15 s），依次加载参数文件（同 rosparam load）
    ros_tool.py wait <超时 s> <话题> ...            等这些话题都收到至少一条消息；超时则打印没收到的话题，退出码 1
"""
import sys
import threading
import time


def wait_master(timeout=15.0):
    import rosgraph
    t0 = time.time()
    while not rosgraph.is_master_online():
        if time.time() - t0 > timeout:
            raise SystemExit("roscore 未启动")
        time.sleep(0.1)


def load(specs):
    import rosparam
    wait_master()       # roscore 与读标定同时启动，这里等它就绪
    for spec in specs:
        path, _, ns = spec.partition(":")
        for params, pns in rosparam.load_file(path, default_namespace=ns or "/"):
            rosparam.upload_params(pns, params)


def wait(timeout, topics):
    import rospy
    rospy.init_node("ros_tool_wait", anonymous=True, disable_signals=True)
    got = set()
    lock = threading.Lock()

    def cb(_msg, topic):
        with lock:
            got.add(topic)

    subs = [rospy.Subscriber(t, rospy.AnyMsg, cb, callback_args=t, queue_size=1) for t in topics]
    t0 = time.time()
    while time.time() - t0 < timeout:
        with lock:
            if len(got) == len(topics):
                break
        time.sleep(0.05)
    for s in subs:
        s.unregister()
    missing = [t for t in topics if t not in got]
    if missing:
        print("未收到 " + " ".join(missing))
        return 1
    print(f"收到 {' '.join(topics)}（{time.time() - t0:.1f} s）")
    return 0


def main():
    if len(sys.argv) < 3 or sys.argv[1] not in ("load", "wait"):
        print(__doc__)
        return 2
    if sys.argv[1] == "load":
        load(sys.argv[2:])
        return 0
    return wait(float(sys.argv[2]), sys.argv[3:])


if __name__ == "__main__":
    sys.exit(main())
