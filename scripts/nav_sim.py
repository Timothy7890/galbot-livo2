#!/usr/bin/env python3
"""导航闭环仿真（不需要 ROS、机器人）：真的 navigator.Navigator + 假的 rospy / 传感器 / 底盘。

    python3 nav_sim.py <地图目录 nav/> [--start x,y,yaw] [--goal x,y [--goal x,y ...]] [--speed 1]

假底盘：按 navigator 发出的 (vx, vy, wz) 积分出真实位姿（地图系）；
假里程计：真实位姿换到一个随机偏移的 odom 系（重定位要自己找出这个偏移），10 Hz；
假雷达：地图定位点云里离雷达 RANGE 内、在雷达高度上下视场内的点，随机取一部分，换到雷达系，10 Hz。
检查：自动重定位是否找对（与真实偏移比较）、每个目标是否到达、全程离障碍最近多少。
"""
import argparse
import math
import os
import sys
import threading
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


# ------------------------------------------------------------------ 假 rospy 与消息
class _Shutdown:
    flag = False


def _install_fake_ros(params):
    rp = types.ModuleType("rospy")
    rp.get_param = lambda name, default=None: params.get(name, default)
    rp.init_node = lambda *a, **k: None
    rp.Subscriber = lambda *a, **k: None
    rp.is_shutdown = lambda: _Shutdown.flag
    rp.loginfo = lambda fmt, *a: print("[info]", fmt % a if a else fmt, flush=True)
    rp.logwarn = lambda fmt, *a: print("[warn]", fmt % a if a else fmt, flush=True)
    rp.logerr = lambda fmt, *a: print("[err ]", fmt % a if a else fmt, flush=True)
    rp.logwarn_throttle = lambda p, fmt, *a: None

    class Rate:
        def __init__(self, hz):
            self.dt = 1.0 / hz / SPEED[0]
            self.t = time.time()

        def sleep(self):
            self.t += self.dt
            time.sleep(max(0.0, self.t - time.time()))
    rp.Rate = Rate
    sys.modules["rospy"] = rp
    for mod, names in (("nav_msgs", ["Odometry"]), ("sensor_msgs", ["Imu", "PointCloud2"])):
        m, mm = types.ModuleType(mod), types.ModuleType(mod + ".msg")
        for n in names:
            setattr(mm, n, type(n, (), {}))
        m.msg = mm
        sys.modules[mod], sys.modules[mod + ".msg"] = m, mm


SPEED = [1.0]       # 仿真加速倍数（只加速控制循环与传感器，规划 / 定位按实际耗时）


class NS(types.SimpleNamespace):
    pass


def odom_msg(T, stamp):
    from nav_common import from_matrix
    q = from_matrix(T[:3, :3]).as_quat()
    return NS(header=NS(stamp=NS(to_sec=lambda: stamp)),
              pose=NS(pose=NS(position=NS(x=T[0, 3], y=T[1, 3], z=T[2, 3]),
                              orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3]))))


def cloud_msg(xyz, stamp):
    xyz = np.ascontiguousarray(xyz, dtype="<f4")
    fields = [NS(name=n, offset=4 * i) for i, n in enumerate("xyz")]
    return NS(header=NS(stamp=NS(to_sec=lambda: stamp)), fields=fields, data=xyz.tobytes(), point_step=12,
              width=len(xyz), height=1)


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def T_xyyaw(x, y, z, yaw):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rz(yaw), (x, y, z)
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("nav")
    ap.add_argument("--start", default=None)
    ap.add_argument("--goal", action="append", default=[])
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--timeout", type=float, default=240.0)
    a = ap.parse_args()
    SPEED[0] = a.speed

    # 外参取包内 g1_calib.yaml（与真机一致）
    from nav_common import load_calib
    T_b_i, t_i_l = load_calib(os.path.join(HERE, "..", "catkin_ws/src/galbot_livo2/config/g1_calib.yaml"))
    from nav_common import from_matrix
    q = from_matrix(T_b_i[:3, :3]).as_quat()
    params = {"galbot/imu_in_base": list(T_b_i[:3, 3]) + list(q), "extrin_calib/extrinsic_T": list(t_i_l),
              "~session": "", "~auto_start": False, "~nav_dir": a.nav}
    _install_fake_ros(params)
    os.environ["NAV_DRY_RUN"] = "1"
    import explorer
    import navigator
    import nav_loc as nl
    explorer.DRY_RUN = True

    nv = navigator.Navigator(a.nav)
    nav = nv.nav
    rng = np.random.default_rng(1)
    if a.start:
        sx, sy, syaw = (float(v) for v in a.start.split(","))
    else:       # 建图轨迹中间的一个点
        tr = nav.traj[len(nav.traj) // 2]
        sx, sy, syaw = tr[1], tr[2], tr[3]
    true = [sx, sy, syaw]
    z_base = nav.floor_z
    # 本次 odom 系相对地图的真实偏移（重定位要找出它）
    T_om_true = T_xyyaw(*rng.uniform(-20, 20, 2), 0.0, rng.uniform(-math.pi, math.pi))   # odom ← map
    T_b_l = T_b_i.copy()
    T_b_l[:3, 3] = T_b_i[:3, :3] @ t_i_l + T_b_i[:3, 3]
    lock = threading.Lock()
    log = {"min_clear": math.inf, "path": []}

    def sensors():
        t0 = time.time()
        k = 0
        while not _Shutdown.flag:
            stamp = t0 + k * 0.1
            k += 1
            with lock:
                x, y, yaw = true
                vx, vy, wz = nv.cmd
                # 底盘：按指令积分 0.1 s（底盘系速度）
                true[0] += (math.cos(yaw) * vx - math.sin(yaw) * vy) * 0.1
                true[1] += (math.sin(yaw) * vx + math.cos(yaw) * vy) * 0.1
                true[2] = math.remainder(yaw + wz * 0.1, 2 * math.pi)
                x, y, yaw = true
            T_mb = T_xyyaw(x, y, z_base, yaw)
            ij = nav.to_cell((x, y))
            if 0 <= ij[0] < nav.df.shape[0] and 0 <= ij[1] < nav.df.shape[1]:
                log["min_clear"] = min(log["min_clear"], float(nav.df[ij[0], ij[1]]))
            log["path"].append((x, y))
            # 雷达：附近的地图点 → 雷达系
            T_ml = T_mb @ T_b_l
            d = nav.pts - T_ml[:3, 3]
            r = np.linalg.norm(d, axis=1)
            vis = (r < 15) & (r > 0.3) & (d[:, 2] / np.maximum(r, 1e-6) > -0.9)
            p = nav.pts[vis]
            if len(p) > 12000:
                p = p[rng.choice(len(p), 12000, replace=False)]
            pl = (p - T_ml[:3, 3]) @ T_ml[:3, :3]
            pl += rng.normal(0, 0.01, pl.shape)
            nv.on_cloud(cloud_msg(pl, stamp))
            # 里程计（IMU 在 odom 系），帧末时刻
            T_oi = T_om_true @ T_mb @ T_b_i
            nv.on_odom(odom_msg(T_oi, stamp + 0.1))
            time.sleep(max(0.0, t0 + k * 0.1 / SPEED[0] - time.time()))

    threading.Thread(target=sensors, daemon=True).start()
    threading.Thread(target=nv.plan_loop, daemon=True).start()
    threading.Thread(target=nv.loc_loop, daemon=True).start()
    threading.Thread(target=nv.run, daemon=True).start()

    t_start = time.time()
    ok_all = True
    # 等自动重定位
    while time.time() - t_start < 60:
        if nv.loc.get("state") in ("localized", "unlocalized") and nv.auto_tried and nv.loc.get("state") != "relocalizing" \
                and time.time() - t_start > navigator.AUTO_RELOC_AFTER + 1:
            break
        time.sleep(0.2)
    print("重定位：", {k: v for k, v in nv.loc.items() if k != "candidates"}, flush=True)
    if nv.T_mo is not None:
        T_mo_true = np.linalg.inv(T_om_true)
        err = np.linalg.norm(nv.T_mo[:2, 3] - T_mo_true[:2, 3])
        derr = math.degrees(abs(math.remainder(nl.math.atan2(nv.T_mo[1, 0], nv.T_mo[0, 0])
                                               - math.atan2(T_mo_true[1, 0], T_mo_true[0, 0]), 2 * math.pi)))
        print(f"  与真实偏移相差 {err * 100:.1f} cm、{derr:.2f}°", flush=True)
        ok_all &= err < 0.2 and derr < 2
        print("  确认定位：", nv.confirm(), flush=True)
    else:
        print("  自动重定位失败，改用手动点选（真实位姿偏 0.5 m、15°）", flush=True)
        nv.request_reloc({"mode": "manual", "x": true[0] + 0.5, "y": true[1] - 0.3, "yaw": true[2] + 0.26})
        time.sleep(1)
        while nv.loc.get("state") == "relocalizing":
            time.sleep(0.2)
        print("  手动：", {k: v for k, v in nv.loc.items() if k != "candidates"}, flush=True)
        if nv.T_mo is None:
            _Shutdown.flag = True
            sys.exit(1)

    goals = [tuple(float(v) for v in g.split(",")) for g in a.goal]
    if not goals:       # 默认：建图轨迹上离起点较远的两个点
        tr = nav.traj
        d = np.hypot(tr[:, 1] - true[0], tr[:, 2] - true[1])
        goals = [tuple(tr[int(np.argmax(d)), 1:3]), (true[0], true[1])]
    for gx, gy in goals:
        ok, msg = nv.goto(gx, gy)
        print(f"前往 ({gx:.1f}, {gy:.1f})：{msg}", flush=True)
        if not ok:
            ok_all = False
            continue
        t0 = time.time()
        last = 0
        while time.time() - t0 < a.timeout / SPEED[0]:
            time.sleep(0.5)
            if time.time() - last > 5 / SPEED[0]:
                last = time.time()
                print(f"  [{(time.time() - t0) * SPEED[0]:5.0f}s] 真实 ({true[0]:.2f}, {true[1]:.2f}) "
                      f"定位 {nv.loc.get('state')} 修正 {nv.loc.get('correction_cm')} cm  {nv.msg}", flush=True)
            if nv.state in ("arrived", "error") or not nv.enabled:
                break
        err = math.hypot(true[0] - gx, true[1] - gy)
        print(f"  结果：{nv.state} {nv.msg}；离目标 {err:.2f} m，用时 {(time.time() - t0) * SPEED[0]:.0f} s", flush=True)
        ok_all &= nv.state == "arrived" and err < 0.5
    print(f"全程离障碍最近 {log['min_clear']:.2f} m（膨胀半径 {explorer.eg.INFLATE:.2f} m）；"
          f"{'通过' if ok_all else '未通过'}", flush=True)
    _Shutdown.flag = True
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
