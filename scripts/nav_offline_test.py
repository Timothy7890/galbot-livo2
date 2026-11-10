#!/usr/bin/env python3
"""离线检验导航定位：用一次录制的真实点云，在另一次（或同一次）录制生成的成熟地图里重定位、持续跟踪。

    python3 nav_offline_test.py <地图目录 nav/> <录制目录> [--every 60] [--track 120] [--hint x,y,yaw]

重定位：录制里每隔 --every 秒取一个时刻，用之前 1 s 的点云重定位，算出"地图→本次里程计"变换 T_map_odom。
同一次录制里 T_map_odom 应该处处相同（里程计漂移除外），各时刻结果的离散程度就是定位一致性；
地图与录制是同一次时 T_map_odom 应接近单位阵。
跟踪：从第一个成功的时刻起，每 1 s 用预测位姿做一次跟踪 ICP，持续 --track 秒，统计修正量与失败次数。
"""
import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nav_loc as nl                    # noqa: E402
from nav_common import PoseInterp, iter_lidar, load_calib, load_trajectory, session_paths, yaw_of   # noqa: E402


def level_scan(sp, pose, t_i_l, T_i_b, t_end, window=1.0):
    """t_end 之前 window 秒的点云 → 当前（t_end 时）水平底盘系点，以及 T_odom_level。"""
    pts = []
    for ts, xyz, dt in iter_lidar(sp["bag"], start=t_end - window - 0.3, stop=t_end + 0.3):
        if ts < t_end - window or ts + 0.1 > t_end or not pose.covers(ts + 0.1):
            continue
        r2 = np.einsum("ij,ij->i", xyz, xyz)
        ok = np.isfinite(r2) & (r2 > 0.25)
        xyz = xyz[ok]
        dt = dt[ok] if dt is not None else np.full(len(xyz), 0.05)
        b = np.clip((dt / 0.1 * 5).astype(int), 0, 4)
        Rb, pb = pose.at(ts + (np.arange(5) + 0.5) * 0.02)
        pts.append(np.einsum("nij,nj->ni", Rb[b], xyz + t_i_l) + pb[b])
    if not pts:
        return None, None
    R, p = pose.at(t_end)
    T_oi = np.eye(4)
    T_oi[:3, :3], T_oi[:3, 3] = R, p
    T_ob = T_oi @ T_i_b
    yaw, _ = nl.level_frame(T_ob)
    T_ol = np.eye(4)
    T_ol[:3, :3], T_ol[:3, 3] = nl.Rz(yaw), T_ob[:3, 3]
    w = nl.voxel(np.vstack(pts), 0.05)
    lev = (w - T_ol[:3, 3]) @ T_ol[:3, :3]
    return lev, T_ol


def fmt(T):
    return f"x={T[0, 3]:7.2f} y={T[1, 3]:7.2f} z={T[2, 3]:5.2f} yaw={math.degrees(yaw_of(T)):7.1f}°"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("nav")
    ap.add_argument("session")
    ap.add_argument("--every", type=float, default=60.0)
    ap.add_argument("--track", type=float, default=120.0)
    ap.add_argument("--hint", default=None)
    a = ap.parse_args()
    t0 = time.time()
    nav = nl.NavMap(a.nav)
    print(f"地图载入 {time.time() - t0:.1f} s：定位点 {len(nav.pts):,}，候选位置 {len(nav.cand)}", flush=True)
    sp = session_paths(a.session)
    T_b_i, t_i_l = load_calib(sp["calib"])
    T_i_b = np.linalg.inv(T_b_i)
    t, p, rot = load_trajectory(sp["traj"])
    pose = PoseInterp(t, p, rot)
    hint = tuple(float(v) for v in a.hint.split(",")) if a.hint else None

    results = []
    for te in np.arange(t[0] + 5.0, t[-1] - 1.0, a.every):
        lev, T_ol = level_scan(sp, pose, t_i_l, T_i_b, te)
        if lev is None:
            continue
        t1 = time.time()
        r = nl.relocalize(nav, lev, hint)
        dt = time.time() - t1
        T_mo = r["T"] @ np.linalg.inv(T_ol) if "T" in r else None
        results.append((te, r, T_mo))
        print(f"t+{te - t[0]:5.0f}s  {'成功' if r['ok'] else '失败'}  {dt:4.1f} s  fit={r.get('fitness', 0):.2f} "
              f"rmse={r.get('rmse', 1):.3f}  {'歧义 ' if r.get('ambiguous') else ''}"
              f"T_map_odom: {fmt(T_mo) if T_mo is not None else '-'}  {r['msg']}  {r.get('time', '')}", flush=True)
    good = [T for _, r, T in results if r["ok"]]
    if good:
        xy = np.array([T[:2, 3] for T in good])
        yw = np.array([yaw_of(T) for T in good])
        yw = np.degrees(np.angle(np.exp(1j * (yw - yw[0])))) 
        med = np.median(xy, axis=0)
        print(f"\n成功 {len(good)}/{len(results)}；T_map_odom 中位 x={med[0]:.2f} y={med[1]:.2f}，"
              f"各次偏离中位：位置最大 {np.max(np.hypot(*(xy - med).T)):.2f} m，朝向范围 {np.ptp(yw):.1f}°")
    # 失败时刻的诊断：以各次成功结果的中位 T_map_odom 为准，算出该时刻的"正确"位姿，
    # 从那里直接 ICP 看匹配度（低 = 那里与地图对不上；高 = 粗搜没找到），并模拟手动点选（偏 0.5 m、10°）
    if good:
        T_c = good[int(np.argmin([np.hypot(*(T[:2, 3] - med)) for T in good]))]
        for te, r, _ in results:
            if r["ok"]:
                continue
            lev, T_ol = level_scan(sp, pose, t_i_l, T_i_b, te)
            T_true = T_c @ T_ol
            src, _ = nl.prepare_source(lev)
            _, fit, rmse = nl.icp(nav, nl.subsample(src, nl.ICP_FULL_PTS), T_true)
            th = yaw_of(T_true)
            hint = (T_true[0, 3] + 0.35, T_true[1, 3] - 0.35, th + math.radians(10))
            rm = nl.relocalize(nav, lev, hint)
            err = np.hypot(*(rm["T"][:2, 3] - T_true[:2, 3])) if "T" in rm else float("nan")
            print(f"  诊断 t+{te - t[0]:.0f}s 正确位姿 ({T_true[0, 3]:.1f}, {T_true[1, 3]:.1f}) 处 ICP fit={fit:.2f} "
                  f"rmse={rmse:.3f}；模拟手动点选：{'成功' if rm['ok'] else '失败'} 误差 {err:.2f} m "
                  f"fit={rm.get('fitness', 0):.2f}", flush=True)
    # 跟踪
    first = next(((te, T) for te, r, T in results if r["ok"]), None)
    if first is None or a.track <= 0:
        return
    te0, T_mo = first
    fails, jumps, times = 0, [], []
    for te in np.arange(te0 + 1.0, min(te0 + a.track, t[-1] - 1.0), 1.0):
        lev, T_ol = level_scan(sp, pose, t_i_l, T_i_b, te, window=0.5)
        if lev is None:
            continue
        t1 = time.time()
        r = nl.track(nav, lev, T_mo @ T_ol)
        times.append(time.time() - t1)
        if r["ok"]:
            T_mo = r["T"] @ np.linalg.inv(T_ol)
            jumps.append(r["jump"])
        else:
            fails += 1
            print(f"  跟踪失败 t+{te - t[0]:.0f}s fit={r['fitness']:.2f} rmse={r['rmse']:.3f} 跳 {r['jump']:.2f} m "
                  f"{math.degrees(r['jump_yaw']):.1f}°", flush=True)
    if times:
        print(f"跟踪 {len(times)} 次，失败 {fails}；每次修正量 中位 {np.median(jumps) * 100:.1f} cm、"
              f"最大 {np.max(jumps) * 100:.1f} cm；耗时 中位 {np.median(times) * 1000:.0f} ms、"
              f"最大 {np.max(times) * 1000:.0f} ms；最终 T_map_odom {fmt(T_mo)}")


if __name__ == "__main__":
    main()
