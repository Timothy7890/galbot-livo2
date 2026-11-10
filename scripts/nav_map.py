#!/usr/bin/env python3
"""由一次录制生成成熟地图（导航用）。

    python3 nav_map.py <录制目录> [名称] [输出目录，默认 <录制目录>/nav] [--from-bag]

来源（不重跑 FAST-LIVO2，直接用录制时的位姿，与录制时的实时地图一致），按优先级：
  1 快：录制时自主建图程序（explorer.py）存的 2D 栅格 explore_grid.npz + 3D 点云体素计数 explore_cloud.npz，几秒
  2 慢：没有这两个文件的旧录制（或 --from-bag）：重放 raw.bag 的 Mid-360 点云 + trajectory.csv 位姿，
      逐点按时间插值去畸变，重建栅格与点云；raw.bag 里图像占绝大部分，读一遍要很久（800 s 的录制在服务器上约 6 分钟）
  3 没有 raw.bag 时：explore_grid.npz + FAST-LIVO2 的 map_raw.pcd。map_raw 只有主相机视野内的点（要上色），
      点云不全，重定位匹配度明显偏低（离线：对的解 0.83–0.94，bag 重放的地图 0.94–0.99），不得已才用
输出到 <录制目录>/nav/（先写 nav.tmp/，全部成功后才换上；原有地图改名 nav.<时间> 留底，不删除）：
    grid.npz    2D 栅格计数（与自主建图 explore_grid.Grid 同一格式，导航直接载入规划）
    grid.png    网页显示用（与 explore_grid.render 一致），范围见 meta.json 的 grid
    cloud.npz   重定位 / 持续定位用的 3D 点（10 cm 体素）与法线；去掉只被扫到几次的（走动的人）与
                落在栅格可通行格上方、障碍高度带内的（被之后的射线洗掉的动态物体）
    traj.npz    建图时底盘轨迹 t, x, y, yaw（map 系）
    meta.json   名称、来源录制、地面高度、栅格范围、点数
进度写在输出目录上一级的 nav_progress.json，网页轮询。
"""
import json
import os
import shutil
import sys
import time

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "web"))
import explore_grid as eg       # noqa: E402
from nav_common import PoseInterp, as_matrix, iter_lidar, load_calib, load_trajectory, session_paths   # noqa: E402
from safety import Safety       # noqa: E402

VOXEL = eg.CLOUD_VOXEL      # 定位点云体素 (m)，与录制时 explorer 存的 explore_cloud.npz 一致
RANGE_MAX = eg.CLOUD_RANGE  # 重放 raw.bag 时只用这么远以内的点（远处稀、误差大）
MIN_HITS_BAG = 3            # 重放时体素至少被这么多帧扫到（墙被扫上百次，路过的人只有几次）
MIN_HITS_REC = 2            # explore_cloud.npz（录制时每 2 帧取 1、点 3 抽 1，较稀）。离线：2 时对的解 0.91–0.97、
                            # 重复结构的错解 0.84；3 时对的解低到 0.85，自动重定位 5/9；1 时错解 0.86，离门槛太近
MIN_PTS_PCD = 1             # map_raw.pcd 本来就缺（只有主相机视野内），不再按点数筛（离线：筛 4 点时重定位 1/9，不筛 6/9）
DESKEW_BINS = 5             # 一帧 0.1 s 分成几段分别插值位姿去畸变
GRID_SUB = 2                # 重放时写栅格点云几抽一
NORMAL_K = 12               # 估法线用的近邻数


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def pcd_keys_counts(xyz):
    """点 → (体素键, 每个体素里的点数)，编码与 explore_grid.voxel_keys 一致。"""
    iv = np.floor(xyz / VOXEL).astype(np.int64)
    off = 1 << 20
    return np.unique(((iv[:, 0] + off) << 42) | ((iv[:, 1] + off) << 21) | (iv[:, 2] + off), return_counts=True)


def normals(pts, k=NORMAL_K):
    """PCA 法线（最小特征值方向），朝向不定（点到面 ICP 不在乎符号）；flat 越小越像平面。分块算，省内存。"""
    tree = cKDTree(pts)
    nrm = np.empty((len(pts), 3), np.float32)
    flat = np.empty(len(pts), np.float32)
    for s in range(0, len(pts), 200000):
        q = pts[s:s + 200000]
        _, idx = tree.query(q, k=k)
        nb = pts[idx] - pts[idx].mean(axis=1, keepdims=True)
        w, v = np.linalg.eigh(np.einsum("nki,nkj->nij", nb, nb) / k)
        nrm[s:s + len(q)] = v[:, :, 0]
        flat[s:s + len(q)] = w[:, 0] / np.maximum(w.sum(axis=1), 1e-12)
    return nrm, flat


def read_pcd_xyz(path):
    """FAST-LIVO2 存的二进制 PCD（x y z rgb，各 4 字节）→ Nx3 float32。"""
    with open(path, "rb") as f:
        hdr = {}
        while True:
            line = f.readline().decode("ascii", "ignore").strip()
            if not line or line.startswith("#"):
                continue
            k, *v = line.split()
            hdr[k] = v
            if k == "DATA":
                break
        if hdr["DATA"][0] != "binary":
            raise ValueError(f"{path} 不是二进制 PCD")
        fields, sizes = hdr["FIELDS"], [int(s) for s in hdr["SIZE"]]
        n = int(hdr["POINTS"][0])
        step = sum(sizes)
        offs = {name: sum(sizes[:i]) for i, name in enumerate(fields)}
        a = np.frombuffer(f.read(n * step), dtype=np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                                                           "offsets": [offs["x"], offs["y"], offs["z"]],
                                                           "itemsize": step}), count=n)
    return np.stack([a["x"], a["y"], a["z"]], axis=1)


class Progress:
    def __init__(self, path):
        self.path = path
        self.t0 = time.time()

    def set(self, state, msg, frac=None, **kw):
        d = {"state": state, "msg": msg, "progress": frac, "elapsed_s": round(time.time() - self.t0), **kw}
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f, ensure_ascii=False)
        os.replace(tmp, self.path)


def grid_state(hits, free):
    h, f = hits.astype(np.int32), free.astype(np.int32)
    st = np.full(h.shape, eg.UNKNOWN, np.uint8)
    occ = (h >= 2) & (h * 4 > f)
    st[(f >= 2) & ~occ] = eg.FREE
    st[occ] = eg.OCC
    return st


def replay_bag(sp, pose, t_i_l, T_i_b, T_b_l, floor_z, duration, t_first, prog):
    """重放 raw.bag：返回 (eg.Grid, 体素键, 次数)。"""
    safety = Safety()           # 车身轮廓 + 自标定，去掉机器人自身的点
    grid = eg.Grid(floor_z=floor_z)
    keys, counts = np.empty(0, np.int64), np.empty(0, np.int64)
    batch, n_used = [], 0
    for ts, xyz, dt in iter_lidar(sp["bag"]):
        if not pose.covers(ts + 0.1):
            continue
        r2 = np.einsum("ij,ij->i", xyz, xyz)
        ok = np.isfinite(r2) & (r2 > 0.04) & (r2 < RANGE_MAX ** 2)
        xyz = xyz[ok]
        dt = dt[ok] if dt is not None else np.full(len(xyz), 0.05)
        base = xyz @ T_b_l[:3, :3].T + T_b_l[:3, 3]
        keep = ~safety.is_self(base[:, :2])
        xyz, dt = xyz[keep], dt[keep]
        if len(xyz) < 100:
            continue
        # 逐段插值位姿去畸变：点在 IMU 系 = 雷达系 + t_i_l
        b = np.clip((dt / 0.1 * DESKEW_BINS).astype(int), 0, DESKEW_BINS - 1)
        Rb, pb = pose.at(ts + (np.arange(DESKEW_BINS) + 0.5) * 0.1 / DESKEW_BINS)
        w = np.einsum("nij,nj->ni", Rb[b], xyz + t_i_l) + pb[b]
        Re, pe = pose.at(ts + 0.1)          # 射线起点 = 帧末雷达位置（与自主建图一致）
        grid.integrate(Re @ t_i_l + pe, w[::GRID_SUB])
        grid.mark_free_disk((Re @ T_i_b[:3, 3] + pe)[:2])
        batch.append(eg.voxel_keys(w, VOXEL))
        n_used += 1
        if len(batch) >= 100:
            keys, counts = eg.merge_counts(keys, counts, np.concatenate(batch))
            batch = []
            frac = (ts - t_first) / duration
            prog.set("running", f"重放 raw.bag：{n_used} 帧（{frac * 100:.0f}%）", round(0.85 * frac, 3))
            if n_used % 1000 == 0:
                log(f"  {n_used} 帧，体素 {len(keys):,}")
    if batch:
        keys, counts = eg.merge_counts(keys, counts, np.concatenate(batch))
    if n_used < 50:
        raise RuntimeError(f"能和位姿对上的点云只有 {n_used} 帧")
    log(f"重放 {n_used} 帧；体素 {len(keys):,}")
    return grid, keys, counts, n_used


def build(session, name, prog, from_bag=False):
    sp = session_paths(session)
    T_b_i, t_i_l = load_calib(sp["calib"])
    T_i_b = np.linalg.inv(T_b_i)
    t, p, rot = load_trajectory(sp["traj"])
    if len(t) < 50:
        raise RuntimeError("轨迹太短（trajectory.csv 不到 50 个位姿）")
    T_b_l = T_b_i.copy()
    T_b_l[:3, 3] = T_b_i[:3, :3] @ t_i_l + T_b_i[:3, 3]

    # 底盘轨迹（map 系）与地面高度（base_link 原点在地面）
    Tw_i = np.tile(np.eye(4), (len(t), 1, 1))
    Tw_i[:, :3, :3], Tw_i[:, :3, 3] = as_matrix(rot), p
    Tw_b = Tw_i @ T_i_b
    traj = np.stack([t, Tw_b[:, 0, 3], Tw_b[:, 1, 3], np.arctan2(Tw_b[:, 1, 0], Tw_b[:, 0, 0])], 1)
    floor_z = float(np.median(Tw_b[:, 2, 3]))
    duration = float(t[-1] - t[0])
    log(f"位姿 {len(t)} 个，时长 {duration:.0f} s，地面 z = {floor_z:.3f}")

    gnpz, cnpz = os.path.join(session, "explore_grid.npz"), os.path.join(session, "explore_cloud.npz")
    pcd = next((f for f in (os.path.join(session, "map_raw.pcd"), os.path.join(session, "map.pcd"))
                if os.path.exists(f)), None)
    has_bag = os.path.exists(sp["bag"])
    use_npz = os.path.exists(gnpz) and not from_bag and (os.path.exists(cnpz) or (pcd and not has_bag))
    if use_npz:
        prog.set("running", "读取录制时的栅格", 0.05)
        g = np.load(gnpz)
        G = dict(hits=g["hits"], free=g["free"], i0=int(g["i0"]), j0=int(g["j0"]), n=int(g["n"]),
                 res=float(g["res"]), floor_z=float(g["floor_z"]))
        if os.path.exists(cnpz):
            source = "explore_grid.npz + explore_cloud.npz"
            c = np.load(cnpz)
            if abs(float(c["voxel"]) - VOXEL) > 1e-6:
                raise RuntimeError(f"explore_cloud.npz 体素 {float(c['voxel'])} 与 {VOXEL} 不符")
            keys, counts = c["keys"], c["counts"]
            min_hits = MIN_HITS_REC
        else:
            source = f"explore_grid.npz + {os.path.basename(pcd)}（只有主相机视野内的点，定位匹配度会偏低）"
            prog.set("running", f"读取 {os.path.basename(pcd)}", 0.15)
            xyz = read_pcd_xyz(pcd)
            log(f"{os.path.basename(pcd)} {len(xyz):,} 点")
            keys, counts = pcd_keys_counts(xyz)
            del xyz
            min_hits = MIN_PTS_PCD if pcd.endswith("map_raw.pcd") else 1
        frames = None
    else:
        if not has_bag:
            raise RuntimeError("没有 raw.bag，也没有录制时存的栅格 explore_grid.npz，无法生成")
        source = "raw.bag"
        pose = PoseInterp(t, p, rot)
        grid, keys, counts, frames = replay_bag(sp, pose, t_i_l, T_i_b, T_b_l, floor_z, duration, t[0], prog)
        st0, (i0, j0) = grid.state()
        i1, j1 = i0 + st0.shape[0], j0 + st0.shape[1]
        G = dict(hits=grid.hits[i0:i1, j0:j1], free=grid.free[i0:i1, j0:j1], i0=i0, j0=j0, n=grid.n,
                 res=eg.RES, floor_z=floor_z)
        min_hits = MIN_HITS_BAG
    st = grid_state(G["hits"], G["free"])

    prog.set("running", "筛选定位点云、估计法线", 0.88)
    pts = eg.voxel_centers(keys[counts >= min_hits], VOXEL)
    # 去掉落在可通行格上方、障碍高度带内的点：录制时被后来的射线洗掉了（人、推车等动态物体）
    ij = np.floor(pts[:, :2] / G["res"] + G["n"] / 2).astype(np.int64) - (G["i0"], G["j0"])
    inside = (ij[:, 0] >= 0) & (ij[:, 0] < st.shape[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < st.shape[1])
    h = pts[:, 2] - G["floor_z"]
    on_free = np.zeros(len(pts), bool)
    on_free[inside] = st[ij[inside, 0], ij[inside, 1]] == eg.FREE
    pts = pts[~(on_free & (h > eg.Z_LOW + 0.05) & (h < eg.Z_HIGH))]
    log(f"定位点云 {len(pts):,} 点（体素 {VOXEL} m）")
    nrm, flat = normals(pts)
    return {
        "grid": G, "png": eg.render(st),
        "cloud": dict(pts=pts.astype(np.float32), nrm=nrm, flat=flat, voxel=VOXEL),
        "traj": dict(traj=traj),
        "meta": {
            "name": name, "session": os.path.basename(os.path.normpath(session)), "source": source,
            "created": time.strftime("%Y-%m-%d %H:%M:%S"), "floor_z": round(G["floor_z"], 3),
            "grid": {"x0": float((G["i0"] - G["n"] / 2) * G["res"]), "y0": float((G["j0"] - G["n"] / 2) * G["res"]),
                     "w": float(st.shape[0] * G["res"]), "h": float(st.shape[1] * G["res"]),
                     "free": int((st == eg.FREE).sum()), "occupied": int((st == eg.OCC).sum())},
            "cloud_points": int(len(pts)), "frames": frames,
            "path_length_m": round(float(np.sum(np.hypot(*np.diff(traj[:, 1:3], axis=0).T))), 1),
            "duration_s": round(duration),
        },
    }


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    session = os.path.abspath(args[0])
    name = args[1] if len(args) > 1 else os.path.basename(session)
    out = os.path.abspath(args[2]) if len(args) > 2 else os.path.join(session, "nav")   # 离线测试可另指定
    os.makedirs(os.path.dirname(out), exist_ok=True)
    prog = Progress(os.path.join(os.path.dirname(out), "nav_progress.json"))
    tmp = out + ".tmp"
    try:
        prog.set("running", "开始生成", 0.0)
        res = build(session, name, prog, from_bag="--from-bag" in sys.argv)
        shutil.rmtree(tmp, ignore_errors=True)
        os.makedirs(tmp)
        np.savez_compressed(os.path.join(tmp, "grid.npz"), **res["grid"])
        np.savez_compressed(os.path.join(tmp, "cloud.npz"), **res["cloud"])
        np.savez_compressed(os.path.join(tmp, "traj.npz"), **res["traj"])
        with open(os.path.join(tmp, "grid.png"), "wb") as f:
            f.write(res["png"])
        with open(os.path.join(tmp, "meta.json"), "w") as f:
            json.dump(res["meta"], f, ensure_ascii=False, indent=1)
        # 原有地图不删，改名为 nav.<原生成时间> 留底，再换上新的
        if os.path.isdir(out):
            os.rename(out, f"{out}.{time.strftime('%Y%m%d_%H%M%S', time.localtime(os.path.getmtime(out)))}")
        os.rename(tmp, out)
        prog.set("done", "导航地图已生成", 1.0, meta=res["meta"])
        log(f"完成：{json.dumps(res['meta'], ensure_ascii=False)}")
    except Exception as e:      # noqa: BLE001
        prog.set("failed", f"生成失败：{e}")
        shutil.rmtree(tmp, ignore_errors=True)
        raise


if __name__ == "__main__":
    main()
