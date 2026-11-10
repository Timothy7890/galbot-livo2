"""成熟地图内定位：全局重定位 + 持续跟踪。只依赖 numpy / scipy（不用 open3d），可离线测试。

输入是"水平底盘系"下的点：底盘系按当前 roll / pitch 摆正（z 朝上、朝向为 0），FAST-LIVO2 的世界系 z 对齐重力，
所以地图里与本次里程计之间只差 x、y、z 与偏航，只搜这 4 个自由度（参考 Leishen-CX 的 localizer.py，那边用 open3d）。

重定位：
  1. 2D 粗搜：地图障碍的距离场（似然场）上，对所有可通行位置（每 0.3 m）× 每 5° 朝向，
     把扫描里障碍高度带内的点投上去打分；手动给了大概位置时只搜它附近
  2. 分数前几名（彼此相距 1 m 或 20° 以上）做 3D 点到面 ICP（x, y, z, 偏航）
  3. 内点比例（fitness）与内点误差（rmse）达标者取最好；另有一个相距较远、分数接近的解则判为有歧义，要人点选大概位置
持续跟踪：每秒一次，从"地图→里程计"推算的位姿出发做 ICP，修正量过大或匹配差算失败，连续失败判为丢失。
"""
import math
import os
import time

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree

import explore_grid as eg

# 2D 粗搜
SEARCH_STEP = 0.4           # 候选位置间距 (m)
SEARCH_YAW = math.radians(6)
SEARCH_CLEAR = 0.3          # 候选位置离障碍至少这么远（机器人不可能在墙里）
SIGMA = 0.15                # 似然场宽度 (m)
SCAN2D_MAX = 500            # 粗搜用的扫描点数上限
BAND = (0.15, 1.6)          # 粗搜用的高度带（底盘系，地面以上）
TOP_K = 12                  # 粗搜前几名做 ICP
NMS_DIST, NMS_YAW = 1.0, math.radians(20)
# 手动点选：只搜点选位置 MANUAL_R 内、朝向 ±MANUAL_YAW，范围小，搜得细：每 0.2 m、2°，不管栅格是什么
# （建图时没走到的未知区、后来挪了东西的障碍格都可能是机器人现在的位置），前 MANUAL_TOP_K 名去重后做 ICP
MANUAL_R, MANUAL_YAW = 2.0, math.radians(35)
MANUAL_STEP, MANUAL_DYAW = 0.2, math.radians(2)
MANUAL_TOP_K, MANUAL_NMS = 16, (0.4, math.radians(8))

# ICP
SRC_VOXEL = 0.15            # ICP 用的扫描点体素 (m)
SRC_RANGE = 20.0
ICP_SCHEDULE = (1.0, 0.6, 0.35, 0.25, 0.2, 0.2)     # 每轮的最大对应距离 (m)，逐步收紧
ICP_ITERS = 4               # 每个距离下的迭代次数
INLIER = 0.2                # 算 fitness / rmse 的内点距离 (m)
# 接受门槛。离线（地图 031615，另三次录制共 40 次成功）：对的解 fitness 0.89–1.00，错的 0.49–0.85（两次 0.81 / 0.85
# 是同一处朝向全对、平移错 9 m 的重复结构错位，真实位置在建图没走到的区域）。自动重定位全图搜，门槛高，
# 结果还要人在网页上核对（navigator 的 confirmed）；手动点选只搜点选处 2 m 内，远处的相似位置进不来，门槛低些
FIT_ACCEPT_AUTO, FIT_ACCEPT_MANUAL, RMSE_ACCEPT = 0.88, 0.75, 0.09
ICP_FAST_PTS = 2000         # 粗搜前几名先用这么多点快速 ICP 筛一遍
ICP_FULL_PTS = 6000         # 最后几名用这么多点精配
FULL_K = 3
AMBIG_RATIO = 0.93          # 次优的不同解 fitness 达到最优的这个比例就算有歧义

# 跟踪
TRACK_MAX_JUMP, TRACK_MAX_YAW = 0.5, math.radians(8)
TRACK_FIT, TRACK_RMSE = 0.45, 0.10
LOST_AFTER = 5


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def voxel(xyz, v):
    k = np.floor(xyz / v).astype(np.int64)
    _, idx = np.unique(k, axis=0, return_index=True)
    return xyz[idx]


def level_frame(T_odom_base):
    """返回 (偏航, R_level)：T_odom_base 的姿态 = Rz(偏航) @ R_level，R_level 只含 roll / pitch。"""
    R = T_odom_base[:3, :3]
    yaw = math.atan2(R[1, 0], R[0, 0])
    return yaw, Rz(-yaw) @ R


class NavMap:
    def __init__(self, nav_dir):
        g = np.load(os.path.join(nav_dir, "grid.npz"))
        self.i0, self.j0, self.n = int(g["i0"]), int(g["j0"]), int(g["n"])
        self.res, self.floor_z = float(g["res"]), float(g["floor_z"])
        self.hits, self.free = g["hits"], g["free"]
        h, f = self.hits.astype(np.int32), self.free.astype(np.int32)
        st = np.full(h.shape, eg.UNKNOWN, np.uint8)
        occ = (h >= 2) & (h * 4 > f)
        st[(f >= 2) & ~occ] = eg.FREE
        st[occ] = eg.OCC
        self.st = st
        self.df = distance_transform_edt(st != eg.OCC) * self.res        # 到最近障碍的距离 (m)
        self.like = np.exp(-self.df ** 2 / (2 * SIGMA ** 2)).astype(np.float32)
        clear = self.df
        c = np.load(os.path.join(nav_dir, "cloud.npz"))
        self.pts, self.nrm = c["pts"].astype(np.float64), c["nrm"].astype(np.float64)
        self.tree = cKDTree(self.pts)
        self.traj = np.load(os.path.join(nav_dir, "traj.npz"))["traj"]
        self.base_z = self.floor_z      # base_link 原点在地面
        # 自动重定位的候选位置：可通行、离障碍够远，每 SEARCH_STEP 取一个。
        # 建图时没走到的未知区不在内（加进来粗搜慢一倍，离线也只多找回少数几次）：机器人在那里时要手动点选
        k = max(1, int(round(SEARCH_STEP / self.res)))
        ok = (st == eg.FREE) & (clear >= SEARCH_CLEAR)
        sub = np.zeros_like(ok)
        sub[::k, ::k] = True
        self.cand = np.argwhere(ok & sub)         # 局部格 (i, j)
        km = max(1, int(round(MANUAL_STEP / self.res)))
        allc = np.zeros(st.shape, bool)
        allc[::km, ::km] = True
        self.cand_all = np.argwhere(allc)       # 手动点选用：全部格子（再按点选处裁）

    # 局部格 <-> 世界
    def to_cell(self, xy):
        return np.floor(np.asarray(xy) / self.res + self.n / 2).astype(np.int64) - (self.i0, self.j0)

    def to_world(self, ij):
        return (np.asarray(ij, np.float64) + (self.i0, self.j0) + 0.5 - self.n / 2) * self.res


def icp(nav, src, T0, schedule=ICP_SCHEDULE, iters=ICP_ITERS):
    """4 自由度（x, y, z, 偏航）点到面 ICP。src：水平底盘系点 Nx3；T0：初始 T_map_base（姿态需为 Rz(θ) @ R_level 形式，
    这里 src 已摆正，所以 T 的姿态就是 Rz(θ)）。返回 (T, fitness, rmse)。"""
    th = math.atan2(T0[1, 0], T0[0, 0])
    t = T0[:3, 3].copy()
    for dmax in schedule:
        for _ in range(iters):
            R = Rz(th)
            p = src @ R.T + t
            d, j = nav.tree.query(p, distance_upper_bound=dmax)
            m = np.isfinite(d)
            if m.sum() < 30:
                return T_of(th, t), 0.0, 1.0
            pm, q, n = p[m], nav.pts[j[m]], nav.nrm[j[m]]
            r = np.einsum("ij,ij->i", pm - q, n)
            # Huber 权重：|r| 超过 5 cm 的降权
            w = np.where(np.abs(r) < 0.05, 1.0, 0.05 / np.maximum(np.abs(r), 1e-9))
            # 对 θ 的导数：d(Rz(θ) s)/dθ = [-y, x, 0]（以当前 p - t 为 R s）
            rs = pm - t
            J = np.column_stack([n[:, 0], n[:, 1], n[:, 2], -rs[:, 1] * n[:, 0] + rs[:, 0] * n[:, 1]])
            A = (J * w[:, None]).T @ J + np.diag([1e-6, 1e-6, 1e-3, 1e-6])
            b = -(J * w[:, None]).T @ r
            dx = np.linalg.solve(A, b)
            # 线性化时把旋转放在 p - t（= R s）上：R ← Rz(dθ) R，t ← t + dt
            dth = float(dx[3])
            t = t + dx[:3]
            th += dth
            if abs(dth) < 1e-4 and np.linalg.norm(dx[:3]) < 1e-3:
                break
    p = src @ Rz(th).T + t
    d, _ = nav.tree.query(p, distance_upper_bound=INLIER)
    m = np.isfinite(d)
    fit = float(m.mean())
    rmse = float(np.sqrt(np.mean(d[m] ** 2))) if m.any() else 1.0
    return T_of(th, t), fit, rmse


def T_of(th, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rz(th), t
    return T


def coarse_search(nav, pts2d, hint=None):
    """2D 似然场粗搜。pts2d：水平底盘系下障碍高度带内的点 Nx2。返回 [(分数, x, y, θ)]，已去重，按分数降序。"""
    if len(pts2d) > SCAN2D_MAX:
        pts2d = pts2d[np.random.default_rng(0).choice(len(pts2d), SCAN2D_MAX, replace=False)]
    cand = nav.cand
    yaws = np.arange(0, 2 * math.pi, SEARCH_YAW)
    top_k, nms_d, nms_y = TOP_K, NMS_DIST, NMS_YAW
    if hint is not None:
        cw = nav.to_world(nav.cand_all)
        cand = nav.cand_all[np.hypot(cw[:, 0] - hint[0], cw[:, 1] - hint[1]) < MANUAL_R]
        yaws = hint[2] + np.arange(-MANUAL_YAW, MANUAL_YAW + 1e-9, MANUAL_DYAW)
        top_k, (nms_d, nms_y) = MANUAL_TOP_K, MANUAL_NMS
    if not len(cand):
        return []
    h, w = nav.like.shape
    flat = nav.like.ravel()
    best = np.zeros(len(cand), np.float32)
    best_yaw = np.zeros(len(cand))
    cf = cand.astype(np.float64) + 0.5
    for a in yaws:
        c, s = math.cos(a), math.sin(a)
        off = np.column_stack([c * pts2d[:, 0] - s * pts2d[:, 1], s * pts2d[:, 0] + c * pts2d[:, 1]]) / nav.res
        ii = np.floor(cf[:, None, 0] + off[None, :, 0]).astype(np.int64)
        jj = np.floor(cf[:, None, 1] + off[None, :, 1]).astype(np.int64)
        inside = (ii >= 0) & (ii < h) & (jj >= 0) & (jj < w)
        v = np.where(inside, flat[np.clip(ii, 0, h - 1) * w + np.clip(jj, 0, w - 1)], 0.0)
        sc = v.mean(axis=1)
        better = sc > best
        best[better], best_yaw[better] = sc[better], a
    order = np.argsort(-best)
    out = []
    xy = nav.to_world(cand)
    for k in order:
        x, y, a = xy[k, 0], xy[k, 1], best_yaw[k]
        if any(math.hypot(x - o[1], y - o[2]) < nms_d and abs(math.remainder(a - o[3], 2 * math.pi)) < nms_y
               for o in out):
            continue
        out.append((float(best[k]), float(x), float(y), float(a)))
        if len(out) >= top_k:
            break
    return out


def prepare_source(level_pts):
    """水平底盘系点 → (ICP 用的 3D 点, 粗搜用的 2D 点)。"""
    r = np.hypot(level_pts[:, 0], level_pts[:, 1])
    p = level_pts[(r > 0.4) & (r < SRC_RANGE)]
    src = voxel(p, SRC_VOXEL)
    band = p[(p[:, 2] > BAND[0]) & (p[:, 2] < BAND[1])]
    pts2d = voxel(band[:, :2], 0.1)
    return src, pts2d


def subsample(p, n, seed=0):
    return p if len(p) <= n else p[np.random.default_rng(seed).choice(len(p), n, replace=False)]


def relocalize(nav, level_pts, hint=None):
    """全局重定位。level_pts：最近约 1 s 的扫描拼起来、变换到当前水平底盘系。hint：(x, y, θ) 手动点选的大概位姿。
    返回 dict：ok, T（T_map_level：水平底盘系 → map）, fitness, rmse, ambiguous, msg, candidates, time。"""
    t0 = time.time()
    src, pts2d = prepare_source(level_pts)
    if len(src) < 100 or len(pts2d) < 30:
        return {"ok": False, "msg": "扫描点太少（周围太空旷或雷达数据不足）"}
    top = coarse_search(nav, pts2d, hint)
    t1 = time.time()
    if not top:
        return {"ok": False, "msg": "点选位置附近没有可通行区域" if hint else "地图里没有候选位置"}
    # 先少量点、宽松距离快速 ICP 把粗搜前几名都拉到位，再挑最好的几个用更多点精配
    fast_src = subsample(src, ICP_FAST_PTS)
    res = []
    for sc, x, y, a in top:
        T, fit, rmse = icp(nav, fast_src, T_of(a, [x, y, nav.base_z]), schedule=(1.0, 0.6, 0.35, 0.25), iters=3)
        res.append({"T": T, "fitness": fit, "rmse": rmse, "coarse": sc})
    res.sort(key=lambda r: r["fitness"], reverse=True)
    full_src = subsample(src, ICP_FULL_PTS)
    for r in res[:FULL_K]:
        r["T"], r["fitness"], r["rmse"] = icp(nav, full_src, r["T"], schedule=(0.35, 0.25, 0.2, 0.2), iters=4)
        r["full"] = True
    full = [r for r in res if r.get("full")]
    best = max(full, key=lambda r: r["fitness"] - 2.0 * r["rmse"])
    rival = None
    for r in res:
        if r is best:
            continue
        d = np.linalg.norm(r["T"][:2, 3] - best["T"][:2, 3])
        dy = abs(math.remainder(math.atan2(r["T"][1, 0], r["T"][0, 0]) - math.atan2(best["T"][1, 0], best["T"][0, 0]),
                                2 * math.pi))
        if (d > NMS_DIST or dy > NMS_YAW) and r["fitness"] >= AMBIG_RATIO * best["fitness"] and r["rmse"] < RMSE_ACCEPT:
            rival = r
            break
    fit_need = FIT_ACCEPT_MANUAL if hint is not None else FIT_ACCEPT_AUTO
    ok = best["fitness"] >= fit_need and best["rmse"] <= RMSE_ACCEPT and (rival is None or hint is not None)
    msg = "定位成功" if ok else (
        "有多个相似位置，请在地图上点选大概位置" if rival is not None and best["fitness"] >= fit_need
        else f"匹配度不够（{best['fitness']:.2f} / {best['rmse']:.3f} m），请在地图上点选大概位置")
    return {"ok": ok, "T": best["T"], "fitness": best["fitness"], "rmse": best["rmse"], "ambiguous": rival is not None,
            "msg": msg, "src_points": int(len(src)), "time": {"coarse": round(t1 - t0, 2), "icp": round(time.time() - t1, 2)},
            "candidates": [{"x": round(r["T"][0, 3], 2), "y": round(r["T"][1, 3], 2),
                            "yaw": round(math.atan2(r["T"][1, 0], r["T"][0, 0]), 3),
                            "fitness": round(r["fitness"], 3), "rmse": round(r["rmse"], 3)} for r in res]}


def track(nav, level_pts, T_pred):
    """持续跟踪：从预测位姿 T_pred（T_map_level）出发做 ICP。返回 dict：ok, T, fitness, rmse, jump, jump_yaw。"""
    src, _ = prepare_source(level_pts)
    if len(src) < 100:
        return {"ok": False, "msg": "扫描点太少", "fitness": 0.0, "rmse": 1.0, "jump": 0.0, "jump_yaw": 0.0}
    T, fit, rmse = icp(nav, subsample(src, ICP_FULL_PTS), T_pred, schedule=(0.6, 0.35, 0.25, 0.2), iters=3)
    jump = float(np.linalg.norm(T[:2, 3] - T_pred[:2, 3]))
    jy = abs(math.remainder(math.atan2(T[1, 0], T[0, 0]) - math.atan2(T_pred[1, 0], T_pred[0, 0]), 2 * math.pi))
    ok = fit >= TRACK_FIT and rmse <= TRACK_RMSE and jump < TRACK_MAX_JUMP and jy < TRACK_MAX_YAW
    return {"ok": ok, "T": T, "fitness": fit, "rmse": rmse, "jump": jump, "jump_yaw": jy}
