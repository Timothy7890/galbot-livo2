"""自主探索用的 2D 占据栅格、边界（frontier）检测与路径规划。只依赖 numpy / scipy / cv2，可离线测试。

世界坐标系 = FAST-LIVO2 的 camera_init（起始时 IMU 位置，z 对齐重力）。栅格 RES 一格，以起点为中心。

每帧点云做 2D 射线投射：激光从雷达到打中点、处在"地面以上 Z_LOW 到 Z_HIGH"高度带内的那一段经过的格子记一次可通行，
打中点在高度带内则该格记一次障碍。打到地面（低于带）的射线整段都在带内，只记可通行；打到天花板的只记到出带为止。
一个格子：障碍次数 ≥ 2 且障碍次数 × 4 > 可通行次数 → 障碍；可通行次数 ≥ 2 且非障碍 → 可通行；其余未知。
（人走过留下的障碍会被之后穿过的射线洗掉。）
"""
import math

import cv2
import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

RES = 0.1                   # 栅格 (m)
HALF = 100.0                # 栅格覆盖起点周围 ±HALF m
Z_LOW, Z_HIGH = 0.08, 1.60  # 障碍高度带（地面以上，m）
RAY_MAX = 10.0              # 只用 10 m 内的射线记可通行，远处太稀
ROBOT_R = 0.3125            # 底盘投影圆半径 (m)
INFLATE = ROBOT_R + 0.10    # 规划时障碍膨胀半径：车身 + 余量
CLEAR_PREF = 0.8            # 离障碍小于此距离的格子加代价，路线尽量走中间 (m)
MIN_FRONTIER = 5            # 边界至少这么多格（0.5 m）才算
SIZE_GAIN = 0.5             # 选目标时每米边界抵消多少米路程

FREE, OCC, UNKNOWN = 0, 1, 2


class Grid:
    def __init__(self, floor_z):
        self.floor_z = floor_z
        self.n = int(round(2 * HALF / RES))
        self.hits = np.zeros((self.n, self.n), np.uint16)
        self.free = np.zeros((self.n, self.n), np.uint16)
        self.bbox = None            # 已观测区域 [i0, i1, j0, j1)（行 = x，列 = y）

    def cell(self, xy):
        return np.floor(np.asarray(xy) / RES + self.n / 2).astype(np.int64)

    def center(self, ij):
        return (np.asarray(ij, np.float64) + 0.5 - self.n / 2) * RES

    def _grow_bbox(self, ij):
        lo, hi = ij.min(axis=0), ij.max(axis=0) + 1
        if self.bbox is None:
            self.bbox = [lo[0], hi[0], lo[1], hi[1]]
        else:
            b = self.bbox
            self.bbox = [min(b[0], lo[0]), max(b[1], hi[0]), min(b[2], lo[1]), max(b[3], hi[1])]

    def _bump(self, arr, ij):
        ok = (ij >= 0).all(axis=1) & (ij < self.n).all(axis=1)
        ij = ij[ok]
        if not len(ij):
            return
        flat = np.unique(ij[:, 0] * self.n + ij[:, 1])      # 每帧每格只记一次
        v = arr.reshape(-1)
        v[flat] = np.minimum(v[flat].astype(np.int64) + 1, 60000)
        self._grow_bbox(np.stack([flat // self.n, flat % self.n], axis=1))

    def integrate(self, origin, pts):
        """origin：雷达在世界系的位置 (3,)；pts：同一帧世界系点 Nx3（已去掉机器人自身）。"""
        lo, hi = self.floor_z + Z_LOW, self.floor_z + Z_HIGH
        d = pts - origin
        L = np.hypot(d[:, 0], d[:, 1])
        keep = L > 0.05
        d, L, pts = d[keep], L[keep], pts[keep]
        # 射线在高度带内的参数区间 [0, t_end]（雷达本身在带内）
        with np.errstate(divide="ignore", invalid="ignore"):
            t_top = np.where(d[:, 2] > 0, (hi - origin[2]) / d[:, 2], np.inf)
            t_bot = np.where(d[:, 2] < 0, (lo - origin[2]) / d[:, 2], np.inf)
        t_end = np.minimum.reduce([np.ones_like(L), t_top, t_bot, RAY_MAX / L])
        in_band = (pts[:, 2] > lo) & (pts[:, 2] < hi) & (L < RAY_MAX)
        # 沿射线每半格采样一次
        steps = np.maximum(1, np.ceil(L * t_end / (RES / 2))).astype(np.int64)
        ray = np.repeat(np.arange(len(L)), steps)
        k = np.arange(len(ray)) - np.repeat(np.cumsum(steps) - steps, steps)
        t = (k / np.repeat(steps, steps)) * np.repeat(t_end, steps)
        xy = origin[:2] + d[ray, :2] * t[:, None]
        self._bump(self.free, self.cell(xy))
        if in_band.any():
            self._bump(self.hits, self.cell(pts[in_band, :2]))

    def mark_free_disk(self, xy, r=ROBOT_R):
        """机器人所在处必然可通行（底盘比雷达低，脚下扫不到）。"""
        c = self.cell(xy)
        k = int(math.ceil(r / RES))
        ii, jj = np.mgrid[-k:k + 1, -k:k + 1]
        m = ii ** 2 + jj ** 2 <= (r / RES) ** 2
        self._bump(self.free, np.stack([c[0] + ii[m], c[1] + jj[m]], axis=1))

    def state(self):
        """裁到已观测区域（外扩 2 格）的状态图与其左上角格坐标。"""
        if self.bbox is None:
            return None, (0, 0)
        i0, i1, j0, j1 = self.bbox
        i0, j0 = max(0, i0 - 2), max(0, j0 - 2)
        i1, j1 = min(self.n, i1 + 2), min(self.n, j1 + 2)
        h = self.hits[i0:i1, j0:j1].astype(np.int32)
        f = self.free[i0:i1, j0:j1].astype(np.int32)
        st = np.full(h.shape, UNKNOWN, np.uint8)
        occ = (h >= 2) & (h * 4 > f)
        st[(f >= 2) & ~occ] = FREE
        st[occ] = OCC
        return st, (i0, j0)


def plan(st, start, blacklist=(), home=None):
    """在状态图 st 上从 start（局部格坐标）出发：
    home 为 None 时找边界目标，否则规划回到 home。
    返回 dict：goal / path（局部格坐标）、frontiers（[(中心格, 格数)]）、reason。"""
    h, w = st.shape
    occ = st == OCC
    clear = distance_transform_edt(~occ) * RES                 # 到最近障碍的距离 (m)
    passable = (st == FREE) & (clear > INFLATE)
    # 起点附近（车身所在）强制可走，否则贴墙时会规划不出来
    si, sj = start
    k = int(math.ceil(ROBOT_R / RES))
    passable[max(0, si - k):si + k + 1, max(0, sj - k):sj + k + 1] |= st[max(0, si - k):si + k + 1,
                                                                        max(0, sj - k):sj + k + 1] != OCC
    if not (0 <= si < h and 0 <= sj < w) or not passable[si, sj]:
        return {"reason": "起点不在可通行区域"}

    cost = 1.0 + 3.0 * np.clip((CLEAR_PREF - clear) / CLEAR_PREF, 0, 1)
    idx = -np.ones((h, w), np.int64)
    cells = np.argwhere(passable)
    idx[passable] = np.arange(len(cells))
    rows, cols, wts = [], [], []
    for di, dj in ((0, 1), (1, 0), (1, 1), (1, -1)):
        a = cells
        b = a + (di, dj)
        ok = (b[:, 0] >= 0) & (b[:, 0] < h) & (b[:, 1] >= 0) & (b[:, 1] < w)
        a, b = a[ok], b[ok]
        ok = passable[b[:, 0], b[:, 1]]
        a, b = a[ok], b[ok]
        step = math.hypot(di, dj) * RES
        wt = step * (cost[a[:, 0], a[:, 1]] + cost[b[:, 0], b[:, 1]]) / 2
        rows += [idx[a[:, 0], a[:, 1]], idx[b[:, 0], b[:, 1]]]
        cols += [idx[b[:, 0], b[:, 1]], idx[a[:, 0], a[:, 1]]]
        wts += [wt, wt]
    g = coo_matrix((np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))),
                   shape=(len(cells), len(cells))).tocsr()
    dist, pred = dijkstra(g, indices=idx[si, sj], return_predecessors=True)

    # 边界：可通行格子（未膨胀）四邻域有未知格
    free = st == FREE
    unk = st == UNKNOWN
    nb = np.zeros_like(unk)
    nb[1:, :] |= unk[:-1, :]; nb[:-1, :] |= unk[1:, :]; nb[:, 1:] |= unk[:, :-1]; nb[:, :-1] |= unk[:, 1:]
    frontier = free & nb
    nlab, lab, stats, cent = cv2.connectedComponentsWithStats(frontier.astype(np.uint8), connectivity=8)
    reach = np.full((h, w), np.inf)
    reach[passable] = dist
    out = {"frontiers": []}

    if home is not None:
        target = [tuple(home)]
    else:
        best, target = math.inf, None
        # 每个边界附近 INFLATE + 0.3 m 内的可达格子都可以当目标（边界本身常贴着障碍）
        r = int(math.ceil((INFLATE + 0.3) / RES))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        # 拉黑的目标（去过但卡住）1 m 内的格子不再作为目标
        banned = np.zeros((h, w), np.uint8)
        for bi, bj in blacklist:
            cv2.circle(banned, (int(bj), int(bi)), int(round(1.0 / RES)), 1, -1)
        allowed = np.isfinite(reach) & ~banned.astype(bool)
        for c in range(1, nlab):
            size = stats[c, cv2.CC_STAT_AREA]
            if size < MIN_FRONTIER:
                continue
            ci, cj = cent[c][1], cent[c][0]
            out["frontiers"].append(((ci, cj), int(size)))
            near = cv2.dilate((lab == c).astype(np.uint8), kernel).astype(bool) & allowed
            if not near.any():
                continue
            cand = np.argwhere(near)
            dd = reach[near]
            j = int(np.argmin(dd))
            score = dd[j] - SIZE_GAIN * size * RES
            if score < best:
                best, target = score, [tuple(cand[j])]
        if target is None:
            out["reason"] = "没有可到达的边界" if out["frontiers"] else "没有边界，探索完成"
            return out

    ti, tj = target[0]
    if not (0 <= ti < h and 0 <= tj < w) or not np.isfinite(reach[ti, tj]):
        out["reason"] = "目标不可达"
        return out
    path, node, s_node = [], idx[ti, tj], idx[si, sj]
    while node >= 0 and node != s_node:
        path.append(tuple(cells[node]))
        node = pred[node]
    path.append((si, sj))
    out.update(goal=(ti, tj), path=path[::-1], length=float(reach[ti, tj]))
    return out


def render(st):
    """状态图 -> RGBA PNG 字节。与 7010 实时地图一致：世界 x 向右、y 向上，网页按世界坐标贴图。"""
    rgba = np.zeros(st.shape + (4,), np.uint8)
    rgba[st == FREE] = (255, 255, 255, 60)
    rgba[st == OCC] = (255, 80, 80, 200)
    # st[i, j]：i 为 x、j 为 y → 图像列 = i，图像行 = y 从大到小
    img = np.ascontiguousarray(rgba.transpose(1, 0, 2)[::-1])
    return cv2.imencode(".png", cv2.cvtColor(img, cv2.COLOR_RGBA2BGRA))[1].tobytes()
