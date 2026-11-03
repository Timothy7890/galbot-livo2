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
MIN_GOAL_DIST = 1.0         # 目标至少离机器人这么远 (m)：脚边总有看不清的边界（桌下、车身旁），否则会一直选脚下、原地不动

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


def line_clear(a, b, passable):
    """a、b 两格之间的直线经过的格子是否都可通行（每半格采样一次）。"""
    n = int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) * 2)) + 1
    ii = np.round(np.linspace(a[0], b[0], n)).astype(int)
    jj = np.round(np.linspace(a[1], b[1], n)).astype(int)
    h, w = passable.shape
    if ii.min() < 0 or jj.min() < 0 or ii.max() >= h or jj.max() >= w:
        return False
    return bool(passable[ii, jj].all())


def shortcut(path, passable):
    """把栅格上的阶梯路径拉直：从当前点直接连到能直线到达的最远点，只保留拐点。"""
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_clear(path[i], path[j], passable):
            j -= 1
        out.append(path[j])
        i = j
    return out


def point_seg_dist(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    L2 = (bx - ax) ** 2 + (by - ay) ** 2
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * (bx - ax) + (p[1] - ay) * (by - ay)) / L2))
    return math.hypot(p[0] - ax - t * (bx - ax), p[1] - ay - t * (by - ay))


def poly_len(pts):
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))) * RES


def plan(st, start, blacklist=(), home=None, keep_goal=None, keep_path=None):
    """在状态图 st 上从 start（局部格坐标）出发：
    home 为 None 时找边界目标，否则规划回到 home。
    keep_goal：当前目标，附近仍有边界且可达就继续用它，不每次重新挑（否则目标来回跳、路线扭来扭去、永远到不了）；
    keep_path：当前路线（拐点），仍全线可通行且机器人没偏离就原样保留。
    返回 dict：goal / path（局部格坐标的拐点列表）、frontiers（[(中心格, 格数)]）、reason。"""
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

    # 每个边界附近 INFLATE + 0.3 m 内的可达格子都可以当目标（边界本身常贴着障碍）
    r = int(math.ceil((INFLATE + 0.3) / RES))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
    # 拉黑的目标（去过但卡住）1 m 内的格子不再作为目标
    banned = np.zeros((h, w), np.uint8)
    for bi, bj in blacklist:
        cv2.circle(banned, (int(bj), int(bi)), int(round(1.0 / RES)), 1, -1)
    allowed = np.isfinite(reach) & ~banned.astype(bool)
    ii, jj = np.ogrid[:h, :w]
    far = (ii - si) ** 2 + (jj - sj) ** 2 >= (MIN_GOAL_DIST / RES) ** 2
    big = np.zeros((h, w), np.uint8)       # 够大的边界
    for c in range(1, nlab):
        size = stats[c, cv2.CC_STAT_AREA]
        if size >= MIN_FRONTIER:
            big[lab == c] = 1
            out["frontiers"].append(((cent[c][1], cent[c][0]), int(size)))

    def goal_still_good(g):
        gi, gj = g
        if not (0 <= gi < h and 0 <= gj < w) or not allowed[gi, gj]:
            return False
        return bool(big[max(0, gi - r):gi + r + 1, max(0, gj - r):gj + r + 1].any())

    if home is not None:
        target = [tuple(home)]
    elif keep_goal is not None and goal_still_good(keep_goal):
        target = [tuple(keep_goal)]
    else:
        best, target = math.inf, None
        for c in range(1, nlab):
            size = stats[c, cv2.CC_STAT_AREA]
            if size < MIN_FRONTIER:
                continue
            near = cv2.dilate((lab == c).astype(np.uint8), kernel).astype(bool) & allowed & far
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
    # 目标没变、原路线仍全线可通行、机器人离路线不远：沿用原路线，避免每秒换一条
    if keep_path and tuple(keep_path[-1]) == (ti, tj) and len(keep_path) >= 2:
        kp = [tuple(p) for p in keep_path]
        seg = [point_seg_dist((si, sj), a, b) for a, b in zip(kp, kp[1:])]
        k = int(np.argmin(seg))            # 机器人所在的那一段
        if seg[k] * RES < 0.5 and all(line_clear(a, b, passable) for a, b in zip(kp[k + 1:], kp[k + 2:])) \
                and line_clear((si, sj), kp[k + 1], passable):
            rest = [(si, sj)] + kp[k + 1:]
            out.update(goal=(ti, tj), path=rest, length=poly_len(rest), kept=True)
            return out
    path, node, s_node = [], idx[ti, tj], idx[si, sj]
    while node >= 0 and node != s_node:
        path.append(tuple(int(v) for v in cells[node]))
        node = pred[node]
    path.append((si, sj))
    path = shortcut(path[::-1], passable)
    out.update(goal=(ti, tj), path=path, length=poly_len(path))
    return out


def render(st):
    """状态图 -> RGBA PNG 字节。与 7010 实时地图一致：世界 x 向右、y 向上，网页按世界坐标贴图。"""
    rgba = np.zeros(st.shape + (4,), np.uint8)
    rgba[st == FREE] = (255, 255, 255, 60)
    rgba[st == OCC] = (255, 80, 80, 200)
    # st[i, j]：i 为 x、j 为 y → 图像列 = i，图像行 = y 从大到小
    img = np.ascontiguousarray(rgba.transpose(1, 0, 2)[::-1])
    return cv2.imencode(".png", cv2.cvtColor(img, cv2.COLOR_RGBA2BGRA))[1].tobytes()
