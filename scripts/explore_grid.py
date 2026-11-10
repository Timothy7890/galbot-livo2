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
CLEAR_PREF = 1.2            # 离障碍小于此距离的格子加代价，宽的地方路线尽量走中间 (m)
CLEAR_WEIGHT = 4.0          # 贴着膨胀边界时每米代价是空旷处的 1 + CLEAR_WEIGHT 倍
SHORTCUT_CLEAR = 0.9        # 拉直时直线离障碍至少要有原路线那么远（最多要求这么远），否则拉直会重新贴边 (m)
TURN_CLEAR = 0.6            # 能原地转身的地方离障碍至少这么远（底盘半径 0.31 + 避障转向余量 0.25）(m)；
                            # 拐点和目标尽量放在这样的地方，转向不会被墙角卡住，只有窄路才放宽
MIN_FRONTIER = 5            # 边界至少这么多格（0.5 m）才算
SIZE_GAIN = 0.5             # 选目标时每米边界抵消多少米路程
MIN_GOAL_DIST = 1.0         # 目标至少离机器人这么远 (m)：脚边总有看不清的边界（桌下、车身旁），否则会一直选脚下、原地不动

# 选目标策略（plan 的 strategy）：
#   balanced 均衡：路程 - SIZE_GAIN × 边界长度，全图挑最划算的（大边界能抵消很远的路程，易在两片区域间来回）
#   nearest  就近清扫：只看路程，掉头加代价（HEADING_COST），把身边的缺口一路清掉
#   region   分区清扫：地图按 REGION_SIZE 划块，当前块里还有可达边界就留在块内（离开加 LEAVE_COST），
#            块内按就近清扫；块扫完才去下一块。采完一片再去另一片，同一区域的图像在时间上集中
#   在 20261104_002715 的真实地图上离线仿真（4 个起点平均）：回到已离开的 5 m 分区 均衡 12.5 次、就近 10.8、
#   分区 5.7、顺序 7.2；总路程 均衡 174 m，其余约 146–150 m；但均衡前期覆盖最快（90% 覆盖时路程 40 m，其余 51–60 m）
#   tour     顺序清扫：把所有边界排成一条最短的访问顺序（开放式旅行商，见 tour_first），去第一个；
#            自然先清完一头再去另一头
STRATEGIES = {"balanced": "均衡", "nearest": "就近清扫", "region": "分区清扫", "tour": "顺序清扫"}
TOUR_DETOUR = 1.3           # 顺序清扫：边界之间实际路程约为直线距离的这么多倍
HEADING_COST = 3.0          # 目标在正后方时加的路程 (m)，正前方为 0，按 (1 - cos 夹角) / 2 插值
REGION_SIZE = 10.0          # 分区边长 (m)：与雷达有效距离相当；5 m 太小，块里的边界一下就看清，又满场找下一块
LEAVE_COST = 15.0           # 分区清扫时去当前块之外的目标加的路程 (m)

FREE, OCC, UNKNOWN = 0, 1, 2

# 导航地图的 3D 定位点云：录制时（explorer.py）与重放 raw.bag 时（nav_map.py）都按 CLOUD_VOXEL 体素计数
CLOUD_VOXEL = 0.10
CLOUD_RANGE = 25.0          # 只记这么远以内的点（远处稀、误差大）
_OFF = 1 << 20


def voxel_keys(pts, voxel=CLOUD_VOXEL):
    """世界系点 Nx3 → 去重的体素键（int64）。"""
    iv = np.floor(pts / voxel).astype(np.int64)
    return np.unique(((iv[:, 0] + _OFF) << 42) | ((iv[:, 1] + _OFF) << 21) | (iv[:, 2] + _OFF))


def voxel_centers(keys, voxel=CLOUD_VOXEL):
    k = np.asarray(keys, np.int64)
    iv = np.stack([(k >> 42) - _OFF, ((k >> 21) & (2 * _OFF - 1)) - _OFF, (k & (2 * _OFF - 1)) - _OFF], 1)
    return (iv + 0.5) * voxel


def merge_counts(keys, counts, new_keys, new_counts=None):
    """体素键计数合并：new_keys 每个记 1 次（或 new_counts 次）。"""
    allk = np.concatenate([keys, new_keys])
    allc = np.concatenate([counts, np.ones(len(new_keys), np.int64) if new_counts is None else new_counts])
    uk, inv = np.unique(allk, return_inverse=True)
    return uk, np.bincount(inv, weights=allc).astype(np.int64)


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


def line_cells(a, b):
    n = int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) * 2)) + 1
    return np.round(np.linspace(a[0], b[0], n)).astype(int), np.round(np.linspace(a[1], b[1], n)).astype(int)


def line_clear(a, b, passable, clear=None, need=0.0):
    """a、b 两格之间的直线经过的格子是否都可通行（每半格采样一次），且离障碍都不小于 need (m)。"""
    ii, jj = line_cells(a, b)
    h, w = passable.shape
    if ii.min() < 0 or jj.min() < 0 or ii.max() >= h or jj.max() >= w:
        return False
    if not passable[ii, jj].all():
        return False
    return clear is None or bool((clear[ii, jj] >= need).all())


def shortcut(path, passable, clear):
    """把栅格上的阶梯路径拉直：从当前点直接连到能直线到达的最远点，只保留拐点。
    直线离障碍的距离不得小于原路线这一段的最小距离（最多要求 SHORTCUT_CLEAR），保持走中间。"""
    pc = np.array([clear[p] for p in path])
    last = len(path) - 1
    out, i = [path[0]], 0
    while i < last:
        visible = []
        for j in range(last, i, -1):
            need = min(SHORTCUT_CLEAR, float(pc[i:j + 1].min())) - RES
            if j == i + 1 or line_clear(path[i], path[j], passable, clear, need):
                visible.append(j)
                if j == last or pc[j] >= TURN_CLEAR:
                    break           # 最远的、能在那里转身的可见点
        # 拐点（要转向的地方）优先放在能转身处；实在没有（窄路）就取最远的可见点
        good = [j for j in visible if j == last or pc[j] >= TURN_CLEAR]
        j = good[0] if good else visible[0]
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


def tour_first(pts, d0):
    """开放式旅行商：从机器人出发走遍全部边界目标 pts（格坐标）的最短顺序，返回第一个的下标。
    机器人到各点用实际路程 d0（m），点与点之间用直线距离 × TOUR_DETOUR 近似；最近邻法起步，再 2-opt 改进。"""
    n = len(pts)
    p = np.asarray(pts, np.float64) * RES
    D = np.linalg.norm(p[:, None] - p[None], axis=2) * TOUR_DETOUR
    d0 = np.asarray(d0, np.float64)
    order = [int(np.argmin(d0))]
    left = set(range(n)) - set(order)
    while left:
        k = min(left, key=lambda m: D[order[-1], m])
        order.append(k)
        left.remove(k)

    def cost(o):
        return d0[o[0]] + sum(D[a, b] for a, b in zip(o, o[1:]))

    best = cost(order)
    improved = True
    while improved:
        improved = False
        for i in range(n - 1):
            for j in range(i + 1, n):
                o = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                c = cost(o)
                if c < best - 1e-6:
                    order, best, improved = o, c, True
    return order[0]


def region_of(xy):
    """世界坐标 (x, y) 所在分区（整数块号）。"""
    return int(math.floor(xy[0] / REGION_SIZE)), int(math.floor(xy[1] / REGION_SIZE))


def plan(st, start, blacklist=(), home=None, keep_goal=None, keep_path=None,
         strategy="balanced", heading=None, region=None, origin=(0.0, 0.0)):
    """在状态图 st 上从 start（局部格坐标）出发：
    home 为 None 时找边界目标，否则规划回到 home。
    keep_goal：当前目标，附近仍有边界且可达就继续用它，不每次重新挑（否则目标来回跳、路线扭来扭去、永远到不了）；
    keep_path：当前路线（拐点），仍全线可通行且机器人没偏离就原样保留。
    strategy：选目标策略（见 STRATEGIES）；heading：机器人朝向 (rad)，就近 / 分区清扫用来少掉头；
    region：当前分区块号（region_of），分区清扫用；origin：局部格 (0, 0) 中心的世界坐标，用来算分区。
    返回 dict：goal / path（局部格坐标的拐点列表）、frontiers（[(中心格, 格数)]）、region（目标所在分区）、reason。"""
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

    cost = 1.0 + CLEAR_WEIGHT * np.clip((CLEAR_PREF - clear) / (CLEAR_PREF - INFLATE), 0, 1)
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
        cands = []                  # [(目标格, 路程 m)]，顺序清扫用
        for c in range(1, nlab):
            size = stats[c, cv2.CC_STAT_AREA]
            if size < MIN_FRONTIER:
                continue
            near = cv2.dilate((lab == c).astype(np.uint8), kernel).astype(bool) & allowed & far
            if not near.any():
                continue
            roomy = near & (clear >= TURN_CLEAR)      # 目标尽量选能转身的地方
            if roomy.any():
                near = roomy
            cand = np.argwhere(near)
            dd = reach[near]
            j = int(np.argmin(dd))
            g = cand[j]
            if strategy == "balanced":
                score = dd[j] - SIZE_GAIN * size * RES
            else:
                score = dd[j]
                if heading is not None:
                    d = g - (si, sj)
                    n = math.hypot(*d)
                    if n > 0:
                        cos = (d[0] * math.cos(heading) + d[1] * math.sin(heading)) / n
                        score += HEADING_COST * (1 - cos) / 2
                if strategy == "region" and region is not None:
                    if region_of((origin[0] + g[0] * RES, origin[1] + g[1] * RES)) != tuple(region):
                        score += LEAVE_COST
            cands.append((tuple(int(v) for v in g), float(dd[j])))
            if score < best:
                best, target = score, [tuple(g)]
        if strategy == "tour" and len(cands) > 1:
            target = [cands[tour_first([c[0] for c in cands], [c[1] for c in cands])][0]]
        if target is None:
            out["reason"] = "没有可到达的边界" if out["frontiers"] else "没有边界，探索完成"
            return out

    ti, tj = target[0]
    if not (0 <= ti < h and 0 <= tj < w) or not np.isfinite(reach[ti, tj]):
        out["reason"] = "目标不可达"
        return out
    out["region"] = region_of((origin[0] + ti * RES, origin[1] + tj * RES))
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
    path = shortcut(path[::-1], passable, clear)
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
