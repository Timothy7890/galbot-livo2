"""底盘避障：Mid-360 点云 + 8 个超声波，按运动方向限速或停车。遥控（teleop_server.py）与以后的自主探索共用。

坐标系 base_link：x 向前、y 向左、z 向上，原点在地面。底盘可全向移动，按 (vx, vy) 的方向检查：
- Mid-360：车身沿运动方向扫过的通道内，最近障碍到车身边缘的距离 gap；
- 超声波：朝向与运动方向夹角 60° 以内的探头读数；
gap 小于 STOP 停车；否则限制速度不超过 sqrt(2·BRAKE·(gap − STOP))，即按 BRAKE 减速度刚好能在 STOP 处停下的速度：
速度越快、离障碍越近越早减速，低速时不必在远处就慢吞吞。原地转向时检查车身转一圈扫过的圆环。
退离障碍的方向不受限制，所以贴近后总能倒出来。

车身自标定（calibrate）：周围 CAL_RADIUS 内清空后静止采几帧，始终有点的 5 cm 格子当作机器人自身
（躯干、垂在两侧的手臂等），之后过滤掉；安全轮廓扩大到包含这些格子。结果存 SELF_MASK_FILE，手臂姿态变了要重新标定。
"""
import json
import math
import time
from pathlib import Path

import numpy as np

# 底盘轮廓（base_link 下的矩形，m）：600 x 600 mm，投影圆直径 625 mm（圆角），按以 base_link 为中心。
# 底盘比雷达低扫不到，自标定只会在此基础上扩大
BASE_FOOTPRINT = {"front": 0.30, "back": -0.30, "left": 0.30, "right": -0.30}
BASE_RADIUS = 0.3125        # 原地转向时底盘扫过的半径
SELF_MARGIN = 0.05          # 轮廓外扩这么多以内的点当作机器人自身
CELL = 0.05                 # 自标定与去重的栅格 (m)
CAL_RADIUS = 0.8            # 自标定只看这个半径内
CAL_RATIO = 0.5             # 至少这么大比例的帧里有点，才算机器人自身
CAL_MAX_EXTENT = 0.6        # 标定出的轮廓任一侧超过此值，多半是周围有障碍物被当成了自身，不采用
SELF_MASK_FILE = Path(__file__).resolve().parent.parent / "catkin_ws/src/galbot_livo2/config/self_mask.json"
Z_MIN, Z_MAX = 0.05, 1.60   # 只看这个高度区间的点（地面以上、机器人高度以下）
RANGE = 3.0                 # 只保留 3 m 内的点
SIDE_MARGIN = 0.05          # 运动通道比车身两侧各宽这么多

LIDAR_STOP = 0.25                       # m，障碍到车身边缘小于此值停车
US_STOP = 0.30                          # m，超声波读数小于此值停车
BRAKE = 0.3                             # 计算允许速度用的减速度 (m/s²)，比 7011 实际减速度 0.8 保守，留出延迟余量
US_MAX_VALID = 4.0                      # 超过视为无回波
ROT_STOP, ROT_SLOW = 0.08, 0.25         # 原地转向：车身扫过圆外的余量
US_ROT_STOP = 0.15                      # 原地转向：任一超声波小于此值不允许转
STALE = 0.5                             # 传感器数据超过这么久没更新就不允许运动 (s)

# 超声波名称 -> 朝向（度，0 为正前方，逆时针为正）与网页上的中文名
ULTRASONIC = {
    "FRONT_LEFT": (0, "前左"), "FRONT_RIGHT": (0, "前右"),
    "LEFT_LEFT": (90, "左左"), "LEFT_RIGHT": (90, "左右"),
    "BACK_LEFT": (180, "后左"), "BACK_RIGHT": (180, "后右"),
    "RIGHT_LEFT": (-90, "右左"), "RIGHT_RIGHT": (-90, "右右"),
}



def ramp(gap, stop, slow):
    return float(np.clip((gap - stop) / (slow - stop), 0.0, 1.0))


def v_allow(gap, stop):
    """离障碍 gap 时允许的最大速度：按 BRAKE 减速能在 stop 处停下。"""
    return math.sqrt(2 * BRAKE * (gap - stop)) if gap > stop else 0.0


def cell_keys(xy):
    c = np.floor(xy / CELL).astype(np.int64)
    return (c[:, 0] << 32) | (c[:, 1] & 0xFFFFFFFF)


class Safety:
    def __init__(self):
        self.self_keys = np.empty(0, dtype=np.int64)    # 机器人自身占据的格子
        self.set_footprint(BASE_FOOTPRINT)
        self.calibrated = None          # 自标定时间
        self.load_self_mask()
        self.pts = np.empty((0, 2))     # 障碍点（base_link 平面坐标）
        self.lidar_t = 0.0
        self.us = {}                    # 名称 -> 读数 (m)，无效为 None
        self.us_t = 0.0
        self.last = {}                  # 最近一次限速的明细，给网页显示

    # ------------------------------------------------------------ 车身轮廓
    def set_footprint(self, fp):
        self.footprint = dict(fp)
        self.corners = np.array([[fp["front"], fp["left"]], [fp["front"], fp["right"]],
                                 [fp["back"], fp["left"]], [fp["back"], fp["right"]]])
        # 原地转向扫过的半径：底盘投影圆与机器人自身格子（手臂等）的最远点取大
        r = BASE_RADIUS
        if len(self.self_keys):
            cx = (self.self_keys >> 32).astype(np.float64)
            cy = (self.self_keys & 0xFFFFFFFF).astype(np.float64)
            cy = np.where(cy >= 2 ** 31, cy - 2 ** 32, cy)
            r = max(r, float(np.max(np.hypot(np.where(cx >= 0, cx + 1, cx), np.where(cy >= 0, cy + 1, cy)))) * CELL)
        self.sweep_r = round(r, 3)

    def load_self_mask(self):
        try:
            d = json.loads(SELF_MASK_FILE.read_text())
        except (OSError, ValueError):
            return
        self.self_keys = np.array(d["keys"], dtype=np.int64)
        self.set_footprint(d["footprint"])
        self.calibrated = d.get("created")

    def calibrate(self, frames):
        """frames：若干帧 base_link 点云（周围 CAL_RADIUS 内无障碍、机器人静止）。返回说明文字。"""
        counts = {}
        for xyz in frames:
            p = xyz[(xyz[:, 2] > Z_MIN) & (xyz[:, 2] < Z_MAX), :2]
            p = p[np.hypot(p[:, 0], p[:, 1]) < CAL_RADIUS]
            for k in np.unique(cell_keys(p)) if len(p) else []:
                counts[int(k)] = counts.get(int(k), 0) + 1
        keys = np.array(sorted(k for k, n in counts.items() if n >= CAL_RATIO * len(frames)), dtype=np.int64)
        fp = dict(BASE_FOOTPRINT)
        if len(keys):
            cx = (keys >> 32).astype(np.int64)
            cy = (keys & 0xFFFFFFFF).astype(np.int64)
            cy = np.where(cy >= 2 ** 31, cy - 2 ** 32, cy)
            fp = {"front": max(fp["front"], (cx.max() + 1) * CELL), "back": min(fp["back"], cx.min() * CELL),
                  "left": max(fp["left"], (cy.max() + 1) * CELL), "right": min(fp["right"], cy.min() * CELL)}
            fp = {k: round(float(v), 3) for k, v in fp.items()}
            if max(abs(v) for v in fp.values()) > CAL_MAX_EXTENT:
                return (f"标定未采用：{CAL_MAX_EXTENT} m 外仍有固定的点（轮廓 前 {fp['front']} 后 {fp['back']} "
                        f"左 {fp['left']} 右 {fp['right']} m），周围可能有障碍物，请移到四周 {CAL_RADIUS} m 内空旷处再标定")
        self.self_keys = keys
        self.set_footprint(fp)
        self.calibrated = time.strftime("%Y-%m-%d %H:%M:%S")
        SELF_MASK_FILE.write_text(json.dumps({"created": self.calibrated, "frames": len(frames),
                                              "footprint": fp, "keys": keys.tolist()}))
        return f"已标定：机器人自身 {len(keys)} 个格子，安全轮廓 前 {fp['front']} 后 {fp['back']} 左 {fp['left']} 右 {fp['right']} m"

    def is_self(self, xy):
        """base_link 平面坐标 Nx2 中哪些点属于机器人自身（车身轮廓内或自标定的格子）。"""
        fp = self.footprint
        inside = ((xy[:, 0] < fp["front"] + SELF_MARGIN) & (xy[:, 0] > fp["back"] - SELF_MARGIN)
                  & (xy[:, 1] < fp["left"] + SELF_MARGIN) & (xy[:, 1] > fp["right"] - SELF_MARGIN))
        if len(self.self_keys) and len(xy):
            inside |= np.isin(cell_keys(xy), self.self_keys)
        return inside

    # ------------------------------------------------------------ 数据输入
    def update_lidar(self, xyz_base):
        """一帧 Mid-360 点云（已变换到 base_link，Nx3）。"""
        p = xyz_base[(xyz_base[:, 2] > Z_MIN) & (xyz_base[:, 2] < Z_MAX)]
        p = p[np.einsum("ij,ij->i", p[:, :2], p[:, :2]) < RANGE * RANGE]
        p = p[~self.is_self(p[:, :2]), :2]
        if len(p):
            # 5 cm 栅格去重，减少计算量和网页传输量
            _, idx = np.unique(cell_keys(p), return_index=True)
            p = p[idx]
        self.pts = p
        self.lidar_t = time.time()

    def update_ultrasonic(self, readings):
        self.us = {k: (v if v is not None and 0 < v < US_MAX_VALID else None) for k, v in readings.items()}
        self.us_t = time.time()

    # ------------------------------------------------------------ 限速
    def lidar_gap(self, u):
        """车身（四周外扩 SIDE_MARGIN 的矩形）沿单位方向 u 平移多远会碰到障碍点。
        等价于障碍点沿 -u 方向的射线与矩形求交（slab 法）；车身侧旁、平移时擦肩而过的点不算。"""
        if not len(self.pts):
            return math.inf
        fp = self.footprint
        lo = np.array([fp["back"] - SIDE_MARGIN, fp["right"] - SIDE_MARGIN])
        hi = np.array([fp["front"] + SIDE_MARGIN, fp["left"] + SIDE_MARGIN])
        t_in = np.zeros(len(self.pts))
        t_out = np.full(len(self.pts), np.inf)
        for k in (0, 1):
            d = -u[k]
            p = self.pts[:, k]
            if abs(d) < 1e-9:
                inside = (p >= lo[k]) & (p <= hi[k])
                t_out = np.where(inside, t_out, -np.inf)
                continue
            t1, t2 = (lo[k] - p) / d, (hi[k] - p) / d
            t_in = np.maximum(t_in, np.minimum(t1, t2))
            t_out = np.minimum(t_out, np.maximum(t1, t2))
        hit = t_in <= t_out
        return float(t_in[hit].min()) if hit.any() else math.inf

    def us_gap(self, u):
        heading = math.degrees(math.atan2(u[1], u[0]))
        gaps = [(d, k) for k, d in self.us.items() if d is not None
                and abs((ULTRASONIC[k][0] - heading + 180) % 360 - 180) < 60]
        return min(gaps) if gaps else (math.inf, "")

    def limit(self, vx, vy, wz):
        """返回限速后的 (vx, vy, wz) 和原因（不限速时为空）。"""
        now = time.time()
        if now - self.lidar_t > STALE or now - self.us_t > STALE:
            self.last = {"reason": "雷达或超声波数据中断，禁止运动"}
            return 0.0, 0.0, 0.0, self.last["reason"]

        reasons = []
        v = math.hypot(vx, vy)
        lin_scale = 1.0
        lg = ug = math.inf
        uk = ""
        if v > 1e-3:
            u = np.array([vx, vy]) / v
            lg = self.lidar_gap(u)
            ug, uk = self.us_gap(u)
            va_l, va_u = v_allow(lg, LIDAR_STOP), v_allow(ug, US_STOP)
            lin_scale = min(1.0, min(va_l, va_u) / v)
            if lin_scale < 1:
                src = f"雷达 {lg:.2f} m" if va_l <= va_u else f"超声波{ULTRASONIC[uk][1]} {ug:.2f} m"
                reasons.append(f"{'停止' if lin_scale == 0 else '减速'}：运动方向障碍（{src}）")

        rot_scale = 1.0
        if abs(wz) > 1e-3:
            r = np.hypot(self.pts[:, 0], self.pts[:, 1]) if len(self.pts) else np.array([math.inf])
            rot_scale = ramp(float(r.min()) - self.sweep_r, ROT_STOP, ROT_SLOW)
            us_min = min((d for d in self.us.values() if d is not None), default=math.inf)
            if us_min < US_ROT_STOP:
                rot_scale = 0.0
            if rot_scale < 1:
                reasons.append(f"{'禁止' if rot_scale == 0 else '减慢'}转向：车身旋转范围内有障碍")

        self.last = {"lidar_gap": None if math.isinf(lg) else round(lg, 3),
                     "us_gap": None if math.isinf(ug) else round(ug, 3), "us_name": uk,
                     "lin_scale": round(lin_scale, 2), "rot_scale": round(rot_scale, 2),
                     "reason": "；".join(reasons)}
        return vx * lin_scale, vy * lin_scale, wz * rot_scale, self.last["reason"]

    def scan(self):
        """给网页画俯视图：障碍点、超声波读数、车身轮廓。"""
        return {"pts": np.round(self.pts, 2).tolist(), "us": self.us,
                "us_dir": {k: v[0] for k, v in ULTRASONIC.items()},
                "us_label": {k: v[1] for k, v in ULTRASONIC.items()},
                "footprint": self.footprint, "sweep_r": self.sweep_r, "limit": self.last,
                "calibrated": self.calibrated, "self_cells": len(self.self_keys),
                "age": round(time.time() - min(self.lidar_t, self.us_t), 2) if self.lidar_t else None}
