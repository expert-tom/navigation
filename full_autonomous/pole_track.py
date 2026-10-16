"""稳定层：跨帧累积观测，约束出稳定的杆位。

为什么需要它 —— 单帧检测有两个固有弱点：

1. **遮挡**：车靠近后一根杆会挡住另一根，单帧只看得到部分球面，
   圆心误差可达 10 cm。而 `pole_detect` 的门槛会（正确地）拒绝这种帧。
2. **噪声**：单帧的半径与圆心都有抖动。

关键认识：**不需要每一帧都测准，只需要在某个时刻测准一次。**
车从停车位出发时两根杆都完整可见（实测可见 360° 时误差 1.4 mm），
用那时的观测锁定坐标即可；靠近后互相遮挡时，路线早已生成，
跟踪不需要重新检测。

本模块因此做三件事：
  - 按"离上一次估计位置最近"把本帧候选关联到已跟踪的杆；
  - 累积多帧的点云（体素降采样，内存有界），并用全部累积点一起做圆拟合；
  - 达到帧数门槛后锁定，锁定后默认不再更新，避免里程计漂移污染已锁定坐标。

只依赖标准库。
"""
import math

try:
    from dataclasses import dataclass, field
except ImportError:
    dataclass = None

import pole_detect


@dataclass
class PoleEstimate:
    """一根杆的融合估计。"""
    x: float
    y: float
    radius: float
    frames: int
    points: int
    residual: float
    spread: float
    locked: bool = False


@dataclass
class Track:
    """内部跟踪状态。"""
    x: float
    y: float
    radius: float
    frames: int = 0
    voxels: dict = field(default_factory=dict)
    last_stamp: float = 0.0
    recent_centers: list = field(default_factory=list)
    recent_spreads: list = field(default_factory=list)
    spread: float = 0.0
    residual: float = 0.0
    locked: bool = False


class PoleTracker:
    """把多帧 Detection 融合成稳定的杆位。

    典型用法（每收到一帧点云就调一次 accumulate）：
        tracker = PoleTracker()
        tracker.accumulate(detect_poles(points, stamp=t, base_pose=pose))
        result = tracker.estimates()
        if result and result[0].locked: -> 可以生成路线了
    """

    def __init__(self, match_distance=0.6, voxel_size=0.02, max_points_per_pole=4000,
                 lock_frames=5, lock_min_spread=0.80):
        self.match_distance = match_distance
        self.voxel_size = voxel_size
        self.max_points_per_pole = max_points_per_pole
        self.lock_frames = lock_frames
        self.lock_min_spread = lock_min_spread
        self.tracks = []
        self.last_stamp = None
        self.frame = ""
        self.rejected = 0

    def _add_points(self, track, points):
        """体素降采样后并入累积；超预算时按步长丢弃，保证内存有界。"""
        for x, y, z in points:
            key = (round(x/self.voxel_size), round(y/self.voxel_size),
                   round(z/self.voxel_size))
            track.voxels[key] = (x, y, z)
        if len(track.voxels) > self.max_points_per_pole:
            stride = math.ceil(len(track.voxels)/self.max_points_per_pole)
            track.voxels = dict(list(track.voxels.items())[::stride])

    def _refine(self, track):
        """用累积点重新拟合圆心与半径。

        注意：**spread 不用累积点算**。累积点是体素降采样后的，密度分布与原始
        点云不同（体素化会把点拉向格心），实测 spread 会系统性偏低
        （原始点 0.85 → 降采样点 0.81），用于锁定判断会误拒。
        因此 spread 取最近若干帧**原始候选簇**的中位数，
        而圆心/半径用累积点（越多越稳）。
        """
        points = list(track.voxels.values())
        if len(points) < 12:
            return
        fit = pole_detect.fit_circle([(p[0], p[1]) for p in points])
        if fit is not None:
            cx, cy, _fitted, rms, _kept = fit
            track.x, track.y = cx, cy
        else:
            rms = 0.0
        distances = [math.hypot(p[0]-track.x, p[1]-track.y) for p in points]
        track.radius = pole_detect._percentile(distances, 0.95)
        track.residual = rms
        if track.recent_spreads:
            ordered = sorted(track.recent_spreads)
            track.spread = ordered[len(ordered)//2]

    def accumulate(self, detection):
        """并入一帧检测结果。返回本帧被接受的杆数。

        注意：**不因为车体移动而清空累积**。传入的簇点已经随 stamp 一起
        转到同一坐标系（map），所以车正常行驶不影响累积；实测"移动超过0.5m
        就重置"会让正常行驶途中把已锁定的坐标丢掉。
        位姿跳变这类真实故障由 vehicle_core 的失效分级处理，不在这里重复判断。
        """
        if detection.stamp and self.last_stamp is not None \
                and detection.stamp < self.last_stamp:
            raise ValueError("detection stamps must increase")
        if detection.frame:
            if self.frame and self.frame != detection.frame:
                raise ValueError("detection frame changed from %r to %r; reset before mixing"
                                 % (self.frame, detection.frame))
            self.frame = detection.frame
        self.last_stamp = detection.stamp or (self.last_stamp or 0.0)
        if not detection.saw_two or not detection.points:
            self.rejected += 1
            return 0
        accepted = 0
        for candidate, cluster in zip(detection.poles, detection.points):
            track = self._nearest_track(candidate)
            if track is None:
                track = Track(candidate.x, candidate.y, candidate.radius)
                self.tracks.append(track)
            if track.locked:
                continue
            track.x, track.y, track.radius = candidate.x, candidate.y, candidate.radius
            track.frames += 1
            track.recent_centers.append((candidate.x, candidate.y))
            if len(track.recent_centers) > 20:
                track.recent_centers.pop(0)
            track.recent_spreads.append(candidate.spread)
            if len(track.recent_spreads) > 10:
                track.recent_spreads.pop(0)
            self._add_points(track, cluster)
            self._refine(track)
            if track.frames >= self.lock_frames and track.spread >= self.lock_min_spread:
                track.locked = True
            accepted += 1
        return accepted

    def _nearest_track(self, candidate):
        best, best_distance = None, self.match_distance
        for track in self.tracks:
            gap = math.hypot(track.x-candidate.x, track.y-candidate.y)
            if gap < best_distance:
                best, best_distance = track, gap
        return best

    def estimates(self):
        """按锁定状态与帧数返回估计，最多两个。

        用近期单帧中心的中位数做最终输出：对个别离群帧稳健。
        """
        ready = [t for t in self.tracks if t.frames >= 2]
        ready.sort(key=lambda t: (not t.locked, -t.frames))
        result = []
        for track in ready[:2]:
            xs = sorted(c[0] for c in track.recent_centers)
            ys = sorted(c[1] for c in track.recent_centers)
            median_x = xs[len(xs)//2]
            median_y = ys[len(ys)//2]
            result.append(PoleEstimate(median_x, median_y, track.radius, track.frames,
                                       len(track.voxels), getattr(track, "residual", 0.0),
                                       track.spread, track.locked))
        return result

    def locked_pair(self, min_separation=0.6):
        """两根杆都锁定且间距足够时返回两个估计，否则返回 None。

        间距门槛与单帧检测一致：太近的两个"杆"不是两根杆。
        """
        estimates = [e for e in self.estimates() if e.locked]
        if len(estimates) < 2:
            return None
        gap = math.hypot(estimates[0].x-estimates[1].x, estimates[0].y-estimates[1].y)
        if gap < min_separation:
            return None
        return estimates[:2]

    def reset(self):
        self.tracks = []
        self.last_stamp = None
        self.frame = ""
        self.rejected = 0


def route_config(config, estimates, radius_margin=0.02):
    """把杆位估计写成一份可用于 make_route 的配置副本。

    约定：以两杆中点为交叉点，下桩取 y 较小的一根（与规则"先绕下方"一致）。
    radius_margin 是给检测半径加的安全裕量：宁可绕远一点，也不要碰底座。
    """
    if len(estimates) < 2:
        raise ValueError("need two pole estimates")
    ordered = sorted(estimates, key=lambda e: e.y)
    lower, upper = ordered[0], ordered[1]
    merged = dict(config)
    merged["lower_pole"] = [lower.x, lower.y]
    merged["upper_pole"] = [upper.x, upper.y]
    merged["pole_radius"] = max(lower.radius, upper.radius) + radius_margin
    merged["robot_radius"] = config["robot_radius"]
    merged["margin"] = config["margin"]
    return merged
