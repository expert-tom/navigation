"""杆检测：从一帧点云里找出两根立杆（含半球底座）的中心与半径。

设计要点（针对"半球底座 + 细杆 + 地面噪声"这个现场条件）：

1. 只取【低高度带】做中心与半径的估计。半球在离地最低处最宽，
   若用全高度带（含细杆）拟合，半径会被细杆拉小 → 车会贴着底座过。
2. 用占用栅格的连通分量做聚类，而不是逐点距离比较：复杂度与点数近似线性。
3. 半径取"到中心距离的高分位数"，对部分弧面可见（遮挡）和离群点稳健。
4. 只接受"半径落在合理区间、点数足够"的候选，并按点数排序取前两个；
   看不到两根时返回 saw_two=False，由调用方决定等待，**绝不硬凑路线**。

纯标准库实现，不依赖 ROS 与 numpy，可在任意机器上离线验证。
性能由降采样保证：入口先截断到 max_points。
"""
import argparse
import math
try:
    from dataclasses import dataclass
except ImportError:
    dataclass = None


@dataclass
class Candidate:
    """一个候选杆（聚类结果）。"""
    x: float
    y: float
    radius: float
    points: int
    z_min: float
    z_max: float
    inliers: int = 0
    rms: float = 0.0
    spread: float = 0.0
    taper: float = 0.0


@dataclass
class Detection:
    """一帧的检测结论。"""
    poles: list
    saw_two: bool
    reason: str
    raw_candidates: int
    stamp: float = 0.0
    base_pose: tuple = None
    points: list = None
    frame: str = ""


def limit_points(points, max_points):
    """按预算截断点流（等间隔抽样保留全视场覆盖）。"""
    if max_points is None or max_points <= 0:
        return list(points)
    try:
        total = len(points)
    except TypeError:
        points = list(points)
        total = len(points)
    if total <= max_points:
        return list(points)
    stride = max(1, math.ceil(total / max_points))
    return list(points)[::stride]


def height_band(points, min_height, max_height):
    """保留 base_link 坐标系下高度落在区间内的点，返回 (x, y, z) 三元组列表。"""
    band = []
    for point in points:
        if len(point) < 3:
            continue
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        if min_height <= z <= max_height:
            band.append((x, y, z))
    return band


def cluster_cells(band, cell_size):
    """占用栅格 + 连通分量（8邻域）。

    返回 (points_by_cell, groups)：points_by_cell 保留每个栅格里的原始点，
    供后续圆拟合使用。
    """
    if cell_size <= 0:
        raise ValueError("cell_size 必须为正")
    points_by_cell = {}
    for x, y, z in band:
        key = (math.floor(x/cell_size), math.floor(y/cell_size))
        entry = points_by_cell.get(key)
        if entry is None:
            points_by_cell[key] = [(x, y, z)]
        else:
            entry.append((x, y, z))
    seen = set()
    groups = []
    for start in points_by_cell:
        if start in seen:
            continue
        stack = [start]
        seen.add(start)
        cells = []
        while stack:
            cell = stack.pop()
            cells.append(cell)
            cx, cy = cell
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    neighbour = (cx+dx, cy+dy)
                    if neighbour in points_by_cell and neighbour not in seen:
                        seen.add(neighbour)
                        stack.append(neighbour)
        groups.append(cells)
    return points_by_cell, groups


def _percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered)-1, max(0, int(round(fraction*(len(ordered)-1)))))
    return ordered[index]


def fit_circle(points, trim=2.5):
    """Taubin 圆拟合 + 一轮局部离群点剔除。

    返回 (cx, cy, r, rms, count)；退化时返回 None。
    用拟合而不是"质心+分位数"的原因：只看得到球面的一部分时，
    质心会被拉向可见一侧（实测偏移可达0.08 m），而圆拟合能用弧的曲率
    同时约束圆心与半径。
    """
    data = [(float(p[0]), float(p[1])) for p in points]
    if len(data) < 8:
        return None
    keep = data
    for _ in range(2):
        fit = _taubin(keep)
        if fit is None:
            return None
        cx, cy, radius, rms = fit
        residuals = [abs(math.hypot(x-cx, y-cy)-radius) for x, y in keep]
        ordered = sorted(residuals)
        median = ordered[len(ordered)//2]
        mad = sorted(abs(value-median) for value in residuals)[len(residuals)//2] or 1e-6
        limit = median + trim*1.4826*mad
        filtered = [p for p, residual in zip(keep, residuals) if residual <= limit]
        if len(filtered) < 8 or len(filtered) == len(keep):
            keep = filtered if len(filtered) >= 8 else keep
            break
        keep = filtered
    fit = _taubin(keep)
    if fit is None:
        return None
    cx, cy, radius, rms = fit
    return cx, cy, radius, rms, len(keep)


def _taubin(points):
    """Taubin 代数圆拟合（对部分弧与噪声比最小二乘更稳）。"""
    count = len(points)
    if count < 3:
        return None
    mx = sum(p[0] for p in points)/count
    my = sum(p[1] for p in points)/count
    suu = suv = svv = suuu = svvv = suvv = svuu = 0.0
    for x, y in points:
        u, v = x-mx, y-my
        suu += u*u
        svv += v*v
        suv += u*v
        suuu += u*u*u
        svvv += v*v*v
        suvv += u*v*v
        svuu += v*u*u
    det = suu*svv-suv*suv
    if abs(det) < 1e-12:
        return None
    rhs_u = 0.5*(suuu+suvv)
    rhs_v = 0.5*(svvv+svuu)
    uc = (rhs_u*svv-rhs_v*suv)/det
    vc = (rhs_v*suu-rhs_u*suv)/det
    cx, cy = uc+mx, vc+my
    radii = [math.hypot(x-cx, y-cy) for x, y in points]
    radius = sum(radii)/count
    if not (math.isfinite(radius) and radius > 0):
        return None
    rms = math.sqrt(sum((value-radius)**2 for value in radii)/count)
    return cx, cy, radius, rms


def candidate_from_cells(points_by_cell, cells, cell_size, radius_percentile=0.95):
    """还原候选杆：圆心用圆拟合，半径用"到该圆心的距离高分位数"。

    为什么半径不能用拟合值：拟合的是点云的**平均**半径。半球底座的投影是
    实心圆盘，点并不均匀充满（外缘更密），代数拟合会把半径估小（实测 0.18→0.14）。
    而半径偏小意味着车会贴着底座过 —— 对避障是危险方向。
    所以半径取高分位数，逼近真实外廓。
    """
    raw = []
    for cell in cells:
        raw.extend(points_by_cell.get(cell, ()))
    if not raw:
        return None
    total = len(raw)
    z_values = [p[2] for p in raw]
    fit = fit_circle(raw)
    if fit is not None:
        cx, cy, _fitted_radius, _rms, _kept = fit
    else:
        cx = sum(p[0] for p in raw)/total
        cy = sum(p[1] for p in raw)/total
    distances = [math.hypot(p[0]-cx, p[1]-cy) for p in raw]
    radius = _percentile(distances, radius_percentile)
    rms = fit[3] if fit is not None else 0.0
    p50 = _percentile(distances, 0.5) or 1e-9
    spread = p50/max(radius, 1e-9)
    return Candidate(cx, cy, radius, total, min(z_values), max(z_values),
                     inliers=fit[4] if fit is not None else total,
                     rms=rms, spread=spread, taper=height_taper(raw))


def height_taper(points, bins=4):
    """高度剖面的收窄程度：各层半宽的最小值 / 最大值。

    半球底座"下宽上窄"（实测剖面 [0.238,0.234,0.149,0.02] → 锥度≈0.08），
    而圆柱上下一样宽（[0.15,0.15,0.15,0.15] → 锥度≈1.0）。
    这是区分"底座"与"椅子腿/柱子/人腿"最可靠的判据：
    只看半径和点数时，一个 r≈0.18 的圆柱与底座几乎无法区分。

    返回 0..1：越小越像底座，接近1说明是直筒。
    """
    if not points:
        return 1.0
    z_values = [p[2] for p in points]
    z0, z1 = min(z_values), max(z_values)
    if z1-z0 < 1e-6:
        return 1.0
    widths = []
    for index in range(bins):
        lo = z0 + (z1-z0)*index/bins
        hi = z0 + (z1-z0)*(index+1)/bins
        xs, ys = [], []
        for p in points:
            if lo <= p[2] <= hi:
                xs.append(p[0])
                ys.append(p[1])
        if len(xs) < 5:
            continue
        widths.append(max(max(xs)-min(xs), max(ys)-min(ys))/2.0)
    if len(widths) < 2:
        return 1.0
    return min(widths)/max(max(widths), 1e-9)


def angular_coverage(points, cx, cy):
    """保留为诊断工具：点方向的覆盖角度。

    注意：**不能**用这个量做质量门槛。半球底座的二维投影是实心圆盘，
    点的方向本身就铺满360°，即使只看到半个球，方向覆盖依然接近满值
    （实测弧面只剩90°时该量仍给出 6.0 rad）。衡量可见完整度请用 spread。
    """
    angles = sorted(math.atan2(p[1]-cy, p[0]-cx) for p in points)
    if len(angles) < 3:
        return 0.0
    gaps = [b-a for a, b in zip(angles, angles[1:])]
    gaps.append((angles[0]+2.0*math.pi)-angles[-1])
    return max(0.0, 2.0*math.pi-max(gaps))


def detect_poles(points, min_height=0.02, max_height=0.45, cell_size=0.08,
                 min_radius=0.05, max_radius=0.40, min_points=25,
                 max_points=20000, max_span=1.2, min_inliers=20, min_spread=0.78,
                 max_taper=0.5, min_separation=0.6, stamp=0.0, base_pose=None):
    """返回 Detection。只接受"半球底座"形状的团块。

    高度带要比底座本身略高（默认 0.02~0.45 m）：锥度判据需要看到
    "下宽上窄"的剖面，只取最底下很薄一层时底座与直筒无法区分。

    min_height/max_height 不要用整车高度带：细杆会把半径估计拉小。

    四道质量门槛，都是为了"宁可说看不到，也不给错坐标"：
      min_spread      p50距离/半径，衡量底座是否完整可见（全周≈0.87，仅90°≈0.60）
      max_taper       高度剖面收窄程度，排除椅子腿/柱子/人腿
                      （实测底座0.07~0.15，圆柱0.99~1.00，区分度极大）
      min_inliers     圆拟合内点数
      min_separation  两个候选中点的最小间距。**没有这道门槛时，底座旁边的
                      椅子腿会被当成第二根杆**，于是"两根杆"位置全错。
    """
    band = height_band(limit_points(points, max_points), min_height, max_height)
    if not band:
        return Detection([], False, "height band is empty; check min_height/max_height", 0,
                         stamp=stamp, base_pose=base_pose)
    points_by_cell, groups = cluster_cells(band, cell_size)
    candidates = []
    cluster_points = []
    rejected = []
    for cells in groups:
        candidate = candidate_from_cells(points_by_cell, cells, cell_size)
        if candidate is None or candidate.points < min_points:
            continue
        if candidate.inliers < min_inliers:
            rejected.append("inliers")
            continue
        if candidate.spread < min_spread:
            rejected.append("spread%.2f" % candidate.spread)
            continue
        if candidate.taper > max_taper:
            rejected.append("taper%.2f" % candidate.taper)
            continue
        if not (min_radius <= candidate.radius <= max_radius):
            rejected.append("radius%.3f" % candidate.radius)
            continue
        if candidate.z_max-candidate.z_min > max_span:
            rejected.append("span")
            continue
        candidates.append(candidate)
        cluster_points.append([point for key in cells
                               for point in points_by_cell.get(key, ())])
    order = sorted(range(len(candidates)), key=lambda i: candidates[i].points, reverse=True)
    candidates = [candidates[i] for i in order]
    cluster_points = [cluster_points[i] for i in order]
    if len(candidates) < 2:
        return Detection(candidates, False,
                         "only %d usable pole candidate(s) of %d groups (%s)"
                         % (len(candidates), len(groups), ",".join(sorted(set(rejected))) or "none"),
                         len(candidates), stamp=stamp, base_pose=base_pose)
    pair = None
    for i in range(len(candidates)):
        for j in range(i+1, len(candidates)):
            gap = math.hypot(candidates[i].x-candidates[j].x, candidates[i].y-candidates[j].y)
            if gap >= min_separation:
                pair = (i, j)
                break
        if pair:
            break
    if pair is None:
        return Detection(candidates[:2], False,
                         "two candidates but they are closer than %.2f m; likely not two poles"
                         % min_separation, len(candidates), stamp=stamp, base_pose=base_pose)
    i, j = pair
    return Detection([candidates[i], candidates[j]], True, "ok", len(candidates),
                     stamp=stamp, base_pose=base_pose,
                     points=[cluster_points[i], cluster_points[j]])


def main():
    parser = argparse.ArgumentParser(description="对合成点云跑杆检测，输出误差")
    parser.add_argument("--spacing", type=float, default=1.6)
    parser.add_argument("--dome-diameter", type=float, default=0.36)
    parser.add_argument("--ground-noise", type=float, default=0.012)
    parser.add_argument("--clutter", type=int, default=0)
    parser.add_argument("--occlude-deg", type=float, default=0.0)
    parser.add_argument("--min-height", type=float, default=0.02)
    parser.add_argument("--max-height", type=float, default=0.35)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--repeat", type=int, default=1, help="换种子重复多次，统计误差")
    args = parser.parse_args()

    import pole_sim

    half = args.spacing/2.0
    poles = [(-half, 0.0), (half, 0.0)]
    clutter = [(-1.2, 1.0, 0.12, 0.9), (1.3, -0.9, 0.15, 0.7)][:args.clutter]
    occlude = (0.0, -1.5, args.occlude_deg) if args.occlude_deg > 0 else None

    errors, radii, misses = [], [], 0
    for trial in range(args.repeat):
        points, truth = pole_sim.scene(
            poles, bounds=(-3, 3, -3, 3), dome_radius=args.dome_diameter/2.0,
            ground_noise=args.ground_noise, clutter=clutter, occlude=occlude,
            seed=args.seed+trial)
        result = detect_poles(points, min_height=args.min_height, max_height=args.max_height)
        if not result.saw_two:
            misses += 1
            if trial == 0:
                print("未看到两根：%s（候选 %d 个）" % (result.reason, result.raw_candidates))
            continue
        truth_centers = [p["center"] for p in truth["poles"]]
        remaining = list(truth_centers)
        for candidate in result.poles:
            best = min(remaining, key=lambda c: math.hypot(candidate.x-c[0], candidate.y-c[1]))
            remaining.remove(best)
            error = math.hypot(candidate.x-best[0], candidate.y-best[1])
            errors.append(error)
            radii.append(candidate.radius)
            if trial == 0:
                print("检测杆心 (%.3f, %.3f)  真值 %s  误差 %.4f m  拟合半径 %.4f m  点数 %d"
                      % (candidate.x, candidate.y, best, error, candidate.radius, candidate.points))
        if trial == 0:
            true_radius = truth["dome_radius"]
            print("真值底座半径 %.4f m" % true_radius)

    if args.repeat > 1:
        print("-"*60)
        print("重复 %d 次：漏检 %d 次" % (args.repeat, misses))
    if errors:
        print("杆心误差: 平均 %.4f m  最大 %.4f m" % (
            sum(errors)/len(errors), max(errors)))
        print("拟合半径: 平均 %.4f m  范围 %.4f~%.4f m" % (
            sum(radii)/len(radii), min(radii), max(radii)))


if __name__ == "__main__":
    main()
