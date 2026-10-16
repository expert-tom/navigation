"""端到端离线验证：里程计 + 点云 → 检测 → 稳定层 → 8 字路线。

这一步专门用来回答接线前必须确认的问题：
**点云先转到 map 系再累积，能否得到正确的杆位和可用的 8 字路线？**

模拟的是一条真实的工作流：
  1. 车从停车位朝两根杆行驶，每帧有一组真值位姿（当作 FAST-LIO 的里程计）；
  2. 每帧按该位姿生成 base_link 系的点云（pole_sim）；
  3. 用位姿把点云转到 map 系（这就是真实链路里 TF 做的事）；
  4. 检测 → 稳定层累积 → 锁定 → 生成路线配置；
  5. 与真值对比杆位、半径、间距，并检查路线不穿杆。

只依赖标准库。
"""
import argparse
import math

import pole_detect
import pole_sim
import pole_track


def rotate_translate(points, pose):
    """把 base_link 系的点转到 map 系：先绕 yaw 旋转，再平移。"""
    x, y, yaw = pose
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    out = []
    for px, py, pz in points:
        out.append((x + cos_yaw*px - sin_yaw*py,
                    y + sin_yaw*px + cos_yaw*py,
                    pz))
    return out


def to_base_frame(points, pose):
    """map → base_link：先减去车体位置，再反向旋转。"""
    x, y, yaw = pose
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    out = []
    for px, py, pz in points:
        dx, dy = px-x, py-y
        out.append((cos_yaw*dx + sin_yaw*dy, -sin_yaw*dx + cos_yaw*dy, pz))
    return out


def approach(robot_start, pole_map, frames=8, step=0.2, visibility=None):
    """生成一串 (位姿, 点云_base, 可见锥角) 的时间序列。

    **杆固定在世界（map）系**，每帧再换算到 base_link 交给生成器 ——
    这才是物理正确的：车动的时候，杆相对车的位置会变。
    """
    sequence = []
    mid_x = sum(p[0] for p in pole_map)/len(pole_map)
    mid_y = sum(p[1] for p in pole_map)/len(pole_map)
    for index in range(frames):
        y = robot_start[1] + index*step
        robot = (robot_start[0], y)
        yaw = math.atan2(mid_y-robot[1], mid_x-robot[0])
        pose = (robot[0], robot[1], yaw)
        keep = 360.0 if visibility is None else visibility(index)
        base_poles = to_base_frame([(p[0], p[1], 0.0) for p in pole_map], pose)
        sequence.append((pose, keep, index, [(p[0], p[1]) for p in base_poles]))
    return sequence


def run(pole_map, frames=8, step=0.2, dome_radius=0.18, ground_noise=0.012,
        visibility=None, seed=0):
    """pole_map 是**世界(map)系**的杆位；每帧换算到 base_link 后生成点云。"""
    tracker = pole_track.PoleTracker()
    accepted = 0
    truth = None
    for pose, keep, _index, base_poles in approach((0.0, -2.5), pole_map, frames,
                                                   step, visibility):
        occlude = None if keep >= 359.0 else (0.0, 0.0, keep)
        map_points, truth = pole_sim.scene(base_poles, pose=pose,
                                           bounds=(-4, 4, -4, 4),
                                           dome_radius=dome_radius,
                                           ground_noise=ground_noise,
                                           occlude=occlude,
                                           seed=seed+_index, map_frame=True)
        index = _index
        detection = pole_detect.detect_poles(map_points, stamp=index*0.1,
                                             base_pose=pose)
        detection.frame = "map"
        accepted += tracker.accumulate(detection)
    return tracker, accepted, truth


def report(poles, tracker, accepted, truth, config):
    from navigation import make_route, distance
    truth_centers = [tuple(p["center"]) for p in truth["poles"]]
    print("=" * 66)
    print("真值杆位(map系): %s   底座半径 %.3f m"
          % ([tuple(round(v, 3) for v in p) for p in truth_centers], truth["dome_radius"]))
    print("累积接受的杆观测数: %d   被拒帧数: %d" % (accepted, tracker.rejected))
    for track in tracker.tracks:
        print("  轨道: 位置(%.4f, %.4f) 半径 %.4f  帧数 %d spread %.3f 锁定 %s"
              % (track.x, track.y, track.radius, track.frames, track.spread, track.locked))
    pair = tracker.locked_pair()
    if pair is None:
        print("结果: **未锁定** —— 拿不到可用杆位")
        return 1
    remaining = list(truth_centers)
    errors = []
    for estimate in pair:
        best = min(remaining, key=lambda c: math.hypot(estimate.x-c[0], estimate.y-c[1]))
        remaining.remove(best)
        errors.append(math.hypot(estimate.x-best[0], estimate.y-best[1]))
    true_spacing = distance(truth_centers[0], truth_centers[1])
    measured_spacing = math.hypot(pair[0].x-pair[1].x, pair[0].y-pair[1].y)
    print("-" * 66)
    print("锁定杆位: %s" % [(round(e.x, 4), round(e.y, 4)) for e in pair])
    print("位置误差: 最大 %.4f m" % max(errors))
    print("间距: 真值 %.3f m  检测 %.3f m  误差 %.4f m"
          % (true_spacing, measured_spacing, abs(true_spacing-measured_spacing)))
    merged = pole_track.route_config(config, pair)
    print("生成的绕行半径 pole_radius = %.4f m（含安全裕量）" % merged["pole_radius"])
    try:
        route = make_route(merged)
    except Exception as exc:
        print("结果: **make_route 失败** —— %s" % exc)
        return 1
    need = merged["pole_radius"] + merged["robot_radius"] + merged["margin"]
    violations = 0
    for point in route:
        for pole in truth_centers:
            if distance(point, pole) < need:
                violations += 1
    order = []
    for point in route:
        if not order or order[-1] != point[2]:
            order.append(point[2])
    print("路线点数 %d  顺序 %s" % (len(route), " → ".join(order)))
    print("路线距真实杆心小于安全半径(%.3f m)的点数: %d" % (need, violations))
    ok = max(errors) < 0.03 and violations == 0 \
        and order == ["DEPART", "LOWER_CCW", "UPPER_CW", "LOWER_EXIT", "RETURN"]
    print("结果: %s" % ("通过" if ok else "**不通过**"))
    return 0 if ok else 1


def main():
    import json
    from pathlib import Path
    parser = argparse.ArgumentParser(description="端到端离线验证：点云→检测→稳定层→8字路线")
    parser.add_argument("--spacing", type=float, default=1.6)
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument("--step", type=float, default=0.2)
    parser.add_argument("--occlude-from", type=int, default=6,
                        help="从第几帧开始收窄可见锥角。默认6：先让前几帧完整可见以完成锁定，"
                             "再模拟靠近后的遮挡。设0会让第一帧就只有 --occlude-min 的可见面，"
                             "检测器会正确地拒绝（可见面不足时给不出可信圆心）")
    parser.add_argument("--occlude-min", type=float, default=120.0)
    parser.add_argument("--ground-noise", type=float, default=0.012)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    config = json.loads((root/"config.json").read_text(encoding="utf-8"))
    half = args.spacing/2.0
    pole_map = [(-half, 0.4), (half, -1.2)]

    def visibility(index):
        if index < args.occlude_from:
            return 360.0
        return args.occlude_min

    tracker, accepted, truth = run(pole_map, frames=args.frames, step=args.step,
                                   ground_noise=args.ground_noise,
                                   visibility=visibility)
    return report(pole_map, tracker, accepted, truth, config)


if __name__ == "__main__":
    raise SystemExit(main())
