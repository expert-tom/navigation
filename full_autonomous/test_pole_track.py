"""稳定层测试：跨帧累积能否在"车靠近→互相遮挡"的过程中锁定正确坐标。"""
import math
import unittest

import pole_detect
import pole_sim
import pole_track


HALF = 0.8
POLES = [(-HALF, 0.0), (HALF, 0.0)]
DOME_RADIUS = 0.18
BOUNDS = (-3.0, 3.0, -3.0, 3.0)
CLUTTER = [(-1.2, 1.0, 0.12, 0.9), (1.3, -0.9, 0.15, 0.7)]


def make_frame(robot, keep_deg, seed):
    """生成"车在 robot 处，每根杆只看到 keep_deg 锥角"的一帧点云。

    keep_deg 用极坐标锥角建模遮挡：雷达只能看到朝向车体一侧的球面，
    车越近、杆越容易被车体或另一根杆挡住，可见锥角越小。
    """
    occlude = None if keep_deg >= 359.0 else (robot[0], robot[1], keep_deg)
    points, truth = pole_sim.scene(POLES, pose=(robot[0], robot[1], 0.0), bounds=BOUNDS,
                                   dome_radius=DOME_RADIUS, ground_noise=0.012,
                                   clutter=CLUTTER, occlude=occlude, seed=seed)
    return points, truth


class TrackerTests(unittest.TestCase):
    def test_locks_correct_position_from_multiple_frames(self):
        """从多个视角累积后应锁定到正确杆位。

        帧数必须达到 lock_frames(默认5) 才允许锁定：这是刻意的，
        单帧就锁定会把一次坏观测永久固化。
        """
        tracker = pole_track.PoleTracker()
        steps = ((-2.0, 360.0), (-1.7, 360.0), (-1.4, 320.0),
                 (-1.2, 300.0), (-1.0, 280.0), (-0.9, 260.0))
        for index, (robot_y, keep) in enumerate(steps):
            robot = (0.0, robot_y)
            points, _truth = make_frame(robot, keep, seed=20+index)
            tracker.accumulate(pole_detect.detect_poles(
                points, stamp=index*0.1, base_pose=(robot[0], robot[1], 0.0)))
        pair = tracker.locked_pair()
        self.assertIsNotNone(pair, "累积 %d 帧后仍未锁定: %s" % (len(steps), [
            (round(t.x, 3), round(t.y, 3), t.frames, round(t.spread, 2), t.locked)
            for t in tracker.tracks]))
        remaining = list(POLES)
        for estimate in pair:
            best = min(remaining, key=lambda c: math.hypot(estimate.x-c[0], estimate.y-c[1]))
            remaining.remove(best)
            error = math.hypot(estimate.x-best[0], estimate.y-best[1])
            self.assertLess(error, 0.03, "锁定位置误差 %.4f m" % error)
            self.assertAlmostEqual(estimate.radius, DOME_RADIUS, delta=0.03)

    def test_locked_estimate_survives_occlusion_and_motion(self):
        """关键场景：车边开边靠近、遮挡越来越重，已锁定的坐标不能被污染。

        这也是"不需要每帧都测准"的核心：锁定后即使检测全部失败，
        坐标仍然可用（此时路线早已生成，跟踪不再需要重新检测）。
        """
        tracker = pole_track.PoleTracker()
        for index in range(6):
            robot = (0.0, -2.5+index*0.2)
            points, _truth = make_frame(robot, 360.0, seed=40+index)
            tracker.accumulate(pole_detect.detect_poles(
                points, stamp=index*0.1, base_pose=(robot[0], robot[1], 0.0)))
        locked = tracker.locked_pair()
        self.assertIsNotNone(locked, "远距离完整可见时应该能锁定: %s" % [
            (round(t.x, 3), round(t.y, 3), t.frames, round(t.spread, 2), t.locked)
            for t in tracker.tracks])
        before = sorted((round(e.x, 4), round(e.y, 4)) for e in locked)
        before_radius = sorted(round(e.radius, 4) for e in locked)
        for index, keep in enumerate((300.0, 240.0, 180.0, 120.0)):
            robot = (0.0, -1.4+index*0.2)
            points, _truth = make_frame(robot, keep, seed=60+index)
            tracker.accumulate(pole_detect.detect_poles(
                points, stamp=1.0+index*0.1, base_pose=(robot[0], robot[1], 0.0)))
        after_pair = tracker.locked_pair()
        self.assertIsNotNone(after_pair, "遮挡后锁定状态丢失了")
        after = sorted((round(e.x, 4), round(e.y, 4)) for e in after_pair)
        after_radius = sorted(round(e.radius, 4) for e in after_pair)
        self.assertEqual(before, after, "遮挡后锁定坐标被改动: %s -> %s" % (before, after))
        self.assertEqual(before_radius, after_radius, "遮挡后锁定半径被改动")

    def test_incomplete_frames_are_counted_not_used(self):
        tracker = pole_track.PoleTracker()
        empty = pole_detect.Detection([], False, "nothing", 0, stamp=0.1)
        self.assertEqual(tracker.accumulate(empty), 0)
        self.assertEqual(tracker.rejected, 1)
        self.assertIsNone(tracker.locked_pair())

    def test_stamp_must_increase(self):
        tracker = pole_track.PoleTracker()
        points, _truth = make_frame((0.0, -2.0), 360.0, seed=80)
        tracker.accumulate(pole_detect.detect_poles(points, stamp=1.0,
                                                    base_pose=(0.0, -2.0, 0.0)))
        with self.assertRaises(ValueError):
            tracker.accumulate(pole_detect.detect_poles(points, stamp=0.5,
                                                        base_pose=(0.0, -2.0, 0.0)))

    def test_voxel_accumulation_is_bounded(self):
        """累积点必须有上界，否则长跑会吃光内存。"""
        tracker = pole_track.PoleTracker(max_points_per_pole=100)
        for index in range(10):
            points, _truth = make_frame((0.0, -2.0), 360.0, seed=90+index)
            tracker.accumulate(pole_detect.detect_poles(points, stamp=index*0.1,
                                                        base_pose=(0.0, -2.0, 0.0)))
        for track in tracker.tracks:
            self.assertLessEqual(len(track.voxels), 200)


class RouteConfigTests(unittest.TestCase):
    def setUp(self):
        import json
        from pathlib import Path
        root = Path(__file__).resolve().parent
        self.config = json.loads((root/"config.json").read_text(encoding="utf-8"))

    def test_route_config_produces_usable_route(self):
        """把估计写成配置后，必须能真的构造出 8 字路线。"""
        from navigation import make_route
        estimates = [pole_track.PoleEstimate(-0.8, -0.05, 0.18, 5, 900, 0.02, 0.85, True),
                     pole_track.PoleEstimate(0.8, 0.05, 0.18, 5, 900, 0.02, 0.85, True)]
        merged = pole_track.route_config(self.config, estimates)
        route = make_route(merged)
        order = []
        for point in route:
            if not order or order[-1] != point[2]:
                order.append(point[2])
        self.assertEqual(order, ["DEPART", "LOWER_CCW", "UPPER_CW", "LOWER_EXIT", "RETURN"])

    def test_lower_pole_is_the_one_with_smaller_y(self):
        """规则要求先绕下方那根：下桩必须取 y 较小的一根。"""
        estimates = [pole_track.PoleEstimate(0.5, 1.4, 0.18, 5, 900, 0.02, 0.85, True),
                     pole_track.PoleEstimate(-0.5, -1.4, 0.18, 5, 900, 0.02, 0.85, True)]
        merged = pole_track.route_config(self.config, estimates)
        self.assertLess(merged["lower_pole"][1], merged["upper_pole"][1])

    def test_radius_margin_is_added(self):
        """检测半径要加安全裕量：宁可绕远，不要碰底座。"""
        estimates = [pole_track.PoleEstimate(-0.8, -0.05, 0.18, 5, 900, 0.02, 0.85, True),
                     pole_track.PoleEstimate(0.8, 0.05, 0.20, 5, 900, 0.02, 0.85, True)]
        merged = pole_track.route_config(self.config, estimates, radius_margin=0.03)
        self.assertAlmostEqual(merged["pole_radius"], 0.23)

    def test_one_estimate_is_rejected(self):
        estimates = [pole_track.PoleEstimate(0.0, 0.0, 0.18, 5, 900, 0.02, 0.85, True)]
        with self.assertRaises(ValueError):
            pole_track.route_config(self.config, estimates)


if __name__ == "__main__":
    unittest.main(verbosity=2)
