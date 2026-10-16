"""杆检测的离线验证：用合成点云量误差，并检查安全行为。

这些测试不需要 ROS、雷达、NUC 或真车，Windows 上直接跑：
    python3 -m unittest test_pole_detect -v
    python3 -m unittest discover -p "test_*.py"
"""
import math
import unittest

import pole_detect
import pole_sim


HALF = 0.8
POLES = [(-HALF, 0.0), (HALF, 0.0)]
DOME_RADIUS = 0.18
BOUNDS = (-3.0, 3.0, -3.0, 3.0)


def make_scene(keep_deg=None, clutter=(), ground_noise=0.012, seed=7,
               dome_radius=DOME_RADIUS, poles=POLES):
    occlude = (0.0, -1.5, keep_deg) if keep_deg else None
    return pole_sim.scene(poles, bounds=BOUNDS, dome_radius=dome_radius,
                          ground_noise=ground_noise, clutter=clutter,
                          occlude=occlude, seed=seed)


def pair_with_truth(detection, truth):
    """把检测结果与真值配对，返回 [(误差, 半径, 候选), ...]。"""
    remaining = [p["center"] for p in truth["poles"]]
    paired = []
    for candidate in detection.poles:
        best = min(remaining, key=lambda c: math.hypot(candidate.x-c[0], candidate.y-c[1]))
        remaining.remove(best)
        paired.append((math.hypot(candidate.x-best[0], candidate.y-best[1]),
                       candidate.radius, candidate))
    return paired


class DetectionAccuracyTests(unittest.TestCase):
    def test_ideal_cloud_is_centimetre_accurate(self):
        """无噪声、全周可见时误差应在毫米级，半径接近真值。"""
        points, truth = make_scene()
        detection = pole_detect.detect_poles(points)
        self.assertTrue(detection.saw_two, detection.reason)
        for error, radius, _candidate in pair_with_truth(detection, truth):
            self.assertLess(error, 0.01)
            self.assertAlmostEqual(radius, DOME_RADIUS, delta=0.01)

    def test_ground_noise_clutter_and_partial_view(self):
        """地面噪声 + 两个干扰圆柱 + 遮挡到250度仍应可用（误差<2cm）。"""
        clutter = [(-1.2, 1.0, 0.12, 0.9), (1.3, -0.9, 0.15, 0.7)]
        points, truth = make_scene(keep_deg=250, clutter=clutter, ground_noise=0.015)
        detection = pole_detect.detect_poles(points)
        self.assertTrue(detection.saw_two, detection.reason)
        for error, radius, _candidate in pair_with_truth(detection, truth):
            self.assertLess(error, 0.02)
            self.assertAlmostEqual(radius, DOME_RADIUS, delta=0.02)

    def test_radius_is_never_underestimated_badly(self):
        """半径偏小会让车贴着底座过；合格的检测半径必须接近真实外廓。"""
        for keep in (None, 300, 270, 250):
            with self.subTest(keep=keep):
                points, truth = make_scene(keep_deg=keep, ground_noise=0.015)
                detection = pole_detect.detect_poles(points)
                self.assertTrue(detection.saw_two, detection.reason)
                for _error, radius, _candidate in pair_with_truth(detection, truth):
                    self.assertAlmostEqual(radius, DOME_RADIUS, delta=0.02)

    def test_larger_dome_scales(self):
        """换成更大的底座（半径0.25 m）仍应检出且半径跟着变。"""
        points, truth = make_scene(dome_radius=0.25)
        detection = pole_detect.detect_poles(points)
        self.assertTrue(detection.saw_two, detection.reason)
        for _error, radius, _candidate in pair_with_truth(detection, truth):
            self.assertAlmostEqual(radius, 0.25, delta=0.03)

    def test_spacing_is_recovered_for_route_generation(self):
        """路线要用两杆间距，检测出的间距误差必须很小。"""
        for spacing in (1.2, 1.6, 2.4):
            with self.subTest(spacing=spacing):
                half = spacing/2.0
                points, _truth = make_scene(poles=[(-half, 0.0), (half, 0.0)])
                detection = pole_detect.detect_poles(points)
                self.assertTrue(detection.saw_two, detection.reason)
                measured = math.hypot(detection.poles[0].x-detection.poles[1].x,
                                      detection.poles[0].y-detection.poles[1].y)
                self.assertAlmostEqual(measured, spacing, delta=0.03)


class SafetyBehaviourTests(unittest.TestCase):
    """检测必须"宁可说看不到，也不给错坐标"。"""

    def test_severe_occlusion_is_refused_not_guessed(self):
        """只看到约90度球面时圆心偏10cm以上，必须拒绝而不是照给。"""
        points, _truth = make_scene(keep_deg=90)
        detection = pole_detect.detect_poles(points)
        self.assertFalse(detection.saw_two)
        self.assertIn("usable pole candidate", detection.reason)

    def test_spread_gate_separates_usable_from_dangerous(self):
        """门槛必须切在"误差约5cm"上：这是安全性，不是调参偏好。

        实测（底座半径0.18）：
          可见 90度 -> spread 0.62 误差 11.0 cm  -> 必须拒绝
          可见200度 -> spread 0.70 误差  4.1 cm  -> 拒绝（临界）
          可见250度 -> spread 0.80 误差  1.1 cm  -> 必须通过
        """
        for keep, should_pass in ((90, False), (180, False), (250, True), (360, True)):
            with self.subTest(keep=keep):
                points, truth = make_scene(keep_deg=keep if keep < 360 else None)
                detection = pole_detect.detect_poles(points)
                self.assertEqual(detection.saw_two, should_pass, detection.reason)
                if should_pass:
                    for error, _radius, _c in pair_with_truth(detection, truth):
                        self.assertLess(error, 0.02)

    def test_single_pole_is_reported_as_incomplete(self):
        """只看到一根时必须报"不足"，不能拿一根硬凑路线。"""
        points, _truth = make_scene(poles=[(0.0, 0.0)])
        detection = pole_detect.detect_poles(points)
        self.assertFalse(detection.saw_two)
        self.assertLess(len(detection.poles), 2)

    def test_clutter_alone_is_not_two_poles(self):
        """只有干扰圆柱、没有真杆时不得报出两根杆。"""
        clutter = [(-1.2, 1.0, 0.12, 0.9), (1.3, -0.9, 0.15, 0.7)]
        points, _truth = pole_sim.scene([], bounds=BOUNDS, clutter=clutter,
                                        ground_samples=800, self_samples=0, seed=11)
        detection = pole_detect.detect_poles(points)
        self.assertFalse(detection.saw_two, detection.reason)

    def test_empty_height_band_is_reported(self):
        points = [(0.0, 0.0, 2.0), (1.0, 0.0, 3.0)]
        detection = pole_detect.detect_poles(points, min_height=0.02, max_height=0.35)
        self.assertFalse(detection.saw_two)
        self.assertIn("height band is empty", detection.reason)

    def test_rejection_reason_is_specific(self):
        """拒绝时必须说明原因，否则现场无法判断是高度带错了还是点数不够。"""
        points, _truth = make_scene(keep_deg=90)
        detection = pole_detect.detect_poles(points)
        self.assertTrue("spread" in detection.reason or "inliers" in detection.reason,
                        detection.reason)


class GeometryHelperTests(unittest.TestCase):
    def test_fit_circle_recovers_known_circle(self):
        points = [(1.5+0.2*math.cos(i*2*math.pi/60), -0.7+0.2*math.sin(i*2*math.pi/60))
                  for i in range(60)]
        cx, cy, radius, rms, count = pole_detect.fit_circle(points)
        self.assertAlmostEqual(cx, 1.5, delta=0.005)
        self.assertAlmostEqual(cy, -0.7, delta=0.005)
        self.assertAlmostEqual(radius, 0.2, delta=0.005)
        self.assertLess(rms, 0.01)
        self.assertEqual(count, 60)

    def test_fit_circle_rejects_outliers(self):
        """局部离群点不应把圆心拉走。"""
        points = [(0.2*math.cos(i*2*math.pi/60), 0.2*math.sin(i*2*math.pi/60))
                  for i in range(60)]
        points.extend([(5.0, 5.0), (-4.0, 3.0), (3.0, -5.0)])
        cx, cy, radius, _rms, count = pole_detect.fit_circle(points)
        self.assertLess(math.hypot(cx, cy), 0.02)
        self.assertAlmostEqual(radius, 0.2, delta=0.02)
        self.assertLess(count, len(points))

    def test_limit_points_keeps_budget_and_coverage(self):
        points = [(float(i), 0.0, 0.1) for i in range(1000)]
        kept = pole_detect.limit_points(points, 100)
        self.assertLessEqual(len(kept), 100)
        self.assertEqual(kept[0], points[0])

    def test_limit_points_handles_generators(self):
        generator = ((float(i), 0.0, 0.1) for i in range(1000))
        kept = pole_detect.limit_points(generator, 50)
        self.assertEqual(len(kept), 50)

    def test_height_band_filters_and_drops_non_finite(self):
        band = pole_detect.height_band(
            [(0.0, 0.0, 0.1), (0.0, 0.0, 5.0), (0.0, 0.0, -1.0),
             (float("nan"), 0.0, 0.1)], 0.02, 0.35)
        self.assertEqual(len(band), 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
