import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

from navigation import Planner, validate_config
from robot_model import inspect_robot, number

ROOT = Path(__file__).resolve().parent


class RobotModelTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT/"config.json").read_text(encoding="utf-8"))

    def test_oriented_wheels_and_rounded_radius(self):
        report = inspect_robot()
        exact = math.hypot(math.sqrt(2)*0.18+0.04/2, 0.075)
        self.assertGreaterEqual(report["collision_bound_m"], exact)
        self.assertLess(report["collision_bound_m"]-exact, 0.0001)
        self.assertEqual(report["recommended_radius_m"], 0.29)
        self.assertEqual(len(report["components"]), 5)
        self.assertAlmostEqual(report["body_radius_m"], 0.27)

    def test_full_outlines_inside_planning_circle(self):
        report = inspect_robot()
        for part in report["components"]:
            for point in part["outline_xy"]:
                self.assertLessEqual(math.hypot(*point), self.config["robot_radius"])

    def test_old_radius_rejected(self):
        for radius in (0.2, 0.27, 0.28):
            c = copy.deepcopy(self.config)
            c["robot_radius"] = radius
            with self.assertRaisesRegex(ValueError, "小于模型"):
                validate_config(c)

    def test_lidar_visual_is_not_used_as_extrinsic(self):
        report = inspect_robot()
        self.assertEqual(report["lidar_joint_origin"]["xyz"], "$(arg xyz)")
        self.assertEqual(report["lidar_argument_defaults"]["xyz"], "0.0 0.0 0.0")
        self.assertFalse(report["lidar_extrinsics_confirmed"])
        self.assertIn("livox_frame", report["links_without_collision"])

    def test_expression_does_not_execute_python(self):
        with self.assertRaises(ValueError):
            number("${__import__('os').getcwd()}", {})

    def test_unknown_collision_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"robot.urdf"
            path.write_text('<robot name="test"><link name="base_link"><collision><geometry>'
                            '<box size="1 1 1"/></geometry></collision></link></robot>', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "不支持的碰撞"):
                inspect_robot(path)

    def test_small_obstacle_between_sampling_points_is_detected(self):
        c = dict(self.config, robot_radius=0.0001, margin=0.0001)
        planner = Planner(c)
        self.assertFalse(planner.clear((0,0), (1,0), [(0.015,0,0.001)]))
        self.assertTrue(planner.clear((0,0), (1,0), [(0.015,0.01,0.001)]))

    def test_new_clearance_and_boundaries(self):
        planner = Planner(self.config)
        self.assertFalse(planner.free((0.45,0), [(0,0,0.1)]))
        self.assertTrue(planner.free((0.50,0), [(0,0,0.1)]))
        self.assertFalse(planner.free((-2.7,0), []))


if __name__ == "__main__":
    unittest.main(verbosity=2)
