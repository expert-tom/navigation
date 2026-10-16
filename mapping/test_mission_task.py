import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

from mission_audit import EightAudit, STAGES
from navigation import Navigator, distance, fixed_obstacles
from prepare_task import inspect_task
from real_vehicle import load_settings
from task_config import apply_task
from vehicle_core import VehicleRuntime

ROOT = Path(__file__).resolve().parent


class MissionTaskTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT/"config.json").read_text(encoding="utf-8"))
        self.settings = load_settings(ROOT/"config/real_vehicle.json")

    def test_single_goal_does_not_require_example_eight_geometry(self):
        self.config["parking"] = [0, -1]  # At a legacy example pole, not a real pole.
        config = apply_task(self.config, goal=[1, -1])
        nav = Navigator(config)
        self.assertEqual(fixed_obstacles(config), [])
        self.assertEqual(nav.route, [(1, -1, "WAYPOINT_1")])
        self.assertNotIn("mission_waypoints", self.config)

    def test_template_cannot_be_armed_or_used_in_wrong_frame(self):
        path = ROOT/"config/waypoints_template.json"
        with self.assertRaisesRegex(ValueError, "confirmed"):
            apply_task(self.config, task_path=path, map_frame="map", require_confirmed=True)
        with self.assertRaisesRegex(ValueError, "map_frame"):
            apply_task(self.config, task_path=path, map_frame="odom")

    def test_bad_measured_goals_are_rejected(self):
        for goal in ([], [0], [math.nan, 0], [math.inf, 0]):
            with self.subTest(goal=goal), self.assertRaises(ValueError):
                apply_task(self.config, goal=goal)
        config = apply_task(self.config, goal=[100, 0])
        with self.assertRaisesRegex(ValueError, "越界"):
            Navigator(config)

    def test_task_requires_explicit_measurements(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"task.json"
            path.write_text('{}', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "missing"):
                apply_task(self.config, task_path=path)

    def test_barrel_between_start_and_goal_is_detoured(self):
        config = apply_task(self.config, goal=[1, 0])
        config["parking"] = [-1, 0]
        config["fixed_obstacles"] = [[0, 0, 0.25]]
        report = inspect_task(config, self.settings)
        self.assertTrue(report["planning_feasible"])
        path = report["segments"][0]["path"]
        self.assertGreater(max(abs(p[1]) for p in path), 0.6)
        nav = Navigator(config)
        for a, b in zip([config["parking"]]+path, path):
            self.assertTrue(nav.planner.clear(a, b, fixed_obstacles(config)))

    def test_multi_goal_runtime_completes_in_order_and_stops(self):
        config = apply_task(self.config, task_path=ROOT/"config/waypoints_template.json", map_frame="map")
        settings = copy.deepcopy(self.settings)
        now = 10.0
        runtime = VehicleRuntime(config, settings, clock=lambda: now)
        x, y = config["parking"]
        runtime.update_pose([x, y, 0], now)
        runtime.update_quality(True, now)
        runtime.update_obstacles([], now)
        runtime.map_ready = True
        self.assertTrue(runtime.start()[0])
        previous_index = 0
        for tick in range(16000):
            now = 10+tick*0.05
            runtime.update_pose([x, y, 0], now)
            runtime.update_quality(True, now)
            runtime.update_obstacles([], now)
            command = runtime.step()
            self.assertIn(runtime.nav.index-previous_index, (0, 1))
            previous_index = runtime.nav.index
            matrix = runtime.c["body_from_command"]
            nx = x+(matrix[0][0]*command[0]+matrix[0][1]*command[1])*0.05
            ny = y+(matrix[1][0]*command[0]+matrix[1][1]*command[1])*0.05
            self.assertTrue(runtime.nav.planner.clear((x, y), (nx, ny), fixed_obstacles(config)))
            x, y = nx, ny
            if runtime.nav.state == "COMPLETE":
                break
        self.assertEqual(runtime.nav.state, "COMPLETE")
        self.assertEqual(command, (0, 0, 0))
        self.assertLessEqual(distance((x, y), config["parking"]), config["waypoint_tolerance"])
        report = runtime.mission_report()
        self.assertEqual(report["waypoints_completed"], len(config["mission_waypoints"]))
        self.assertGreater(report["travelled_m"], 5)
        elapsed = report["elapsed_seconds"]
        now += 20
        self.assertEqual(runtime.mission_report()["elapsed_seconds"], elapsed)

    def test_occupied_goal_cannot_be_skipped(self):
        config = apply_task(self.config, goal=[-1, -2])
        nav = Navigator(config)
        nav.start()
        sample = {"pose": [*config["parking"], 0], "stamp": 1,
                  "obstacle_stamp": 1, "localized": True, "obstacles": [(-1, -2, 0.2)]}
        self.assertEqual(nav.step(sample, 1), (0, 0, 0))
        self.assertEqual(nav.index, 0)
        self.assertEqual(nav.state, "BLOCKED")
        sample["obstacles"] = []
        self.assertNotEqual(nav.step(sample, 1), (0, 0, 0))

    def test_eight_claim_rejected_without_measured_loop(self):
        audit = EightAudit(self.config)
        audit.begin(self.config["parking"], 10)
        for stage in STAGES:
            audit.stage(stage)
        audit.observe(self.config["parking"], 11)
        self.assertFalse(audit.report(11)["verified"])

    def test_eight_route_has_correct_measured_winding(self):
        nav = Navigator(self.config)
        audit = EightAudit(self.config)
        audit.begin(self.config["parking"], 1)
        for index, point in enumerate(nav.route):
            audit.stage(point[2])
            audit.observe(point, 2+index)
        report = audit.report(2+len(nav.route))
        self.assertTrue(report["verified"], report)
        self.assertAlmostEqual(report["lower_turns_ccw"], 1)
        self.assertAlmostEqual(report["upper_turns_ccw"], -1)

    def test_eight_runtime_cannot_claim_complete_without_motion(self):
        now = 10
        runtime = VehicleRuntime(self.config, self.settings, clock=lambda: now)
        runtime.update_pose([*self.config["parking"], 0], now)
        runtime.update_quality(True, now)
        runtime.update_obstacles([], now)
        runtime.map_ready = True
        self.assertTrue(runtime.start()[0])
        runtime.nav.index = len(runtime.nav.route)
        self.assertEqual(runtime.step(), (0, 0, 0))
        self.assertTrue(runtime.latched)
        self.assertEqual(runtime.reason, "ROUTE_NOT_VERIFIED")


if __name__ == "__main__":
    unittest.main(verbosity=2)
