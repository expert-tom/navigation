import copy
import json
import math
from pathlib import Path
import random
import unittest

from localization_monitor import covariance_valid
from navigation import Planner, distance
from navigation_protocol import crc8, crc16
from real_vehicle import load_settings
from safety_control import SafetyController
from vehicle_core import VehicleRuntime, cloud_obstacles, grid_obstacles, message_stamp, quaternion_matrix, transform_point


ROOT = Path(__file__).resolve().parent


class VehicleTests(unittest.TestCase):
    def setUp(self):
        self.navigation = json.loads((ROOT/"config.json").read_text(encoding="utf-8"))
        self.settings = load_settings(ROOT/"config/real_vehicle.json")
        self.now = 10.0

    def runtime(self):
        runtime = VehicleRuntime(self.navigation, self.settings, clock=lambda: self.now)
        runtime.update_pose([*self.navigation["parking"], 0], self.now)
        runtime.update_quality(True, self.now)
        runtime.update_obstacles([], self.now)
        runtime.map_ready = True
        return runtime

    def test_start_requires_sensor_map_and_localization(self):
        runtime = self.runtime()
        runtime.map_ready = False
        self.assertEqual(runtime.start(), (False, "WAIT_MAP"))
        runtime.map_ready = True
        runtime.update_quality(False, self.now)
        self.assertEqual(runtime.start(), (False, "WAIT_LOCALIZATION_QUALITY"))
        runtime.update_quality(True, self.now)
        self.assertTrue(runtime.start()[0])

    def test_motion_requires_real_measurement_confirmations(self):
        runtime = self.runtime()
        self.assertFalse(runtime.start(armed=True)[0])
        for key in ("field_confirmed", "calibration_confirmed", "lidar_extrinsics_confirmed", "emergency_stop_confirmed"):
            runtime.settings[key] = True
        self.assertTrue(runtime.start(armed=True)[0])

    def test_start_away_from_parking_is_rejected(self):
        runtime = self.runtime()
        self.now += 0.01
        runtime.update_pose([-1.5, -2, 0], self.now)
        self.assertEqual(runtime.start(), (False, "NOT_AT_PARKING"))

    def test_stale_sensor_stops_even_with_fresh_pose(self):
        runtime = self.runtime()
        runtime.start()
        self.now += 0.35
        runtime.update_pose([*self.navigation["parking"], 0], self.now)
        runtime.update_quality(True, self.now)
        self.assertEqual(runtime.step(), (0,0,0))
        self.assertEqual(runtime.reason, "WAIT_OBSTACLES")

    def test_stale_quality_stops_even_with_fresh_sensors(self):
        runtime = self.runtime()
        runtime.start()
        self.now += 0.6
        runtime.update_pose([*self.navigation["parking"], 0], self.now)
        runtime.update_obstacles([], self.now)
        self.assertEqual(runtime.step(), (0,0,0))
        self.assertEqual(runtime.reason, "WAIT_LOCALIZATION_QUALITY")

    def test_pose_jump_latches_stop(self):
        runtime = self.runtime()
        runtime.start()
        runtime.update_pose([0,-2,0], self.now+0.01)
        self.assertTrue(runtime.latched)
        self.assertEqual(runtime.step(), (0,0,0))
        self.assertFalse(runtime.start()[0])

    def test_old_pose_cannot_refresh_freshness(self):
        runtime = self.runtime()
        with self.assertRaises(ValueError):
            runtime.update_pose([0,0,0], self.now-1)
        self.assertEqual(runtime.pose_stamp, self.now)

    def test_abort_cannot_be_restarted(self):
        runtime = self.runtime()
        runtime.start()
        runtime.abort()
        self.assertFalse(runtime.start()[0])
        self.assertEqual(runtime.step(), (0,0,0))

    def test_message_age_preserved_and_future_rejected(self):
        self.assertAlmostEqual(message_stamp(99.8,100,10,0.3), 9.8)
        for stamp in (0,99,100.01,float("nan")):
            with self.assertRaises(ValueError):
                message_stamp(stamp,100,10,0.3)

    def test_full_3d_rotation_for_tilted_lidar(self):
        matrix = quaternion_matrix((0,math.sin(math.pi/4),0,math.cos(math.pi/4)))
        point = transform_point((1,0,0),(2,3,4),matrix)
        for actual, expected in zip(point,(2,3,3)):
            self.assertAlmostEqual(actual,expected)

    def test_voxel_circles_cover_points(self):
        identity = quaternion_matrix((0,0,0,1))
        points = [(1.001,0.049,0.2),(1.099,0.099,0.2)]
        obstacles = cloud_obstacles(points,(0,0,0),identity,(0,0,0),identity,(0,0,0),self.settings["cloud"],[-3,3,-3,3])
        for p in points:
            self.assertTrue(any(distance(p,o) <= o[2] for o in obstacles))

    def test_cloud_overload_does_not_silently_drop_obstacles(self):
        identity = quaternion_matrix((0,0,0,1))
        cloud = dict(self.settings["cloud"],max_obstacles=1)
        with self.assertRaisesRegex(ValueError,"too many"):
            cloud_obstacles([(1,0,0.2),(2,0,0.2)],(0,0,0),identity,(0,0,0),identity,(0,0,0),cloud,[-3,3,-3,3])
        with self.assertRaisesRegex(ValueError,"no valid"):
            cloud_obstacles([(1,0,-1)],(0,0,0),identity,(0,0,0),identity,(0,0,0),cloud,[-3,3,-3,3])

    def test_unknown_map_cells_are_obstacles(self):
        obstacles = grid_obstacles([-1,0,100,0],2,2,1,(0,0),0,50,True,[0,2,0,2],10)
        self.assertEqual(len(obstacles),2)
        self.assertAlmostEqual(obstacles[0][2],1/math.sqrt(2))

    def test_map_must_cover_field(self):
        with self.assertRaisesRegex(ValueError,"cover"):
            grid_obstacles([0]*4,2,2,1,(0,0),0,50,True,[-3,3,-3,3],10)

    def test_covariance_limits_and_unknown_zero(self):
        settings = self.settings["quality_monitor"]
        values = [0.0]*36
        self.assertFalse(covariance_valid(values,settings))
        values[0],values[7],values[35] = 0.01,0.01,0.02
        self.assertTrue(covariance_valid(values,settings))
        values[7] = 1
        self.assertFalse(covariance_valid(values,settings))

    def test_spatial_index_matches_brute_force_geometry(self):
        rng = random.Random(42)
        planner = Planner(self.navigation)
        obstacles = [(rng.uniform(-2,2),rng.uniform(-2,2),rng.uniform(0.01,0.3)) for _ in range(200)]
        padding = self.navigation["robot_radius"]+self.navigation["margin"]
        for _ in range(100):
            a,b = [(rng.uniform(-2.5,2.5),rng.uniform(-2.5,2.5)) for _ in range(2)]
            dx,dy = b[0]-a[0],b[1]-a[1]
            expected = True
            for o in obstacles:
                t = max(0,min(1,((o[0]-a[0])*dx+(o[1]-a[1])*dy)/(dx*dx+dy*dy)))
                if distance((a[0]+t*dx,a[1]+t*dy),o) <= o[2]+padding:
                    expected = False
                    break
            self.assertEqual(planner.clear(a,b,obstacles),expected)

    def test_runtime_three_scenarios_complete_and_encode_frames(self):
        for scenario in ("normal","obstacle","localization_loss"):
            with self.subTest(scenario=scenario):
                self.now = 10.0
                runtime = self.runtime()
                runtime.start()
                x,y = self.navigation["parking"]
                safe = SafetyController(limit=0.05,clock=lambda:self.now)
                for tick in range(16000):
                    self.now = 10+tick*0.05
                    runtime.update_pose([x,y,0],self.now)
                    runtime.update_quality(not(scenario=="localization_loss" and 40<=tick<60),self.now)
                    obstacles = [(*runtime.nav.route[8][:2],0.12)] if scenario=="obstacle" and 10<=tick<100 else []
                    runtime.update_obstacles(obstacles,self.now)
                    command = runtime.step()
                    safe.update(*command)
                    frame = safe.build_frame()
                    self.assertEqual(crc8(frame[:4]),frame[4])
                    self.assertEqual(crc16(frame[:-2]),int.from_bytes(frame[-2:],"little"))
                    matrix = runtime.c["body_from_command"]
                    x += (matrix[0][0]*command[0]+matrix[0][1]*command[1])*0.05
                    y += (matrix[1][0]*command[0]+matrix[1][1]*command[1])*0.05
                    if runtime.nav.state=="COMPLETE":
                        break
                self.assertEqual(runtime.nav.state,"COMPLETE")
                self.assertLessEqual(distance((x,y),self.navigation["parking"]),runtime.c["waypoint_tolerance"])
                self.assertEqual(command,(0,0,0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
