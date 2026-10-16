import json
from pathlib import Path
import struct
import time
import unittest

from feedback import FeedbackParser
from navigation import Navigator, Planner, distance
from navigation_protocol import crc8, crc16
from safety_control import SafetyController
from serial_link import SerialLink
from simulate import simulate

ROOT = Path(__file__).resolve().parent
OFFICIAL_FEEDBACK = bytes.fromhex("A5 0B 00 01 5C 01 00 AA 2C 01 88 77 66 55 44 33 22 11 EF 74")


class FeedbackTests(unittest.TestCase):
    def test_official_sample_every_split(self):
        for split in range(len(OFFICIAL_FEEDBACK)+1):
            parser = FeedbackParser()
            results = parser.feed(OFFICIAL_FEEDBACK[:split]) + parser.feed(OFFICIAL_FEEDBACK[split:])
            self.assertEqual(len(results), 1)
            self.assertEqual(results[0]["stage_remain_time"], 300)
            self.assertEqual(results[0]["sync_time_stamp"], 0x1122334455667788)
            self.assertEqual(results[0]["game_type"], 10)
            self.assertEqual(results[0]["game_progress"], 10)

    def test_noise_corruption_and_sticky_frames(self):
        bad = bytearray(OFFICIAL_FEEDBACK)
        bad[8] ^= 1
        parser = FeedbackParser()
        results = parser.feed(b"noise" + bad + OFFICIAL_FEEDBACK*2)
        self.assertEqual(len(results), 2)
        self.assertGreater(parser.rejected, 0)
        self.assertEqual(len(parser.buffer), 0)

    def test_robot_status_fields(self):
        body = struct.pack("<BBHHHHHB", 7, 2, 300, 400, 10, 100, 80, 5)
        head = struct.pack("<BHB", 0xA5, len(body), 2)
        prefix = head + bytes([crc8(head)]) + b"\x0b\x00" + body
        frame = prefix + struct.pack("<H", crc16(prefix))
        result = FeedbackParser().feed(frame)[0]
        self.assertEqual(result["current_HP"], 300)
        self.assertTrue(result["gimbal_power"])
        self.assertFalse(result["chassis_power"])
        self.assertTrue(result["shooter_power"])

    def test_invalid_header_does_not_grow_buffer(self):
        parser = FeedbackParser()
        parser.feed(b"\xa5\xff\xff\x00\x00"*1000)
        self.assertLess(len(parser.buffer), 5)


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT/"config.json").read_text(encoding="utf-8"))

    def sample(self, now=0):
        return {"pose": [*self.config["parking"], 0], "stamp": now,
                "obstacle_stamp": now, "localized": True, "obstacles": []}

    def test_wait_start_abort_and_stale_sensor(self):
        nav = Navigator(self.config)
        self.assertEqual(nav.step(self.sample(), 0), (0, 0, 0))
        nav.start()
        self.assertNotEqual(nav.step(self.sample(), 0), (0, 0, 0))
        self.assertEqual(nav.step(self.sample(), 1), (0, 0, 0))
        self.assertEqual(nav.state, "WAIT_SENSOR")
        nav.abort()
        nav.start()
        self.assertEqual(nav.step(self.sample(1), 1), (0, 0, 0))

    def test_bad_pose_and_scan(self):
        nav = Navigator(self.config)
        nav.start()
        for key, value in (("pose", [float("nan"),0,0]), ("localized", False),
                           ("obstacles", [[0,0,-1]]), ("obstacle_stamp", -2)):
            sample = self.sample()
            sample[key] = value
            self.assertEqual(nav.step(sample, 0), (0, 0, 0))

    def test_astar_detour_keeps_clearance(self):
        planner = Planner(self.config)
        start, goal, obstacles = (-1.0, 0.0), (1.0, 0.0), [(0,0,0.2)]
        self.assertFalse(planner.clear(start, goal, obstacles))
        path = planner.plan(start, goal, obstacles)
        self.assertTrue(path)
        self.assertGreater(max(abs(p[1]) for p in path), 0.5)
        for a, b in zip([start]+path, path):
            self.assertTrue(planner.clear(a, b, obstacles))

    def test_no_route_when_goal_occupied(self):
        planner = Planner(self.config)
        self.assertEqual(planner.plan((-1,0), (0,0), [(0,0,0.2)]), [])

    def test_complete_three_scenarios(self):
        for scenario in ("normal", "obstacle", "localization_loss"):
            with self.subTest(scenario=scenario):
                route, rows, report = simulate(self.config, scenario)
                self.assertTrue(report["completed"])
                self.assertLessEqual(report["parking_error_m"], self.config["waypoint_tolerance"])
                self.assertGreaterEqual(report["min_obstacle_clearance_m"], self.config["margin"])
                indices = [row[6] for row in rows]
                self.assertTrue(all(0 <= b-a <= 1 for a,b in zip(indices,indices[1:])))
                states = [t["state"] for t in report["transitions"]]
                self.assertLess(states.index("LOWER_CCW"), states.index("UPPER_CW"))
                if scenario == "obstacle":
                    self.assertIn("BLOCKED", states)
                if scenario == "localization_loss":
                    self.assertIn("WAIT_SENSOR", states)

    def test_circle_winding_directions(self):
        nav = Navigator(self.config)
        for label, center_key, sign in (("LOWER_CCW","lower_pole",1), ("UPPER_CW","upper_pole",-1)):
            center = self.config[center_key]
            pts = [p for p in nav.route if p[2] == label]
            angles = [__import__("math").atan2(p[1]-center[1],p[0]-center[0]) for p in pts]
            for a,b in zip(angles,angles[1:]):
                delta = (b-a+__import__("math").pi)%(2*__import__("math").pi)-__import__("math").pi
                self.assertGreater(sign*delta, 0)


class FakePort:
    def __init__(self, **kwargs):
        self.options = kwargs
        self.written = []
        self.closed = False

    def write(self, frame):
        self.written.append(frame)
        return len(frame)

    def read(self, count):
        time.sleep(0.002)
        return b""

    def close(self):
        self.closed = True


class SerialTests(unittest.TestCase):
    def test_fake_port_zero_start_motion_and_shutdown(self):
        controller = SafetyController()
        link = SerialLink("fake", controller, port_factory=FakePort)
        try:
            link.start()
            time.sleep(0.03)
            controller.update(0.05, 0)
            time.sleep(0.04)
        finally:
            link.close()
        self.assertIsNone(link.error)
        self.assertTrue(link.port.closed)
        self.assertEqual(link.port.options["baudrate"], 115200)
        self.assertTrue(any(struct.unpack("<fff", f[5:17])[0] > 0 for f in link.port.written))
        for f in link.port.written[-3:]:
            self.assertEqual(struct.unpack("<fff", f[5:17]), (0,0,0))

    def test_partial_write_is_failure(self):
        link = SerialLink("fake", SafetyController(), port_factory=FakePort)
        link.port.write = lambda f: 1
        try:
            with self.assertRaises(IOError):
                link._write(bytes(19))
        finally:
            link.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
