import argparse
from collections import deque
import json
import os
from pathlib import Path
from types import SimpleNamespace
import struct
import tempfile
import time
import unittest
from unittest import mock

import check_serial_port
import calibrate
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
        self.assertFalse(result["chassis_enabled"])
        self.assertTrue(result["any_power"])

    def test_sweep_summary_flags_disabled_chassis(self):
        from main import SerialRunner
        body = struct.pack("<BBHHHHHB", 7, 2, 300, 400, 10, 100, 80, 0)
        head = struct.pack("<BHB", 0xA5, len(body), 2)
        prefix = head + bytes([crc8(head)]) + b"\x0b\x00" + body
        message = FeedbackParser().feed(prefix + struct.pack("<H", crc16(prefix)))[0]
        runner = SerialRunner(argparse.Namespace(sweep=True, k=0.05))
        runner.collect(SimpleNamespace(messages=deque([message])))
        self.assertFalse(runner.status[1])
        self.assertEqual(runner.status_changes, 1)
        self.assertFalse(message["chassis_enabled"])

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


class BrokenPort(FakePort):
    """模拟pySerial在select自报可读但读到0字节时抛出的异常。"""

    def read(self, count):
        time.sleep(0.002)
        raise OSError("device reports readiness to read but returned no data "
                      "(device disconnected or multiple access on port?)")


class VanishingPort(BrokenPort):
    """读到0字节后设备节点消失，模拟适配器被拔出。"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.path = kwargs.get("port")

    def read(self, count):
        if self.path:
            try:
                os.remove(self.path)
            except OSError:
                pass
        return super().read(count)


class RecoveringPort(FakePort):
    """多数读为空，偶尔读到一帧完整反馈。"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.calls = 0

    def read(self, count):
        self.calls += 1
        if self.calls % 5 == 0:
            return OFFICIAL_FEEDBACK
        time.sleep(0.001)
        return b""


class SerialTests(unittest.TestCase):
    """所有用例都使用真实存在的临时设备路径。

    link内部会把“设备节点消失”当作掉线立即判故障，因此路径必须真实存在；
    只有设备还在、却读不到数据的场景才走宽限逻辑。
    """

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.device = os.path.join(self._tmp.name, "fake-tty")
        Path(self.device).write_text("")
        self._links = []

    def make_link(self, port_factory, **kwargs):
        """登记每一个link：一个进程反复打开同一个tty，正是multi-access的来源。"""
        link = SerialLink(self.device, kwargs.pop("controller", SafetyController()),
                          port_factory=port_factory, **kwargs)
        self._links.append(link)
        return link

    def tearDown(self):
        for link in self._links:
            link.close()
        self._tmp.cleanup()

    def test_close_is_idempotent_and_leaves_no_open_port(self):
        """close()可重复调用；否则解除占用失败会让下位机一直保留旧指令。"""
        link = self.make_link(FakePort)
        link.start()
        time.sleep(0.02)
        link.close()
        self.assertTrue(link.port is None)
        self.assertIsNone(link.port)
        link.close()

    def test_stop_frames_are_idempotent(self):
        """发送线程的finally与close()都会补零速，但只允许真正执行一次。

        旧实现里写异常会break掉补发循环，可能一帧零速都发不出去，
        而下位机没有超时保护 —— 车会保持最后一条运动指令。
        """
        link = self.make_link(FakePort)
        port = link.port
        link.send_stop_frames(count=2, period=0.001)
        self.assertEqual(link.sent_frames, 2)
        link.send_stop_frames(count=2, period=0.001)
        self.assertEqual(link.sent_frames, 2)

    def test_close_always_attempts_stop_frames_even_if_tx_never_ran(self):
        """未调用start()时close()也必须补零速：异常路径不能漏掉这一步。"""
        link = self.make_link(FakePort)
        port = link.port
        started = time.monotonic()
        link.close()
        self.assertGreaterEqual(time.monotonic() - started, link.STOP_PERIOD)
        self.assertEqual(len(port.written), link.STOP_FRAMES)
        for frame in port.written:
            self.assertEqual(struct.unpack("<fff", frame[5:17]), (0.0, 0.0, 0.0))

    def test_close_failure_still_releases_the_port(self):
        class StubbornPort(FakePort):
            def close(self):
                self.closed = True
                raise OSError("close failed")

        link = self.make_link(StubbornPort)
        link.close()
        self.assertIsNone(link.port)

    def test_read_error_still_reports_device_diagnostics(self):
        """读异常后仍要继续判断设备是否还在/是否被占用，否则只能盲目重试。"""
        shown = []
        with tempfile.TemporaryDirectory() as tmp:
            device = os.path.join(tmp, "tty")
            Path(device).write_text("")
            link = SerialLink(device, SafetyController(), port_factory=BrokenPort,
                              zero_read_grace=0.02)
            original = link.registration_name_hint
            link.registration_name_hint = lambda: shown.append(1) or "hint"
            try:
                link.start()
                time.sleep(0.25)
            finally:
                link.close()
        self.assertIn("串口故障", link.fault_message())
        self.assertTrue(link.error is not None)

    def test_fake_port_zero_start_motion_and_shutdown(self):
        controller = SafetyController()
        link = self.make_link(FakePort, controller=controller)
        port = link.port
        try:
            link.start()
            time.sleep(0.03)
            controller.update(0.05, 0)
            time.sleep(0.04)
        finally:
            link.close()
        self.assertIsNone(link.error)
        self.assertTrue(port.closed)
        self.assertIsNone(link.port)
        self.assertEqual(port.options["baudrate"], 115200)
        self.assertTrue(any(struct.unpack("<fff", f[5:17])[0] > 0 for f in port.written))
        for f in port.written[-3:]:
            self.assertEqual(struct.unpack("<fff", f[5:17]), (0,0,0))

    def test_partial_write_is_failure(self):
        link = self.make_link(FakePort)
        link.port.write = lambda f: 1
        try:
            with self.assertRaises(IOError):
                link._write(bytes(19))
        finally:
            link.close()

    def test_repeated_empty_read_is_not_an_instant_fault(self):
        """这正是实车调试遇到的异常：短时间读空不该立刻判定串口故障。"""
        link = self.make_link(BrokenPort, zero_read_grace=5)
        try:
            link.start()
            time.sleep(0.05)
            self.assertIsNone(link.error)
            self.assertGreater(link.no_data_errors, 1)
            self.assertGreater(link.stats()["sent_frames"], 0)
        finally:
            link.close()

    def test_sustained_empty_read_latches_fault_and_stops_motion(self):
        controller = SafetyController()
        link = self.make_link(BrokenPort, controller=controller, zero_read_grace=0.05)
        try:
            link.start()
            time.sleep(0.3)
            self.assertIsNotNone(link.error)
            self.assertIn("no-data", link.error_kind)
            self.assertEqual(controller.output().vx, 0.0)
            self.assertEqual(controller.output().reason, "stopped")
            with self.assertRaises(RuntimeError):
                link.check()
            self.assertIn("串口故障", link.fault_message())
            sent_at_fault = link.stats()["sent_frames"]
            time.sleep(0.3)
            self.assertLessEqual(link.stats()["sent_frames"] - 3, sent_at_fault)
        finally:
            link.close()

    def test_missing_device_is_faulted_without_grace(self):
        link = self.make_link(VanishingPort, zero_read_grace=5)
        try:
            link.start()
            time.sleep(0.1)
            self.assertIsNotNone(link.error)
            self.assertIn("device-missing", link.error_kind)
            self.assertIn("设备节点已消失", link.fault_message())
        finally:
            link.close()

    def test_zero_read_gives_feedback_time_to_recover(self):
        """只要随后真的读到数据，宽限计数就清零，不会累计成故障。"""
        link = self.make_link(RecoveringPort, zero_read_grace=0.3)
        try:
            link.start()
            time.sleep(0.2)
            self.assertIsNone(link.error)
            self.assertGreater(link.stats()["feedback_messages"], 0)
        finally:
            link.close()


class PreflightTests(unittest.TestCase):
    def test_busy_port_is_reported_before_any_command(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "fake-tty")
            Path(path).write_text("")
            with mock.patch.object(check_serial_port, "o_direct_open",
                                   staticmethod(lambda *a, **k: {"ok": False,
                                                                 "error": "Device or resource busy",
                                                                 "hint": "lsof"})), \
                 mock.patch.object(check_serial_port, "find_hold_pids", return_value={}):
                report = check_serial_port.preflight(path, probe_env=False)
            self.assertFalse(report["clean"])
            self.assertTrue(any("独占打开失败" in item for item in report["problems"]))

    def test_missing_device_is_reported(self):
        report = check_serial_port.preflight(os.path.join(tempfile.gettempdir(),
                                                          "dsh-no-such-tty"))
        self.assertFalse(report["clean"])
        self.assertTrue(any("设备节点不存在" in item for item in report["problems"]))

    def test_available_port_is_clean(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "fake-tty")
            Path(path).write_text("")
            with mock.patch.object(check_serial_port, "o_direct_open",
                                   staticmethod(lambda *a, **k: {"ok": True, "flock": "ok"})), \
                 mock.patch.object(check_serial_port, "find_hold_pids", return_value={}):
                report = check_serial_port.preflight(path, probe_env=False)
            self.assertTrue(report["clean"])
            self.assertEqual(report["problems"], [])


class CalibrateTests(unittest.TestCase):
    """保底方案的几何校验：实测值经它把关后才允许写进配置。"""

    def setUp(self):
        self.config = json.loads((ROOT/"config.json").read_text(encoding="utf-8"))

    def geometry(self, spacing=1.6, base_diameter=0.36, angle=90.0, margin=0.1,
                 robot_radius=None):
        radius = self.config["robot_radius"] if robot_radius is None else robot_radius
        return calibrate.build((0.0, 0.0), spacing, angle, base_diameter/2,
                               radius, margin, [-3.0, 3.0, -3.0, 3.0])

    def test_plausible_measurement_passes_and_route_builds(self):
        geo = self.geometry()
        problems, _warnings, facts = calibrate.verify(geo, self.config)
        self.assertEqual(problems, [])
        _route, order = calibrate.dry_run(geo, self.config)
        self.assertEqual(order, ["DEPART", "LOWER_CCW", "UPPER_CW", "LOWER_EXIT", "RETURN"])
        self.assertAlmostEqual(facts["min_safe_radius_m"], 0.57, places=6)
        self.assertAlmostEqual(facts["spare_m"], 0.23, places=6)

    def test_pole_centres_are_symmetric_about_the_axis(self):
        geo = self.geometry(spacing=2.0, angle=90.0)
        low, high = geo["lower_pole"], geo["upper_pole"]
        self.assertAlmostEqual(low[0], 1.0, places=6)
        self.assertAlmostEqual(high[0], -1.0, places=6)
        self.assertAlmostEqual(low[1], high[1], places=6)
        self.assertAlmostEqual(distance(low, high), 2.0, places=6)

    def test_spacing_too_small_is_rejected(self):
        """弧半径必须大于 底座半径+车体半径+margin，否则make_route直接抛错。"""
        geo = self.geometry(spacing=1.0, base_diameter=0.36)
        problems, _warnings, facts = calibrate.verify(geo, self.config)
        self.assertTrue(any("间距" in item for item in problems))
        self.assertLess(facts["spare_m"], 0)
        route, detail = calibrate.dry_run(geo, self.config)
        self.assertIsNone(route)
        self.assertIn("make_route() 失败", detail)
        self.assertIn("间距不足", detail)

    def test_robot_radius_below_model_is_rejected(self):
        """车体半径小于随车xacro推荐值时，规划器会把车体当空气。"""
        geo = self.geometry(spacing=2.0, robot_radius=0.2)
        problems, _warnings, _facts = calibrate.verify(geo, self.config)
        self.assertTrue(any("robot_radius" in item for item in problems))

    def test_thin_pole_measurement_warns_about_the_dome(self):
        """按细杆(4cm)测量时，必须警告真实的碰撞体是更粗的半球底座。"""
        geo = self.geometry(spacing=2.0, base_diameter=0.04)
        _problems, warnings, _facts = calibrate.verify(geo, self.config)
        self.assertTrue(any("半球底座" in item for item in warnings))

    def test_tight_margin_warns_but_still_builds(self):
        """余量在0到TIGHT_MARGIN之间：只警告，不拒绝——实车可试但风险自负。"""
        geo = self.geometry(spacing=1.18, base_diameter=0.36)
        problems, warnings, facts = calibrate.verify(geo, self.config)
        self.assertEqual(problems, [])
        self.assertGreater(facts["spare_m"], 0)
        self.assertLess(facts["spare_m"], calibrate.TIGHT_MARGIN)
        self.assertTrue(any("余量" in item for item in warnings))
        route, order = calibrate.dry_run(geo, self.config)
        self.assertIsNotNone(route)
        self.assertEqual(order, ["DEPART", "LOWER_CCW", "UPPER_CW", "LOWER_EXIT", "RETURN"])

    def test_points_outside_bounds_are_rejected(self):
        geo = calibrate.build((0.0, 0.0), 8.0, 90.0, 0.18, 0.29, 0.1,
                              [-1.0, 1.0, -1.0, 1.0])
        problems, _warnings, _facts = calibrate.verify(geo, self.config)
        self.assertTrue(any("bounds 之外" in item for item in problems))


if __name__ == "__main__":
    unittest.main(verbosity=2)
