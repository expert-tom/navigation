import struct
import unittest

from offline_sender import run_loop
from safety_control import SafetyController


class FakeTime:
    def __init__(self):
        self.now = 0.0

    def clock(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class SenderTests(unittest.TestCase):
    def test_period_timeout_and_exit(self):
        timer = FakeTime()
        controller = SafetyController(clock=timer.clock)
        controller.update(0.1, 0)
        records = []
        run_loop(controller, lambda frame: records.append((timer.now, frame)),
                 duration=0.65, clock=timer.clock, sleep=timer.sleep)
        for a, b in zip(records, records[1:]):
            self.assertGreaterEqual(b[0] - a[0], 0.01 - 1e-12)
        for timestamp, frame in records[:-1]:
            vx = struct.unpack("<fff", frame[5:17])[0]
            if timestamp < 0.5:
                self.assertGreater(vx, 0)
            else:
                self.assertEqual(vx, 0)
        self.assertEqual(struct.unpack("<fff", records[-1][1][5:17]), (0, 0, 0))

    def test_slow_sink_does_not_trigger_burst(self):
        timer = FakeTime()
        controller = SafetyController(clock=timer.clock)
        timestamps = []

        def sink(frame):
            timestamps.append(timer.now)
            timer.sleep(0.03)

        run_loop(controller, sink, duration=0.2, clock=timer.clock, sleep=timer.sleep)
        for a, b in zip(timestamps, timestamps[1:]):
            self.assertGreaterEqual(b - a, 0.04 - 1e-12)

    def test_exception_outputs_zero_then_propagates(self):
        timer = FakeTime()
        controller = SafetyController(clock=timer.clock)
        controller.update(0.1, 0)
        frames = []

        def fail(elapsed):
            raise RuntimeError("模拟导航错误")

        with self.assertRaises(RuntimeError):
            run_loop(controller, frames.append, demo_update=fail,
                     clock=timer.clock, sleep=timer.sleep)
        self.assertEqual(len(frames), 1)
        self.assertEqual(struct.unpack("<fff", frames[0][5:17]), (0, 0, 0))

    def test_invalid_parameters(self):
        for key in ("duration", "frequency"):
            for value in (0, -1, float("nan"), float("inf")):
                with self.assertRaises(ValueError):
                    run_loop(SafetyController(), lambda frame: None, **{key: value})


if __name__ == "__main__":
    unittest.main(verbosity=2)
