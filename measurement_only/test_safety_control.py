import struct
import unittest

from navigation_protocol import crc8, crc16
from safety_control import SafetyController


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.controller = SafetyController(clock=lambda: self.now)

    def values(self):
        return struct.unpack("<fff", self.controller.build_frame()[5:17])

    def test_startup_and_stop_are_zero(self):
        self.assertEqual(self.values(), (0, 0, 0))
        self.controller.update(0.1, 0.1)
        self.controller.stop()
        self.assertEqual(self.values(), (0, 0, 0))

    def test_clamp_both_axes_and_disable_rotation(self):
        self.controller.update(1, -1, 3)
        x, y, z = self.values()
        self.assertAlmostEqual(x, 0.2)
        self.assertAlmostEqual(y, -0.2)
        self.assertEqual(z, 0)

    def test_invalid_update_clears_previous_motion(self):
        for bad in (float("nan"), float("inf"), float("-inf")):
            for axis in range(3):
                self.controller.update(0.1, 0.1)
                values = [0.1, 0.1, 0.0]
                values[axis] = bad
                self.controller.update(*values)
                self.assertEqual(self.values(), (0, 0, 0))
                self.assertEqual(self.controller.output().reason, "invalid")

    def test_timeout_boundary_and_sending_does_not_refresh(self):
        self.controller.update(0.1, 0)
        for tick in range(50):
            self.now = tick / 100
            self.assertGreater(self.values()[0], 0)
        self.now = 0.5
        self.assertEqual(self.values(), (0, 0, 0))
        self.assertEqual(self.controller.output().reason, "timeout")
        self.now = 10
        self.assertEqual(self.values(), (0, 0, 0))

    def test_new_command_recovers_after_timeout(self):
        self.controller.update(0.1, 0)
        self.now = 0.6
        self.assertEqual(self.values(), (0, 0, 0))
        self.controller.update(-0.1, 0)
        self.assertAlmostEqual(self.values()[0], -0.1)

    def test_frames_have_valid_checksums_and_sequence_wrap(self):
        for index in range(257):
            frame = self.controller.build_frame()
            self.assertEqual(len(frame), 19)
            self.assertEqual(frame[3], (index + 1) % 256)
            self.assertEqual(frame[4], crc8(frame[:4]))
            self.assertEqual(int.from_bytes(frame[-2:], "little"), crc16(frame[:-2]))

    def test_bad_configuration(self):
        for limit in (0, -1, 1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                SafetyController(limit=limit)
        for timeout in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                SafetyController(timeout=timeout)


if __name__ == "__main__":
    unittest.main(verbosity=2)
