import os
from pathlib import Path
import stat
import struct
import tempfile
import threading
import time
import unittest
from unittest import mock

import check_serial_port
from safety_control import SafetyController
from serial_link import SerialLink, NO_DATA_MARKER, READ_RETRY_PERIOD


FEEDBACK = bytes.fromhex("A5 0B 00 01 5C 01 00 AA 2C 01 88 77 66 55 44 33 22 11 EF 74")


class Port:
    def __init__(self, **options):
        self.options = options
        self.written = []
        self.closed = False

    def write(self, frame):
        self.written.append(frame)
        return len(frame)

    def read(self, size):
        return b""

    def close(self):
        self.closed = True


class SerialGuardTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.controller = SafetyController(clock=lambda: self.now)
        self.link = SerialLink("fake", self.controller, port_factory=Port,
                               clock=lambda: self.now, zero_read_grace=1)

    def tearDown(self):
        self.link.close()

    def test_normal_empty_timeouts_never_start_anomaly_grace(self):
        for _ in range(200):
            self.now += 0.1
            self.link._accept_read(b"")
        self.assertIsNone(self.link.error)
        self.assertFalse(self.link.motion_inhibited)
        self.assertEqual(self.link.stats()["read_timeouts"], 200)

    def test_first_anomaly_immediately_forces_zero_even_after_update(self):
        self.controller.update(0.02, 0)
        self.link._send_once()
        self.assertGreater(struct.unpack("<fff", self.link.port.written[-1][5:17])[0], 0)
        self.link._handle_read_error(OSError(NO_DATA_MARKER))
        self.assertIsNone(self.link.error)  # Only observation is allowed in grace.
        self.controller.update(0.02, 0.03)
        self.link._send_once()
        self.assertEqual(struct.unpack("<fff", self.link.port.written[-1][5:17]), (0, 0, 0))
        with self.assertRaisesRegex(RuntimeError, "锁定"):
            self.link.check(for_motion=True)

    def test_racing_update_inside_stop_cannot_restore_nonzero_frame(self):
        self.link._handle_read_error(OSError(NO_DATA_MARKER))
        original_stop = self.controller.stop
        def racing_stop():
            original_stop()
            self.controller.update(0.05, 0.05)
        with mock.patch.object(self.controller, "stop", side_effect=racing_stop):
            self.link._send_once()
        self.assertEqual(struct.unpack("<fff", self.link.port.written[-1][5:17]), (0, 0, 0))

    def test_valid_feedback_clears_timer_but_never_motion_latch(self):
        self.link._handle_read_error(OSError(NO_DATA_MARKER))
        self.now += 0.2
        self.link._accept_read(FEEDBACK)
        self.now += 5
        self.link._accept_read(b"")
        self.assertIsNone(self.link.error)
        self.assertIsNone(self.link._anomaly_since)
        self.assertTrue(self.link.motion_inhibited)
        self.controller.update(0.03, 0)
        self.link._send_once()
        self.assertEqual(self.link.last_written_command, (0, 0, 0))

    def test_noise_does_not_reset_anomaly_timer(self):
        self.link._handle_read_error(OSError(NO_DATA_MARKER))
        self.now += 0.5
        self.link._accept_read(b"garbage")
        self.now += 0.6
        self.link._accept_read(b"")
        self.assertEqual(self.link.error_kind, "no-data-persistent")

    def test_recovery_after_expired_grace_cannot_erase_fault(self):
        self.link._handle_read_error(OSError(NO_DATA_MARKER))
        self.now += 1.1
        self.link._accept_read(FEEDBACK)
        self.assertEqual(self.link.error_kind, "no-data-persistent")
        with self.assertRaises(RuntimeError):
            self.link.check()

    def test_device_disappearance_has_no_grace(self):
        self.link._device_probe = lambda: False
        self.link._handle_read_error(OSError(NO_DATA_MARKER))
        self.assertEqual(self.link.error_kind, "device-missing-or-replaced")
        self.assertTrue(self.link._stop.is_set())

    def test_other_io_error_is_immediate(self):
        self.link._handle_read_error(OSError("Input/output error"))
        self.assertEqual(self.link.error_kind, "read-error")
        self.assertTrue(self.link._stop.is_set())

    def test_feedback_counter_survives_consuming_queue(self):
        self.link._accept_read(FEEDBACK+FEEDBACK)
        self.link.messages.clear()
        self.assertEqual(self.link.stats()["feedback_messages"], 2)
        self.assertEqual(self.link.stats()["queued_messages"], 0)

    def test_invalid_grace_rejected(self):
        for value in (-1, 6, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                SerialLink("fake", self.controller, port_factory=Port, zero_read_grace=value)

    def test_anomaly_loop_is_throttled(self):
        class BrokenPort(Port):
            def read(self, size):
                raise OSError(NO_DATA_MARKER)
        link = SerialLink("fake", SafetyController(), port_factory=BrokenPort)
        worker = threading.Thread(target=link._rx)
        started = time.monotonic()
        worker.start()
        time.sleep(0.06)
        link._stop.set()
        worker.join(timeout=1)
        elapsed = time.monotonic()-started
        self.assertFalse(worker.is_alive())
        self.assertGreater(link.no_data_errors, 0)
        self.assertLessEqual(link.no_data_errors, elapsed/READ_RETRY_PERIOD+2)
        link.close()

    def test_repeated_start_or_closed_link_is_rejected(self):
        self.link.close()
        with self.assertRaises(RuntimeError):
            self.link.start()
        self.link.close()  # Idempotent cleanup.


class PreflightTests(unittest.TestCase):
    def test_missing_or_regular_file_never_passes_or_opens_device(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"not-a-tty"
            with mock.patch("os.open", side_effect=AssertionError("must not open")):
                self.assertFalse(check_serial_port.preflight(str(path))["clean"])
            path.touch()
            with mock.patch("os.open", side_effect=AssertionError("must not open")):
                self.assertFalse(check_serial_port.preflight(str(path))["clean"])

    def report(self, holders, unreadable):
        info = os.stat_result((stat.S_IFCHR | 0o660, 1, 1, 1, 0, 20, 0, 0, 0, 0))
        with mock.patch.object(check_serial_port.os, "stat", return_value=info), \
             mock.patch.object(check_serial_port.os, "access", return_value=True), \
             mock.patch.object(check_serial_port.os, "name", "posix"), \
             mock.patch.object(check_serial_port, "Path") as path, \
             mock.patch.object(check_serial_port, "find_holders", return_value=(holders, unreadable)), \
             mock.patch("os.open", side_effect=AssertionError("must not open")):
            path.return_value.is_dir.return_value = True
            return check_serial_port.preflight("/dev/ttyACM0")

    def test_holder_blocks_preflight(self):
        report = self.report([{"pid": 42}], 0)
        self.assertFalse(report["clean"])
        self.assertFalse(report["opens_device"])

    def test_permission_gaps_are_not_claimed_as_exclusive_access(self):
        report = self.report([], 7)
        self.assertTrue(report["clean"])
        self.assertEqual(report["unreadable_processes"], 7)
        self.assertFalse(report["exclusive_access_proven"])
        self.assertTrue(any("无权" in warning for warning in report["warnings"]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
