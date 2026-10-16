"""Timed empty reads are normal; exception grace permits ZERO output only.

Motion inhibition is latched until the process is restarted. No auto reconnect.
"""
import math
import os
import struct
import threading
import time
from collections import deque

from feedback import FeedbackParser
from navigation_protocol import pack_command

NO_DATA_MARKER = "device reports readiness to read but returned no data"
READ_RETRY_PERIOD = 0.005


class SerialLink:
    def __init__(self, device, controller, port_factory=None, *, zero_read_grace=1.0,
                 device_probe=None, clock=time.monotonic, motion_permit=None, dtr=False, rts=False):
        if not math.isfinite(zero_read_grace) or not 0 <= zero_read_grace <= 5:
            raise ValueError("zero_read_grace必须为0到5秒的有限数；宽限期仅发送零速")
        self.controller, self.device, self.clock = controller, device, clock
        self.motion_permit = motion_permit
        self.stop_write_attempts = self.stop_write_successes = 0
        self.zero_read_grace = zero_read_grace
        self._guard = threading.RLock()
        self._stop = threading.Event()
        self._threads = []
        self._started = self._closed = False
        self.parser = FeedbackParser()
        self.messages, self.times = deque(maxlen=1000), deque(maxlen=1000)
        self.error = self.error_kind = None
        self.motion_inhibited = False
        self._anomaly_since = self.last_anomaly = None
        self.sent_frames = self.received_bytes = self.feedback_messages = 0
        self.read_timeouts = self.no_data_errors = self.read_errors = 0
        self.last_read_at = self.last_feedback_at = None
        self.last_written_command = self.last_written_at = None
        real_port = port_factory is None
        if real_port:
            try:
                import serial
            except ImportError as exc:
                raise RuntimeError("缺少pySerial；请按项目离线依赖说明安装") from exc
            port_factory = serial.Serial
        options = dict(port=device, baudrate=115200, bytesize=8, parity="N", stopbits=1,
                       timeout=0.02, write_timeout=0.02, xonxoff=False, rtscts=False, dsrdtr=False)
        if os.name == "posix":
            options["exclusive"] = True
        try:
            if real_port:
                # pySerial defaults both lines to True. Set desired states BEFORE
                # open; OS/adapter glitches are still possible and must be tested.
                options["port"] = None
                self.port = port_factory(**options)
                self.port.dtr, self.port.rts = bool(dtr), bool(rts)
                self.port.port = device
                self.port.open()
            else:
                self.port = port_factory(**options)
        except Exception as exc:
            if getattr(self, "port", None) is not None:
                self.port.close()
            raise RuntimeError(f"无法打开串口 {device}: {exc}；请运行 python3 check_serial_port.py --device 路径") from exc
        if device_probe is not None:
            self._device_probe = device_probe
        elif real_port and os.name == "posix":
            try:
                identity = self._identity()
            except OSError:
                self.port.close()
                raise
            self._device_probe = lambda: self._identity() == identity
        else:
            self._device_probe = lambda: True  # Test injection / non-POSIX backend.

    def _identity(self):
        info = os.stat(self.device)
        return info.st_rdev, info.st_ino

    def _device_present(self):
        try:
            return bool(self._device_probe())
        except OSError:
            return False

    def _fault(self, exc, kind):
        with self._guard:
            if self.error is None:
                self.error, self.error_kind = exc, kind
            self.motion_inhibited = True
            self.controller.stop()
            self._stop.set()

    def _handle_read_error(self, exc):
        with self._guard:
            if NO_DATA_MARKER not in str(exc):
                self.read_errors += 1
                self._fault(exc, "read-error")
                return
            self.no_data_errors += 1
            self.motion_inhibited = True
            self.controller.stop()
            self.last_anomaly = str(exc)
            if not self._device_present():
                self._fault(exc, "device-missing-or-replaced")
                return
            if self._anomaly_since is None:
                self._anomaly_since = self.clock()
            self._check_grace()

    def _check_grace(self):
        with self._guard:
            if self._anomaly_since is not None and self.clock()-self._anomaly_since >= self.zero_read_grace:
                self._fault(RuntimeError(self.last_anomaly), "no-data-persistent")

    def _accept_read(self, data):
        with self._guard:
            self._check_grace()
            if self.error is not None:
                return
            if not data:
                self.read_timeouts += 1
                self._check_grace()
                return
            self.received_bytes += len(data)
            self.last_read_at = self.clock()
            parsed = self.parser.feed(data)
            self.messages.extend(parsed)
            self.feedback_messages += len(parsed)
            if parsed:
                self.last_feedback_at = self.clock()
                self._anomaly_since = None
                # Recovery NEVER releases motion_inhibited.
            self._check_grace()

    def start(self):
        with self._guard:
            if self._started or self._closed or self.motion_inhibited:
                raise RuntimeError("串口链路不能重复启动；故障后需重新创建进程")
            self._started = True
            self.controller.stop()
            self._threads = [threading.Thread(target=self._tx, name="serial-tx", daemon=True),
                             threading.Thread(target=self._rx, name="serial-rx", daemon=True)]
        for thread in self._threads:
            thread.start()

    def _write(self, frame):
        if self.port.write(frame) != len(frame):
            raise IOError("串口仅写入部分帧，停止运行")
        self.sent_frames += 1
        self.last_written_command = struct.unpack("<fff", frame[5:17])
        self.last_written_at = self.clock()

    def _send_once(self):
        # Latch and construction/write share a lock. Concurrent controller.update
        # cannot resume nonzero frames after the read thread has inhibited motion.
        with self._guard:
            permitted = True
            if self.motion_permit is not None:
                try:
                    permitted = self.motion_permit() is True
                except Exception:
                    permitted = False
            if self.motion_inhibited or self._stop.is_set() or not permitted:
                self.controller.stop()
                # Independent frame: controller.update may race with stop().
                frame = pack_command(0.0, 0.0, 0.0, self.sent_frames % 256)
            else:
                frame = self.controller.build_frame()
            self._write(frame)
            self.times.append(self.clock())

    def _tx(self):
        period = 0.01
        deadline = self.clock()
        try:
            while not self._stop.is_set():
                remaining = deadline-self.clock()
                if remaining > 0:
                    self._stop.wait(remaining)
                    continue
                self._send_once()
                deadline = max(deadline+period, self.clock()+0.003)
        except Exception as exc:
            self._fault(exc, "write-error")
        finally:
            self.controller.stop()
            self._shutdown_zeros(period)

    def _shutdown_zeros(self, period=0.01):
        # Bounded best effort. A failed write must NOT cancel later attempts.
        # A disconnected link can still make all attempts fail: no physical
        # stop guarantee without a downstream watchdog / independent e-stop.
        for seq in range(20):
            time.sleep(period)
            self.stop_write_attempts += 1
            try:
                with self._guard:
                    self._write(pack_command(0.0, 0.0, 0.0, seq))
                    self.stop_write_successes += 1
            except Exception as exc:
                self._fault(exc, "shutdown-write-error")

    def _rx(self):
        while not self._stop.is_set():
            if not self._device_present():
                self._fault(RuntimeError("设备节点消失或已被替换"), "device-missing-or-replaced")
                break
            try:
                self._accept_read(self.port.read(256))
            except Exception as exc:
                self._handle_read_error(exc)
            self._stop.wait(READ_RETRY_PERIOD)

    def close(self):
        with self._guard:
            if self._closed:
                return
            self._closed = True
            self.motion_inhibited = True
            self._stop.set()
            self.controller.stop()
        for thread in self._threads:
            thread.join(timeout=1)
        self.port.close()

    def check(self, for_motion=False):
        with self._guard:
            if self.error is not None:
                raise RuntimeError(f"串口故障，禁止继续运动 [{self.error_kind}]: {self.error}") from self.error
            if for_motion and self.motion_inhibited:
                raise RuntimeError("串口读取异常，运动已锁定；观察恢复后也必须重新启动任务")

    def stats(self):
        with self._guard:
            now = self.clock()
            send_times=list(self.times)
            hz=None
            if len(send_times)>1 and send_times[-1]>send_times[0]:
                hz=(len(send_times)-1)/(send_times[-1]-send_times[0])
            return {"sent_frames": self.sent_frames, "received_bytes": self.received_bytes,
                    "normal_tx_hz_recent": hz,
                    "stop_write_attempts": self.stop_write_attempts,
                    "stop_write_successes": self.stop_write_successes,
                    "feedback_messages": self.feedback_messages, "queued_messages": len(self.messages),
                    "rejected_frames": self.parser.rejected, "unparsed_bytes": len(self.parser.buffer),
                    "read_timeouts": self.read_timeouts, "no_data_errors": self.no_data_errors,
                    "read_errors": self.read_errors, "motion_inhibited": self.motion_inhibited,
                    "read_anomaly_latched": self.last_anomaly is not None,
                    "fault": self.error_kind, "last_anomaly": self.last_anomaly,
                    "last_read_age_s": None if self.last_read_at is None else now-self.last_read_at,
                    "last_feedback_age_s": None if self.last_feedback_at is None else now-self.last_feedback_at,
                    "last_written_command": self.last_written_command,
                    "last_written_age_s": None if self.last_written_at is None else now-self.last_written_at,
                    "closed": self._closed, "execution_acknowledged": False}
