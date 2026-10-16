"""串口发送线程与反馈接收线程；不自动重连后恢复运动。

关于“device reports readiness to read but returned no data”：
pySerial 的 posix 实现在 read 前用 select 等待可读，若 select 报告可读而
os.read 返回 0 字节，它就抛 SerialException。该现象只有两种来源：
端口已被别的进程占用（多重访问），或适配器/USB 链路已经掉线。
一次或少数几次这种返回不代表链路已坏，因此这里先做短暂宽限并统计，
只有在宽限期内持续出现、或设备节点消失、或出现真正的 I/O 错误时，
才判定为串口故障并锁定停止。
"""
import os
import sys
import threading
import time
from collections import deque

from feedback import FeedbackParser
from navigation_protocol import pack_command, next_seq

try:
    import serial as _serial_module
except ImportError:
    _serial_module = None

NO_DATA_MARKER = "returned no data"
ZERO_READ_GRACE_SECONDS = 1.0
READ_RETRY_PERIOD = 0.005

if _serial_module is not None:
    _SerialExceptionTypes = (_serial_module.SerialException, _serial_module.SerialTimeoutException)
else:
    _SerialExceptionTypes = ()


def device_exists(path):
    """设备节点是否存在。"""
    try:
        return os.path.exists(path)
    except (OSError, ValueError):
        return False


def classify_serial_exception(exc):
    """区分'可观察的瞬时症状'与'必须立刻停车'的硬故障。"""
    text = str(exc)
    if NO_DATA_MARKER in text:
        return "no-data"
    if isinstance(exc, (OSError, IOError)) and not isinstance(exc, _SerialExceptionTypes):
        return "io"
    return "hard"


def request_size(port):
    """select 自报可读但读到0字节时，重试一次只取1字节，避免死循环放大。"""
    waiting = getattr(port, "in_waiting", None)
    if callable(waiting):
        try:
            return 1 if waiting() == 0 else 256
        except Exception:
            return 256
    return 256


class SerialLink:
    """一个串口只允许一个进程；本类不做自动重连。

    port_factory 仅用于测试注入；device_root 用于把 /dev/serial/by-id 的
    符号链接映射到真实设备节点，从而判断“设备已消失”还是“设备还在但读不到数据”。
    """

    def __init__(self, device, controller, port_factory=None, *,
                 zero_read_grace=ZERO_READ_GRACE_SECONDS, device_root=None,
                 exclusive=True):
        self.controller = controller
        self.device = device
        self.device_root = device_root
        self.zero_read_grace = float(zero_read_grace)
        if not 0 <= self.zero_read_grace <= 30:
            raise ValueError("zero_read_grace必须是0到30秒之间的有限数")
        if port_factory is None:
            if _serial_module is None:
                raise RuntimeError("串口功能需要：python -m pip install -r requirements.txt")
            port_factory = _serial_module.Serial
        options = dict(port=device, baudrate=115200, bytesize=8, parity="N", stopbits=1,
                       timeout=0.02, write_timeout=0.02, xonxoff=False, rtscts=False, dsrdtr=False)
        if exclusive and os.name == "posix":
            options["exclusive"] = True
        self.port = self._open(port_factory, options)
        self.parser = FeedbackParser()
        self.messages = deque(maxlen=1000)
        self.times = deque(maxlen=1000)
        self.error = None
        self.error_kind = None
        self.fault_time = None
        self.sent_frames = 0
        self.zero_reads = 0
        self.no_data_errors = 0
        self.read_errors = 0
        self.feedback_received = 0
        self.last_read_at = None
        self._grace_started_at = None
        self._stop = threading.Event()
        self._threads = []
        self._closed = False
        self._stop_lock = threading.Lock()
        self._stop_frames_sent = False
        self.device_probe = device if device_root is None else os.path.join(
            device_root, "serial", "by-id", os.path.basename(os.path.normpath(device)))

    def _open(self, port_factory, options):
        try:
            return port_factory(**options)
        except Exception as exc:
            raise RuntimeError(self.describe_open_failure(exc)) from exc

    def describe_open_failure(self, exc):
        text = str(exc)
        lines = ["无法打开串口 %s：%s" % (self.device, text)]
        if isinstance(exc, PermissionError) or "Permission denied" in text:
            lines.append("权限不足：把当前用户加入 dialout 组（sudo usermod -aG dialout $USER）后重新登录。")
        elif "exclusively lock" in text or "Device or resource busy" in text:
            lines.append("端口已被占用：lsof %s；fuser -v %s" % (self.device, self.device))
        return "\n".join(lines)

    def _fault(self, exc, note):
        if self.error is None:
            self.error = exc if exc is not None else RuntimeError(note)
            self.error_kind = note
            self.fault_time = time.monotonic()
        self.controller.stop()
        self._stop.set()

    def _device_present(self):
        if device_exists(self.device):
            return True
        return self.device_root is None and device_exists(self.device_probe)

    def _registration_name(self):
        """检查设备是否被 brltty 等内核驱动抢先占用。"""
        root = "/sys/class"
        if not sys.platform.startswith("linux"):
            return None
        try:
            entries = os.listdir(root)
        except OSError:
            return None
        for entry in entries:
            if "tty" not in entry:
                continue
            path = os.path.join(root, entry, "device", "name")
            try:
                with open(path, "r", encoding="utf-8", errors="replace") as handle:
                    if self.device in handle.read():
                        return entry
            except OSError:
                continue
        return None

    def _on_transient(self, exc, note):
        """可观察症状：设备还在就继续运行，设备已消失就立刻停车。"""
        self.no_data_errors += 1
        if not self._device_present():
            self._fault(exc, "fault: device-missing: " + note)
            return False
        return self._inside_grace(note, exc)

    def _inside_grace(self, note, exc=None):
        """宽限窗口：持续出现同一个症状超过zero_read_grace秒才判故障。"""
        now = time.monotonic()
        if self._grace_started_at is None:
            self._grace_started_at = now
        if now - self._grace_started_at < self.zero_read_grace:
            return True
        self._fault(exc, "fault: no-data: %s 连续%.1f秒自报可读却返回0字节"
                    % (note, self.zero_read_grace))
        return False

    def _on_zero_read(self, note, now):
        """自报可读却读到0字节：先宽限观察，持续出现才判故障。"""
        self.zero_reads += 1
        if not self._device_present():
            self._fault(None, "fault: device-missing: " + note)
            return False
        return self._inside_grace(note)

    def start(self):
        self.controller.stop()
        self._stop.clear()
        self._threads = [threading.Thread(target=self._tx, name="serial-tx", daemon=True),
                         threading.Thread(target=self._rx, name="serial-rx", daemon=True)]
        for thread in self._threads:
            thread.start()

    def _write(self, frame):
        if self.port.write(frame) != len(frame):
            raise IOError("串口仅写入部分帧，停止运行")
        self.sent_frames += 1

    STOP_FRAMES = 8
    STOP_PERIOD = 0.02

    def send_stop_frames(self, count=None, period=None):
        """补发零速帧，返回成功帧数；同一进程内只真正执行一次。

        与发送线程解耦：写异常导致发送线程提前退出时，退出路径仍会再试一次。
        """
        with self._stop_lock:
            if self._stop_frames_sent:
                return 0
            count = self.STOP_FRAMES if count is None else count
            period = self.STOP_PERIOD if period is None else period
            sent = 0
            seq = 0
            for _ in range(count):
                time.sleep(period)
                try:
                    self._write(pack_command(0.0, 0.0, 0.0, seq))
                except Exception as exc:
                    if self.error is None:
                        self.error = exc
                        self.error_kind = "fault: tx-error"
                    break
                seq = next_seq(seq)
                sent += 1
            self._stop_frames_sent = True
            return sent

    def _tx(self):
        period = 0.01
        deadline = time.monotonic()
        try:
            while not self._stop.is_set():
                remaining = deadline-time.monotonic()
                if remaining > 0:
                    self._stop.wait(remaining)
                    continue
                self._write(self.controller.build_frame())
                self.times.append(time.monotonic())
                deadline = max(deadline+period, time.monotonic()+0.003)
        except Exception as exc:
            self.error = exc
            self.error_kind = "fault: tx-error"
            self._stop.set()
        finally:
            self.controller.stop()
            self.send_stop_frames()

    def _fail(self, exc, note):
        """读线程遇到无法归类的异常：记录并锁定停止，不重连。"""
        if self.error is None:
            self.error = exc
            self.error_kind = note
            self.fault_time = time.monotonic()
        self.controller.stop()
        self._stop.set()

    def _rx(self):
        try:
            while not self._stop.is_set():
                try:
                    data = self.port.read(request_size(self.port))
                except Exception as exc:
                    kind = classify_serial_exception(exc)
                    if kind == "hard":
                        self.read_errors += 1
                        self._fault(exc, "fault: read-error: " + str(exc))
                        continue
                    if not self._on_transient(exc, str(exc)):
                        continue
                    self._stop.wait(READ_RETRY_PERIOD)
                    continue
                now = time.monotonic()
                if data:
                    parsed = self.parser.feed(data)
                    self.messages.extend(parsed)
                    self.feedback_received += len(parsed)
                    self.last_read_at = now
                    self._grace_started_at = None
                else:
                    self._on_zero_read("read()返回空", now)
        except Exception as exc:
            self._fail(exc, "fault: rx-error")

    def close(self):
        """可重复调用，并确保退出前补发零速帧、端口一定被关闭。"""
        self._closed = True
        self._stop.set()
        self.controller.stop()
        for thread in self._threads:
            thread.join(timeout=2)
        self.send_stop_frames()
        self._close_port()

    def _close_port(self):
        """幂等关闭端口；关闭失败不抛异常，避免打断调用方的finally。"""
        port, self.port = self.port, None
        if port is None:
            return
        try:
            port.close()
        except Exception as exc:
            self.read_errors += 1
            if self.error is None:
                self.error = exc
                self.error_kind = "fault: close-error"

    def check(self):
        if self.error is not None:
            raise RuntimeError(self.fault_message()) from self.error

    def fault_message(self):
        kind = self.error_kind or ""
        lines = ["串口故障，禁止继续运动：%s" % self.error]
        if self.zero_reads or self.no_data_errors or self.read_errors:
            lines.append("统计：读空 %d 次、'可读却无数据'异常 %d 次、其他读异常 %d 次、已发送 %d 帧"
                         % (self.zero_reads, self.no_data_errors, self.read_errors,
                            self.sent_frames))
        if "no-data" in kind:
            lines.append(self.registration_name_hint())
        elif "device-missing" in kind:
            lines.append("设备节点已消失：适配器可能被拔出或USB供电瞬断（dmesg | tail 查看 USB disconnect）。")
        elif "read-error" in kind or "tx-error" in kind:
            lines.append("端口可能被别的进程占用：lsof %s；fuser -v %s" % (self.device, self.device))
        lines.append("排查后再重新运行；本程序不会自动重连，也不会在故障后恢复运动。")
        return "\n".join(lines)

    def registration_name_hint(self):
        name = self._registration_name()
        if name:
            return ("注意：设备已被内核驱动 %s 抢占（常见为 brltty），"
                    "请先停掉该驱动再打开串口。" % name)
        return "先确认没有第二个进程占用同一个串口，再检查USB线缆、转接板供电和车上下位机是否上电。"

    def stats(self):
        return {"sent_frames": self.sent_frames,
                "feedback_messages": self.feedback_received,
                "pending_messages": len(self.messages),
                "unparsed_bytes": len(self.parser.buffer),
                "rejected_frames": self.parser.rejected,
                "zero_reads": self.zero_reads,
                "no_data_errors": self.no_data_errors,
                "read_errors": self.read_errors,
                "last_read_age_s": (None if self.last_read_at is None
                                    else round(time.monotonic()-self.last_read_at, 3)),
                "healthy": self.error is None}
