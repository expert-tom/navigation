"""导航控制保护层；只生成帧，不操作串口。"""

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable

from navigation_protocol import next_seq, pack_command


@dataclass(frozen=True)
class ControlOutput:
    vx: float
    vy: float
    wz: float
    reason: str


class SafetyController:
    """导航算法调用update，发送侧调用output或build_frame。

    只有update刷新控制时间；读取、打包不会刷新，避免旧指令一直有效。
    每轴独立限幅不是总速度限制。wz目前强制为0。
    默认0.2是协议建议的起步量级，不是经过实车验证的安全速度。
    """

    def __init__(self, limit: float = 0.2, timeout: float = 0.5,
                 clock: Callable[[], float] = time.monotonic, *, commissioned_limit=0.2):
        if not math.isfinite(commissioned_limit) or not 0 < commissioned_limit <= 1:
            raise ValueError("commissioned_limit必须为已验收的控制量上限，范围(0,1]")
        if not math.isfinite(limit) or not 0 < limit <= commissioned_limit:
            raise ValueError("limit超过已验收上限；默认上限0.2")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout必须是有限正数，单位秒")
        self.limit = limit
        self.timeout = timeout
        self._clock = clock
        self._lock = threading.Lock()
        self._command = ControlOutput(0.0, 0.0, 0.0, "startup")
        self._updated_at = None
        self._seq = 1

    def update(self, vx: float, vy: float, wz: float = 0.0) -> None:
        """提交新控制量；任何轴非有限时，整条指令归零。"""
        with self._lock:
            if not all(math.isfinite(v) for v in (vx, vy, wz)):
                self._command = ControlOutput(0.0, 0.0, 0.0, "invalid")
                self._updated_at = None
                return
            x = max(-self.limit, min(self.limit, vx))
            y = max(-self.limit, min(self.limit, vy))
            reason = "limited" if (x != vx or y != vy) else "valid"
            self._command = ControlOutput(x, y, 0.0, reason)
            self._updated_at = self._clock()

    def stop(self) -> None:
        """清除当前指令；后续新update可重新恢复。不是锁定式急停。"""
        with self._lock:
            self._command = ControlOutput(0.0, 0.0, 0.0, "stopped")
            self._updated_at = None

    def _output_locked(self) -> ControlOutput:
        if self._updated_at is None:
            return self._command
        age = self._clock() - self._updated_at
        if age < 0 or age >= self.timeout:
            return ControlOutput(0.0, 0.0, 0.0, "timeout")
        return self._command

    def output(self) -> ControlOutput:
        with self._lock:
            return self._output_locked()

    def build_frame(self) -> bytes:
        """保护后的控制量打包为19字节，并递增流水号。"""
        with self._lock:
            command = self._output_locked()
            frame = pack_command(command.vx, command.vy, command.wz, self._seq)
            self._seq = next_seq(self._seq)
            return frame
