"""导航下行反馈流式解析：兼容拆包、粘包、噪声和CRC错误。"""
import struct
from navigation_protocol import crc8, crc16


class FeedbackParser:
    def __init__(self):
        self.buffer = bytearray()
        self.rejected = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        messages = []
        while self.buffer:
            index = self.buffer.find(b"\xa5")
            if index < 0:
                self.buffer.clear()
                break
            del self.buffer[:index]
            if len(self.buffer) < 5:
                break
            size = int.from_bytes(self.buffer[1:3], "little")
            if crc8(self.buffer[:4]) != self.buffer[4] or size not in (11, 13):
                del self.buffer[0]
                self.rejected += 1
                continue
            if len(self.buffer) < 7:
                break
            cmd = int.from_bytes(self.buffer[5:7], "little")
            if (cmd, size) not in ((1, 11), (11, 13)):
                del self.buffer[0]
                self.rejected += 1
                continue
            total = 9 + size
            if len(self.buffer) < total:
                break
            frame = bytes(self.buffer[:total])
            if crc16(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
                del self.buffer[0]
                self.rejected += 1
                continue
            del self.buffer[:total]
            payload = frame[7:-2]
            result = {"cmd_id": cmd, "seq": frame[3]}
            if cmd == 1:
                flags, remaining, timestamp = struct.unpack("<BHQ", payload)
                result.update(game_type=flags & 15, game_progress=flags >> 4,
                              stage_remain_time=remaining, sync_time_stamp=timestamp)
            else:
                values = struct.unpack("<BBHHHHHB", payload)
                names = ("robot_id", "robot_level", "current_HP", "maximum_HP",
                         "shooter_barrel_cooling_value", "shooter_barrel_heat_limit",
                         "chassis_power_limit", "power_flags")
                result.update(zip(names, values))
                result.update(gimbal_power=bool(values[-1] & 1),
                              chassis_power=bool(values[-1] & 2),
                              shooter_power=bool(values[-1] & 4))
            messages.append(result)
        return messages
