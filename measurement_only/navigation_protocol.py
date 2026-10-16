"""导航赛道上行协议。仅生成数据，不打开串口、不控制车辆。"""

import argparse
import math
import struct


def crc8(data: bytes) -> int:
    """反射多项式0x8C，初值0xFF，无最终异或。"""
    value = 0xFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8C if value & 1 else 0)
    return value


def crc16(data: bytes) -> int:
    """反射多项式0x8408，初值0xFFFF，无最终异或。"""
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8408 if value & 1 else 0)
    return value


def pack_command(vx: float, vy: float, wz: float, seq: int = 1) -> bytes:
    """生成19字节原始协议帧；不缩放、不限幅，不含cmd_id。

    此函数用于协议编码与离线测试。实际控制前应另行进行限幅与超时保护。
    非有限值或超出float32范围的值会抛出ValueError。
    """
    if type(seq) is not int or not 0 <= seq <= 255:
        raise ValueError("seq必须是0到255之间的整数")
    if not all(math.isfinite(v) for v in (vx, vy, wz)):
        raise ValueError("速度指令不能包含NaN或Inf")
    try:
        payload = struct.pack("<fff", vx, vy, wz)
    except (OverflowError, struct.error) as exc:
        raise ValueError("速度指令超出float32可编码范围") from exc
    header = struct.pack("<BHB", 0xA5, 12, seq)
    frame = header + bytes([crc8(header)]) + payload
    return frame + struct.pack("<H", crc16(frame))


def next_seq(seq: int) -> int:
    """流水号从255回绕到0。"""
    if type(seq) is not int or not 0 <= seq <= 255:
        raise ValueError("seq必须是0到255之间的整数")
    return (seq + 1) % 256


def main() -> None:
    parser = argparse.ArgumentParser(description="离线生成导航指令帧，不发送给车辆")
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--wz", type=float, default=0.0)
    parser.add_argument("--seq", type=int, default=1)
    args = parser.parse_args()
    try:
        frame = pack_command(args.vx, args.vy, args.wz, args.seq)
    except ValueError as exc:
        parser.error(str(exc))
    print(f"Offline only | vx={args.vx}, vy={args.vy}, wz={args.wz}, seq={args.seq}")
    print(f"Length: {len(frame)} bytes")
    print(frame.hex(" ").upper())
    print(f"Header CRC8: 0x{frame[4]:02X}")
    print(f"Frame CRC16: 0x{int.from_bytes(frame[-2:], 'little'):04X}")


if __name__ == "__main__":
    main()
