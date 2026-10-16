"""100 Hz离线循环：sink仅接收内存帧，本模块不访问串口。"""

import math
import time

from safety_control import SafetyController


def run_loop(controller, sink, duration=2.0, frequency=100.0,
             clock=time.monotonic, sleep=time.sleep, demo_update=None):
    """sink(frame)接收单帧；demo_update仅用于演示生成控制量。

    落后时跳过周期，不连续补发。发送完成后至少留一个周期再开始下一帧。
    因而目标频率为100 Hz，操作系统调度和处理耗时可能使实际频率降低。
    未来实车控制应从独立导航模块更新controller，不在此处执行导航计算。
    """
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("duration必须为有限正数")
    if not math.isfinite(frequency) or frequency <= 0:
        raise ValueError("frequency必须为有限正数")
    period = 1.0 / frequency
    start = clock()
    deadline = start
    try:
        while True:
            remaining = deadline - clock()
            if remaining > 0:
                sleep(remaining)
                continue  # 再次检查，兼容提前唤醒。
            elapsed = clock() - start
            if elapsed >= duration:
                break
            if demo_update is not None:
                demo_update(elapsed)
            sink(controller.build_frame())
            # 不追赶过期周期；与上次输出结束保持间隔。
            deadline = max(deadline + period, clock() + period)
    finally:
        controller.stop()
        # 退出帧也保留间隔；Ctrl-C/普通异常时尽力输出零速。
        sleep(period)
        sink(controller.build_frame())


def main():
    import struct

    controller = SafetyController()
    records = []
    start = time.monotonic()

    def memory_sink(frame):
        records.append((time.monotonic() - start, frame))

    def simulated_navigation(elapsed):
        # 开始200ms保持零速；随后更新400ms；600ms后模拟导航停止更新。
        if 0.2 <= elapsed < 0.6:
            controller.update(0.1, -0.1)

    print("离线运行约2秒，帧仅保存在内存中，不访问串口。", flush=True)
    run_loop(controller, memory_sink, demo_update=simulated_navigation)

    print("输出状态变化（时间为实际测量）：")
    previous = None
    for timestamp, frame in records:
        values = struct.unpack("<fff", frame[5:17])
        moving = any(values)
        if moving != previous:
            print(f"  t={timestamp:.3f}s: " + ("运动指令" if moving else "零速指令"))
            previous = moving
    timestamps = [t for t, _ in records[:-1]]  # 单独排除退出零速帧。
    intervals = [b - a for a, b in zip(timestamps, timestamps[1:])]
    print(f"周期帧数：{len(timestamps)}；另加1帧退出零速")
    if intervals:
        average = sum(intervals) / len(intervals)
        print(f"平均周期：{average * 1000:.3f}ms；测得频率：{1 / average:.2f}Hz")
        print(f"周期范围：{min(intervals)*1000:.3f}～{max(intervals)*1000:.3f}ms")
    print("最后一帧：", records[-1][1].hex(" ").upper())
    print("预期：约0.2秒开始运动，约1.1秒因控制量未更新而归零。")


if __name__ == "__main__":
    main()
