"""使用模拟时间展示保护效果，不等待真实时间，也不连接车辆。"""

import struct

from safety_control import SafetyController


def main():
    now = [0.0]
    controller = SafetyController(clock=lambda: now[0])
    labels = {
        "startup": "启动归零", "valid": "有效指令", "limited": "已限幅",
        "invalid": "异常归零", "timeout": "超时归零", "stopped": "主动停止",
    }

    def show(event):
        output = controller.output()
        frame = controller.build_frame()
        values = struct.unpack("<fff", frame[5:17])
        print(f"t={now[0]:.2f}s | {event} | {labels[output.reason]}")
        print("  vx={:.3f}, vy={:.3f}, wz={:.3f}, 帧长={}".format(*values, len(frame)))

    print("离线演示：以下时间为模拟时间，不代表发送频率。")
    show("尚未收到导航指令")
    now[0] = 0.1
    controller.update(0.1, -0.1)
    show("收到正常指令")
    now[0] = 0.2
    controller.update(1.0, -1.0)
    show("收到超范围指令")
    now[0] = 0.69
    show("距上次更新490ms，读取不会刷新计时")
    now[0] = 0.71
    show("距上次更新510ms")
    now[0] = 0.8
    controller.update(0.1, 0)
    show("新的有效指令恢复控制")
    now[0] = 0.9
    controller.update(float("nan"), 0.1)
    show("收到NaN，所有轴归零")
    now[0] = 1.0
    controller.update(0.1, 0.1)
    controller.stop()
    show("主动清除指令")
    print("最后生成的停车帧：", controller.build_frame().hex(" ").upper())


if __name__ == "__main__":
    main()
