"""统一入口：默认帮助；仿真、测试、反馈和串口调试。"""
import argparse
import json
from pathlib import Path
import sys
import time
import unittest

from feedback import FeedbackParser
from safety_control import SafetyController

ROOT = Path(__file__).resolve().parent
VERSION = "2026-10-15-serial-diag"


def known_options(parser):
    """收集parser与其子parser已知的所有选项字符串。"""
    options = set()
    stack = [parser]
    while stack:
        current = stack.pop()
        for action in current._actions:
            options.update(action.option_strings)
            if isinstance(action, argparse._SubParsersAction):
                stack.extend(action.choices.values())
    return options


def parse_argv(parser, argv=None):
    """旧版NUC代码不认识--sweep等新参数，这里把报错说清楚，避免误判成设备问题。"""
    if argv is None:
        argv = sys.argv[1:]
    if "--version" in argv:
        print(VERSION)
        raise SystemExit(0)
    unknown = [item for item in argv if item.startswith("--")
               and item.split("=")[0] not in known_options(parser)]
    try:
        return parser.parse_args(argv)
    except SystemExit:
        if unknown:
            print("\n提示：%s 不被当前代码识别，说明NUC上运行的不是最新代码。"
                  % ", ".join(unknown))
            print("      当前版本标记为 %s；请在项目目录执行：" % VERSION)
            print("        python3 main.py serial --help")
            print("      若帮助里没有--sweep/--k/--fast，就把整个navigation_starter目录")
            print("      重新同步到NUC，不要只复制main.py。")
        raise


def main():
    parser = argparse.ArgumentParser(description="SRM导航整合项目")
    sub = parser.add_subparsers(dest="mode", required=True)
    sub.add_parser("test")
    sub.add_parser("simulate")
    sub.add_parser("feedback")
    serial = sub.add_parser("serial")
    serial.add_argument("--device", required=True)
    serial.add_argument("--seconds", type=float, default=5)
    serial.add_argument("--vx", type=float, default=0)
    serial.add_argument("--vy", type=float, default=0)
    serial.add_argument("--arm", action="store_true", help="允许低速调试运动")
    serial.add_argument("--k", type=float, default=0.05,
                        help="首轮调试的速度上限；确认方向后可用--fast放宽到0.2")
    serial.add_argument("--fast", action="store_true",
                        help="解除首轮0.05限幅（需先确认方向与空旷区域），协议建议不超过0.2")
    serial.add_argument("--sweep", action="store_true",
                        help="依次发送+vx/-vx/+vy/-vy，每段后回零，用来测轴向")
    serial.add_argument("--segment", type=float, default=1.5, help="sweep每段时间，秒")
    serial.add_argument("--pause", type=float, default=1.0, help="sweep段间回零时间，秒")
    serial.add_argument("--grace", type=float, default=1.0,
                        help="串口读空后的宽限秒数，超过该时间仍持续读空才判定故障")
    serial.add_argument("--no-preflight", action="store_true",
                        help="跳过打开串口前的设备占用检查")
    serial.add_argument("--version", action="store_true", help="打印代码版本标记后退出")
    args = parse_argv(parser)
    if getattr(args, "version", False):
        print(VERSION)
        return
    if args.mode == "test":
        suite = unittest.defaultTestLoader.discover(str(ROOT), pattern="test_*.py")
        raise SystemExit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
    if args.mode == "simulate":
        from simulate import simulate, write_artifacts, write_playback
        c = json.loads((ROOT/"config.json").read_text(encoding="utf-8"))
        success = True
        playback = {"config": c, "scenarios": {}}
        for scenario in ("normal", "obstacle", "localization_loss"):
            route, rows, result = simulate(c, scenario)
            write_artifacts(ROOT/"simulation_results", c, route, rows, result)
            playback["scenarios"][scenario] = {"route": route, "rows": rows, "report": result}
            print(json.dumps(result, ensure_ascii=False, indent=2))
            success &= result["completed"]
        write_playback(ROOT/"simulation_results", playback)
        raise SystemExit(not success)
    if args.mode == "feedback":
        sample = bytes.fromhex("A5 0B 00 01 5C 01 00 AA 2C 01 88 77 66 55 44 33 22 11 EF 74")
        print(json.dumps(FeedbackParser().feed(sample), ensure_ascii=False, indent=2))
        return
    import math
    if not all(math.isfinite(v) for v in (args.seconds, args.vx, args.vy)) or not 0 < args.seconds <= 30:
        parser.error("调试时长必须为0到30秒之间的有限数，控制量必须有限")
    if not math.isfinite(args.grace) or not 0 <= args.grace <= 30:
        parser.error("宽限时间必须是0到30秒之间的有限数")
    if not math.isfinite(args.k) or not 0.001 <= args.k <= 0.2:
        parser.error("速度上限--k必须在0.001到0.2之间")
    if not all(math.isfinite(v) and 0 < v <= 10 for v in (args.segment, args.pause)):
        parser.error("sweep的段时间与回零时间必须在0到10秒之间")
    speed_limit = 0.2 if args.fast else 0.05
    if max(abs(args.vx), abs(args.vy)) > speed_limit:
        parser.error("单次调试控制量不得超过%s；确认方向后可用--fast放宽到0.2" % speed_limit)
    moving = bool(args.vx or args.vy or args.sweep)
    if moving and not args.arm:
        parser.error("运动调试需要--arm；先确认现场急停和调试区域")
    if moving and not args.fast and args.seconds > 2:
        parser.error("首轮调试运动时长最多2秒；确认方向后加--fast放宽")
    runner = SerialRunner(args)
    runner.run()


class SerialRunner:
    """串口调试流程：预检查、定频发送、可选sweep、使能状态与统计汇总。"""

    SWEEP_SIGNS = ((1, 0, "+vx"), (-1, 0, "-vx"), (0, 1, "+vy"), (0, -1, "-vy"))

    def __init__(self, args):
        self.a = args
        self.status = None
        self.last_status_at = None
        self.status_changes = 0

    def preflight(self):
        if self.a.no_preflight:
            return
        from check_serial_port import preflight
        report = preflight(self.a.device)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        if not report["clean"]:
            for problem in report["problems"]:
                print("预检查未通过：" + problem)
            raise SystemExit("串口预检查未通过，未打开串口，也未发送任何指令")

    def collect(self, link):
        """打印反馈帧，并跟踪chassis_power使能状态的变化。"""
        while link.messages:
            message = link.messages.popleft()
            print(json.dumps(message, ensure_ascii=False))
            if message.get("cmd_id") == 11:
                snapshot = (message.get("robot_id"), message.get("chassis_enabled"),
                            message.get("chassis_power_limit"))
                if snapshot != self.status:
                    self.status = snapshot
                    self.status_changes += 1
                    self.last_status_at = time.monotonic()
                    print(">>> 底盘状态变化: robot_id=%s chassis_enabled=%s "
                          "chassis_power_limit=%s"
                          % (message.get("robot_id"), message.get("chassis_enabled"),
                             message.get("chassis_power_limit")))

    def drive(self, link, controller, seconds, vx, vy):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            link.check()
            controller.update(vx, vy)
            self.collect(link)
            time.sleep(0.02)

    def run(self):
        self.preflight()
        from serial_link import SerialLink
        controller = SafetyController(limit=self.a.k)
        link = SerialLink(self.a.device, controller, zero_read_grace=self.a.grace)
        try:
            link.start()
            self.drive(link, controller, 0.2, 0.0, 0.0)
            if self.a.sweep:
                for sx, sy, label in self.SWEEP_SIGNS:
                    print(">>> %s：%s 持续%.1f秒（回零后测下一轴）"
                          % (label, self.a.k, self.a.segment))
                    self.drive(link, controller, self.a.segment,
                               sx*self.a.k, sy*self.a.k)
                    print(">>> 回零 %.1f秒" % self.a.pause)
                    self.drive(link, controller, self.a.pause, 0.0, 0.0)
            elif self.a.vx or self.a.vy:
                self.drive(link, controller, self.a.seconds, self.a.vx, self.a.vy)
            else:
                self.drive(link, controller, self.a.seconds, 0.0, 0.0)
        finally:
            link.close()
            print(json.dumps(link.stats(), ensure_ascii=False, indent=2))
        link.check()
        self.report(link)

    def report(self, link):
        stats = link.stats()
        if stats["feedback_messages"] == 0:
            print("警告：本次没有解析到任何反馈帧。收不到反馈不等于发送失败，"
                  "但没有反馈就无法判断底盘是否使能；先检查下位机是否上电。")
            return
        if self.status is None or self.status[1] is not True:
            print("重要：本次始终没有收到 chassis_power=true。")
            print("      底盘使能标志为 false 时，无论发多大的速度指令车都不会动。")
            print("      请确认：遥控器是否切到自动/上位机控制、安全开关是否解除、")
            print("      底盘电源与急停是否已放开，然后重跑本命令并观察上面的“底盘状态变化”。")
        elif self.a.sweep:
            print("底盘已使能（chassis_power=true）。逐段核对：")
            for _, _, label in self.SWEEP_SIGNS:
                print("  %s 段：车往哪个方向动、动了多少 → 记录下来就是标定矩阵的一列" % label)
            print("  注意：协议存在90°轴向交叉，vx进底盘y方向、vy取负进x方向。")
        if self.status_changes <= 1:
            print("提示：整段过程中底盘状态只出现%d种取值，说明使能条件没有变化。"
                  % max(self.status_changes, 1))


if __name__ == "__main__":
    main()
