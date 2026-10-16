"""统一入口：默认帮助；仿真、测试、反馈和串口调试。"""
import argparse
import json
from pathlib import Path
import time
import unittest

from feedback import FeedbackParser
from safety_control import SafetyController

ROOT = Path(__file__).resolve().parent


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
    serial.add_argument("--grace", type=float, default=1.0, help="读异常后仅零速观察的秒数，0到5；不会自动恢复运动")
    serial.add_argument("--stats-output", type=Path, help="串口统计JSON路径，默认保存在logs中")
    args = parser.parse_args()
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
    if not math.isfinite(args.grace) or not 0 <= args.grace <= 5:
        parser.error("grace必须为0到5秒的有限数，观察期间只发送零速")
    if args.vx or args.vy:
        if not args.arm:
            parser.error("运动调试需要--arm；先确认现场急停和调试区域")
        if max(abs(args.vx), abs(args.vy)) > 0.05 or args.seconds > 2:
            parser.error("首次调试限幅0.05，运动时长最多2秒")
    from serial_link import SerialLink
    from check_serial_port import preflight
    report = preflight(args.device)
    print(json.dumps({"preflight": report}, ensure_ascii=False))
    if not report["clean"]:
        parser.exit(2, "预检查失败，未打开串口；"+"; ".join(report["problems"])+"\n")
    stats_path = args.stats_output or ROOT/"logs"/f"serial_run_{time.time_ns()}.json"
    stats_path.parent.mkdir(parents=True, exist_ok=True)
    controller = SafetyController(limit=0.05)
    try:
        link = SerialLink(args.device, controller, zero_read_grace=args.grace)
    except RuntimeError as exc:
        parser.exit(2, str(exc)+"\n")
    try:
        link.start()
        time.sleep(0.2)  # 先发送零速。
        start = time.monotonic()
        while time.monotonic()-start < args.seconds:
            link.check(for_motion=bool(args.vx or args.vy))
            controller.update(args.vx, args.vy)
            while link.messages:
                print(json.dumps(link.messages.popleft(), ensure_ascii=False))
            time.sleep(0.02)
    finally:
        link.close()
        result = {"preflight": report, "serial": link.stats(),
                  "requested": {"vx": args.vx, "vy": args.vy, "seconds": args.seconds}}
        stats_path.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        print(json.dumps(result["serial"], ensure_ascii=False, indent=2))
        print("串口统计：", stats_path)
    link.check()
    if link.last_anomaly is not None:
        parser.exit(1, "观察期间出现读异常；运动已锁定，即使反馈恢复也需排查后重新启动。\n")


if __name__ == "__main__":
    main()
