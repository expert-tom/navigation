import argparse
import glob
import importlib.util
import json
import os
from pathlib import Path
import platform
import subprocess
import time


def inspect():
    results = {"platform": platform.platform(), "user": os.environ.get("USER", os.environ.get("USERNAME")),
               "ROS_DISTRO": os.environ.get("ROS_DISTRO"), "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"),
               "serial_by_id": glob.glob("/dev/serial/by-id/*"), "serial_candidates": glob.glob("/dev/ttyACM*")+glob.glob("/dev/ttyUSB*")}
    results["imports"] = {name: importlib.util.find_spec(name) is not None for name in ("rclpy", "tf2_ros", "sensor_msgs_py", "serial")}
    commands = {"network": ["ip", "-br", "addr"], "ros_topics": ["ros2", "topic", "list", "-t"],
                "ros_nodes": ["ros2", "node", "list"], "ros_packages": ["ros2", "pkg", "list"]}
    for key, command in commands.items():
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=12)
            results[key] = {"returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr}
        except (OSError, subprocess.TimeoutExpired) as exc:
            results[key] = {"error": str(exc)}
    return results


def main():
    parser = argparse.ArgumentParser(description="Read-only ROS, network and serial inventory; does not open serial ports")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent/"logs"/f"device_check_{time.time_ns()}.json")
    args = parser.parse_args()
    report = inspect()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print("Saved:", args.output)


if __name__ == "__main__":
    main()
