"""Read-only inventory. Never opens tty, starts/kills a driver or alters ROS settings."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import platform
import subprocess
import sys


def inspect():
    report=dict(python=sys.version,executable=sys.executable,platform=platform.platform(),
        environment={key:os.environ.get(key) for key in ("ROS_DISTRO","ROS_DOMAIN_ID","RMW_IMPLEMENTATION",
            "ROS_LOCALHOST_ONLY","ROS_DISCOVERY_SERVER","CYCLONEDDS_URI","FASTRTPS_DEFAULT_PROFILES_FILE")},
        imports={key:importlib.util.find_spec(key) is not None for key in ("numpy","yaml","serial","rclpy","tf2_ros")},
        opens_serial=False,packages={})
    for package in ("nav2_map_server","nav2_amcl","nav2_lifecycle_manager","slam_toolbox","pointcloud_to_laserscan","rviz2"):
        try:
            result=subprocess.run(["ros2","pkg","prefix",package],capture_output=True,text=True,timeout=5)
            report["packages"][package]=dict(found=result.returncode==0,prefix=result.stdout.strip(),error=result.stderr.strip())
        except (OSError,subprocess.TimeoutExpired) as exc:
            report["packages"][package]=dict(found=False,error=str(exc))
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    report=inspect()
    payload=json.dumps(report,ensure_ascii=False,indent=2)
    print(payload)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        with args.output.open("x",encoding="utf-8") as stream:
            stream.write(payload)
    if not all(report["imports"].values()) or not all(p["found"] for p in report["packages"].values()):
        raise SystemExit(1)


if __name__=="__main__":
    main()
