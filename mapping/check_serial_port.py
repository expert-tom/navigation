"""Metadata-only preflight: never opens a tty or changes system settings."""
import argparse
import json
import os
import stat
from pathlib import Path


def find_holders(device, proc_root="/proc"):
    holders, unreadable = [], 0
    try:
        target = os.stat(device)
        processes = list(Path(proc_root).iterdir())
    except OSError:
        return [], 1
    for process in processes:
        if not process.name.isdigit():
            continue
        try:
            descriptors = list((process/"fd").iterdir())
        except PermissionError:
            unreadable += 1
            continue
        except OSError:
            continue
        for descriptor in descriptors:
            try:
                info = descriptor.stat()
                match = stat.S_ISCHR(info.st_mode) and info.st_rdev == target.st_rdev
            except OSError:
                continue
            if match:
                holders.append({"pid": int(process.name)})
                break
    return holders, unreadable


def preflight(device):
    report = {"device": str(device), "real_path": os.path.realpath(device),
              "opens_device": False, "exclusive_access_proven": False,
              "problems": [], "warnings": []}
    try:
        info = os.stat(device)
    except OSError as exc:
        report["problems"].append("设备不存在或无法读取属性: "+str(exc))
        report["clean"] = False
        return report
    if not stat.S_ISCHR(info.st_mode):
        report["problems"].append("路径不是字符设备")
    if not os.access(device, os.R_OK | os.W_OK):
        report["problems"].append("当前用户没有串口读写权限；联系设备管理员处理，不修改系统权限")
    if os.name == "posix" and Path("/proc").is_dir():
        holders, unreadable = find_holders(device)
        report.update(holders=holders, unreadable_processes=unreadable)
        if holders:
            report["problems"].append("发现持有该串口的进程；确认所有者及用途后处理")
        if unreadable:
            report["warnings"].append("部分进程无权检查，未发现占用不等于无人占用")
        name = os.path.basename(report["real_path"])
        driver = Path("/sys/class/tty")/name/"device/driver"
        report["driver"] = os.path.basename(os.path.realpath(driver)) if driver.exists() else None
    else:
        report["warnings"].append("本平台无法检查Linux进程占用")
    report["warnings"].append("检查有权限和时序限制；真正打开时仍需处理失败，不能据此保证链路健康")
    report["clean"] = not report["problems"]
    return report


def require_preflight(device):
    result = preflight(device)
    if not result["clean"]:
        raise RuntimeError("串口预检查失败；未打开设备: "+"; ".join(result["problems"]))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True)
    args = parser.parse_args()
    report = preflight(args.device)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    raise SystemExit(0 if report["clean"] else 1)


if __name__ == "__main__":
    main()
