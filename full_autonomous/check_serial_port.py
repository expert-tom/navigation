"""串口故障预检查：打开串口前判断“占用”还是“掉线”。

只读检查，不发送任何指令，不使用任何第三方库。可用系统python3直接运行：
    python3 check_serial_port.py /dev/serial/by-id/usb-1a86_USB_Single_Serial_xxx-if00
    python3 check_serial_port.py          # 自动列出候选设备
main.py serial 在打开串口前会自动调用同一套检查（可用 --no-preflight 跳过）。
"""
import json
import os
import re
import stat
import subprocess
import sys

BY_ID_DIR = "/dev/serial/by-id"


def run(command):
    """执行诊断命令；失败或超时返回None，不抛出。"""
    try:
        done = subprocess.run(command, capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return None
    text = (done.stdout or "") + (done.stderr or "")
    return text.strip() or None


def find_hold_pids(real_path, device):
    """在/proc里找出仍持有该串口的进程；需要能读到别人的fd权限。"""
    targets = {real_path, device, os.path.realpath(real_path)}
    holders = {}
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        fd_dir = "/proc/%s/fd" % entry
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                link = os.readlink(os.path.join(fd_dir, fd))
            except OSError:
                continue
            if link in targets:
                command = run(["ps", "-o", "args=", "-p", entry])
                holders[entry] = command or "?"
                break
    return holders


def o_direct_open(path, flags):
    """只验证能否独占打开；非Linux或测试注入时跳过。"""
    if getattr(os, "name", "") != "posix" or not hasattr(os, "O_NOCTTY"):
        return None
    try:
        fd = os.open(path, flags | os.O_NOCTTY | os.O_NONBLOCK)
    except PermissionError as exc:
        return {"ok": False, "error": "权限不足：%s" % exc,
                "hint": "sudo usermod -aG dialout $USER，然后重新登录"}
    except OSError as exc:
        return {"ok": False, "error": str(exc),
                "hint": "端口可能已被别的进程占用：lsof %s；fuser -v %s" % (path, path)}
    try:
        try:
            import fcntl
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            flock = "ok"
        except ImportError:
            flock = "unsupported"
        except OSError as exc:
            flock = "busy: %s" % exc
        return {"ok": True, "flock": flock}
    finally:
        os.close(fd)


def resolve_device(device):
    report = {"device": device, "real_path": None, "exists": False, "resolve_error": None}
    try:
        report["exists"] = os.path.exists(device)
        report["real_path"] = os.path.realpath(device)
    except OSError as exc:
        report["resolve_error"] = str(exc)
    return report


def preflight(device, inject=None, probe_env=True):
    """返回串口可用性报告；clean为False时不得继续发送运动指令。

    inject/probe_env 仅用于单元测试：注入独占打开结果并跳过真实设备探测。
    """
    problems = []
    hints = []
    report = resolve_device(device)
    real_path = report["real_path"] or device
    if not report["exists"]:
        problems.append("设备节点不存在：%s（适配器未插好、未上电，或已被拔出）" % device)
        hints.append("插好适配器后用 ls -l %s 确认；也可以直接查看 ls /dev/ttyUSB*" % BY_ID_DIR)
        report.update(clean=False, problems=problems, hints=hints)
        return report
    if not os.path.islink(device):
        hints.append("设备路径不是by-id符号链接：重新插拔后编号可能变化")
    try:
        device_stat = os.stat(real_path)
    except OSError as exc:
        device_stat = None
        problems.append("无法读取设备属性：%s" % exc)
    if device_stat is not None:
        report["mode_octal"] = oct(device_stat.st_mode & 0o777)
        report["uid"] = device_stat.st_uid
        report["gid"] = device_stat.st_gid
        report["is_char_device"] = stat.S_ISCHR(device_stat.st_mode)
        if os.name == "posix" and not stat.S_ISCHR(device_stat.st_mode):
            problems.append("该路径不是字符设备：%s" % real_path)
        if not device_stat.st_mode & 0o222:
            problems.append("设备对当前用户不可写：权限为 %s" % report["mode_octal"])
        if probe_env and hasattr(os, "getuid") and device_stat.st_uid == os.getuid():
            hints.append("设备属主是当前用户：通常是brltty等驱动抢占了CDC设备，"
                         "请检查 sudo systemctl stop brltty-udev.service")

    direct = inject if inject is not None else o_direct_open(real_path, os.O_RDWR)
    report["direct_open"] = direct
    if direct is not None and not direct.get("ok", True):
        problems.append("独占打开失败：%s" % direct.get("error"))
        if direct.get("hint"):
            hints.append(direct["hint"])
    if direct is not None and isinstance(direct.get("flock"), str) and direct["flock"].startswith("busy"):
        problems.append("端口已被加文件锁：%s" % direct["flock"])

    if probe_env:
        holders = find_hold_pids(real_path, device)
        report["holders"] = holders
        if holders:
            problems.append("发现仍占用该串口的进程：%s" % "; ".join(
                "%s -> %s" % (pid, command) for pid, command in sorted(holders.items())))
            hints.append("先结束这些进程（ros2节点、minicom、上一次没退干净的python）再重试")
        report["tty_driver"] = read_driver(real_path)
        report["usb_path"] = find_usb_ancestor(real_path)
        report["dmesg_tail"] = read_dmesg_tail()
        report["lsof"] = run(["lsof", device])
        report["fuser"] = run(["fuser", "-v", device])
        report["stty"] = run(["stty", "-F", real_path, "-a"])
        if report["dmesg_tail"]:
            hints.append("dmesg里若出现 USB disconnect / device descriptor read error，"
                         "说明是线缆、接口或供电问题，换口或换线")
        if report["tty_driver"]:
            hints.append("内核驱动为 %s；ch341 系列在供电瞬断时会直接掉线" % report["tty_driver"])
    if not problems:
        hints.append("没有发现占用或权限问题：设备已就绪，可以开始零速调试")
    report.update(clean=not problems, problems=problems, hints=hints)
    return report


def read_driver(real_path):
    """读取串口对应的内核驱动名，例如 ch341。"""
    name = os.path.basename(real_path)
    try:
        return os.path.basename(os.readlink("/sys/class/tty/%s/device/driver" % name))
    except OSError:
        return None


def find_usb_ancestor(real_path):
    """向上找到USB设备节点，便于核对供电/接口。"""
    name = os.path.basename(real_path)
    path = "/sys/class/tty/%s/device" % name
    for _ in range(8):
        try:
            path = os.path.dirname(os.path.realpath(path))
        except OSError:
            return None
        if re.search(r"/usb\d+/[0-9]+-[0-9.]+$", path):
            return path
        if path in ("/", "/sys"):
            return None
    return None


def read_dmesg_tail():
    """dmesg可能因kernel.dmesg_restrict不可读；不可读时给出替代命令。"""
    try:
        with open("/proc/sys/kernel/dmesg_restrict", "r", encoding="utf-8") as handle:
            restrict = handle.read().strip()
    except OSError:
        restrict = "unknown"
    if restrict == "1":
        return {"available": False, "note": "dmesg受限，请在终端执行 sudo dmesg | tail -40"}
    text = run(["dmesg", "--ctime", "--level=err,warn,info"])
    if text is None:
        return {"available": False, "note": "dmesg不可读，请在终端执行 sudo dmesg | tail -40"}
    lines = [line for line in text.splitlines()
             if re.search(r"usb|tty|ch34|serial", line, re.IGNORECASE)]
    return {"available": True, "lines": lines[-25:]}


def list_candidates():
    try:
        return sorted(os.listdir(BY_ID_DIR))
    except OSError:
        return []


def main():
    if len(sys.argv) > 1:
        targets = sys.argv[1:]
    else:
        candidates = list_candidates()
        if not candidates:
            print(json.dumps({"error": "没有找到 %s 下的设备；"
                                       "检查适配器是否插好，或查看 /dev/ttyUSB*" % BY_ID_DIR},
                             ensure_ascii=False, indent=2))
            return 1
        targets = [os.path.join(BY_ID_DIR, name) for name in candidates]
    failed = False
    for device in targets:
        report = preflight(device)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        failed = failed or not report["clean"]
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
