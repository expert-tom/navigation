"""Offline waypoint planning check. Never imports ROS or opens a serial port."""
import argparse
import json
from pathlib import Path

from navigation import Navigator, fixed_obstacles
from real_vehicle import load_settings
from task_config import apply_task


ROOT = Path(__file__).resolve().parent


def inspect_task(config, settings):
    nav = Navigator(config)
    obstacles = fixed_obstacles(config)
    segments = []
    start = config["parking"]
    for x, y, label in nav.route:
        path = nav.planner.plan(start, (x, y), obstacles)
        segments.append({"label": label, "start": list(start), "goal": [x, y],
                         "feasible_with_configured_obstacles": bool(path), "path": path})
        start = (x, y)
    confirmations = ("field_confirmed", "calibration_confirmed", "lidar_extrinsics_confirmed", "emergency_stop_confirmed")
    warnings = [key+" is not confirmed" for key in confirmations if settings.get(key) is not True]
    if not settings.get("quality_monitor", {}).get("source_confirmed"):
        warnings.append("localization quality source is not confirmed")
    if config.get("task_measurements_confirmed") is not True:
        warnings.append("task coordinates are examples/unconfirmed")
    return {"mode": "OFFLINE_ONLY", "hardware_verified": False,
            "map_frame": settings["map_frame"], "sensor_topic": settings["sensor_topic"],
            "sensor_type": settings["sensor_type"], "warnings": warnings,
            "planning_feasible": all(s["feasible_with_configured_obstacles"] for s in segments),
            "segments": segments,
            "limits": "Checks only configured circles/bounds. Does not verify live map, TF, sensor visibility, hardware, protocol scale or physical stopping."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=ROOT/"config.json")
    parser.add_argument("--settings", type=Path, default=ROOT/"config/real_vehicle.json")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        settings = load_settings(args.settings)
        config = apply_task(json.loads(args.config.read_text(encoding="utf-8")),
                            task_path=args.task, map_frame=settings["map_frame"])
        result = inspect_task(config, settings)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.exit(2, "Task check failed: "+str(exc)+"\n")
    payload = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
    print(payload)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8")
    if not result["planning_feasible"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
