"""Strict premapped contest entry point. Defaults to dry-run with no tty open."""
import argparse
import json
import os
from pathlib import Path

from map_bundle import validate_bundle
from navigation import Navigator
from real_vehicle import create_node, load_settings
from vehicle_core import grid_obstacles

ROOT = Path(__file__).resolve().parent


def prepare(bundle, field, settings_path, armed=False):
    manifest, grid = validate_bundle(bundle)
    config = json.loads(Path(field).read_text(encoding="utf-8"))
    settings = load_settings(settings_path)
    if "mission_waypoints" in config or "fixed_obstacles" in config:
        raise ValueError("contest entry requires the ordered two-pole eight, not arbitrary waypoints")
    if config.get("map_digest") != manifest["map_digest"]:
        raise ValueError("field coordinates must be selected on THIS map and bound to map_digest")
    if settings["map_frame"] != manifest["map_frame"]:
        raise ValueError("map frame mismatch")
    if not settings["map_required"] or settings["pose_mode"] != "tf" or settings["unknown_blocked"] is not True:
        raise ValueError("premapped entry requires map and TF localization")
    effective = dict(config, **settings["navigation_overrides"])
    if effective.get("robot_radius",0) < .29:
        raise ValueError("whole robot envelope must be confirmed; supplied xacro already requires >=0.29m")
    if armed:
        missing = [key for key in ("field_confirmed","calibration_confirmed","lidar_extrinsics_confirmed",
                   "emergency_stop_confirmed") if settings.get(key) is not True]
        if config.get("field_confirmed") is not True or config.get("calibration_confirmed") is not True:
            missing.append("field file: field_confirmed/calibration_confirmed")
        if missing:
            raise ValueError("unconfirmed: "+", ".join(missing))
    nav = Navigator(effective)
    obstacles = grid_obstacles(grid["data"],grid["width"],grid["height"],grid["resolution"],
        grid["origin"][:2],grid["origin"][2],settings["occupancy_threshold"],True,
        effective["bounds"],settings["map_max_cells"])
    scene = obstacles + [( *effective[key], effective["pole_radius"]) for key in ("lower_pole","upper_pole")]
    points = [effective["parking"]]+[point[:2] for point in nav.route]
    for index,(a,b) in enumerate(zip(points,points[1:])):
        if not nav.planner.clear(a,b,scene):
            raise ValueError(f"reference route segment {index} blocked/unknown/outside map: fix map, pole base radius or path geometry; do not disable checking")
    settings["expected_map_digest"] = manifest["map_digest"]
    return config,settings,dict(route_points=len(nav.route),static_obstacles=len(obstacles),
        map_digest=manifest["map_digest"],estimated_minimum_s=sum(__import__('math').dist(a,b) for a,b in zip(points,points[1:]))/effective["max_speed_mps"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--field", type=Path, required=True)
    parser.add_argument("--settings", type=Path, default=ROOT/"config/premapped.json")
    parser.add_argument("--check", action="store_true", help="offline map/route check only")
    parser.add_argument("--connect-serial", action="store_true")
    parser.add_argument("--arm", action="store_true")
    parser.add_argument("--device")
    parser.add_argument("--log-dir", type=Path, default=ROOT/"logs")
    args, ros_args = parser.parse_known_args()
    if args.arm and not args.connect_serial:
        parser.error("--arm requires --connect-serial")
    if args.connect_serial and (os.name != "posix" or os.environ.get("ROS_DOMAIN_ID") == "77"):
        parser.error("serial only permitted on commissioned Linux hardware, never mock domain 77")
    try:
        config,settings,report = prepare(args.bundle,args.field,args.settings,args.arm)
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.check:
        return
    import rclpy
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_node(settings, config, args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
