"""无需ROS或第三方包的二维导航闭环仿真。"""
import argparse
import csv
import html
import json
import math
from pathlib import Path
import struct

from navigation import Navigator, distance
from safety_control import SafetyController
from robot_model import model_for_config


def simulate(config, scenario="normal", max_time=400.0, dt=0.05):
    nav = Navigator(config)
    nav.start()
    now = 0.0
    safe = SafetyController(limit=config["control_limit"], clock=lambda: now)
    x, y = config["parking"]
    yaw = 0.0
    rows, transitions = [], []
    previous = None
    obstacle = (*nav.route[8][:2], 0.12)
    min_clearance = math.inf
    while now <= max_time:
        obstacles = [obstacle] if scenario == "obstacle" and 0.5 <= now < 5.0 else []
        localized = not (scenario == "localization_loss" and 2.0 <= now < 3.0)
        sample = {"pose": [x, y, yaw], "stamp": now, "obstacle_stamp": now,
                  "obstacles": obstacles, "localized": localized}
        command = nav.step(sample, now)
        safe.update(*command)
        frame = safe.build_frame()
        vx, vy, wz = struct.unpack("<fff", frame[5:17])
        matrix = config["body_from_command"]
        bx = matrix[0][0]*vx + matrix[0][1]*vy
        by = matrix[1][0]*vx + matrix[1][1]*vy
        wx, wy = math.cos(yaw)*bx-math.sin(yaw)*by, math.sin(yaw)*bx+math.cos(yaw)*by
        nx, ny = x+wx*dt, y+wy*dt
        all_obstacles = obstacles + [(*config[k], config["pole_radius"])
                                     for k in ("lower_pole", "upper_pole")]
        if not nav.planner.clear((x, y), (nx, ny), all_obstacles):
            raise RuntimeError(f"仿真碰撞/出界，t={now:.2f}, state={nav.state}")
        for o in all_obstacles:
            min_clearance = min(min_clearance, distance((nx, ny), o)-o[2]-config["robot_radius"])
        if nav.state != previous:
            transitions.append({"time": round(now, 3), "state": nav.state, "waypoint": nav.index})
            previous = nav.state
        rows.append((now, x, y, yaw, vx, vy, nav.index, nav.state))
        x, y = nx, ny
        if nav.state == "COMPLETE":
            break
        now += dt
    result = {"scenario": scenario, "completed": nav.state == "COMPLETE",
              "simulated_seconds": round(now, 3), "route_points": len(nav.route),
              "completed_points": nav.index, "parking_error_m": round(distance((x, y), config["parking"]), 4),
              "min_obstacle_clearance_m": round(min_clearance, 4), "transitions": transitions,
              "robot_radius_m": config["robot_radius"], "safety_margin_m": config["margin"],
              "robot_model": config.get("robot_model"),
              "assumptions": "碰撞尺寸取自所附xacro；理想二维底盘、精确位姿、假定场地和速度标定；非实车验收"}
    return nav.route, rows, result


def write_artifacts(directory, config, route, rows, result):
    directory.mkdir(parents=True, exist_ok=True)
    name = result["scenario"]
    with (directory / (name+"_trajectory.csv")).open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(("time", "x", "y", "yaw", "vx", "vy", "waypoint", "state"))
        writer.writerows(rows)
    (directory / (name+"_report.json")).write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    x0, x1, y0, y1 = config["bounds"]

    def transform(p):
        return 50+(p[0]-x0)/(x1-x0)*600, 650-(p[1]-y0)/(y1-y0)*600

    def poly(points):
        return " ".join("{:.2f},{:.2f}".format(*transform(p)) for p in points)

    radius_scale = 600/(x1-x0)
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="750" height="780" viewBox="0 0 750 780">',
             '<rect width="750" height="780" fill="white"/>',
             '<rect x="50" y="50" width="600" height="600" fill="#f6f8fa" stroke="#333"/>',
             f'<polyline points="{poly([config["parking"]]+route)}" fill="none" stroke="#999" stroke-dasharray="6 4"/>',
             f'<polyline points="{poly([(r[1],r[2]) for r in rows])}" fill="none" stroke="#1769aa" stroke-width="2"/>']
    for key in ("lower_pole", "upper_pole"):
        cx, cy = transform(config[key])
        parts.append(f'<circle cx="{cx}" cy="{cy}" r="{config["pole_radius"]*radius_scale}" fill="#e89532"/>')
    cx, cy = transform(config["parking"])
    parts.append(f'<rect x="{cx-7}" y="{cy-7}" width="14" height="14" fill="#15803d"/>')
    if name == "obstacle":
        cx, cy = transform(route[8])
        parts.append(f'<circle cx="{cx}" cy="{cy}" r="{0.12*radius_scale}" fill="#dc2626" opacity="0.7"/>')
    parts.extend([f'<text x="50" y="690" font-size="18">{html.escape(name)} | complete={result["completed"]}</text>',
                  '<text x="50" y="720" font-size="16">Blue: actual trajectory | Grey: reference | Green: parking</text>',
                  f'<text x="50" y="750" font-size="16">Robot radius: {config["robot_radius"]} m; ideal dynamics, assumed field/calibration.</text>', '</svg>'])
    (directory / (name+"_route.svg")).write_text("\n".join(parts), encoding="utf-8")


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="二维离线导航仿真")
    parser.add_argument("--config", type=Path, default=root/"config.json")
    parser.add_argument("--scenario", choices=("normal", "obstacle", "localization_loss", "all"), default="all")
    parser.add_argument("--output", type=Path, default=root/"simulation_results")
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    scenarios = ("normal", "obstacle", "localization_loss") if args.scenario == "all" else (args.scenario,)
    failed = False
    playback = {"config": config, "scenarios": {}}
    for scenario in scenarios:
        route, rows, result = simulate(config, scenario)
        write_artifacts(args.output, config, route, rows, result)
        playback["scenarios"][scenario] = {"route": route, "rows": rows, "report": result}
        print(f"{scenario}: 完成={result['completed']}, 模拟用时={result['simulated_seconds']}s, "
              f"停车误差={result['parking_error_m']}m")
        print(" → ".join(t["state"] for t in result["transitions"]))
        failed |= not result["completed"]
    if args.scenario == "all":
        write_playback(args.output, playback)
    print("结果目录：", args.output.resolve())
    if failed:
        raise SystemExit(1)


def write_playback(directory, playback):
    playback["robot_model"] = model_for_config(playback["config"])
    template = (Path(__file__).resolve().parent / "viewer_template.html").read_text(encoding="utf-8")
    payload = json.dumps(playback, ensure_ascii=False).replace("<", "\\u003c")
    (directory/"playback.html").write_text(template.replace("__DATA__", payload), encoding="utf-8")


if __name__ == "__main__":
    main()
