"""Measured waypoint tasks; no ROS imports and no hardware access."""
import copy
import json
import math
from pathlib import Path

from navigation import validate_config


def apply_task(config, goal=None, task_path=None, map_frame=None, require_confirmed=False):
    if goal is not None and task_path is not None:
        raise ValueError("choose either goal or waypoint task")
    result = copy.deepcopy(config)
    if goal is not None:
        if len(goal) != 2 or not all(math.isfinite(v) for v in goal):
            raise ValueError("goal must contain finite map coordinates")
        result["mission_waypoints"] = [list(goal)]
        result["fixed_obstacles"] = []
    if task_path is not None:
        task = json.loads(Path(task_path).read_text(encoding="utf-8"))
        required = ("map_frame", "parking", "bounds", "waypoints", "fixed_obstacles")
        missing = [key for key in required if key not in task]
        if missing:
            raise ValueError("missing task measurements: "+", ".join(missing))
        if not isinstance(task["map_frame"], str) or not task["map_frame"]:
            raise ValueError("task map_frame must be a nonempty string")
        if map_frame is not None and task["map_frame"] != map_frame:
            raise ValueError("task map_frame does not match localization/map settings")
        if require_confirmed and task.get("measurements_confirmed") is not True:
            raise ValueError("task measurements must be confirmed before armed operation")
        if not isinstance(task["fixed_obstacles"], list):
            raise ValueError("fixed_obstacles must be a list; [] uses only map and live obstacles")
        for key in ("parking", "bounds", "fixed_obstacles"):
            result[key] = task[key]
        result["mission_waypoints"] = task["waypoints"]
        result["task_map_frame"] = task["map_frame"]
        result["task_measurements_confirmed"] = task.get("measurements_confirmed") is True
    validate_config(result)
    return result
