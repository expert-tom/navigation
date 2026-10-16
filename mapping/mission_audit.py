"""Audit the measured trajectory, independently of waypoint completion.

This is a software acceptance check, not a replacement for referee judgement.
"""
import math


STAGES = ("DEPART", "LOWER_CCW", "UPPER_CW", "LOWER_EXIT", "RETURN")


class EightAudit:
    def __init__(self, config):
        self.config = config
        self.started_at = None
        self.last_time = None
        self.last = None
        self.length = 0.0
        self.angles = [0.0, 0.0]
        self.stages = []
        self.violation = None

    def begin(self, pose, now):
        self.started_at = self.last_time = now
        self.last = tuple(pose[:2])

    def observe(self, pose, now):
        if self.last is None or now <= self.last_time:
            return
        point = tuple(pose[:2])
        if not all(math.isfinite(v) for v in (*point, now)):
            raise ValueError("nonfinite trajectory sample")
        a = self.last
        dx, dy = point[0]-a[0], point[1]-a[1]
        squared = dx*dx+dy*dy
        padding = self.config["robot_radius"]+self.config["margin"]
        x0, x1, y0, y1 = self.config["bounds"]
        for p in (a, point):
            if not (x0+padding <= p[0] <= x1-padding and y0+padding <= p[1] <= y1-padding):
                self.violation = "FIELD_BOUNDARY"
        circles = self.config.get("fixed_obstacles", [
            (*self.config[key], self.config["pole_radius"]) for key in ("lower_pole", "upper_pole")])
        for index, circle in enumerate(circles):
            center = circle[:2]
            t = max(0.0, min(1.0, ((center[0]-a[0])*dx+(center[1]-a[1])*dy)/squared)) if squared else 0.0
            clearance = math.hypot(a[0]+t*dx-center[0], a[1]+t*dy-center[1])
            if clearance <= circle[2]+padding:
                self.violation = "POLE_CLEARANCE"
            if "mission_waypoints" not in self.config and index < 2:
                before = math.atan2(a[1]-center[1], a[0]-center[0])
                after = math.atan2(point[1]-center[1], point[0]-center[0])
                self.angles[index] += (after-before+math.pi) % (2*math.pi)-math.pi
        self.length += math.sqrt(squared)
        self.last, self.last_time = point, now

    def stage(self, name):
        if name in STAGES and (not self.stages or self.stages[-1] != name):
            self.stages.append(name)

    def report(self, now):
        turns = [angle/(2*math.pi) for angle in self.angles]
        parking = self.config["parking"]
        error = None if self.last is None else math.hypot(self.last[0]-parking[0], self.last[1]-parking[1])
        # Near-closed trajectories have a small angular residual at the parking tolerance.
        winding_ok = abs(turns[0]-1.0) <= 0.1 and abs(turns[1]+1.0) <= 0.1
        stages_ok = self.stages == list(STAGES)
        returned = error is not None and error <= self.config["waypoint_tolerance"]
        return {
            "elapsed_seconds": None if self.started_at is None else max(0.0, now-self.started_at),
            "travelled_m": self.length, "lower_turns_ccw": turns[0], "upper_turns_ccw": turns[1],
            "stages": list(self.stages), "parking_error_m": error,
            "winding_ok": winding_ok, "stages_ok": stages_ok, "returned": returned,
            "violation": self.violation,
            "verified": self.started_at is not None and winding_ok and stages_ok and returned and self.violation is None,
        }
