import copy
import math
import time

from navigation import Navigator, distance, fixed_obstacles
from mission_audit import EightAudit


def quaternion_matrix(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError("invalid quaternion")
    length = math.sqrt(sum(v*v for v in q))
    if length < 1e-8 or abs(length-1) > 0.05:
        raise ValueError("quaternion is not normalized")
    x, y, z, w = [v/length for v in q]
    return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def transform_point(point, translation, rotation):
    return tuple(sum(rotation[i][j]*point[j] for j in range(3))+translation[i] for i in range(3))


def yaw_from_quaternion(q):
    matrix = quaternion_matrix(q)
    return math.atan2(matrix[1][0], matrix[0][0])


def message_stamp(message_seconds, ros_seconds, monotonic_seconds, timeout):
    age = ros_seconds-message_seconds
    if not all(math.isfinite(v) for v in (message_seconds, ros_seconds, monotonic_seconds)):
        raise ValueError("invalid timestamp")
    if message_seconds <= 0 or not 0 <= age <= timeout:
        raise ValueError("stale or future message")
    return monotonic_seconds-age


def cloud_obstacles(points, translation, rotation, base_translation, base_rotation, pose, settings, bounds):
    voxel = settings["voxel_size"]
    cells = set()
    accepted = 0
    total = 0
    radius = voxel/math.sqrt(2)
    for point in points:
        total += 1
        if total > settings["max_input_points"]:
            raise ValueError("point cloud exceeds processing budget")
        point = tuple(float(v) for v in point[:3])
        if not all(math.isfinite(v) for v in point):
            continue
        base = transform_point(point, base_translation, base_rotation)
        if not settings["min_height"] <= base[2] <= settings["max_height"]:
            continue
        if math.hypot(base[0], base[1]) <= settings["self_filter_radius"]:
            continue
        accepted += 1
        world = transform_point(point, translation, rotation)
        if distance(world, pose) > settings["max_range"]:
            continue
        if not (bounds[0]-radius <= world[0] <= bounds[1]+radius and bounds[2]-radius <= world[1] <= bounds[3]+radius):
            continue
        cells.add((math.floor(world[0]/voxel), math.floor(world[1]/voxel)))
        if len(cells) > settings["max_obstacles"]:
            raise ValueError("too many obstacles; stopped instead of dropping points")
    if accepted == 0 and not settings["allow_empty"]:
        raise ValueError("no valid obstacle-height returns; check height filtering and TF")
    return [((x+0.5)*voxel, (y+0.5)*voxel, radius) for x, y in sorted(cells)]


def grid_obstacles(data, width, height, resolution, origin, yaw, threshold, unknown_blocked, bounds, max_cells):
    if width <= 0 or height <= 0 or len(data) != width*height or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("invalid occupancy grid")
    c, s = math.cos(yaw), math.sin(yaw)
    for x in (bounds[0], bounds[1]):
        for y in (bounds[2], bounds[3]):
            dx, dy = x-origin[0], y-origin[1]
            local_x, local_y = c*dx+s*dy, -s*dx+c*dy
            if not (-1e-8 <= local_x <= width*resolution+1e-8 and -1e-8 <= local_y <= height*resolution+1e-8):
                raise ValueError("map does not cover the configured field bounds")
    radius = resolution/math.sqrt(2)
    obstacles = []
    for index, value in enumerate(data):
        if value < -1 or value > 100:
            raise ValueError("invalid map occupancy")
        if value < threshold and not (value == -1 and unknown_blocked):
            continue
        x, y = (index%width+0.5)*resolution, (index//width+0.5)*resolution
        world = (origin[0]+c*x-s*y, origin[1]+s*x+c*y, radius)
        if bounds[0]-radius <= world[0] <= bounds[1]+radius and bounds[2]-radius <= world[1] <= bounds[3]+radius:
            obstacles.append(world)
            if len(obstacles) > max_cells:
                raise ValueError("map exceeds processing budget")
    return obstacles


class VehicleRuntime:
    def __init__(self, navigation_config, settings, clock=time.monotonic):
        self.c = copy.deepcopy(navigation_config)
        self.c.update(settings["navigation_overrides"])
        self.nav = Navigator(self.c)
        self.settings = settings
        self.clock = clock
        self.pose = None
        self.pose_stamp = -math.inf
        self.obstacles = []
        self.obstacle_stamp = -math.inf
        self.static_obstacles = []
        self.map_ready = not settings["map_required"]
        self.localized = False
        self.quality_stamp = -math.inf
        self.started = False
        self.latched = False
        self.reason = "WAIT_START"
        self.previous_command = (0.0, 0.0, 0.0)
        self.command_stamp = clock()
        self.last_pose = None
        self.last_pose_stamp = None
        self.task_kind = "waypoints" if "mission_waypoints" in self.c else "eight"
        self.audit = EightAudit(self.c)
        self.completed_at = None

    def update_pose(self, pose, stamp):
        if len(pose) != 3 or not all(math.isfinite(v) for v in pose) or not math.isfinite(stamp):
            raise ValueError("invalid pose")
        if self.last_pose_stamp is not None:
            if stamp < self.last_pose_stamp:
                raise ValueError("out-of-order pose")
            if stamp == self.last_pose_stamp:
                return
            dt = stamp-self.last_pose_stamp
            delta_yaw = abs((pose[2]-self.last_pose[2]+math.pi)%(2*math.pi)-math.pi)
            if self.started and (distance(pose, self.last_pose) > self.settings["pose_jump_distance"]+self.settings["max_pose_speed"]*dt or delta_yaw > self.settings["pose_jump_yaw"]):
                self.abort("POSE_JUMP")
                return
        self.pose = list(pose)
        self.pose_stamp = stamp
        self.last_pose = list(pose)
        self.last_pose_stamp = stamp
        if self.started:
            self.audit.observe(pose, stamp)
            if self.audit.violation:
                self.abort("TRAJECTORY_"+self.audit.violation)

    def update_quality(self, valid, stamp):
        self.localized = valid is True
        self.quality_stamp = stamp

    def update_obstacles(self, obstacles, stamp):
        if not math.isfinite(stamp) or any(len(o) != 3 or not all(math.isfinite(v) for v in o) or o[2] < 0 for o in obstacles):
            raise ValueError("invalid obstacles")
        if stamp < self.obstacle_stamp:
            raise ValueError("out-of-order obstacle data")
        self.obstacles = obstacles
        self.obstacle_stamp = stamp

    def invalidate_sensor(self, reason):
        self.obstacle_stamp = -math.inf
        self.reason = reason

    def ready(self, now=None):
        now = self.clock() if now is None else now
        if self.latched:
            return False, self.reason
        if self.pose is None or not 0 <= now-self.pose_stamp <= self.c["state_timeout"]:
            return False, "WAIT_POSE"
        if not self.localized or not 0 <= now-self.quality_stamp <= self.settings["quality_timeout"]:
            return False, "WAIT_LOCALIZATION_QUALITY"
        if not 0 <= now-self.obstacle_stamp <= self.c["state_timeout"]:
            return False, "WAIT_OBSTACLES"
        if not self.map_ready:
            return False, "WAIT_MAP"
        return True, "READY"

    def start(self, armed=False):
        if self.started:
            return False, "already started; restart process for a new round"
        if armed:
            confirmations = ("field_confirmed", "calibration_confirmed", "lidar_extrinsics_confirmed", "emergency_stop_confirmed")
            missing = [key for key in confirmations if self.settings.get(key) is not True]
            if missing:
                return False, "unconfirmed: "+", ".join(missing)
        valid, reason = self.ready()
        if not valid:
            return False, reason
        if distance(self.pose, self.c["parking"]) > self.settings["start_position_tolerance"]:
            return False, "NOT_AT_PARKING"
        poles = fixed_obstacles(self.c)
        if not self.nav.planner.free(self.pose, self.obstacles+self.static_obstacles+poles):
            return False, "START_OCCUPIED"
        self.started = True
        self.audit.begin(self.pose, self.clock())
        self.nav.start()
        return True, "started"

    def abort(self, reason="ABORTED"):
        self.latched = True
        self.reason = reason
        self.nav.abort()
        self.previous_command = (0.0, 0.0, 0.0)

    def step(self):
        now = self.clock()
        valid, reason = self.ready(now)
        if not self.started or not valid:
            if not self.latched:
                self.reason = reason if not valid else "WAIT_START"
            self.previous_command = (0.0, 0.0, 0.0)
            self.command_stamp = now
            return self.previous_command
        sample = {"pose": self.pose, "stamp": self.pose_stamp,
                  "localized": self.localized, "obstacles": self.obstacles+self.static_obstacles,
                  "obstacle_stamp": self.obstacle_stamp}
        command = self.nav.step(sample, now)
        if self.task_kind == "eight":
            self.audit.stage(self.nav.state)
            if self.nav.state == "COMPLETE" and not self.audit.report(now)["verified"]:
                self.abort("ROUTE_NOT_VERIFIED")
                return (0.0, 0.0, 0.0)
        if self.nav.state == "COMPLETE" and self.completed_at is None:
            self.completed_at = now
        self.reason = self.nav.state
        if command == (0.0, 0.0, 0.0):
            self.previous_command = command
            self.command_stamp = now
            return command
        delta = self.settings["command_slew_rate"]*max(0, min(now-self.command_stamp, 0.1))
        differences = [command[i]-self.previous_command[i] for i in range(2)]
        scale = min(1.0, delta/max(max(abs(v) for v in differences), 1e-9))
        output = tuple(self.previous_command[i]+differences[i]*scale for i in range(2))+(0.0,)
        self.previous_command = output
        self.command_stamp = now
        return output

    def mission_report(self):
        now = self.clock() if self.completed_at is None else self.completed_at
        result = self.audit.report(now)
        result.update(task_kind=self.task_kind, complete=self.nav.state == "COMPLETE",
                      state=self.reason, waypoint=self.nav.index, route_points=len(self.nav.route))
        if self.task_kind != "eight":
            result["verified"] = False
            result["lower_turns_ccw"] = result["upper_turns_ccw"] = None
            result["waypoints_completed"] = self.nav.index
        return result
