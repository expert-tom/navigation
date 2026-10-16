import copy
import itertools
import math
import time

from navigation import Navigator, distance


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


def freshness(message_seconds, ros_seconds, monotonic_seconds, timeout):
    """把"消息年龄"换算成本地单调节拍，并对时钟域不一致做降级。

    问题：FAST-LIO等雷达里程计可能用自己的时间基准（例如开机计时），
    与NUC系统时钟相差小时级。此时 age 会变成小时级，本该新鲜的位姿被永久
    判为过期。降级策略：年龄为负(消息来自未来)或超过"明显不合理"阈值
    (max(timeout, 10s))时，视为时钟域不一致，退化为"到达即新鲜"。

    返回 (单调节拍, 是否降级)。真正的时间戳<=0仍按原逻辑拒绝。
    """
    if not all(math.isfinite(v) for v in (message_seconds, ros_seconds, monotonic_seconds)):
        raise ValueError("invalid timestamp")
    if message_seconds <= 0:
        raise ValueError("stale or future message")
    age = ros_seconds-message_seconds
    if age < 0 or age > max(timeout, 10.0):
        return monotonic_seconds, True
    if age > timeout:
        raise ValueError("stale or future message")
    return monotonic_seconds-age, False


def limit_cloud(points, budget):
    """把点流限制在预算内，返回 (已限制的点序列, 是否发生截断)。

    原实现是"处理完才抛错"：要跑满十几个点、耗时一点几秒才报"超出预算"，
    而传感器回调只给0.3秒预算，于是必然再抛一次"处理太慢"并停车。
    这里在处理前截断，保证处理时间有上界；截断由调用方记录为可见的降级。
    """
    if budget is None or budget <= 0:
        return points, False
    try:
        total = len(points)
    except TypeError:
        kept = list(itertools.islice(points, int(budget)))
        return kept, True
    if total <= budget:
        return points, False
    stride = max(1, math.ceil(total / budget))
    kept = list(points)[::stride]
    return kept, True


def sampled_obstacles(points, translation, rotation, base_translation, base_rotation,
                      pose, settings, bounds, max_input_points=None, max_obstacles=None):
    """cloud_obstacles 的可降采样版本；返回 (障碍列表, 是否截断)。"""
    budget = settings.get("max_input_points") if max_input_points is None else max_input_points
    kept, truncated = limit_cloud(points, budget)
    obstacles = cloud_obstacles(kept, translation, rotation, base_translation,
                                base_rotation, pose, settings, bounds,
                                max_obstacles=max_obstacles)
    return obstacles, truncated


def cloud_obstacles(points, translation, rotation, base_translation, base_rotation, pose, settings, bounds, max_obstacles=None):
    voxel = settings["voxel_size"]
    limit = settings.get("max_obstacles") if max_obstacles is None else max_obstacles
    cells = set()
    accepted = 0
    total = 0
    radius = voxel/math.sqrt(2)
    for point in points:
        total += 1
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
        if len(cells) > limit:
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
    """导航运行时：输入帧 → 状态机 → 控制量。

    失效分三档（规则4.2要考核"人为干扰下成功避障"，所以不能一有异常就报废整轮）：
      latched    硬故障（stop服务/关断/串口故障）：本轮永久终止，只能重启进程；
      suspended  软故障（定位跳变/数据长时间断流）：暂停并归零，允许clear_fault后继续；
      瞬时过期    WAIT_POSE/WAIT_OBSTACLES等本来就自愈，不进入任何故障态。
    无论哪一档都不会自动恢复运动：必须显式clear_fault，且恢复后仍不重新校验起步位置
    （因为恢复是"继续本轮"，不是"开新一轮"）。
    """

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
        self.suspended = False
        self.fault_reason = None
        self.reason = "WAIT_START"
        self.previous_command = (0.0, 0.0, 0.0)
        self.command_stamp = clock()
        self.last_pose = None
        self.last_pose_stamp = None

    def update_pose(self, pose, stamp, freshness_stamp=None):
        """stamp是ROS域消息时间（排序与跳变判断用）；freshness_stamp是本地单调节拍。

        两者必须分开：拿ROS时间与time.monotonic()相减会得到小时级的"年龄"，
        于是本该新鲜的位姿被永久判为过期（WAIT_POSE）。缺省时退化为stamp，兼容原调用。
        """
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
                self.suspend("POSE_JUMP")
                return
        self.pose = list(pose)
        self.pose_stamp = stamp if freshness_stamp is None else freshness_stamp
        self.last_pose = list(pose)
        self.last_pose_stamp = stamp

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
        if self.suspended:
            return False, self.reason
        stream_timeout = self.settings.get("stream_stale_timeout")
        if stream_timeout is not None:
            for label, stamp in (("POSE", self.pose_stamp), ("OBSTACLES", self.obstacle_stamp),
                                 ("QUALITY", self.quality_stamp)):
                if math.isfinite(stamp) and now-stamp > stream_timeout:
                    self.suspend("STREAM_LOST_"+label)
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
        if not self.nav.planner.free(self.pose, self.obstacles+self.static_obstacles):
            return False, "START_OCCUPIED"
        self.started = True
        self.nav.start()
        return True, "started"

    def abort(self, reason="ABORTED"):
        """硬故障：本轮永久终止，只能重启进程（保持原有语义）。"""
        self.latched = True
        self.fault_reason = reason
        self.reason = reason
        self.nav.abort()
        self.previous_command = (0.0, 0.0, 0.0)

    def suspend(self, reason="SUSPENDED"):
        """软故障：暂停并归零，可用clear_fault恢复，不必重启进程。

        POSE_JUMP、数据断流走这里：比赛要考核"人为干扰下避障"，
        一次跳变就报废整轮是不合适的；但恢复必须显式，不会自动继续运动。
        """
        if self.latched:
            return
        self.suspended = True
        self.fault_reason = reason
        self.reason = reason
        self.previous_command = (0.0, 0.0, 0.0)
        self.command_stamp = self.clock()

    def clear_fault(self):
        """清除软故障并重新锚定当前位姿，返回 (是否成功, 说明)。

        重新锚定是必要的：跳变已经发生过，若不对齐last_pose，
        恢复后的第一帧会被再次判为跳变，形成"恢复即再锁"的死循环。
        硬故障(latched)不允许清除，与原行为一致。
        """
        if self.latched:
            return False, "hard fault cannot be cleared; restart process"
        if not self.suspended:
            return False, "no recoverable fault"
        self.suspended = False
        cleared, self.fault_reason = self.fault_reason, None
        if self.pose is not None:
            self.last_pose = list(self.pose)
        self.last_pose_stamp = None
        self.previous_command = (0.0, 0.0, 0.0)
        self.command_stamp = self.clock()
        self.reason = "RECOVERED_FROM_" + str(cleared)
        return True, self.reason

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
