"""二维全向底盘原型：有序路点、膨胀障碍栅格A*、定位失效停车。

输入位姿和障碍均来自外部定位/感知；不以串口反馈推算位置。
"""
import heapq
import math
from robot_model import model_for_config


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def validate_config(c):
    required = ("parking", "lower_pole", "upper_pole", "bounds", "body_from_command")
    for name in required:
        if name not in c:
            raise ValueError("缺少配置：" + name)
    for name in ("pole_radius", "robot_radius", "margin", "grid_resolution",
                 "waypoint_tolerance", "max_speed_mps", "tracking_gain",
                 "state_timeout", "control_limit"):
        if not math.isfinite(c[name]) or c[name] <= 0:
            raise ValueError("配置必须为有限正数：" + name)
    if c["control_limit"] > 0.2:
        raise ValueError("原型control_limit不能超过0.2")
    for name in ("parking", "lower_pole", "upper_pole"):
        if len(c[name]) != 2 or not all(math.isfinite(v) for v in c[name]):
            raise ValueError("坐标无效：" + name)
    bounds = c["bounds"]
    if len(bounds) != 4 or not all(math.isfinite(v) for v in bounds):
        raise ValueError("场地边界无效")
    if bounds[0] >= bounds[1] or bounds[2] >= bounds[3]:
        raise ValueError("场地边界顺序无效")
    m = c["body_from_command"]
    if len(m) != 2 or any(len(row) != 2 for row in m):
        raise ValueError("标定矩阵必须为2x2")
    if not all(math.isfinite(v) for row in m for v in row):
        raise ValueError("标定矩阵不是有限数")
    if abs(m[0][0]*m[1][1] - m[0][1]*m[1][0]) < 1e-8:
        raise ValueError("标定矩阵不可逆")
    model = model_for_config(c)
    if model and c["robot_radius"] < model["recommended_radius_m"]:
        raise ValueError(f"robot_radius小于模型要求的{model['recommended_radius_m']:.2f}米")


def make_route(c):
    """以两桩中点为交叉点；路线比例由现场桩坐标确定。

    从下桩左侧进入：下瓣逆时针270°→上瓣顺时针360°→下瓣逆时针90°→返回。
    入口和出口共用一条线。有序索引区分交叉点的两次经过。
    """
    low, high = c["lower_pole"], c["upper_pole"]
    r = distance(low, high) / 2
    if r <= c["pole_radius"] + c["robot_radius"] + c["margin"]:
        raise ValueError("两桩间距不足以按当前安全半径绕行")
    theta = math.atan2(high[1] - low[1], high[0] - low[0])
    entry_angle = theta + math.pi/2

    def point(center, angle):
        return (center[0]+r*math.cos(angle), center[1]+r*math.sin(angle))

    route = []
    entry = point(low, entry_angle)

    def line(a, b, label):
        n = max(1, math.ceil(distance(a, b) / 0.08))
        for i in range(1, n+1):
            route.append((a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n, label))

    def arc(center, begin, sweep, label):
        n = max(1, math.ceil(abs(sweep)*r/0.08))
        for i in range(1, n+1):
            x, y = point(center, begin+sweep*i/n)
            route.append((x, y, label))

    line(c["parking"], entry, "DEPART")
    arc(low, entry_angle, 1.5*math.pi, "LOWER_CCW")
    arc(high, theta+math.pi, -2*math.pi, "UPPER_CW")
    arc(low, theta, math.pi/2, "LOWER_EXIT")
    line(entry, c["parking"], "RETURN")
    return route


class Planner:
    def __init__(self, c):
        self.c = c
        self._indexed_obstacles = None
        self._bins = {}
        self._bin_size = max(0.25, c["grid_resolution"]*4)

    def nearby(self, a, b, obstacles):
        if obstacles is not self._indexed_obstacles:
            self._indexed_obstacles = obstacles
            self._bins = {}
            padding = self.c["robot_radius"]+self.c["margin"]
            for index, o in enumerate(obstacles):
                radius = o[2]+padding
                for x in range(math.floor((o[0]-radius)/self._bin_size), math.floor((o[0]+radius)/self._bin_size)+1):
                    for y in range(math.floor((o[1]-radius)/self._bin_size), math.floor((o[1]+radius)/self._bin_size)+1):
                        self._bins.setdefault((x,y), []).append(index)
        found = set()
        for x in range(math.floor(min(a[0],b[0])/self._bin_size), math.floor(max(a[0],b[0])/self._bin_size)+1):
            for y in range(math.floor(min(a[1],b[1])/self._bin_size), math.floor(max(a[1],b[1])/self._bin_size)+1):
                found.update(self._bins.get((x,y), ()))
        return (obstacles[index] for index in sorted(found))

    def free(self, p, obstacles):
        c = self.c
        padding = c["robot_radius"] + c["margin"]
        x0, x1, y0, y1 = c["bounds"]
        if not (x0+padding <= p[0] <= x1-padding and y0+padding <= p[1] <= y1-padding):
            return False
        return all(distance(p, o) > o[2]+padding for o in self.nearby(p, p, obstacles))

    def clear(self, a, b, obstacles):
        if not self.free(a, obstacles) or not self.free(b, obstacles):
            return False
        dx, dy = b[0]-a[0], b[1]-a[1]
        squared = dx*dx+dy*dy
        padding = self.c["robot_radius"]+self.c["margin"]
        for o in self.nearby(a, b, obstacles):
            t = max(0.0, min(1.0, ((o[0]-a[0])*dx+(o[1]-a[1])*dy)/squared)) if squared else 0.0
            closest = a[0]+t*dx, a[1]+t*dy
            if distance(closest, o) <= o[2]+padding:
                return False
        return True

    def plan(self, start, goal, obstacles):
        if self.clear(start, goal, obstacles):
            return [goal]
        if not self.free(start, obstacles) or not self.free(goal, obstacles):
            return []
        res = self.c["grid_resolution"]
        x0, x1, y0, y1 = self.c["bounds"]
        nx, ny = int((x1-x0)/res), int((y1-y0)/res)

        def cell(p):
            return round((p[0]-x0)/res), round((p[1]-y0)/res)

        def pos(k):
            return x0+k[0]*res, y0+k[1]*res

        first, last = cell(start), cell(goal)
        if not self.clear(start, pos(first), obstacles) or not self.clear(pos(last), goal, obstacles):
            return []
        queue = [(0.0, first)]
        costs, parent = {first: 0.0}, {}
        while queue:
            _, current = heapq.heappop(queue)
            if current == last:
                nodes = [last]
                while nodes[-1] != first:
                    nodes.append(parent[nodes[-1]])
                return [pos(k) for k in reversed(nodes)][1:] + [goal]
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt = current[0]+dx, current[1]+dy
                if not (0 <= nxt[0] <= nx and 0 <= nxt[1] <= ny):
                    continue
                cost = costs[current] + math.hypot(dx, dy)*res
                if cost < costs.get(nxt, math.inf) and self.clear(pos(current), pos(nxt), obstacles):
                    costs[nxt], parent[nxt] = cost, current
                    heapq.heappush(queue, (cost+distance(pos(nxt), goal), nxt))
        return []


class Navigator:
    def __init__(self, config):
        validate_config(config)
        self.c = config
        self.route = make_route(config)
        self.planner = Planner(config)
        self.index = 0
        self.state = "WAIT_START"
        self.started = False
        self.halted = False
        self.detour = []
        self.previous_obstacles = None
        poles = [(config[k][0], config[k][1], config["pole_radius"])
                 for k in ("lower_pole", "upper_pole")]
        points = [config["parking"]] + [p[:2] for p in self.route]
        if not all(self.planner.clear(a, b, poles) for a, b in zip(points, points[1:])):
            raise ValueError("参考路线穿过边界或桩；请修正现场配置")

    def start(self):
        if not self.halted:
            self.started = True

    def abort(self):
        self.halted, self.state = True, "ABORTED"

    def step(self, sample, now):
        if self.halted or not self.started:
            return (0.0, 0.0, 0.0)
        try:
            pose = sample["pose"]
            stamp, obstacle_stamp = sample["stamp"], sample["obstacle_stamp"]
            dynamic = sample["obstacles"]
            valid = (len(pose) == 3 and all(math.isfinite(v) for v in pose)
                     and math.isfinite(stamp) and math.isfinite(obstacle_stamp)
                     and sample["localized"] is True
                     and 0 <= now-stamp <= self.c["state_timeout"]
                     and 0 <= now-obstacle_stamp <= self.c["state_timeout"]
                     and all(len(o) == 3 and all(math.isfinite(v) for v in o)
                             and o[2] >= 0 for o in dynamic))
        except (KeyError, TypeError, ValueError):
            valid = False
        if not valid:
            self.detour = []
            self.state = "WAIT_SENSOR"
            return (0.0, 0.0, 0.0)
        obstacles = [tuple(o) for o in dynamic] + [
            (*self.c[k], self.c["pole_radius"]) for k in ("lower_pole", "upper_pole")]
        here = pose[:2]
        if not self.planner.free(here, obstacles):
            self.detour = []
            self.state = "BLOCKED"
            return (0.0, 0.0, 0.0)
        tolerance = self.c["waypoint_tolerance"]
        if self.index < len(self.route) and distance(here, self.route[self.index]) <= tolerance:
            self.index += 1
            self.detour = []
        if self.index == len(self.route):
            self.state = "COMPLETE"
            return (0.0, 0.0, 0.0)
        if obstacles != self.previous_obstacles:
            self.detour = []
            self.previous_obstacles = obstacles
        goal = self.route[self.index][:2]
        while self.detour and distance(here, self.detour[0]) <= tolerance:
            self.detour.pop(0)
        if not self.detour or not self.planner.clear(here, self.detour[0], obstacles):
            self.detour = self.planner.plan(here, goal, obstacles)
        if not self.detour:
            self.state = "BLOCKED"
            return (0.0, 0.0, 0.0)
        self.state = self.route[self.index][2]
        target = self.detour[0]
        dx, dy = target[0]-pose[0], target[1]-pose[1]
        norm = math.hypot(dx, dy)
        speed = min(self.c["max_speed_mps"], self.c["tracking_gain"]*norm)
        wx, wy = (dx/norm*speed, dy/norm*speed) if norm else (0, 0)
        yaw = pose[2]
        bx = math.cos(yaw)*wx + math.sin(yaw)*wy
        by = -math.sin(yaw)*wx + math.cos(yaw)*wy
        a, b = self.c["body_from_command"][0]
        d, e = self.c["body_from_command"][1]
        determinant = a*e-b*d
        vx, vy = (e*bx-b*by)/determinant, (-d*bx+a*by)/determinant
        scale = min(1.0, self.c["control_limit"]/max(abs(vx), abs(vy), 1e-9))
        return (vx*scale, vy*scale, 0.0)
