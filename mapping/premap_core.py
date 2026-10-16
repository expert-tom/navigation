"""ROS-independent, testable parts of the premapped pipeline. Units are metres.

No invented map->odom transform, localization flag, or command acknowledgement.
"""
import hashlib
import math
import threading
import time
from pathlib import Path

import numpy as np

from vehicle_core import quaternion_matrix


def rigid(translation, quaternion):
    result = np.eye(4)
    result[:3, :3] = quaternion_matrix(quaternion)
    if len(translation) != 3 or not np.isfinite(translation).all():
        raise ValueError("invalid rigid translation")
    result[:3, 3] = translation
    return result


def odometry_base(position, quaternion, body_from_base):
    """T_odom_base = T_odom_body @ T_body_base, NOT a frame-name replacement."""
    return rigid(position, quaternion) @ rigid(body_from_base["xyz"], body_from_base["xyzw"])


def xyzyaw(matrix):
    return (float(matrix[0, 3]), float(matrix[1, 3]), float(matrix[2, 3]),
            math.atan2(matrix[1, 0], matrix[0, 0]))


def decode_xyz(message, max_points):
    """Zero-copy strided PointCloud2 view, then bounded XYZ copy; includes row padding."""
    count = int(message.width)*int(message.height)
    if count <= 0 or count > max_points:
        raise ValueError("empty cloud or point count exceeds admission budget")
    if message.point_step <= 0 or message.row_step < message.width*message.point_step:
        raise ValueError("invalid PointCloud2 stride")
    if len(message.data) < message.row_step*message.height:
        raise ValueError("truncated PointCloud2 data")
    fields = {field.name: field for field in message.fields}
    formats, offsets = [], []
    endian = ">" if message.is_bigendian else "<"
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None or field.datatype not in (7, 8) or field.count != 1:
            raise ValueError("XYZ must be scalar float32/float64 fields")
        size = 4 if field.datatype == 7 else 8
        if field.offset < 0 or field.offset+size > message.point_step:
            raise ValueError("invalid XYZ field offset")
        formats.append(endian+("f4" if size == 4 else "f8"))
        offsets.append(field.offset)
    dtype = np.dtype(dict(names=["x", "y", "z"], formats=formats, offsets=offsets,
                         itemsize=message.point_step))
    view = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                      strides=(message.row_step, message.point_step))
    return np.column_stack([view[name].reshape(-1) for name in ("x", "y", "z")])


def vector_obstacles(points, translation, rotation, base_translation, base_rotation,
                     pose, settings, bounds):
    """Vectorized conservative voxel circles. Never silently truncate occupied cells."""
    if len(points) > settings["max_input_points"]:
        raise ValueError("cloud exceeds admission budget")
    points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    points = points[np.isfinite(points).all(axis=1)]
    # Tiny 3x3 transforms: einsum avoids BLAS thread-pool oversubscription
    # alongside ROS workers on a small NUC.
    base = np.einsum("ij,kj->ik",points,np.asarray(base_rotation))+base_translation
    keep = ((base[:, 2] >= settings["min_height"]) & (base[:, 2] <= settings["max_height"])
            & (np.hypot(base[:, 0], base[:, 1]) > settings["self_filter_radius"]))
    if not np.any(keep) and not settings["allow_empty"]:
        raise ValueError("no valid obstacle-height returns")
    world = np.einsum("ij,kj->ik",points[keep],np.asarray(rotation))+translation
    voxel = settings["voxel_size"]
    radius = voxel/math.sqrt(2)
    keep = ((np.hypot(world[:, 0]-pose[0], world[:, 1]-pose[1]) <= settings["max_range"])
            & (world[:, 0] >= bounds[0]-radius) & (world[:, 0] <= bounds[1]+radius)
            & (world[:, 1] >= bounds[2]-radius) & (world[:, 1] <= bounds[3]+radius))
    cells = np.floor(world[keep, :2]/voxel).astype(np.int64)
    # Bounded 2D -> 1D encoding, unique keys then decode. No random sampling,
    # no dropped occupied cell; much cheaper than structured row sorting.
    min_x=math.floor((bounds[0]-radius)/voxel)
    min_y=math.floor((bounds[2]-radius)/voxel)
    ny=math.floor((bounds[3]+radius)/voxel)-min_y+1
    keys=np.unique((cells[:,0]-min_x)*ny+(cells[:,1]-min_y))
    cells=np.column_stack((keys//ny+min_x,keys%ny+min_y))
    # Bound by physical field size, not a small room-dependent arbitrary count.
    budget = (math.ceil((bounds[1]-bounds[0]+4*radius)/voxel)+1) * \
             (math.ceil((bounds[3]-bounds[2]+4*radius)/voxel)+1)
    if len(cells) > min(budget, settings.get("max_grid_cells", 250000)):
        raise ValueError("voxel grid exceeds configured memory budget; no obstacles dropped")
    return [(float((x+.5)*voxel), float((y+.5)*voxel), radius) for x, y in cells]


class LatestWorker:
    """One in-flight job + one replaceable latest job; main executor never waits."""
    def __init__(self, function):
        self.function = function
        self.condition = threading.Condition()
        self.pending = self.result = None
        self.closed = False
        self.dropped = 0
        self.thread = threading.Thread(target=self._run, daemon=True, name="cloud-worker")
        self.thread.start()

    def submit(self, job):
        with self.condition:
            if self.closed:
                return
            if self.pending is not None:
                self.dropped += 1
            self.pending = job
            self.condition.notify()

    def take(self):
        with self.condition:
            result, self.result = self.result, None
            return result

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.closed or self.pending is not None)
                if self.closed:
                    return
                job, self.pending = self.pending, None
            try:
                result = (True, self.function(job))
            except Exception as exc:
                result = (False, str(exc))
            with self.condition:
                self.result = result

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()
        self.thread.join(timeout=1)


class OutputPermit:
    """Independent TX-side expiry, even if planning/logging/executor is blocked."""
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.deadline = -math.inf
        self.enabled = False

    def allow_until(self, deadline):
        self.deadline = deadline
        self.enabled = True

    def stop(self):
        self.enabled = False

    def __call__(self):
        return self.enabled and self.clock() < self.deadline


class LocalizationGate:
    """Covariance + scan/map endpoint consistency + fresh odometry + stable dwell."""
    def __init__(self, max_age=.8, dwell=1.0, min_match=.45):
        self.max_age, self.dwell, self.min_match = max_age, dwell, min_match
        self.pose_time = self.scan_time = self.odom_time = -math.inf
        self.cov_ok = False
        self.match = 0.0
        self.since = None

    def valid(self, now):
        good = (self.cov_ok and self.match >= self.min_match and
                all(0 <= now-stamp <= self.max_age
                    for stamp in (self.pose_time, self.scan_time, self.odom_time)))
        if not good:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now-self.since >= self.dwell


class OccupancyMatcher:
    def __init__(self, data, width, height, resolution, origin, yaw=0.0, tolerance=.2):
        if width <= 0 or height <= 0 or len(data) != width*height or resolution <= 0:
            raise ValueError("invalid match grid")
        self.data = np.asarray(data).reshape((height, width))
        self.resolution, self.origin, self.yaw = resolution, origin, yaw
        self.radius = int(math.ceil(tolerance/resolution))
        self.occupied = self.data >= 65
        # Precompute dilation once; do not scan all walls per scan endpoint.
        source = np.pad(self.occupied, self.radius)
        self.near = np.zeros_like(self.occupied)
        for dy in range(-self.radius, self.radius+1):
            for dx in range(-self.radius, self.radius+1):
                if dx*dx+dy*dy <= self.radius*self.radius:
                    self.near |= source[self.radius+dy:self.radius+dy+height,
                                        self.radius+dx:self.radius+dx+width]

    def fraction(self, world_points):
        if len(world_points) < 30:
            return 0.0
        points = np.asarray(world_points)[:, :2]-self.origin
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        cells = np.floor(points @ np.array([[c, -s], [s, c]])/self.resolution).astype(int)
        inside = ((cells[:, 0] >= 0) & (cells[:, 0] < self.data.shape[1])
                  & (cells[:, 1] >= 0) & (cells[:, 1] < self.data.shape[0]))
        hits = np.zeros(len(points), dtype=bool)
        hits[inside] = self.near[cells[inside, 1], cells[inside, 0]]
        return float(hits.mean())


def map_digest(data, width, height, resolution, origin):
    """Include geometry, not header time. Same published map must match manifest."""
    import json
    meta = json.dumps([width, height, round(resolution, 7),
                       [round(float(v), 7) for v in origin]], separators=(",", ":"))
    return hashlib.sha256(meta.encode()+np.asarray(data, dtype=np.int8).tobytes()).hexdigest()


def file_digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def mapping_parameters(config):
    # Humble slam_toolbox defaults map_name to ABSOLUTE /map. Namespace alone
    # does not isolate it. Keep this explicit and regression-tested.
    return dict(use_sim_time=False,odom_frame=config["odom_frame"],map_frame=config["map_frame"],
        base_frame=config["base_frame"],scan_topic="/race/scan",map_name="/race/map",mode="mapping",
        resolution=.05,minimum_travel_distance=.08,minimum_travel_heading=.08,minimum_time_interval=.1,
        map_update_interval=1.0,transform_publish_period=.05,restamp_tf=False)
