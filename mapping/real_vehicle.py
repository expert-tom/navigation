import argparse
import json
import math
import os
from pathlib import Path
import time

from safety_control import SafetyController
from task_config import apply_task
from vehicle_core import VehicleRuntime, cloud_obstacles, grid_obstacles, message_stamp, quaternion_matrix, yaw_from_quaternion


ROOT = Path(__file__).resolve().parent


def load_settings(path):
    settings = json.loads(Path(path).read_text(encoding="utf-8"))
    if settings["pose_mode"] not in ("tf", "odometry", "pose"):
        raise ValueError("pose_mode must be tf, odometry or pose")
    if settings["sensor_type"] not in ("pointcloud2", "livox_custom", "scan"):
        raise ValueError("sensor_type must be pointcloud2, livox_custom or scan")
    for key in ("control_hz", "quality_timeout", "command_timeout", "command_slew_rate", "start_position_tolerance", "pose_jump_distance", "max_pose_speed", "pose_jump_yaw"):
        if not math.isfinite(settings[key]) or settings[key] <= 0:
            raise ValueError("invalid setting: "+key)
    if settings["control_hz"] > 100 or settings["command_timeout"] > 0.5:
        raise ValueError("control_hz <= 100 and command_timeout <= 0.5 required")
    cloud = settings["cloud"]
    for key in ("voxel_size", "max_range", "max_input_points", "max_obstacles"):
        if not math.isfinite(cloud[key]) or cloud[key] <= 0:
            raise ValueError("invalid cloud setting: "+key)
    if not all(math.isfinite(cloud[k]) for k in ("min_height", "max_height", "self_filter_radius")):
        raise ValueError("invalid cloud height or footprint")
    if cloud["min_height"] >= cloud["max_height"] or cloud["self_filter_radius"] < 0:
        raise ValueError("invalid cloud filter")
    return settings


def create_node(settings, nav_config, args):
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as PathMessage
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Bool, Float32MultiArray, String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener

    class NavigationNode(Node):
        def __init__(self):
            super().__init__("srm_navigation")
            if self.get_parameter("use_sim_time").value and args.connect_serial:
                raise ValueError("serial connection is forbidden with use_sim_time")
            self.runtime = VehicleRuntime(nav_config, settings)
            from premap_core import LatestWorker, OutputPermit
            self.permit = OutputPermit()
            self.controller = SafetyController(limit=self.runtime.c["control_limit"], timeout=settings["command_timeout"],
                commissioned_limit=self.runtime.c.get("commissioned_control_limit", .2))
            self.link = None
            self.log = None
            self.last_error = ""
            self.last_sensor_message = -math.inf
            self.last_quality_error = ""
            self.completion_recorded = False
            self.buffer = Buffer(cache_time=Duration(seconds=5))
            self.listener = TransformListener(self.buffer, self)
            self.cloud_worker = LatestWorker(self.process_sensor)
            self.map_worker = LatestWorker(self.process_map)
            self.preview = self.create_publisher(Float32MultiArray, "/navigation/command_preview", 10)
            self.status = self.create_publisher(String, "/navigation/status", 10)
            self.route_pub = self.create_publisher(PathMessage, "/navigation/reference_path", 10)
            self.create_subscription(Bool, settings["quality_topic"], self.quality_callback, 10)
            if settings["pose_mode"] == "odometry":
                self.create_subscription(Odometry, settings["pose_topic"], self.pose_callback, qos_profile_sensor_data)
            elif settings["pose_mode"] == "pose":
                self.create_subscription(PoseWithCovarianceStamped, settings["pose_topic"], self.pose_callback, qos_profile_sensor_data)
            sensor_type = settings["sensor_type"]
            if sensor_type == "livox_custom":
                from livox_ros_driver2.msg import CustomMsg
                message_type = CustomMsg
            else:
                message_type = PointCloud2 if sensor_type == "pointcloud2" else LaserScan
            self.create_subscription(message_type, settings["sensor_topic"], self.sensor_callback, qos_profile_sensor_data)
            map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, settings["map_topic"], self.map_callback, map_qos)
            self.create_service(Trigger, "~/start", self.start_callback)
            self.create_service(Trigger, "~/stop", self.stop_callback)
            args.log_dir.mkdir(parents=True, exist_ok=True)
            name = f"run_{time.strftime('%Y%m%d_%H%M%S')}_{time.time_ns()}.jsonl"
            self.log = (args.log_dir/name).open("x", encoding="utf-8", buffering=1)
            self.record({"event": "startup", "armed": args.arm, "serial": args.connect_serial,
                         "settings": settings, "navigation_config": nav_config})
            if args.connect_serial:
                from serial_link import SerialLink
                device = args.device or settings["serial_device"]
                if not device:
                    self.log.close()
                    raise ValueError("set serial_device or pass --device; no automatic port selection")
                try:
                    from check_serial_port import require_preflight
                    report = require_preflight(device)
                    self.record({"event": "serial_preflight", "report": report})
                    self.link = SerialLink(device, self.controller,
                                           zero_read_grace=settings.get("serial_zero_read_grace", 1.0),
                                           motion_permit=self.permit,dtr=settings.get("serial_dtr",False),
                                           rts=settings.get("serial_rts",False))
                    self.link.start()
                except Exception:
                    if self.link:
                        self.link.close()
                    self.log.close()
                    raise
            self.create_timer(1/settings["control_hz"], self.tick)
            self.create_timer(1.0, self.publish_status)
            self.get_logger().info("ready for input; dry-run="+str(not args.arm)+"; start service: /srm_navigation/start")

        def record(self, row):
            row["monotonic"] = time.monotonic()
            self.log.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+"\n")

        def stamp(self, message):
            seconds = message.header.stamp.sec+message.header.stamp.nanosec/1e9
            return message_stamp(seconds, self.get_clock().now().nanoseconds/1e9,
                                 time.monotonic(), self.runtime.c["state_timeout"])

        def tf_parts(self, target, source, stamp):
            if not source:
                raise ValueError("empty frame_id")
            tf = self.buffer.lookup_transform(target, source, stamp, timeout=Duration(seconds=.08))
            t, q = tf.transform.translation, tf.transform.rotation
            return (t.x, t.y, t.z), quaternion_matrix((q.x, q.y, q.z, q.w))

        def quality_callback(self, message):
            self.runtime.update_quality(message.data is True, time.monotonic())
            if message.data is not True:
                self.permit.stop()
                self.controller.stop()

        def pose_callback(self, message):
            try:
                if message.header.frame_id != settings["map_frame"]:
                    raise ValueError("pose frame must equal configured map_frame")
                if settings["pose_mode"] == "odometry" and message.child_frame_id != settings["base_frame"]:
                    raise ValueError("odometry child frame is not base_frame; use TF pose mode")
                p, q = message.pose.pose.position, message.pose.pose.orientation
                self.runtime.update_pose((p.x, p.y, yaw_from_quaternion((q.x, q.y, q.z, q.w))), self.stamp(message))
            except Exception as exc:
                self.runtime.pose_stamp = -math.inf
                self.last_error = "pose: "+str(exc)
                self.controller.stop()

        def tf_pose(self):
            tf = self.buffer.lookup_transform(settings["map_frame"], settings["base_frame"], Time(), timeout=Duration(seconds=0))
            stamp = message_stamp(tf.header.stamp.sec+tf.header.stamp.nanosec/1e9,
                                  self.get_clock().now().nanoseconds/1e9, time.monotonic(), self.runtime.c["state_timeout"])
            t, q = tf.transform.translation, tf.transform.rotation
            self.runtime.update_pose((t.x, t.y, yaw_from_quaternion((q.x, q.y, q.z, q.w))), stamp)

        def sensor_callback(self, message):
            # Admission before any point iteration/allocation. Processing and TF
            # wait happen in a bounded latest-only worker, not the ROS executor.
            if settings["sensor_type"] == "pointcloud2" and message.width*message.height > settings["cloud"]["max_input_points"]:
                self.runtime.invalidate_sensor("CLOUD_OVERLOAD")
                self.permit.stop()
                self.controller.stop()
                self.last_error = "cloud admission rejected: too many input points"
                return
            self.cloud_worker.submit((message, self.runtime.pose))

        def process_sensor(self, job):
            message, pose = job
            try:
                monotonic_stamp = self.stamp(message)
                if monotonic_stamp <= self.last_sensor_message:
                    raise ValueError("duplicate or out-of-order sensor message")
                stamp = Time.from_msg(message.header.stamp)
                translation, rotation = self.tf_parts(settings["map_frame"], message.header.frame_id, stamp)
                base_translation, base_rotation = self.tf_parts(settings["base_frame"], message.header.frame_id, stamp)
                if pose is None:
                    raise ValueError("waiting for base pose")
                if settings["sensor_type"] == "pointcloud2":
                    from premap_core import decode_xyz, vector_obstacles
                    points = decode_xyz(message, settings["cloud"]["max_input_points"])
                elif settings["sensor_type"] == "livox_custom":
                    points = ((p.x, p.y, p.z) for p in message.points)
                else:
                    points = ((r*math.cos(message.angle_min+i*message.angle_increment), r*math.sin(message.angle_min+i*message.angle_increment), 0.0)
                              for i, r in enumerate(message.ranges) if math.isfinite(r) and message.range_min <= r < message.range_max)
                function = vector_obstacles if settings["sensor_type"] == "pointcloud2" else cloud_obstacles
                obstacles = function(points, translation, rotation, base_translation, base_rotation,
                                     pose, settings["cloud"], self.runtime.c["bounds"])
                if time.monotonic()-monotonic_stamp > self.runtime.c["state_timeout"]:
                    raise ValueError("sensor processing too slow")
                return obstacles, monotonic_stamp
            except Exception as exc:
                raise ValueError("sensor: "+str(exc)) from exc

        def map_callback(self, message):
            self.permit.stop()
            self.controller.stop()
            self.runtime.map_ready = False
            self.map_worker.submit(message)

        def process_map(self, message):
            try:
                if message.header.frame_id != settings["map_frame"]:
                    raise ValueError("map frame mismatch")
                info = message.info
                q = info.origin.orientation
                matrix = quaternion_matrix((q.x, q.y, q.z, q.w))
                if abs(matrix[2][0])+abs(matrix[2][1]) > 1e-5:
                    raise ValueError("occupancy map must be planar")
                if settings.get("expected_map_digest"):
                    from premap_core import map_digest
                    actual = map_digest(message.data, info.width, info.height, info.resolution,
                        (info.origin.position.x, info.origin.position.y, yaw_from_quaternion((q.x,q.y,q.z,q.w))))
                    if actual != settings["expected_map_digest"]:
                        raise ValueError("published map differs from selected map bundle")
                obstacles = grid_obstacles(message.data, info.width, info.height, info.resolution,
                    (info.origin.position.x, info.origin.position.y), yaw_from_quaternion((q.x, q.y, q.z, q.w)),
                    settings["occupancy_threshold"], settings["unknown_blocked"], self.runtime.c["bounds"], settings["map_max_cells"])
                return obstacles
            except Exception as exc:
                raise ValueError("map: "+str(exc)) from exc

        def start_callback(self, request, response):
            if self.link:
                try:
                    self.link.check(for_motion=args.arm)
                    age = self.link.stats()["last_feedback_age_s"]
                    if args.arm and (age is None or age > settings.get("feedback_timeout", 1.0)):
                        raise RuntimeError("WAIT_SERIAL_FEEDBACK: need fresh CRC-valid feedback; not an execution ACK")
                except RuntimeError as exc:
                    response.success, response.message = False, str(exc)
                    self.record({"event": "start", "success": False, "message": response.message})
                    return response
            response.success, response.message = self.runtime.start(armed=args.arm)
            self.record({"event": "start", "success": response.success, "message": response.message})
            return response

        def stop_callback(self, request, response):
            self.permit.stop()
            self.runtime.abort()
            self.controller.stop()
            response.success, response.message = True, "aborted; restart process before a new round"
            self.record({"event": "stop"})
            return response

        def tick(self):
            tick_started = time.monotonic()
            try:
                result = self.map_worker.take()
                if result:
                    if result[0]:
                        self.runtime.static_obstacles = result[1]
                        self.runtime.map_ready = True
                        self.record({"event": "map", "obstacles": len(result[1])})
                    else:
                        self.runtime.map_ready = False
                        self.last_error = result[1]
                result = self.cloud_worker.take()
                if result:
                    if result[0]:
                        obstacles, stamp = result[1]
                        self.runtime.update_obstacles(obstacles, stamp)
                        self.last_sensor_message = stamp
                    else:
                        # A missed TF lookup does not erase a still-fresh valid
                        # sample. Its original deadline is NEVER refreshed.
                        self.last_error = result[1]
                if self.link:
                    self.link.check(for_motion=args.arm)
                    age = self.link.stats()["last_feedback_age_s"]
                    if args.arm and (age is None or age > settings.get("feedback_timeout", 1.0)):
                        self.permit.stop()
                        self.controller.stop()
                        if self.runtime.started:
                            self.runtime.abort("SERIAL_FEEDBACK_LOST")
                        return
                if settings["pose_mode"] == "tf":
                    try:
                        self.tf_pose()
                    except Exception as exc:
                        self.last_error = "TF: "+str(exc)
                command = self.runtime.step()
                now = time.monotonic()
                if not self.runtime.ready()[0] or now-tick_started > settings["command_timeout"]:
                    command = (0.0, 0.0, 0.0)
                    self.permit.stop()
                elif args.arm and self.runtime.started and not self.runtime.latched and self.runtime.nav.state != "COMPLETE":
                    deadline = min(self.runtime.pose_stamp+self.runtime.c["state_timeout"],
                        self.runtime.obstacle_stamp+self.runtime.c["state_timeout"],
                        self.runtime.quality_stamp+settings["quality_timeout"],
                        tick_started+settings["command_timeout"])
                    if self.link and self.link.last_feedback_at is not None:
                        deadline = min(deadline,self.link.last_feedback_at+settings.get("feedback_timeout",1.0))
                    self.permit.allow_until(deadline)
                else:
                    self.permit.stop()
                self.controller.update(*(command if args.arm else (0.0, 0.0, 0.0)))
                preview = Float32MultiArray()
                preview.data = list(command)
                self.preview.publish(preview)
                self.record({"event": "control", "state": self.runtime.reason, "waypoint": self.runtime.nav.index,
                             "pose": self.runtime.pose, "preview": command,
                             "controller_command": [self.controller.output().vx, self.controller.output().vy, 0.0],
                             "serial_connected": self.link is not None})
                if self.runtime.nav.state == "COMPLETE" and not self.completion_recorded:
                    self.record({"event": "mission_complete", "report": self.runtime.mission_report()})
                    self.completion_recorded = True
                if self.link:
                    while self.link.messages:
                        self.record({"event": "feedback", "data": self.link.messages.popleft()})
            except Exception as exc:
                self.permit.stop()
                self.controller.stop()
                self.last_error = "control: "+str(exc)
                if self.link and (self.link.error or self.link.motion_inhibited):
                    self.runtime.abort("SERIAL_ERROR")

        def publish_status(self):
            ready, reason = self.runtime.ready()
            message = String()
            message.data = json.dumps({"ready": ready, "reason": reason, "state": self.runtime.reason,
                "armed": args.arm, "serial": args.connect_serial, "error": self.last_error,
                "obstacles": len(self.runtime.obstacles), "map_obstacles": len(self.runtime.static_obstacles),
                "serial_stats": self.link.stats() if self.link else None,
                "mission": self.runtime.mission_report()}, ensure_ascii=False)
            if self.link:
                self.record({"event": "serial_stats", "data": self.link.stats()})
            self.status.publish(message)
            self.get_logger().info(message.data)
            route = PathMessage()
            route.header.frame_id = settings["map_frame"]
            route.header.stamp = self.get_clock().now().to_msg()
            from geometry_msgs.msg import PoseStamped
            for x, y, label in self.runtime.nav.route:
                pose = PoseStamped()
                pose.header = route.header
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                pose.pose.orientation.w = 1.0
                route.poses.append(pose)
            self.route_pub.publish(route)

        def close(self):
            self.permit.stop()
            self.runtime.abort("SHUTDOWN")
            self.controller.stop()
            try:
                if self.link:
                    self.link.close()
                    self.record({"event": "serial_shutdown", "data": self.link.stats()})
                if self.log and not self.log.closed:
                    self.record({"event": "shutdown"})
            finally:
                self.cloud_worker.close()
                self.map_worker.close()
                if self.log and not self.log.closed:
                    self.log.close()

    return NavigationNode()


def main():
    parser = argparse.ArgumentParser(description="ROS2 real vehicle navigation; default has no serial output")
    parser.add_argument("--settings", type=Path, default=ROOT/"config/real_vehicle.json")
    parser.add_argument("--config", type=Path, default=ROOT/"config.json")
    parser.add_argument("--connect-serial", action="store_true")
    parser.add_argument("--device")
    parser.add_argument("--arm", action="store_true")
    task = parser.add_mutually_exclusive_group()
    task.add_argument("--goal", type=float, nargs=2, metavar=("X", "Y"))
    task.add_argument("--waypoints", type=Path, help="JSON task with measured map-frame waypoints")
    parser.add_argument("--log-dir", type=Path, default=ROOT/"logs")
    args, ros_args = parser.parse_known_args()
    if args.arm and not args.connect_serial:
        parser.error("--arm requires --connect-serial")
    if args.connect_serial and os.environ.get("ROS_DOMAIN_ID") == "77":
        parser.error("ROS_DOMAIN_ID=77 is reserved for mock inputs; serial output is forbidden")
    settings = load_settings(args.settings)
    nav_config = json.loads(args.config.read_text(encoding="utf-8"))
    nav_config = apply_task(nav_config, goal=args.goal, task_path=args.waypoints,
                            map_frame=settings["map_frame"], require_confirmed=args.arm)
    try:
        import rclpy
    except ImportError:
        parser.error("run on Ubuntu22.04 after source /opt/ros/humble/setup.bash")
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_node(settings, nav_config, args)
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
