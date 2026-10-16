import argparse
import json
import math
from pathlib import Path
import time

from vehicle_core import message_stamp, yaw_from_quaternion


def covariance_valid(values, settings):
    if len(values) != 36 or not all(math.isfinite(v) for v in values):
        return False
    diagonal = [values[0], values[7], values[35]]
    if any(v < 0 for v in diagonal):
        return False
    if not settings["accept_zero_covariance"] and any(v == 0 for v in diagonal):
        return False
    return diagonal[0] <= settings["max_position_variance"] and diagonal[1] <= settings["max_position_variance"] and diagonal[2] <= settings["max_yaw_variance"]


def main():
    parser = argparse.ArgumentParser(description="Localization heartbeat from a confirmed covariance-bearing pose source")
    parser.add_argument("--settings", type=Path, default=Path(__file__).resolve().parent/"config/real_vehicle.json")
    args, ros_args = parser.parse_known_args()
    settings = json.loads(args.settings.read_text(encoding="utf-8"))
    quality = settings["quality_monitor"]
    if quality["source_confirmed"] is not True:
        parser.error("quality_monitor.source_confirmed must be true after checking source, frames and covariance semantics")
    if quality["pose_type"] not in ("odometry", "pose"):
        parser.error("quality_monitor.pose_type must be odometry or pose")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool

    class Monitor(Node):
        def __init__(self):
            super().__init__("srm_localization_monitor")
            self.valid = False
            self.stamp = -math.inf
            self.last_message = -math.inf
            self.publisher = self.create_publisher(Bool, settings["quality_topic"], 10)
            self.create_subscription(Odometry if quality["pose_type"] == "odometry" else PoseWithCovarianceStamped,
                                     quality["pose_topic"], self.receive, qos_profile_sensor_data)
            self.create_timer(0.1, self.publish)

        def receive(self, message):
            self.valid = False
            try:
                if message.header.frame_id != quality["pose_frame"]:
                    raise ValueError("quality pose frame mismatch")
                if quality["pose_type"] == "odometry" and message.child_frame_id != quality["child_frame"]:
                    raise ValueError("quality child frame mismatch")
                t = message.header.stamp.sec+message.header.stamp.nanosec/1e9
                if t <= self.last_message:
                    raise ValueError("duplicate or out-of-order quality source")
                self.stamp = message_stamp(t, self.get_clock().now().nanoseconds/1e9, time.monotonic(), quality["max_age"])
                p, q = message.pose.pose.position, message.pose.pose.orientation
                if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                    raise ValueError("invalid quality pose")
                yaw_from_quaternion((q.x, q.y, q.z, q.w))
                self.valid = covariance_valid(message.pose.covariance, quality)
                self.last_message = t
            except Exception as exc:
                self.get_logger().warn(str(exc))

        def publish(self):
            message = Bool()
            message.data = self.valid and 0 <= time.monotonic()-self.stamp <= quality["max_age"]
            self.publisher.publish(message)

    rclpy.init(args=ros_args)
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
