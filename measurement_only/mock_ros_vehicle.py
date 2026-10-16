import argparse
import json
import math
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description="Synthetic ROS inputs; isolated ROS_DOMAIN_ID required; no serial access")
    parser.add_argument("--settings", type=Path, default=Path(__file__).resolve().parent/"config/real_vehicle.json")
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parent/"config.json")
    args = parser.parse_args()
    import os
    if os.environ.get("ROS_DOMAIN_ID") != "77":
        parser.error("use export ROS_DOMAIN_ID=77 in every demo terminal; never mix mock input with real vehicles")
    settings = json.loads(args.settings.read_text(encoding="utf-8"))
    config = json.loads(args.config.read_text(encoding="utf-8"))
    if settings["sensor_type"] != "pointcloud2" or settings["pose_mode"] != "tf":
        parser.error("demo uses pointcloud2 and tf settings")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Bool, Float32MultiArray
    from std_msgs.msg import Header
    from tf2_ros import TransformBroadcaster

    class Demo(Node):
        def __init__(self):
            super().__init__("srm_mock_vehicle")
            self.x,self.y = config["parking"]
            self.command = [0.0,0.0,0.0]
            self.command_stamp = time.monotonic()
            self.last_tick = time.monotonic()
            self.tf = TransformBroadcaster(self)
            self.cloud = self.create_publisher(PointCloud2,settings["sensor_topic"],qos_profile_sensor_data)
            self.quality = self.create_publisher(Bool,settings["quality_topic"],10)
            self.map_pub = self.create_publisher(OccupancyGrid,settings["map_topic"],QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
            self.create_subscription(Float32MultiArray,"/navigation/command_preview",self.receive,10)
            self.create_timer(0.05,self.tick)
            self.create_timer(1.0,self.publish_map)
            self.get_logger().info("MOCK ONLY: publishes artificial TF, map, cloud and quality; do not arm serial")

        def receive(self,message):
            if len(message.data)==3 and all(math.isfinite(v) for v in message.data):
                self.command = list(message.data)
                self.command_stamp = time.monotonic()

        def publish_map(self):
            message = OccupancyGrid()
            message.header.frame_id = settings["map_frame"]
            message.header.stamp = self.get_clock().now().to_msg()
            x0,x1,y0,y1 = config["bounds"]
            message.info.resolution = 0.1
            message.info.width = math.ceil((x1-x0)/0.1)+1
            message.info.height = math.ceil((y1-y0)/0.1)+1
            message.info.origin.position.x,message.info.origin.position.y = float(x0),float(y0)
            message.info.origin.orientation.w = 1.0
            message.data = [0]*(message.info.width*message.info.height)
            self.map_pub.publish(message)

        def tick(self):
            now = time.monotonic()
            dt = min(now-self.last_tick,0.1)
            self.last_tick = now
            command = self.command if now-self.command_stamp<=0.2 else [0,0,0]
            matrix = config["body_from_command"]
            self.x += (matrix[0][0]*command[0]+matrix[0][1]*command[1])*dt
            self.y += (matrix[1][0]*command[0]+matrix[1][1]*command[1])*dt
            stamp = self.get_clock().now().to_msg()
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = settings["map_frame"]
            transform.child_frame_id = settings["base_frame"]
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation.w = 1.0
            self.tf.sendTransform(transform)
            header = Header()
            header.frame_id = settings["map_frame"]
            header.stamp = stamp
            points = []
            for name in ("lower_pole","upper_pole"):
                x,y = config[name]
                for i in range(32):
                    a = i*2*math.pi/32
                    points.append((x+config["pole_radius"]*math.cos(a),y+config["pole_radius"]*math.sin(a),0.2))
            self.cloud.publish(create_cloud_xyz32(header,points))
            valid = Bool()
            valid.data = True
            self.quality.publish(valid)

    rclpy.init()
    node = Demo()
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
