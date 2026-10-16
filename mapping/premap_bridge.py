"""Explicit LIO body->base conversion and calibrated sensor-frame alias.

Does not launch or modify any team's private driver/launch/configuration.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import time

from premap_core import odometry_base, rigid
from vehicle_core import message_stamp, quaternion_matrix


def multiply(a,b):
    x,y,z,w = a
    X,Y,Z,W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def validate(settings):
    required = ("frames_confirmed","extrinsics_confirmed","timestamps_confirmed")
    if any(settings.get(key) is not True for key in required):
        raise ValueError("check real input frames, extrinsics and ROS timestamp epoch first: "+", ".join(required))
    for key in ("body_from_base","base_from_lidar"):
        rigid(settings[key]["xyz"],settings[key]["xyzw"])
        if abs(sum(v*v for v in settings[key]["xyzw"])-1)>1e-6:
            raise ValueError(key+" quaternion must be unit length")
    targets = [settings[key] for key in ("map_frame","odom_frame","base_frame","lidar_frame")]
    if len(set(targets)) != 4 or any(not f or f.startswith("/") for f in targets):
        raise ValueError("target TF frames must be distinct, nonempty and without leading slash")
    if settings["odom_frame"] == settings["input_odom_frame"] or settings["base_frame"] == settings["input_body_frame"]:
        raise ValueError("use isolated race frames to avoid duplicate TF authorities")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--settings", type=Path, required=True)
    args, ros_args = parser.parse_known_args()
    settings = json.loads(args.settings.read_text(encoding="utf-8"))
    validate(settings)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

    class Bridge(Node):
        def __init__(self):
            super().__init__("race_lio_bridge")
            if self.get_parameter("use_sim_time").value:
                raise ValueError("hardware bridge requires ROS wall clock")
            self.broadcaster = TransformBroadcaster(self)
            self.static = StaticTransformBroadcaster(self)
            self.cloud_pub = self.create_publisher(PointCloud2,"/race/points",qos_profile_sensor_data)
            self.odom_pub = self.create_publisher(Odometry,"/race/odom",qos_profile_sensor_data)
            self.create_subscription(Odometry,settings["odom_topic"],self.odom,qos_profile_sensor_data)
            self.create_subscription(PointCloud2,settings["cloud_topic"],self.cloud,qos_profile_sensor_data)
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id, tf.child_frame_id = settings["base_frame"],settings["lidar_frame"]
            t = settings["base_from_lidar"]
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z = map(float,t["xyz"])
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w = map(float,t["xyzw"])
            self.static.sendTransform(tf)
            self.last_stamp = -math.inf
            self.errors = 0

        def fresh(self,message):
            stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
            message_stamp(stamp,self.get_clock().now().nanoseconds/1e9,time.monotonic(),.5)
            return stamp

        def warn(self, exc):
            self.errors += 1
            if self.errors % 50 == 1:
                self.get_logger().error(str(exc)+" (input rejected; no invented TF)")

        def odom(self,message):
            try:
                stamp = self.fresh(message)
                if message.header.frame_id != settings["input_odom_frame"] or message.child_frame_id != settings["input_body_frame"]:
                    raise ValueError("LIO odometry frame IDs differ from commissioned configuration")
                if stamp <= self.last_stamp:
                    raise ValueError("out-of-order LIO odometry / clock reset: restart mapping or localization")
                p,q = message.pose.pose.position,message.pose.pose.orientation
                matrix = odometry_base((p.x,p.y,p.z),(q.x,q.y,q.z,q.w),settings["body_from_base"])
                qout = multiply((q.x,q.y,q.z,q.w),settings["body_from_base"]["xyzw"])
                norm=math.sqrt(sum(value*value for value in qout))
                qout=tuple(value/norm for value in qout)
                rotation = quaternion_matrix(qout)
                if abs(rotation[2][0])+abs(rotation[2][1]) > .3:
                    raise ValueError("base tilt too large for planar AMCL: check extrinsics / terrain")
                tf = TransformStamped()
                tf.header.stamp = message.header.stamp
                tf.header.frame_id,tf.child_frame_id = settings["odom_frame"],settings["base_frame"]
                tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z = map(float,matrix[:3,3])
                tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w = map(float,qout)
                out = Odometry()
                out.header, out.child_frame_id = tf.header,tf.child_frame_id
                out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z = map(float,matrix[:3,3])
                out.pose.pose.orientation = tf.transform.rotation
                # Deliberately no fabricated covariance / velocity: AMCL uses TF.
                self.broadcaster.sendTransform(tf)
                self.odom_pub.publish(out)
                self.last_stamp = stamp
            except Exception as exc:
                self.warn(exc)

        def cloud(self,message):
            try:
                self.fresh(message)
                if message.header.frame_id != settings["input_lidar_frame"]:
                    raise ValueError("raw cloud is not in the configured physical lidar frame")
                if not 0 < message.width*message.height <= settings["max_input_points"]:
                    raise ValueError("cloud outside admission budget")
                out = copy.copy(message)
                out.header = copy.deepcopy(message.header)
                # Same physical origin and axes, explicitly confirmed in config.
                # Only the TF namespace is isolated. No geometric pose is faked.
                out.header.frame_id = settings["lidar_frame"]
                self.cloud_pub.publish(out)
            except Exception as exc:
                self.warn(exc)

    rclpy.init(args=ros_args)
    node = Bridge()
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
