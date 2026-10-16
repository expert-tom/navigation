"""AMCL covariance is necessary, not sufficient: check scans against loaded map."""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
from localization_monitor import covariance_valid
from premap_core import LatestWorker, LocalizationGate, OccupancyMatcher, rigid
from vehicle_core import message_stamp, yaw_from_quaternion


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--settings", type=Path, required=True)
    args,ros_args = parser.parse_known_args()
    settings = json.loads(args.settings.read_text(encoding="utf-8"))
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.time import Time
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Empty
    from tf2_ros import Buffer,TransformListener

    class Monitor(Node):
        def __init__(self):
            super().__init__("race_localization_quality")
            self.gate = LocalizationGate(**settings["localization_gate"])
            self.buffer = Buffer(cache_time=Duration(seconds=5))
            self.listener = TransformListener(self.buffer,self)
            self.matcher = None
            self.map_worker = LatestWorker(self.make_matcher)
            self.scan_worker = LatestWorker(self.match_scan)
            self.pub = self.create_publisher(Bool,"/race/localization_valid",10)
            self.status = self.create_publisher(String,"/race/localization_status",10)
            qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid,"/race/map",lambda m:self.map_worker.submit(m),qos)
            self.create_subscription(Odometry,"/race/odom",self.odom,qos_profile_sensor_data)
            self.create_subscription(PoseWithCovarianceStamped,"/race/amcl_pose",self.pose,qos_profile_sensor_data)
            self.create_subscription(LaserScan,"/race/scan",lambda m:self.scan_worker.submit((m,self.matcher)),qos_profile_sensor_data)
            self.request = self.create_client(Empty,"/race/request_nomotion_update")
            self.pending_request = None
            self.create_timer(.1,self.tick)
            self.create_timer(.5,self.refresh_amcl)

        def stamp(self,message):
            return message_stamp(message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                self.get_clock().now().nanoseconds/1e9,time.monotonic(),self.gate.max_age)

        def odom(self,message):
            try:
                stamp = self.stamp(message)
                if message.header.frame_id != settings["odom_frame"] or message.child_frame_id != settings["base_frame"]:
                    raise ValueError("odom frame mismatch")
                if stamp > self.gate.odom_time:
                    self.gate.odom_time = stamp
            except Exception:
                self.gate.odom_time = -math.inf

        def pose(self,message):
            self.gate.cov_ok = False
            try:
                stamp = self.stamp(message)
                if message.header.frame_id != settings["map_frame"] or stamp <= self.gate.pose_time:
                    return
                p,q = message.pose.pose.position,message.pose.pose.orientation
                if not np.isfinite([p.x,p.y,p.z]).all():
                    return
                yaw_from_quaternion((q.x,q.y,q.z,q.w))
                self.gate.cov_ok = covariance_valid(message.pose.covariance,settings["quality_monitor"])
                self.gate.pose_time = stamp
            except Exception:
                pass

        def make_matcher(self,message):
            if message.header.frame_id != settings["map_frame"] or len(message.data)>16000000:
                raise ValueError("map frame/size mismatch")
            info = message.info
            q = info.origin.orientation
            return OccupancyMatcher(message.data,info.width,info.height,info.resolution,
                (info.origin.position.x,info.origin.position.y),yaw_from_quaternion((q.x,q.y,q.z,q.w)))

        def match_scan(self,job):
            message,matcher = job
            if matcher is None:
                raise ValueError("WAIT_MAP")
            stamp = self.stamp(message)
            tf = self.buffer.lookup_transform(settings["map_frame"],message.header.frame_id,
                Time.from_msg(message.header.stamp),timeout=Duration(seconds=.08))
            t,q = tf.transform.translation,tf.transform.rotation
            transform = rigid((t.x,t.y,t.z),(q.x,q.y,q.z,q.w))
            ranges = np.asarray(message.ranges)
            angles = message.angle_min+np.arange(len(ranges))*message.angle_increment
            keep = np.isfinite(ranges)&(ranges>=max(.5,message.range_min))&(ranges<message.range_max)
            points = np.column_stack((ranges[keep]*np.cos(angles[keep]),ranges[keep]*np.sin(angles[keep]),np.zeros(keep.sum())))
            points = points @ transform[:3,:3].T+transform[:3,3]
            return matcher.fraction(points),stamp

        def refresh_amcl(self):
            # AMCL otherwise may not publish fresh covariance while stationary.
            if self.request.service_is_ready() and (self.pending_request is None or self.pending_request.done()):
                self.pending_request = self.request.call_async(Empty.Request())

        def tick(self):
            result = self.map_worker.take()
            if result:
                self.matcher = result[1] if result[0] else None
                self.gate.since = None
                self.gate.scan_time = -math.inf
            result = self.scan_worker.take()
            if result and result[0]:
                match,stamp = result[1]
                if stamp > self.gate.scan_time:
                    self.gate.match,self.gate.scan_time = match,stamp
            valid = self.matcher is not None and self.gate.valid(time.monotonic())
            message = Bool()
            message.data = bool(valid)
            self.pub.publish(message)
            state = String()
            state.data = json.dumps(dict(valid=bool(valid),scan_map_match=self.gate.match,
                covariance_ok=self.gate.cov_ok,meaning="readiness heuristic, not localization ground truth"))
            self.status.publish(state)

    rclpy.init(args=ros_args)
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.map_worker.close()
        node.scan_worker.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
