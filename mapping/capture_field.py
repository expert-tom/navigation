"""Pick geometry in RViz on the saved map, not in an arbitrary LIO boot frame."""
import argparse
import json
import math
from pathlib import Path

from map_bundle import validate_bundle
from navigation import Navigator
from premap_core import map_digest
from vehicle_core import yaw_from_quaternion

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args,ros_args = parser.parse_known_args()
    manifest,grid = validate_bundle(args.bundle)
    if args.output.exists():
        parser.error("refuse to overwrite existing field")
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PointStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    labels = ["parking centre","lower pole centre","upper pole centre",
              "inner safe field rectangle corner A","opposite corner B"]

    class Capture(Node):
        def __init__(self):
            super().__init__("race_field_capture")
            self.points = []
            self.done = False
            self.map_verified = False
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid,"/race/map",self.verify_map,qos)
            self.create_subscription(PointStamped,"/clicked_point",self.receive,10)
            self.get_logger().info("RViz Fixed Frame=race_map. Publish Point, click: "+labels[0])

        def verify_map(self,message):
            self.map_verified=False
            try:
                info=message.info
                q=info.origin.orientation
                origin=(info.origin.position.x,info.origin.position.y,yaw_from_quaternion((q.x,q.y,q.z,q.w)))
                self.map_verified=(message.header.frame_id==manifest["map_frame"] and
                    map_digest(message.data,info.width,info.height,info.resolution,origin)==manifest["map_digest"])
            except Exception:
                pass
            if not self.map_verified:
                self.get_logger().error("/race/map does not equal saved bundle. Stop mapping; load the saved map before picking coordinates.")

        def receive(self,message):
            if self.done:
                return
            if not self.map_verified:
                self.get_logger().error("WAIT_SAVED_MAP: point rejected until published map matches bundle")
                return
            if message.header.frame_id != manifest["map_frame"]:
                self.get_logger().error("wrong frame; set RViz Fixed Frame="+manifest["map_frame"])
                return
            if not all(math.isfinite(value) for value in (message.point.x,message.point.y)):
                self.get_logger().error("nonfinite point rejected")
                return
            self.points.append([message.point.x,message.point.y])
            if len(self.points)<5:
                self.get_logger().info("next click: "+labels[len(self.points)])
                return
            field = json.loads((ROOT/"config/field_template.json").read_text(encoding="utf-8"))
            field["parking"],field["lower_pole"],field["upper_pole"] = self.points[:3]
            a,b = self.points[3:]
            field["bounds"] = [min(a[0],b[0]),max(a[0],b[0]),min(a[1],b[1]),max(a[1],b[1])]
            field["map_digest"] = manifest["map_digest"]
            field["description"] = "Map-picked geometry; base radii, whole robot footprint, velocity calibration and initial yaw STILL NEED CONFIRMATION"
            args.output.parent.mkdir(parents=True,exist_ok=True)
            with args.output.open("x",encoding="utf-8") as stream:
                json.dump(field,stream,ensure_ascii=False,indent=2)
            self.done = True
            try:
                Navigator(field)
                self.get_logger().info("Saved. Now confirm pole BASE radius, full vehicle envelope, calibration and run offline --check.")
            except ValueError as exc:
                self.get_logger().error("Saved for correction; geometry not feasible: "+str(exc))

    rclpy.init(args=ros_args)
    node = Capture()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node,timeout_sec=.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
