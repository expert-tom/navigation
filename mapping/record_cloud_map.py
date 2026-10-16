"""Optional 3D voxel PCD reference export, NOT the AMCL navigation-map input.

Collect /race/points transformed to race_map during RC exploration. Loop closure
can move historical scans: this lightweight exporter does not reoptimize history.
Keep a rosbag to rebuild a metrically consistent 3D map if required later.
"""
import argparse
from pathlib import Path
import time

import numpy as np
from premap_core import LatestWorker, decode_xyz, rigid
from vehicle_core import message_stamp


def write_pcd(path, points):
    points = np.asarray(points,dtype="<f4").reshape((-1,3))
    if len(points)==0 or not np.isfinite(points).all():
        raise ValueError("refuse empty/nonfinite PCD")
    header = ("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
              f"COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n")
    with Path(path).open("xb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(points.tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--seconds",type=float,default=60)
    parser.add_argument("--voxel",type=float,default=.05)
    args,ros_args = parser.parse_known_args()
    if args.output.exists() or not .01<=args.voxel<=.5 or not 1<=args.seconds<=1800:
        parser.error("new output required, voxel [.01,.5], seconds [1,1800]")
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from tf2_ros import Buffer,TransformListener

    class Recorder(Node):
        def __init__(self):
            super().__init__("race_cloud_recorder")
            self.buffer = Buffer(cache_time=Duration(seconds=5))
            self.listener = TransformListener(self.buffer,self)
            self.cells = np.empty((0,3),dtype=np.int64)
            self.error = None
            self.accepted = 0
            self.worker = LatestWorker(self.add)
            self.create_subscription(PointCloud2,"/race/points",self.worker.submit,qos_profile_sensor_data)

        def add(self,message):
            message_stamp(message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                self.get_clock().now().nanoseconds/1e9,time.monotonic(),.5)
            points = decode_xyz(message,300000)
            points = points[np.isfinite(points).all(axis=1)]
            tf = self.buffer.lookup_transform("race_map",message.header.frame_id,
                Time.from_msg(message.header.stamp),timeout=Duration(seconds=.08))
            t,q = tf.transform.translation,tf.transform.rotation
            transform = rigid((t.x,t.y,t.z),(q.x,q.y,q.z,q.w))
            world = points @ transform[:3,:3].T+transform[:3,3]
            cells = np.unique(np.concatenate((self.cells,np.floor(world/args.voxel).astype(np.int64))),axis=0)
            if len(cells)>400000:
                raise ValueError("3D voxel budget exceeded; reduce mapped area / increase voxel, no silent truncation")
            self.cells = cells
            self.accepted += 1
            return len(cells)

    rclpy.init(args=ros_args)
    node = Recorder()
    deadline = time.monotonic()+args.seconds
    failed=False
    try:
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            result = node.worker.take()
            if result and not result[0]:
                # Missed TF samples are not usable; do not save them in wrong frame.
                node.get_logger().warn(result[1])
                if "budget" in result[1]:
                    raise RuntimeError(result[1])
    except KeyboardInterrupt:
        pass
    except Exception:
        failed=True
        raise
    finally:
        node.worker.close()
        # Do not race an unfinished NumPy worker while saving.
        if node.worker.thread.is_alive():
            raise RuntimeError("export worker still active; no potentially inconsistent PCD written")
        if node.accepted and not failed:
            args.output.parent.mkdir(parents=True,exist_ok=True)
            write_pcd(args.output,(node.cells+.5)*args.voxel)
            print("Reference PCD saved:",args.output,"accepted sweeps:",node.accepted)
        else:
            print("No valid map-frame sweeps or recorder failed: no PCD written")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
