import copy
import json
import math
from pathlib import Path
import struct
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

from calibrate_motion import fit
from map_bundle import load_map,read_pgm,validate_bundle
from navigation import Navigator,validate_config
from premap_bridge import multiply,validate
from premap_core import (decode_xyz,vector_obstacles,rigid,odometry_base,LatestWorker,
    OutputPermit,LocalizationGate,OccupancyMatcher,map_digest,mapping_parameters)
from premap_vehicle import prepare
from record_cloud_map import write_pcd
from safety_control import SafetyController
from serial_link import SerialLink
from test_serial_guard import Port
from vehicle_core import cloud_obstacles,VehicleRuntime

ROOT=Path(__file__).resolve().parent


class CloudTests(unittest.TestCase):
    def message(self,endian="<"):
        data=b"".join(struct.pack(endian+"fff",*p) for p in ((1,2,3),(4,5,6)))+b"PAD!"
        fields=[SimpleNamespace(name=n,datatype=7,count=1,offset=i*4) for i,n in enumerate(("x","y","z"))]
        return SimpleNamespace(width=2,height=1,point_step=12,row_step=28,data=data,
            is_bigendian=endian==">",fields=fields)

    def test_padding_and_endianness(self):
        for endian in ("<",">"):
            np.testing.assert_equal(decode_xyz(self.message(endian),10),[[1,2,3],[4,5,6]])

    def test_admission_before_data_or_iteration(self):
        message=self.message()
        message.width=300001
        message.data=None
        with self.assertRaisesRegex(ValueError,"admission"):
            decode_xyz(message,300000)

    def test_truncated_or_invalid_fields(self):
        for mode in ("truncated","offset","type","stride"):
            message=self.message()
            if mode=="truncated": message.data=b"x"
            if mode=="offset": message.fields[0].offset=999
            if mode=="type": message.fields[0].datatype=2
            if mode=="stride": message.row_step=1
            with self.assertRaises(ValueError): decode_xyz(message,10)

    def test_vector_matches_conservative_reference(self):
        rng=np.random.default_rng(2)
        points=rng.uniform([-3,-3,-.2],[3,3,1.5],(20000,3))
        config=json.loads((ROOT/"config/premapped.json").read_text())
        cloud=config["cloud"]
        identity=((1,0,0),(0,1,0),(0,0,1))
        args=((0,0,0),identity,(0,0,0),identity,(0,0,0),cloud,[-3,3,-3,3])
        self.assertEqual(vector_obstacles(points,*args),cloud_obstacles(points,*args))

    def test_dense_cloud_no_2500_obstacle_cliff(self):
        x,y=np.meshgrid(np.arange(-2.9,3,.1),np.arange(-2.9,3,.1))
        points=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,.2)))
        cloud=json.loads((ROOT/"config/premapped.json").read_text())["cloud"]
        result=vector_obstacles(points,(0,0,0),np.eye(3),(0,0,0),np.eye(3),(0,0,0),cloud,[-3,3,-3,3])
        self.assertGreater(len(result),2500)

    def test_low_base_returns_preserved(self):
        cloud=json.loads((ROOT/"config/premapped.json").read_text())["cloud"]
        result=vector_obstacles([[1,0,.04]],(0,0,0),np.eye(3),(0,0,0),np.eye(3),(0,0,0),cloud,[-3,3,-3,3])
        self.assertEqual(len(result),1)


class PremapCoreTests(unittest.TestCase):
    def test_slam_absolute_map_topic_is_explicitly_isolated(self):
        config=json.loads((ROOT/"config/infrastructure.json").read_text())
        parameters=mapping_parameters(config)
        self.assertEqual(parameters["map_name"],"/race/map")
        self.assertEqual(parameters["scan_topic"],"/race/scan")
        self.assertEqual(parameters["mode"],"mapping")
        self.assertFalse(parameters["restamp_tf"])

    def test_body_base_rigid_transform_not_rename(self):
        q=(0,0,math.sin(math.pi/4),math.cos(math.pi/4))
        transform=odometry_base((2,3,4),q,dict(xyz=[1,0,-.4],xyzw=[0,0,0,1]))
        np.testing.assert_allclose(transform[:3,3],[2,4,3.6],atol=1e-8)
        np.testing.assert_allclose(rigid([0,0,0],multiply(q,[0,0,0,1]))[:3,:3],transform[:3,:3])

    def test_infrastructure_cannot_run_with_unconfirmed_extrinsics(self):
        config=json.loads((ROOT/"config/infrastructure.json").read_text())
        with self.assertRaisesRegex(ValueError,"first"):
            validate(config)
        for key in ("frames_confirmed","extrinsics_confirmed","timestamps_confirmed"):
            config[key]=True
        validate(config)
        config["odom_frame"]=config["input_odom_frame"]
        with self.assertRaises(ValueError): validate(config)

    def test_transmit_permit_expires_even_if_controller_refreshed(self):
        now=[10.]
        permit=OutputPermit(clock=lambda:now[0])
        controller=SafetyController(clock=lambda:now[0])
        link=SerialLink("fake",controller,port_factory=Port,motion_permit=permit,clock=lambda:now[0])
        permit.allow_until(10.2)
        controller.update(.01,0)
        link._send_once()
        self.assertGreater(link.last_written_command[0],0)
        now[0]=10.2
        controller.update(.1,.1)
        link._send_once()
        self.assertEqual(link.last_written_command,(0,0,0))
        link.close()

    def test_shutdown_retries_after_first_failed_stop(self):
        class Flaky(Port):
            def write(self,frame):
                self.written.append(frame)
                if len(self.written)==1: raise OSError("one failed write")
                return len(frame)
        link=SerialLink("fake",SafetyController(),port_factory=Flaky)
        with mock.patch("serial_link.time.sleep"):
            link._shutdown_zeros()
        self.assertEqual(link.stop_write_attempts,20)
        self.assertEqual(link.stop_write_successes,19)
        self.assertEqual(len(link.port.written),20)
        self.assertTrue(link.motion_inhibited)
        self.assertTrue(all(struct.unpack("<fff",f[5:17])==(0,0,0) for f in link.port.written))
        link.close()

    def test_all_stop_failures_bounded_and_reported(self):
        class Broken(Port):
            def write(self,frame): raise OSError("disconnected")
        link=SerialLink("fake",SafetyController(),port_factory=Broken)
        with mock.patch("serial_link.time.sleep"):
            link._shutdown_zeros()
        self.assertEqual(link.stats()["stop_write_successes"],0)
        self.assertEqual(link.stats()["stop_write_attempts"],20)
        self.assertFalse(link.stats()["execution_acknowledged"])
        link.close()

    def test_dtr_and_rts_set_before_open(self):
        events=[]
        class RealFake(Port):
            def __init__(self,**options):
                super().__init__(**options)
                self.port=options["port"]
                events.append(("construct",self.port))
            def open(self):
                events.append(("open",self.dtr,self.rts,self.port))
        with mock.patch.dict("sys.modules",{"serial":SimpleNamespace(Serial=RealFake)}):
            with mock.patch("serial_link.os.name","nt"):
                link=SerialLink("fake",SafetyController())
                link.close()
        self.assertEqual(events,[("construct",None),("open",False,False,"fake")])

    def test_latest_worker_bounded_queue_and_failure(self):
        entered,release=threading.Event(),threading.Event()
        def process(value):
            if value==1:
                entered.set()
                release.wait(1)
            if value==4: raise ValueError("failure observed")
            return value
        worker=LatestWorker(process)
        worker.submit(1)
        self.assertTrue(entered.wait(1))
        worker.submit(2)
        worker.submit(3)
        self.assertEqual(worker.dropped,1)
        release.set()
        deadline=time.monotonic()+2
        found=None
        while time.monotonic()<deadline:
            result=worker.take()
            if result and result==(True,3):
                found=result
                break
            time.sleep(.005)
        self.assertEqual(found,(True,3))
        worker.submit(4)
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            result=worker.take()
            if result: break
            time.sleep(.005)
        self.assertEqual(result,(False,"failure observed"))
        worker.close()
        self.assertFalse(worker.thread.is_alive())

    def test_localization_requires_real_fresh_sources_and_dwell(self):
        gate=LocalizationGate()
        self.assertFalse(gate.valid(10))
        gate.pose_time=gate.scan_time=gate.odom_time=10
        gate.cov_ok=True
        gate.match=.9
        self.assertFalse(gate.valid(10))
        gate.pose_time=gate.scan_time=gate.odom_time=11.01
        self.assertTrue(gate.valid(11.01))
        gate.match=.1
        self.assertFalse(gate.valid(11.02))
        self.assertIsNone(gate.since)

    def test_scan_matcher_rejects_shifted_unknown_and_sparse_endpoints(self):
        data=np.zeros((100,100),dtype=int)
        data[:,50]=100
        matcher=OccupancyMatcher(data.ravel(),100,100,.1,(0,0),tolerance=.1)
        points=np.column_stack((np.full(40,5.05),np.linspace(.2,9.8,40)))
        self.assertEqual(matcher.fraction(points),1)
        self.assertEqual(matcher.fraction(points+[1,0]),0)
        self.assertEqual(matcher.fraction(points[:10]),0)

    def test_calibration_fits_actual_units_and_rejects_one_axis(self):
        commands=np.array([[.1,0],[-.1,0],[0,.1],[0,-.1],[.05,.05],[-.05,-.05]])
        matrix=np.array([[0,-2],[3,0]])
        rows=np.column_stack((commands,commands@matrix.T))
        np.testing.assert_allclose(fit(rows)["body_from_command"],matrix,atol=1e-10)
        rows[:,1]=0
        with self.assertRaises(ValueError): fit(rows)

    def test_raised_limits_require_calibration_and_explicit_controller_cap(self):
        config=json.loads((ROOT/"config.json").read_text())
        config.update(commissioned_control_limit=.4,control_limit=.3)
        with self.assertRaises(ValueError): validate_config(config)
        config["calibration_confirmed"]=True
        validate_config(config)
        with self.assertRaises(ValueError): SafetyController(limit=.3)
        controller=SafetyController(limit=.3,commissioned_limit=.4)
        controller.update(1,0)
        self.assertAlmostEqual(controller.output().vx,.3)

    def test_binary_pcd_export_never_overwrites(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"reference.pcd"
            write_pcd(path,[[1,2,3]])
            self.assertTrue(path.read_bytes().endswith(struct.pack("<fff",1,2,3)))
            with self.assertRaises(FileExistsError): write_pcd(path,[[0,0,0]])


class MapBundleTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root=Path(self.directory.name)
        pixels=np.full((80,80),254,dtype=np.uint8)
        pixels[0,:]=pixels[-1,:]=pixels[:,0]=pixels[:,-1]=0
        self.pixels=pixels
        (self.root/"room.pgm").write_bytes(b"P5\n# generated test, not real field\n80 80\n255\n"+pixels.tobytes())
        (self.root/"room.yaml").write_text("image: room.pgm\nresolution: 0.1\norigin: [-4.0, -4.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n")
        self.grid=load_map(self.root/"room.yaml")
        manifest={key:self.grid[key] for key in ("yaml_sha256","image_sha256","map_digest")}
        manifest.update(map_yaml="room.yaml",map_frame="race_map")
        self.bundle=self.root/"room.manifest.json"
        self.bundle.write_text(json.dumps(manifest))
        self.field=json.loads((ROOT/"config/field_template.json").read_text())
        self.field["map_digest"]=self.grid["map_digest"]
        self.field_path=self.root/"field.json"
        self.field_path.write_text(json.dumps(self.field))

    def test_pgm_map_conversion_and_exact_bundle(self):
        np.testing.assert_array_equal(read_pgm(self.root/"room.pgm"),self.pixels)
        self.assertEqual(set(self.grid["data"]),{0,100})
        manifest,grid=validate_bundle(self.bundle)
        self.assertEqual(manifest["map_digest"],grid["map_digest"])
        self.assertNotEqual(grid["map_digest"],map_digest(grid["data"],80,80,.1,[-3,-4,0]))

    def test_modified_map_detected(self):
        (self.root/"room.yaml").write_text((self.root/"room.yaml").read_text()+"# changed\n")
        with self.assertRaisesRegex(ValueError,"changed"): validate_bundle(self.bundle)

    def test_unknown_pixel_and_pgm_binary_whitespace_preserved(self):
        path=self.root/"small.pgm"
        path.write_bytes(b"P5\n3 1\n255\n"+bytes([10,35,205]))
        np.testing.assert_array_equal(read_pgm(path),[[10,35,205]])
        self.pixels[1,1]=205
        (self.root/"room.pgm").write_bytes(b"P5\n80 80\n255\n"+self.pixels.tobytes())
        grid=load_map(self.root/"room.yaml")
        self.assertIn(-1,grid["data"])

    def test_rotated_map_rejected_instead_of_ignored_by_amcl(self):
        path=self.root/"room.yaml"
        path.write_text(path.read_text().replace("-4.0, 0.0]","-4.0, 0.5]"))
        with self.assertRaisesRegex(ValueError,"axis-aligned"): load_map(path)

    def test_wrong_map_field_and_unconfirmed_arm_rejected(self):
        _,_,report=prepare(self.bundle,self.field_path,ROOT/"config/premapped.json")
        self.assertGreater(report["route_points"],100)
        with self.assertRaisesRegex(ValueError,"unconfirmed"):
            prepare(self.bundle,self.field_path,ROOT/"config/premapped.json",armed=True)
        self.field["map_digest"]="different map"
        self.field_path.write_text(json.dumps(self.field))
        with self.assertRaisesRegex(ValueError,"THIS map"):
            prepare(self.bundle,self.field_path,ROOT/"config/premapped.json")

    def test_known_obstacle_on_reference_route_blocks_preflight(self):
        # Add an occupied stripe across the departure path, regenerate bundle.
        self.pixels[59:61,5:35]=0
        (self.root/"room.pgm").write_bytes(b"P5\n80 80\n255\n"+self.pixels.tobytes())
        grid=load_map(self.root/"room.yaml")
        manifest=json.loads(self.bundle.read_text())
        for key in ("yaml_sha256","image_sha256","map_digest"): manifest[key]=grid[key]
        self.bundle.write_text(json.dumps(manifest))
        self.field["map_digest"]=grid["map_digest"]
        self.field_path.write_text(json.dumps(self.field))
        with self.assertRaisesRegex(ValueError,"blocked"):
            prepare(self.bundle,self.field_path,ROOT/"config/premapped.json")

    def test_premapped_runtime_lookahead_completes_full_eight(self):
        now=[10.]
        settings=json.loads((ROOT/"config/premapped.json").read_text())
        runtime=VehicleRuntime(self.field,settings,clock=lambda:now[0])
        runtime.map_ready=True
        x,y=self.field["parking"]
        for tick in range(16000):
            now[0]=10+tick*.05
            runtime.update_pose([x,y,.3],now[0])
            runtime.update_quality(True,now[0])
            runtime.update_obstacles([],now[0])
            if tick==0: self.assertTrue(runtime.start()[0])
            command=runtime.step()
            # actual simulation uses the supplied calibration, not proof of vehicle calibration.
            matrix=np.asarray(self.field["body_from_command"])
            bx,by=matrix@np.asarray(command[:2])
            x+=(math.cos(.3)*bx-math.sin(.3)*by)*.05
            y+=(math.sin(.3)*bx+math.cos(.3)*by)*.05
            if runtime.nav.state=="COMPLETE": break
        self.assertEqual(runtime.nav.state,"COMPLETE",runtime.mission_report())
        self.assertTrue(runtime.mission_report()["verified"])
        self.assertEqual(command,(0,0,0))


if __name__=="__main__":
    unittest.main(verbosity=2)
