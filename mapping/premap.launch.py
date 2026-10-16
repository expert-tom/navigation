"""ROS 2 Humble: RC mapping OR loaded-map AMCL. NEVER opens a serial port.

Run directly: ros2 launch /absolute/path/premap.launch.py stage:=mapping
The public lidar driver and LIO must already supply confirmed inputs.
"""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
    from launch.events import Shutdown
    from launch.event_handlers import OnProcessExit
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    def setup(context):
        from ament_index_python.packages import get_package_share_directory
        sys.path.insert(0,str(ROOT))
        sys.path.insert(0,str(ROOT/".deps/python310"))
        import yaml
        from premap_bridge import validate
        from premap_core import mapping_parameters
        infrastructure = Path(LaunchConfiguration("infrastructure").perform(context)).resolve()
        config = json.loads(infrastructure.read_text(encoding="utf-8"))
        validate(config)
        mission_path=Path(LaunchConfiguration("mission_settings").perform(context)).resolve()
        mission=json.loads(mission_path.read_text(encoding="utf-8"))
        if any(mission[key]!=config[key] for key in ("map_frame","odom_frame","base_frame")):
            raise ValueError("mission_settings and infrastructure must use the same isolated TF frames")
        stage = LaunchConfiguration("stage").perform(context)
        if stage not in ("mapping","localization"):
            raise ValueError("stage must be mapping or localization")
        bridge = ExecuteProcess(cmd=[sys.executable,str(ROOT/"run.py"),"premap_bridge.py","--settings",str(infrastructure)],output="screen")
        actions = [bridge]
        # Fail closed if input bridge exits. No automatic process restart / motion.
        actions.append(RegisterEventHandler(OnProcessExit(target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason="input bridge exited"))])))
        actions.append(Node(package="pointcloud_to_laserscan",executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",namespace="race",output="screen",
            remappings=[("cloud_in","/race/points"),("scan","/race/scan")],
            parameters=[dict(use_sim_time=False,target_frame=config["base_frame"],
                transform_tolerance=.1,min_height=config["scan_min_height"],max_height=config["scan_max_height"],
                range_min=config["scan_range_min"],range_max=12.0,angle_min=-3.141592653589793,
                angle_max=3.141592653589793,angle_increment=.008726646259971648,
                scan_time=.1,use_inf=True,queue_size=2)]))
        if stage == "mapping":
            default = Path(get_package_share_directory("slam_toolbox"))/"config/mapper_params_online_async.yaml"
            params = yaml.safe_load(default.read_text())["slam_toolbox"]["ros__parameters"]
            params.update(mapping_parameters(config))
            actions.append(Node(package="slam_toolbox",executable="async_slam_toolbox_node",
                namespace="race",name="slam_toolbox",parameters=[params],output="screen"))
        else:
            from map_bundle import validate_bundle
            manifest, grid = validate_bundle(LaunchConfiguration("bundle").perform(context))
            if manifest["map_frame"] != config["map_frame"]:
                raise ValueError("map bundle / infrastructure TF mismatch")
            actions.extend([
                Node(package="nav2_map_server",executable="map_server",namespace="race",name="map_server",
                    parameters=[dict(use_sim_time=False,yaml_filename=grid["yaml_path"],frame_id=config["map_frame"],topic_name="map")],output="screen"),
                Node(package="nav2_amcl",executable="amcl",namespace="race",name="amcl",output="screen",
                    parameters=[dict(use_sim_time=False,base_frame_id=config["base_frame"],
                        odom_frame_id=config["odom_frame"],global_frame_id=config["map_frame"],scan_topic="/race/scan",map_topic="/race/map",
                        robot_model_type="nav2_amcl::OmniMotionModel",set_initial_pose=False,always_reset_initial_pose=True,
                        tf_broadcast=True,transform_tolerance=.15,update_min_d=.02,update_min_a=.02,
                        min_particles=500,max_particles=2000,max_beams=90,laser_model_type="likelihood_field",
                        laser_min_range=.4,laser_max_range=12.0,alpha1=.1,alpha2=.1,alpha3=.1,alpha4=.1,alpha5=.1)]),
                Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",namespace="race",
                    name="lifecycle_manager_localization",output="screen",
                    parameters=[dict(use_sim_time=False,autostart=True,node_names=["map_server","amcl"],bond_timeout=4.0)]),
                ExecuteProcess(cmd=[sys.executable,str(ROOT/"run.py"),"premap_quality.py","--settings",
                    str(Path(LaunchConfiguration("mission_settings").perform(context)).resolve())],output="screen")])
        return actions

    return LaunchDescription([
        DeclareLaunchArgument("stage",default_value="mapping"),
        DeclareLaunchArgument("infrastructure",default_value=str(ROOT/"config/infrastructure.json")),
        DeclareLaunchArgument("mission_settings",default_value=str(ROOT/"config/premapped.json")),
        DeclareLaunchArgument("bundle",default_value=""),OpaqueFunction(function=setup)])
