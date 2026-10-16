# 实车接入版：明天的调试顺序

本版本另建文件夹，保留旧项目、旧压缩包和之前的GitHub整理副本。它增加ROS2接入与实车运行入口；不包含已经实测成功的SLAM模型，也不代表车辆已通过比赛验收。

## 1. 已完成什么

`real_vehicle.py`订阅定位/TF、定位质量心跳、点云或LaserScan、OccupancyGrid地图；`vehicle_core.py`将这些输入接到八字导航核心；原串口模块负责115200、8N1、目标100Hz发送19字节帧。串口发送独立线程，导航处理停止更新时控制看门狗归零。支持一次启动、锁定停止、定位跳变停止、数据过期停车、未知地图格视为障碍、故障日志和路线预览。

输入支持：

| 配置 | 支持的来源 |
|---|---|
| `pose_mode=tf` | 由TF查询`map_frame -> base_frame`，推荐处理雷达和底盘原点不同的情况 |
| `pose_mode=odometry` | `nav_msgs/Odometry`；header必须是map_frame，child必须是base_frame |
| `pose_mode=pose` | `geometry_msgs/PoseWithCovarianceStamped`；位置必须表示底盘原点，header必须是map_frame |
| `sensor_type=pointcloud2` | `sensor_msgs/PointCloud2`，建议使用经公开定位方案去畸变后的点云 |
| `sensor_type=livox_custom` | `livox_ros_driver2/CustomMsg`，需已构建官方驱动的消息包 |
| `sensor_type=scan` | `sensor_msgs/LaserScan`，仍按真实TF转换、按base_frame高度过滤 |
| 地图 | `nav_msgs/OccupancyGrid`；地图需覆盖配置的整个场地边界 |
| 定位质量 | `std_msgs/Bool`，有效心跳不能过期；不能用永久发布True的命令伪造定位可信 |

`/navigation/command_preview`是Float32MultiArray协议控制量，不是Twist，也不是m/s；程序不发布`/cmd_vel`，避免与别的底盘驱动误接。

## 2. 到NUC先运行这些

把新版压缩包传到个人用户目录，解压后进入包含`main.py`、`real_vehicle.py`的`navigation_starter`文件夹。下面命令都从这个文件夹执行。

```bash
source /opt/ros/humble/setup.bash
python3 main.py test
python3 simulate.py
python3 check_vehicle.py
```

预期54项测试通过，三种离线场景完成。设备检查不会打开串口，只记录ROS包、话题、节点、网卡和串口路径，结果保存于`logs/device_check_*.json`。

检查依赖：`rclpy`、`tf2_ros`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`、`sensor_msgs_py`、`std_msgs`、`std_srvs`；串口另需pySerial。原依赖`requirements.txt`只包含pySerial，不会安装ROS。

如果NUC缺Python串口包，可在自己的venv中安装：

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --no-index --find-links vendor -r requirements.txt
```

已附pySerial官方发布的离线wheel，串口依赖安装不需要互联网。之后用`.venv/bin/python`替换文档里的`python3`。若venv或系统ROS依赖缺失，请器材负责人处理；不在共享NUC擅自执行系统安装或改其他用户配置。Livox SDK2和公开定位栈的依赖没有打入这个wheel，仍需现场检查。

## 3. 不知道驱动或定位程序时

你的规则禁止使用SRM队内既有代码；允许公开开源项目。因此可以询问已有设备参数和公共软件安装位置，但不能直接复用规则禁止的队内实现。

先执行`check_vehicle.py`，判断`livox_ros_driver2`是否已安装。若有，按该版本README与实际网卡/IP配置启动，不要猜雷达地址。官方ROS2驱动的常用启动命令为：

```bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

启动文件输出类型取决于`xfer_format`，请用下面命令确认，不能只根据话题名字判断：

```bash
ros2 topic list -t
ros2 topic info /livox/lidar -v
```

驱动只产生雷达/IMU数据，不等于定位已经完成。公开参考方案是北极熊战队`pb2025_sentry_nav`，将Point-LIO里程计、small_gicp重定位、雷达与底盘TF、地图等分开。它默认命名空间为`red_standard_robot1`，默认帧和URDF也属于它自己的车，不能不改参数就套到本车。

公共源码可单独放个人目录：

```bash
mkdir -p ~/srm_public_ws/src
git clone --recursive https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav.git ~/srm_public_ws/src/pb2025_sentry_nav
```

这是下载源码，还没有完成构建。构建前依照上游README检查small_gicp、ROS依赖、Livox SDK2；不要直接执行上游的sudo安装流程。共有NUC缺系统依赖时交给负责人处理，自编第三方库应安装到个人前缀。构建命令是工作空间内`colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`，成功后才source该工作空间的`install/setup.bash`。

上游实车建图入口为`ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True`。这只是上游入口示例：必须先改其雷达配置、机器人TF/外参和话题命名空间，且不要把上游默认Nav2输出接到比赛底盘。建图模式不等于已有先验地图上的可靠重定位；正式运行须使用你们场地的地图、实际初始位置，并检查定位表现。

## 4. 配置真正的现场输入

编辑`config/real_vehicle.json`：

- `map_frame`、`base_frame`：真实TF帧；例如公开工程可能用`map`、`chassis`，本车xacro是`base_link`。必须先确认，不能仅改名字来掩盖不存在的TF。
- `sensor_topic`、`sensor_type`：实际雷达或去畸变点云话题和类型。
- `pose_mode`、`pose_topic`：真实底盘位姿来源；雷达位置不能当成车体中心。
- `map_topic`：真实地图；默认必须收到地图，未知格阻塞。若现场只有明确测量的矩形空场地且没有地图，才可设置`map_required=false`，此时真实墙面完全依赖传感器输入，不能假装已经接入全局地图。
- `cloud.min_height`、`max_height`、`self_filter_radius`：在base_frame中滤除地面和本车反射；示例高度未经实测，必须用RViz检查，不要把真实障碍滤掉。该版本使用高度过滤，不是复杂地形地面分割；低于最低高度的障碍可能被忽略。
- `serial_device`：经过设备检查确认的真实串口路径。

点云必须有正常时间戳，TF要能在点云时间转换到map和base。程序不会刷新旧消息时间戳来假装数据是新的。点云超出预算会停车报错，不静默丢弃障碍物；修改预算前测量NUC处理时间。

地图输入按格子的外接圆保守转换；未观测的动态障碍仍依赖雷达。扫描有遮挡和盲区，软件不能凭“没看到点”证明全部空间安全。原始Mid-360点云转入此桥接层没有实现逐点去畸变，建议使用定位方案已去畸变的输出；需要通过真实运动测试确认效果。

## 5. 定位有效性

默认订阅`/localization_valid`，需要定位模块持续发布有效性，超时会停车。若公开定位程序没有这个话题，可使用`localization_monitor.py`，但先填写`quality_monitor`：输入话题类型、header帧、child帧、协方差阈值。

```bash
python3 localization_monitor.py
```

默认`source_confirmed=false`，监视器会拒绝启动。核对源数据与质量含义后再设置为true；不能为了通过检查直接改成true。默认零协方差视为质量未知。若定位器总发零协方差，必须确认它的质量指标，改接其真正的状态；显式允许零协方差也仅检查时效和位姿数值，不证明定位可靠。

这层监视器检查时间戳、帧和协方差，不独立检测全局定位漂移。需要现场验证重定位正确、地图坐标一致；已有有效TF也不能单独证明定位可信。

## 6. 先无串口试运行

在真实驱动、定位、地图和质量源启动后：

```bash
python3 real_vehicle.py
```

默认不打开串口。另开终端source ROS与公共软件工作空间后：

```bash
ros2 topic echo /navigation/status
```

状态显示`READY`才具备输入条件。`WAIT_POSE`检查TF/位姿，`WAIT_LOCALIZATION_QUALITY`检查质量心跳，`WAIT_OBSTACLES`检查点云和时间戳，`WAIT_MAP`检查地图、覆盖范围。`error`字段提供具体原因。

如需预演导航计算，可调用下面服务；程序仍不发送底盘指令，但真实车辆不移动时，它不会自己完成路线。

```bash
ros2 service call /srm_navigation/start std_srvs/srv/Trigger '{}'
```

RViz添加Path显示`/navigation/reference_path`，核对八字路线、地图、真实停车点与桩。`logs/run_*.jsonl`保存配置、位姿、状态、计算控制量和实际输出控制量。

## 7. 在NUC上验证ROS闭环，不连接底盘

使用三个终端，在每个终端执行`source /opt/ros/humble/setup.bash`和`export ROS_DOMAIN_ID=77`，进入项目目录。

终端一：

```bash
python3 mock_ros_vehicle.py
```

终端二：

```bash
python3 real_vehicle.py
```

等状态READY，终端三：

```bash
ros2 service call /srm_navigation/start std_srvs/srv/Trigger '{}'
ros2 topic echo /navigation/status
```

演示程序提供假TF、假点云、空地图与有效心跳，并按预览控制量移动虚拟位姿；无需Gazebo，不访问串口。域77明确禁止实车程序连接串口。默认实车速度较低，ROS演示全圈会花几分钟。

结束后关闭演示终端。实车调试使用新的终端，不要沿用域77，也不要把假输入当成真实传感器数据。

本包已在Windows运行纯Python单元与闭环测试，尚未在ROS2环境运行上述节点演示；这一步须在NUC执行确认。

## 8. 实车运动前的标定

沿用`docs/03_lab.md`的默认零速与短时单轴测试入口；一个串口只能由一个进程占用，不能与实车自动入口同时运行。记录正负vx、vy方向及距离，填写`config.json`的真实`body_from_command`，停车点、桩和场地边界全部换成同一地图坐标下的实测数据。

在`config/real_vehicle.json`中，只有实测完成后才填写：

- `field_confirmed=true`
- `calibration_confirmed=true`
- `lidar_extrinsics_confirmed=true`
- `emergency_stop_confirmed=true`

这些标志控制实车启动，不是让用户通过勾选代替测量。`config.json`原有的`calibration_confirmed`是旧示例提示，实际启动以实车配置中的标志为准。

底盘wz字段仍固定0，不能通过本程序主动控制车体旋转；位置控制使用实时yaw换算。如果现场底盘会自动旋转、方向漂移或惯性明显，需要调整实测标定并检查TF、制动表现。

## 9. 串口零速、单点、八字的顺序

先启动仅零速模式：

```bash
python3 real_vehicle.py --connect-serial --device /dev/serial/by-id/实际底盘设备
```

即使调用start，未加`--arm`时串口也只发送零控制量。确认停车响应后退出该进程。

先选择现场空旷范围内的一点，坐标必须实测：

```bash
python3 real_vehicle.py --connect-serial --device /dev/serial/by-id/实际底盘设备 --arm --goal 实测X 实测Y
```

车辆位于实测停车点、状态READY后，在裁判认可的开始方式下调用start服务。单点模式到点停车，不自动返回。不要把字面量“实测X”粘贴到命令中。

单点通过再启动八字：

```bash
python3 real_vehicle.py --connect-serial --device /dev/serial/by-id/实际底盘设备 --arm
```

```bash
ros2 service call /srm_navigation/start std_srvs/srv/Trigger '{}'
```

start之后不需要持续人工输入控制量；定位、感知、任务状态机与串口循环在NUC运行，完成路线后持续输出零控制量。一次运行仅允许开始一次，新一轮需停止并重启程序。开始触发的具体安排需赛务确认，正式运行期间不通过键盘、遥控器或远程桌面干预运动。

调试中可使用stop服务：

```bash
ros2 service call /srm_navigation/stop std_srvs/srv/Trigger '{}'
```

stop与定位跳变、串口故障会锁定停止，不能再start恢复。正常退出尽力发送三帧零速；断电、强制杀进程或断线时不能保证零速送达，下位机无超时停车保护，现场硬件急停必须确认。软件stop不是硬件急停。

## 10. 明天优先带回哪些结果

设备检查JSON、ROS节点启动日志、点云/定位消息类型、TF树、状态报错、单轴标定记录、实测场地坐标。先达到真实数据READY和单点停车，再尝试完整任务。程序交付不能替代这些硬件验证。

## 参考来源

只借鉴模块分工与公开接口，没有复制战队的算法源文件；本项目保留本次校内赛19字节协议，不能换成别队串口协议。

- RoboMaster官方资料索引：https://bbs.robomaster.com/article/9269
- 北极熊战队公开ROS2导航工程：https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav
- Livox官方驱动：https://github.com/Livox-SDK/livox_ros_driver2
- ROS2 Humble TF接口源码：https://github.com/ros2/geometry2/blob/humble/tf2_ros_py/tf2_ros/buffer.py
- ROS2 Humble点云读取接口：https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs_py/sensor_msgs_py/point_cloud2.py

公开项目版本与依赖会更新，实际部署记录对应Git提交；参考仓库的许可证、子模块许可证均应保留。上面的实车入口来自上游README，本包没有在本机编译该公共导航栈。
