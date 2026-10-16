# 实车快速接通：单点 → 绕桶路点 → 最终路线

本版优先验证实车自主到点和绕桶。纯Python测试通过，尚未在本车或ROS 2中验收。
串口模块未修改，之前的间歇异常仍需解决；不忽略接收错误，不自动重连恢复运动。

## 任务模式

- `--goal X Y`：一个地图坐标目标，到点停车，不自动返回。
- `--waypoints config/field_task.json`：按顺序访问实测路点，最后一点停车。
- 不带以上参数：保留旧版八字，仍需实测场地参数。

路点模式不再使用示例中的两根桩，而使用任务文件中的桶圆形障碍、真实地图和实时点云。
不识别桶的类别，也不根据照片自动获取地图坐标。必须从指定侧绕桶时，应在该侧设置中间路点；仅给桶后方终点，规划器会自行选择绕行侧。
需要返回时把停车位作为最后一个路点。最终路线以现场赛务确认要求为准。

## 离线检查（不需要ROS，不访问串口）

```bash
python3 main.py test
python3 prepare_task.py --task config/waypoints_template.json --output logs/task_check.json
```

模板仅为示例。planning_feasible=true只证明在配置圆形障碍和边界下可规划，不能证明实车就绪。

## 第一次到实验室：先接通真实输入

把本版解压到个人用户的新目录，进入包含main.py的文件夹。加载ROS及实际使用的公开驱动/定位工作空间环境。

```bash
source /opt/ros/humble/setup.bash
python3 check_vehicle.py
```

修改`config/real_vehicle.json`中的真实map/base坐标系、地图、定位质量源、点云类型和话题。
本版PointCloud2默认话题改为此前设备清单中的`/livox/lidar/pointcloud`，现场仍应核对。
原始点云没有在本包逐点去畸变，运动时需检查拖影，优先使用已经验证的去畸变点云输出。
必须独立启动Mid-360驱动、建图/定位和地图加载；本程序不会自动终止共享电脑后台程序。
定位质量心跳不能用恒定True代替。WAIT_POSE/WAIT_LOCALIZATION_QUALITY/WAIT_OBSTACLES/WAIT_MAP分别表示位姿、定位质量、障碍或地图尚未就绪。

## 填实测路点并预演

```bash
cp config/waypoints_template.json config/field_task.json
```

编辑自己的field_task.json：

- map_frame：与定位、地图一致。
- parking：本轮开始时车体中心的实测坐标；程序要求从这里附近开始。
- bounds：实测[xmin,xmax,ymin,ymax]；地图需覆盖整个边界。
- fixed_obstacles：每项[桶中心X,桶中心Y,桶半径]；可靠地图已包含桶时，可显式设[]。
- waypoints：依次到达的地图坐标，不能落在桶内或未知区域。
- measurements_confirmed：实测后才设true。模板false会拒绝带--arm运行。

```bash
python3 prepare_task.py --task config/field_task.json --output logs/task_check.json
python3 real_vehicle.py --waypoints config/field_task.json
```

默认不打开串口。另开已加载ROS环境的终端：

```bash
ros2 topic echo /navigation/status
```

RViz查看/navigation/reference_path核对目标顺序；它表示任务路点，不是局部A*绕障路径。
真实输入READY后，可调用start预演计算，车未移动时不会自动完成任务：

```bash
ros2 service call /srm_navigation/start std_srvs/srv/Trigger '{}'
```

## 实车验证顺序

先验证串口持续收发和停车；短时逐轴标定后填写config.json的body_from_command。
串口参数按协议保持115200、8N1、无流控。核实现场尺寸、速度标定、雷达外参、急停后，才填写实车配置中的四个确认标志。
退出无串口预演进程，只保留一个控制进程。先连接但只发零速：

```bash
python3 real_vehicle.py --waypoints config/field_task.json --connect-serial --device /dev/serial/by-id/usb-1a86_USB_Single_Serial_5909093824-if00
```

未加--arm，即使调用start也只发零速。通信失败时不进入运动阶段。
退出零速进程后，将任务先设为一个近距离空旷目标（实测坐标），启动：

```bash
python3 real_vehicle.py --waypoints config/field_task.json --connect-serial --device /dev/serial/by-id/usb-1a86_USB_Single_Serial_5909093824-if00 --arm
```

等待READY，现场允许开始后再调用start服务。单点停车通过后，再填桶侧中间点、桶后目标，最后完整比赛路线。
目标被占据时BLOCKED会等待，不跳过目标。普通输入失效归零，恢复后可能继续；stop、定位跳变、串口故障锁定停止，需要重启任务。

```bash
ros2 service call /srm_navigation/stop std_srvs/srv/Trigger '{}'
```

软件stop不是硬件急停。协议说明下位机保持最后有效指令，断线时不能保证停车帧送达。

## 调试记录

保留设备检查JSON、task_check.json、run_*.jsonl、实际任务配置和录像。
日志controller_command是准备输出，serial_connected只表示创建了链路，均不是下位机执行回执。
任务结束记录mission_complete、用时、轨迹长度和已完成路点数。八字模式另核验绕桩圈数和返回位置，路点模式不受圈数限制。
