# 赛前遥控建图、赛时加载地图的八字绕桩版本

这是独立的新版本，原有目录没有覆盖。新比赛入口是 **`premap_vehicle.py`**，不是旧的 `main.py serial`，也不是直接启动旧 `real_vehicle.py`。

当前交付是通过桌面离线测试的候选实现，**不是已经实车跑通的成品**。记录见 `reports/desktop_validation.json`。ROS2 Humble 启动、真实 TF、AMCL 收敛、USB 故障与物理停车尚需现场验收，所有现场确认开关默认关闭。

## 方案与功能

赛前用原遥控器慢速探索场地，公共雷达驱动与 LIO 提供点云和局部里程计。本项目转换真实车体坐标，通过公开 SLAM Toolbox 生成地图，保存 `yaml + pgm + manifest.json`。在保存的地图上点击停车位、两个杆中心和安全范围，不把示例坐标当实测坐标。

比赛时加载地图，AMCL 重新定位。车辆在停车位就绪后接受一次开始触发，自主按顺序执行下瓣逆时针、上瓣顺时针的八字并返回停车位。交叉点按顺序通过，不按最近航点跳瓣。实时点云参与障碍膨胀和 A* 绕行；必经点被占据时等待，不跳点。记录实际轨迹的绕杆方向、圈数、停车误差和用时。

**两种地图不能混淆：**

- 本版导航使用由三维雷达投影后生成的二维 `yaml + pgm` 地图，AMCL 不是 PCD 匹配定位。
- `record_cloud_map.py` 可另存三维 PCD 供可视化参考；它不进行历史回环优化，不可冒充精确优化后的三维地图，也不作为本版 AMCL 输入。若考核另要求三维地图精度，请保存 rosbag，另用公开建图工具离线优化。

```text
race_map ── SLAM Toolbox（建图）或 AMCL（比赛）──> race_odom
race_odom ── LIO里程计 × 实测body→base变换 ──> race_base
race_base ── 实测雷达外参 ──> race_lidar
```

建图和定位阶段不能同时运行，防止两个 `race_map→race_odom` 发布者。不得发布假的静态 `map→odom`，不得只把 `camera_init` 改名成 `map`。`race_lidar` 是确认过相同原点、相同轴向的物理雷达帧隔离别名，安装变换由 `base_from_lidar` 表达。

## 1. 环境与现场必填项

将整个新目录单独放入自己的目录，例如 `~/srm_premap/navigation_starter`，不要覆盖现场版本。

```bash
cd ~/srm_premap/navigation_starter
source /opt/ros/humble/setup.bash
python3 bootstrap_dependencies.py
python3 run.py check_premap_environment.py
```

附带离线依赖适用于 Ubuntu22.04、x86_64、CPython3.10，校验后解压到本项目 `.deps/python310`。不需要 sudo、pip、venv，不改 `.bashrc`；重启后仍用 `python3 run.py 脚本名.py ...`。

ROS 依赖：`rclpy`、`tf2_ros`、`sensor_msgs_py`、标准消息包、`nav2_map_server`、`nav2_amcl`、`nav2_lifecycle_manager`、`slam_toolbox`、`pointcloud_to_laserscan`、`rviz2`。wheel不能代替ROS包。缺包时请管理员安装公开Humble软件包，或在个人工作空间构建公开源码，不绕过系统权限。

公共 Livox 驱动/公开 FAST-LIO 的启动、雷达网络和 LIO 外参是前置条件。本项目不启动、复制、修改队内私有 `rm_nav_bringup` / `selfdefined`；规则禁止使用队内既有代码。不要 kill 其他用户进程或关闭系统服务。

先核实真实输入，而不仅是进程名：

```bash
ros2 topic list -t
ros2 topic echo /Odometry --once
ros2 topic info /livox/lidar/pointcloud -v
```

核实 ROS_DOMAIN_ID、RMW/发现配置、话题类型、实际帧名、频率和时间戳。source不会启动节点，也不保证发现其他用户节点。

编辑 `config/infrastructure.json`：

- 输入话题和帧名是候选值，必须以现场输出为准。
- `body_from_base = T_body_base`：base中的点变换到LIO body；桥接计算 `T_odom_base = T_odom_body × T_body_base`。
- `base_from_lidar = T_base_lidar`：雷达中的点变换到车体。四元数 `[x,y,z,w]`，平移单位米。示例±0.4米是占位值，不能直接确认。
- `race_base` 取地面车体参考原点，低障碍高度过滤以它为准。确认真实车体中心、安装方向、LIO body/IMU语义。
- 确认雷达/LIO与ROS墙钟同一时间基准。不用接收时间覆盖陈旧消息时间。
- 完成后才设三个 `*_confirmed`。软件不会自动置真。

## 2. 遥控建图与保存

```bash
ros2 launch "$PWD/premap.launch.py" stage:=mapping
```

另开已加载ROS环境的终端运行RViz2，Fixed Frame设 `race_map`，添加 `/race/map`、`/race/points` 和TF。遥控慢速扫描整个待行驶范围，再回到起点观察回环重影。检查杆和低底座可见性；不能调低检查标准掩盖盲区。本项目建图阶段不打开控制串口。

也可直接加载附带的显示配置：`rviz2 -d config/premapped.rviz`，已设置地图、点云、有序路线和 `/race/initialpose` 工具话题；仍须核查界面实际显示。

停车并等地图/回环稳定后：

```bash
python3 run.py map_bundle.py --output maps/classroom
```

生成三个地图文件，禁止覆盖旧地图。布局改变或重新建图必须用新名字。可选三维参考点云采集：

```bash
python3 run.py record_cloud_map.py --output maps/classroom_reference.pcd --seconds 60
```

建议同时按真实输入话题保存rosbag以供重建。PCD参考导出器不回头优化历史点云，不能与精确三维建图混为一谈。

## 3. 地图点选、外廓和速度标定

先退出建图launch，启动已保存地图的定位阶段，再打开RViz查看**已保存**的地图。不要在继续回环变化的在线地图上采点：

```bash
ros2 launch "$PWD/premap.launch.py" stage:=localization bundle:="$PWD/maps/classroom.manifest.json"
```

保留这一个定位launch。另一个终端执行：

```bash
python3 run.py capture_field.py --bundle maps/classroom.manifest.json --output config/field_measured.json
```

在RViz Publish Point工具依次点击：停车中心、下杆中心、上杆中心、安全矩形内角A、对角B。上下杆按规则入口方向区分。矩形必须在比赛边界内，不是整间教室。工具不会自动确认现场数据。

编辑生成的 `field_measured.json`：

- `pole_radius` 取**底座**保守最大半径，不是细杆半径；大小不同时取较大底座。
- `robot_radius` 包含整车突出部分。xacro的0.29米只是所给模型的下限，不代表照片中的整车已验证。
- `margin` 覆盖定位、感知、制动和跟踪误差。未经制动标定不能直接提速。
- `body_from_command` 单位为“m/s 每单位协议控制量”。默认矩阵只表示轴向关系，不是速度标定。
- 采集正负双轴稳态响应样本，离线执行 `python3 run.py calibrate_motion.py --csv 样本.csv`。CSV列为 `command_vx,command_vy,body_vx_mps,body_vy_mps`；世界速度须先旋转到车体。还需测死区、回差、电量和停车距离，拟合工具不会自动确认。
- 默认 `max_speed_mps=0.05`、`control_limit=0.05` 为调试档，不是比赛档。验收后逐级调整；超过默认0.2控制量还需设置已实测的 `commissioned_control_limit`（本实现最高1），不得外推未测试范围。
- 坐标来自地图点选，不需手抄整条路线；外廓、底座和安全距离仍需实测。

模板生成圆形八字，不保证任意杆距/狭窄布局都可行。离线检查路线穿过占据、未知区域或越界时会拒绝启动，须修地图、布局或路线几何，不是关检查。

```bash
python3 run.py premap_vehicle.py --bundle maps/classroom.manifest.json --field config/field_measured.json --check
```

此检查不打开串口，也不证明真实定位/停车正确。换地图须重新选坐标，不能只手改digest字符串。

## 4. 加载地图重新定位，先预演

保留第3步已启动的一个定位launch；如果退出过才重新运行下式，**不要重复启动**。保留正确的公共传感器/LIO输入：

```bash
ros2 launch "$PWD/premap.launch.py" stage:=localization bundle:="$PWD/maps/classroom.manifest.json"
```

将车放停车位。RViz Fixed Frame设 `race_map`，将 **2D Pose Estimate话题设 `/race/initialpose`**，按真实位置/朝向给AMCL初值。初值不是定位结果：检查雷达/地图对齐、协方差收敛和质量持续有效；对称房间可能误匹配。

```bash
ros2 topic echo /race/localization_status
python3 run.py premap_vehicle.py --bundle maps/classroom.manifest.json --field config/field_measured.json
```

新任务默认不打开串口，**车不会动**。查看 `/navigation/status`、`/navigation/reference_path`、`/navigation/command_preview`。可在赛前这次预演用遥控检查沿途定位/感知，但这不是自动运动验收，更不是比赛运行。

定位心跳依据真实AMCL协方差、扫描/地图端点一致性、里程计时效和稳定等待生成；是质量启发式，不是绝对真值。禁止永久手发True，也不能拿FAST-LIO的全零协方差冒充全局定位。

## 5. 现场验收后的一次自主运行

先完成 `docs/08_现场验收与注意事项.md`。然后确认 `config/premapped.json` 的四项开关和field文件的两项开关。没有现场可立即操作的独立急停，不得只凭软件日志声称安全。

关闭只预演的任务进程，不能并行两个同名任务；建图不能同时启动，不能有第二个串口控制者。按赛务方要求在赛前选择上位机控制模式，本项目不猜遥控器开关位置。

```bash
python3 run.py premap_vehicle.py \
  --bundle maps/classroom.manifest.json \
  --field config/field_measured.json \
  --connect-serial --arm \
  --device /dev/serial/by-id/usb-1a86_USB_Single_Serial_5909093824-if00
```

设备名来自此前检测，仍须核对。启动后先零速，要求新鲜CRC正确反馈及地图、TF、感知、定位、起点条件。`armed=true`不等于已发车。

裁判开始时一次触发（先确认赛务方允许这个起跑接口）：

```bash
ros2 service call /srm_navigation/start std_srvs/srv/Trigger '{}'
```

成功后自主执行，不再键盘/遥控修正。临时障碍时停车/重规划，移开且感知定位恢复后继续；串口异常、真实位姿跳变、轨迹碰杆/出界锁定故障，不自动恢复。新一轮安全停车后重启任务清数据。规则未提供裁判电子接口，本版不捏造语音或game_progress起跑语义。

## 停车硬限制

下位机无通信超时停车，USB断开后可能保持最后运动指令。异常时软件禁止非零输出，退出按单帧间隔尝试20次零速，首帧失败仍继续，但**全部失败时无法保证物理停车**。PC掉电/强杀进程也无法补帧。软件stop服务不能替代独立急停。

正常读取超时 `b''` 不报错；pySerial `returned no data` 异常不是普通超时。首次异常即锁运动，宽限期只零速观察，反馈恢复也不解锁。DTR/RTS在打开前设False仍需实测适配器/固件响应，不能保证消除电平毛刺。

## 文件与测试

- 比赛入口 `premap_vehicle.py`；基础链路 `premap.launch.py`、`premap_bridge.py`、`premap_quality.py`。
- 地图 `map_bundle.py`；点选 `capture_field.py`；标定 `calibrate_motion.py`。
- 问题处理 `docs/07_问题审核与新版设计.md`；现场验收/开发禁忌 `docs/08_现场验收与注意事项.md`。
- 旧 `main.py`、`real_vehicle.py`、mock和旧配置保留作回归对照，不是比赛部署入口。mock域77禁止串口。旧docs为历史资料，以本README及07/08为准。

```bash
python3 run.py benchmark_premap.py --output reports/target_validation.json
```

测试和合成性能不是ROS集成/实车通过证明。请保留NUC上的实际验证记录。

公共接口参考：[Humble AMCL](https://github.com/ros-navigation/navigation2/blob/humble/nav2_amcl/src/amcl_node.cpp)、[SLAM Toolbox启动](https://github.com/SteveMacenski/slam_toolbox/blob/humble/launch/online_async_launch.py)、[点云转扫描](https://github.com/ros-perception/pointcloud_to_laserscan/blob/humble/README.md)。各依赖遵守原许可证，wheel保留原许可证；未引用队内私有导航/固件实现。
