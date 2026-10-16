# Ubuntu 22.04 / ROS 2 Humble实验室接入

实车接入版已提供ROS桥接、占据栅格输入和自动任务入口，使用方式与外部SLAM部署见05_real_vehicle.md。下面保留早期原型的串口标定流程；“尚未接入”描述的是早期版本。

更新：已接入用户提供的xacro碰撞尺寸，规划半径为0.29m，详见04_robot_model.md。
以下所说的模型/TF现场接入指在ROS中加载和验证；模型文件本身已保存在assets中。

## 当前不能自动完成的工作

本项目没有Mid-360设备数据，也没有现场地图、定位TF和真实标定；已获得车体xacro。
建图定位必须接入已有开源方案或自行开发后在设备上验证；不能以理想仿真位姿代替。
当前没有Gazebo插件、ROS自动导航节点或Nav2配置，不能直接向TurtleBot3发布当前控制量。

Livox官方驱动支持Mid-360及Ubuntu 22.04 / Humble：
https://github.com/Livox-SDK/livox_ros_driver2
现场若已有驱动，优先检查和复用自己的环境；不要改系统级配置或其他人的用户环境。

## 部署基础项目

将完整navigation_starter复制到自己的用户目录：

```bash
cd ~/navigation_starter
python3 -m venv .venv
.venv/bin/python main.py test
.venv/bin/python simulate.py
.venv/bin/python -m pip install -r requirements.txt
```

若venv组件缺失，联系管理员按比赛要求处理，不擅自修改共用NUC系统。
仿真无需第三方依赖，可以直接使用python3运行。

## 先零速，再短时低速

确认急停、自动控制模式、调试区域和串口权限后，使用实际by-id路径：

```bash
.venv/bin/python main.py serial --device /dev/serial/by-id/实际设备名 --seconds 5
```

默认只发送零速，启动后接收解析反馈，退出尽力间隔发送三帧零速。
反馈不一定在调试时持续发送，收不到反馈不能单独证明发送失败。

逐轴测试时：

```bash
.venv/bin/python main.py serial --device /dev/serial/by-id/实际设备名 --vx 0.03 --seconds 0.5 --arm
```

同样分别测试负vx、正vy、负vy；记录实际方向，不连续大范围运行。
入口限制运动量不超过0.05、时长不超过2秒。
默认不自动重连，不在串口故障后自动恢复运动。
线缆断开或进程强制结束后无法保证零速送达，必须确认现场急停。

## 需要接入导航核心的数据

`Navigator.step(sample, now)`需要：

```python
sample = {
    "pose": [x, y, yaw],              # map坐标米，yaw弧度
    "stamp": pose_monotonic_time,     # 当前电脑的单调时钟域
    "localized": True,               # 定位模块明确判断有效
    "obstacles": [[x, y, radius]],    # map坐标中的圆形障碍
    "obstacle_stamp": scan_monotonic_time
}
```

不要每次读取时重写旧数据时间戳。ROS消息时间不是time.monotonic；适配时需计算消息年龄，
再换算为同一单调时钟域，保留定位/感知的新鲜度。收到消息也不等于定位可信。
定位质量、地图对齐、点云高度过滤、地面剔除、TF外参、动态障碍提取都需独立完成。

当前地图表示为边界和圆形障碍，没有读取ROS占据栅格地图；真实墙面不能只靠两根桩表示。
ROS地图或点云须转换成正确障碍输入，或扩展Planner支持占据栅格地图。

## 标定矩阵

配置中的body_from_command表示：

```text
[车体真实x速度]   [m00 m01] [协议vx]
[车体真实y速度] = [m10 m11] [协议vy]
```

默认[[0,-1],[1,0]]只用于理想仿真，是假定速度增益，不是实测速度换算。
应依据车体坐标定义，测量每轴每单位控制量产生的速度，填入矩阵。
同时测量车体外接半径、桩半径、边界和停车位；保持坐标一致。
calibration_confirmed=false是状态提示，不是实车自动模式的软件解锁逻辑；当前未提供实车全路线入口。

## 验收前必须现场验证

1. 雷达稳定、时间戳和TF正确，地图与场地一致。
2. 定位可靠，遮挡恢复和位姿跳变处理正确。
3. 正负轴向、增益、零速停车和实际发送频率经过测量。
4. 车辆惯性、制动距离和碰撞包络进入速度与安全参数。
5. 完整8字顺序正确，交叉点不跳分支，返回停车位。
6. 人为干扰下无碰撞，定位/感知失效时停车。
7. 全程无人接管，代码上传GitHub，并完成重复运行测试。

NUC实车测试、真实SLAM、地图定位、ROS桥接和Gazebo比赛模型仍是后续工作；本项目不能宣称比赛已完成。
