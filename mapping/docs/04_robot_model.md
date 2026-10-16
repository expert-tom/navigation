# 实车xacro接入说明

来源：用户提供的sentry_robot_cylinder.xacro，原样保存在assets目录。
本次没有修改原始模型，也未安装ROS或Gazebo。

## 已确定并用于代码的几何

| 项目 | 模型数值 |
|---|---|
| base_link车体圆柱 | 半径0.27m，高0.20m，中心z=0.15m |
| 四个轮心 | x/y分别为±0.18m，z=0.075m |
| 轮子圆柱 | 半径0.075m，宽0.04m |
| 关节 | 四轮均通过fixed joint连接base_link |
| 完整碰撞几何平面上界 | 约0.284642m |
| 向上取整的规划半径 | 0.29m |
| 额外安全距离 | 0.10m，为配置策略，不是URDF测量值 |

之前配置半径0.20m小于模型。现在所有碰撞检查、边界检查、绕桩可行性检查、
仿真净空统计和回放安全圈均使用0.29m。
障碍物碰撞判定使用：车体规划半径0.29 + 安全距离0.10 + 障碍自身半径。

## 为什么不是0.33m

此前约0.33m是未计入轮轴旋转的粗略估计。文件中的轮轴沿轮心径向，
根据安装旋转，最远轮面点约为sqrt((sqrt(2)*0.18+0.02)^2+0.075^2)。
代码使用每个圆柱的外切64边形柱，再应用collision origin和joint origin旋转平移，
求所有点相对base_link原点的平面距离上界。外切而不是内接，多边形不会低估圆柱。
结果向上取整到厘米，得到0.29m。此圆形包络不依赖车体yaw。

## 新增代码与检查

- robot_model.py：受限、只读的模型检查器；支持本文件的数字属性、简单算术、直接固定关节、圆柱collision。
- 模型解析不执行任意Python表达式。遇到不支持的碰撞体或关节结构会报错。
- navigation.py：启动时检查配置半径不能小于模型上界要求；线段碰撞从离散采样改为解析最近距离。
- 回放：显示车体与四轮的俯视轮廓、规划安全圈及模型尺寸。
- assets/robot_model_report.json：几何分析结果，可用python robot_model.py重新生成。

## 不能从文件确定的内容

1. 雷达外参：livox_joint用$(arg xyz)和$(arg rpy)，默认都是0。注释提到measurement_params_real.yaml，但没有提供该文件。
2. 雷达visual origin是STL网格的显示偏移，不是base_link到雷达的安装变换。
3. 雷达STL引用package://pb_rm_simulation/meshes/mid360.stl，本项目没有这个外部网格。
4. livox_frame没有collision，因此包络只覆盖文件给出的车体/轮子collision，无法证明涵盖所有真实线缆、贴纸支架等。
5. fixed轮关节与占位惯量不能用于推断可驱动动力学，也没有Gazebo控制插件。
6. 全向移动能力、协议到实际速度映射、制动距离仍需实车测量，当前理想动力学不变。
7. 场地、桩半径、停车位仍为仿真示例，未从URDF获取。

## 运行新版

```bash
python3 robot_model.py
python3 main.py test
python3 simulate.py
```

Windows用python代替python3。打开simulation_results/playback.html，勿打开viewer_template.html。
虚拟机中旧项目不会自动更新；将新版压缩包解压到一个新目录后运行，便于保留旧版本。
