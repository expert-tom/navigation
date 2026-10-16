"""读取本项目xacro中的固定圆柱碰撞几何，不执行xacro中的Python代码。

支持数字属性、简单算术表达式、固定关节、圆柱collision。
遇到未支持的碰撞几何或关节结构时明确报错，不静默漏掉车体部件。
"""
import ast
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
XACRO = "{http://www.ros.org/wiki/xacro}"


def number(text, properties):
    if text.startswith("${") and text.endswith("}"):
        text = text[2:-1]
    def evaluate(node):
        if isinstance(node, ast.Constant) and type(node.value) in (int, float):
            return float(node.value)
        if isinstance(node, ast.Name) and node.id in properties:
            return properties[node.id]
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
            return evaluate(node.operand) * (-1 if isinstance(node.op, ast.USub) else 1)
        if isinstance(node, ast.BinOp):
            a, b = evaluate(node.left), evaluate(node.right)
            if isinstance(node.op, ast.Add): return a+b
            if isinstance(node.op, ast.Sub): return a-b
            if isinstance(node.op, ast.Mult): return a*b
            if isinstance(node.op, ast.Div): return a/b
        raise ValueError("不支持的xacro表达式：" + text)
    value = evaluate(ast.parse(text, mode="eval").body)
    if not math.isfinite(value):
        raise ValueError("模型中含非有限数字")
    return value


def vector(text, properties):
    values = re.findall(r"\$\{[^}]*\}|\S+", text)
    if len(values) != 3:
        raise ValueError("模型向量需要3个分量：" + text)
    return tuple(number(v, properties) for v in values)


def transform(point, origin, properties):
    if origin is None:
        return point
    tx, ty, tz = vector(origin.get("xyz", "0 0 0"), properties)
    roll, pitch, yaw = vector(origin.get("rpy", "0 0 0"), properties)
    x, y, z = point
    y, z = math.cos(roll)*y-math.sin(roll)*z, math.sin(roll)*y+math.cos(roll)*z
    x, z = math.cos(pitch)*x+math.sin(pitch)*z, -math.sin(pitch)*x+math.cos(pitch)*z
    x, y = math.cos(yaw)*x-math.sin(yaw)*y, math.sin(yaw)*x+math.cos(yaw)*y
    return x+tx, y+ty, z+tz


def convex_hull(points):
    points = sorted(set(points))
    def cross(a,b,c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    def half(items):
        result = []
        for p in items:
            while len(result) >= 2 and cross(result[-2], result[-1], p) <= 0:
                result.pop()
            result.append(p)
        return result
    return half(points)[:-1] + half(reversed(points))[:-1]


def inspect_robot(path=None):
    path = Path(path) if path is not None else ROOT/"assets/sentry_robot_cylinder.xacro"
    root = ET.parse(path).getroot()
    if root.findall(XACRO+"include"):
        raise ValueError("当前模型检查器不支持include，请提供完整固定圆柱模型")
    properties = {"pi": math.pi}
    for item in root.findall(XACRO+"property"):
        properties[item.get("name")] = number(item.get("value"), properties)
    joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    all_points, components, no_collision = [], [], []
    for link in root.findall("link"):
        name = link.get("name")
        collisions = link.findall("collision")
        if not collisions:
            no_collision.append(name)
            continue
        joint = None
        if name != "base_link":
            joint = joints.get(name)
            if joint is None or joint.get("type") != "fixed" or joint.find("parent").get("link") != "base_link":
                raise ValueError("仅支持直接固定在base_link上的碰撞部件："+name)
        for collision in collisions:
            geometry = collision.find("geometry")
            cylinder = geometry.find("cylinder") if geometry is not None else None
            if cylinder is None or len(geometry) != 1:
                raise ValueError("不支持的碰撞几何："+name)
            radius = number(cylinder.get("radius"), properties)
            height = number(cylinder.get("length"), properties)
            if radius <= 0 or height <= 0:
                raise ValueError("圆柱尺寸必须大于0")
            outer = radius/math.cos(math.pi/64)
            points = []
            for z in (-height/2, height/2):
                for i in range(64):
                    angle = 2*math.pi*i/64
                    p = transform((outer*math.cos(angle), outer*math.sin(angle), z), collision.find("origin"), properties)
                    p = transform(p, joint.find("origin") if joint is not None else None, properties)
                    points.append(p)
            all_points.extend(points)
            components.append({"link": name, "radius_m": radius, "length_m": height,
                               "outline_xy": convex_hull([(p[0],p[1]) for p in points])})
    if not all_points:
        raise ValueError("模型没有可用碰撞几何")
    bound = max(math.hypot(p[0],p[1]) for p in all_points)
    args = {a.get("name"): a.get("default") for a in root.findall(XACRO+"arg")}
    lidar = joints.get("livox_frame")
    return {"base_frame": "base_link", "lidar_frame": "livox_frame",
            "body_radius_m": next(c["radius_m"] for c in components if c["link"] == "base_link"),
            "collision_bound_m": bound, "recommended_radius_m": math.ceil(bound*100)/100,
            "collision_z_range_m": [min(p[2] for p in all_points),max(p[2] for p in all_points)],
            "footprint_xy": convex_hull([(p[0],p[1]) for p in all_points]),
            "components": components, "links_without_collision": no_collision,
            "lidar_joint_origin": dict(lidar.find("origin").attrib) if lidar is not None else None,
            "lidar_argument_defaults": args, "lidar_extrinsics_confirmed": False,
            "notes": ["半径覆盖模型中明确提供的collision，不包含无碰撞网格的雷达外形。",
                      "雷达visual origin只决定网格显示，不能当成雷达安装TF。",
                      "四轮关节为fixed；文件未提供可直接驱动的Gazebo底盘插件。",
                      "外接圆包含姿态变化的平面投影，额外安全距离由margin设置。"]}


def model_for_config(config):
    source = config.get("robot_model")
    if not source:
        return None
    return inspect_robot(ROOT / source)


if __name__ == "__main__":
    report = inspect_robot()
    target = ROOT/"assets/robot_model_report.json"
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"车体半径={report['body_radius_m']:.3f}m")
    print(f"碰撞包络上界={report['collision_bound_m']:.6f}m；推荐规划半径={report['recommended_radius_m']:.2f}m")
    print(target)
