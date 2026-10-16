"""合成点云生成器：造"带半球底座的立杆 + 地面 + 干扰物"的雷达点云。

用途是让杆检测算法在没有雷达、没有现场的情况下可测：
  - 可以注入地面噪声、遮挡、多出的干扰圆柱、自车反射；
  - 能给出真值（杆心、底座半径），从而量出检测误差；
  - 只依赖标准库，Windows/Ubuntu 都能跑。

几何模型（按现场照片的橙色半球底座 + 细杆）：
    地面         z = 0 附近的带噪平面
    半球底座     球心 (cx, cy, dome_center_z)，半径 dome_radius，只保留 z >= 0 的半球
    杆           竖直圆柱，半径 pole_radius，从底座顶部延伸到 pole_height
    干扰物       任意位置的圆柱（椅子/柱子/人腿）
    自车         车体附近的点（应被 self_filter_radius 滤掉）
"""
import argparse
import math
import random


def dome_points(center, radius, center_z, count, rng):
    """球面上的点（只取露出地面的半球），用球坐标采样。"""
    points = []
    for _ in range(count):
        cos_theta = rng.uniform(0.0, 1.0)
        sin_theta = math.sqrt(max(0.0, 1.0-cos_theta*cos_theta))
        phi = rng.uniform(0.0, 2.0*math.pi)
        z = center_z + radius*cos_theta
        if z >= 0.0:
            points.append((center[0] + radius*sin_theta*math.cos(phi),
                           center[1] + radius*sin_theta*math.sin(phi), z))
    return points


def cylinder_points(center, radius, z0, z1, count, rng):
    """竖直圆柱侧面上的点。"""
    points = []
    for _ in range(count):
        phi = rng.uniform(0.0, 2.0*math.pi)
        z = rng.uniform(z0, z1)
        points.append((center[0]+radius*math.cos(phi), center[1]+radius*math.sin(phi), z))
    return points


def ground_points(bounds, count, noise, rng):
    points = []
    for _ in range(count):
        x = rng.uniform(bounds[0], bounds[1])
        y = rng.uniform(bounds[2], bounds[3])
        points.append((x, y, rng.gauss(0.0, noise)))
    return points


def robot_points(pose, self_radius, count, rng):
    """自车反射：应被 self_filter_radius 滤掉。"""
    x, y, yaw = pose
    points = []
    for _ in range(count):
        phi = rng.uniform(0.0, 2.0*math.pi)
        r = rng.uniform(0.05, self_radius*0.9)
        points.append((x+r*math.cos(phi), y+r*math.sin(phi), rng.uniform(0.0, 0.2)))
    return points


def scene(poles, pose=(0.0, 0.0, 0.0), bounds=(-3.0, 3.0, -3.0, 3.0),
          dome_radius=0.18, dome_center_z=None, pole_radius=0.02, pole_height=1.8,
          dome_samples=900, pole_samples=260, ground_samples=1400, ground_noise=0.012,
          self_radius=0.29, self_samples=60, clutter=(), clutter_samples=300,
          occlude=None, seed=7, self_filter_radius=None, map_frame=False):
    """生成一帧点云。返回 (点列表, 真值字典)。

    poles                : [((x,y), ...)] 每根杆的底座中心，**在 base_link 系**
                           （即"车头朝 +x 时看到的位置"）。map_frame=True 时
                           它们会随所有点一起被转到 map 系。
    pose                 : 车体位姿 (x, y, yaw)，即传感器所在位置
    map_frame            : True 时把每个点转到 pose 所在坐标系（模拟真实链路里的 TF）。
                           **累积多帧时必须用 True**：否则不同时刻的点留在 base_link 系，
                           直接叠加会把车走过的位移"拖"进杆的团块里（实测半径 0.18→0.54）。

    dome_center_z        : 半球底座的球心离地高度。**默认 0** —— 底座是扣在地上的
                           半球（球心在地面），车会撞到的最宽处就是 z≈0 处、
                           半径等于 dome_radius 的那一圈。设成正值会得到"碗"形。
    clutter              : [((x,y), radius, z1), ...] 干扰圆柱（base_link 系）
    occlude              : (cx, cy, keep_angle_span_deg) 只保留朝向该点一定角度内的点
                           （cx, cy 也在 base_link 系）
    self_filter_radius   : 模拟真实链路的自车点过滤：丢弃距车体中心小于该半径的点。
                           默认取 self_radius —— **不做这个过滤会产生伪点**
                           （真实链路里这些点会被滤掉）。
    """
    rng = random.Random(seed)
    if dome_center_z is None:
        dome_center_z = 0.0
    points = []
    truth = {"poles": [], "dome_radius": dome_radius, "dome_center_z": dome_center_z}
    for center in poles:
        truth["poles"].append({"center": tuple(center), "radius": dome_radius})
        dome = dome_points(center, dome_radius, dome_center_z, dome_samples, rng)
        top = dome_center_z + dome_radius*math.cos(math.asin(
            min(1.0, pole_radius/max(dome_radius, 1e-9))))
        pole = cylinder_points(center, pole_radius, top, pole_height, pole_samples, rng)
        block = dome + pole
        if occlude is not None:
            ox, oy, span = occlude
            base = math.atan2(oy-center[1], ox-center[0])
            half = math.radians(span)/2.0
            kept = []
            for p in block:
                delta = abs((math.atan2(p[1]-center[1], p[0]-center[0])-base+math.pi)
                            % (2*math.pi)-math.pi)
                if delta <= half:
                    kept.append(p)
            block = kept
        points.extend(block)
    points.extend(ground_points(bounds, ground_samples, ground_noise, rng))
    points.extend(robot_points(pose, self_radius, self_samples, rng))
    for cx, cy, radius, z1 in clutter:
        points.extend(cylinder_points((cx, cy), radius, 0.0, z1, clutter_samples, rng))
    rng.shuffle(points)
    if self_filter_radius is None:
        self_filter_radius = self_radius
    if self_filter_radius > 0:
        points = [p for p in points
                  if math.hypot(p[0]-pose[0], p[1]-pose[1]) > self_filter_radius]
    if map_frame:
        points = to_map_frame(points, pose)
        truth["poles"] = [{"center": to_map_frame([(p["center"][0], p["center"][1], 0.0)], pose)[0][:2],
                           "radius": p["radius"]} for p in truth["poles"]]
    return points, truth


def to_map_frame(points, pose):
    """base_link → map：先绕 yaw 旋转，再平移。"""
    x, y, yaw = pose
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return [(x + cos_yaw*px - sin_yaw*py,
             y + sin_yaw*px + cos_yaw*py,
             pz) for px, py, pz in points]


def main():
    parser = argparse.ArgumentParser(description="生成合成雷达点云，用于离线验证杆检测")
    parser.add_argument("--spacing", type=float, default=1.6, help="两根杆的间距")
    parser.add_argument("--dome-diameter", type=float, default=0.36)
    parser.add_argument("--ground-noise", type=float, default=0.012)
    parser.add_argument("--clutter", type=int, default=0, help="干扰圆柱数量")
    parser.add_argument("--occlude-deg", type=float, default=0.0, help="遮挡保留角度范围")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--dump", type=int, default=0, help="打印前N个点")
    args = parser.parse_args()

    half = args.spacing/2.0
    poles = [(-half, 0.0), (half, 0.0)]
    clutter = [(-1.2, 1.0, 0.12, 0.9), (1.3, -0.9, 0.15, 0.7)][:args.clutter]
    occlude = (0.0, -1.5, args.occlude_deg) if args.occlude_deg > 0 else None
    points, truth = scene(poles, bounds=(-3, 3, -3, 3),
                          dome_radius=args.dome_diameter/2.0,
                          ground_noise=args.ground_noise,
                          clutter=clutter, occlude=occlude, seed=args.seed)
    print("点数: %d" % len(points))
    print("真值杆心: %s   底座半径 %.3f m" % (
        [p["center"] for p in truth["poles"]], truth["dome_radius"]))
    zs = [p[2] for p in points]
    print("z 范围: %.3f ~ %.3f" % (min(zs), max(zs)))
    for index in range(min(args.dump, len(points))):
        print("  (%.3f, %.3f, %.3f)" % points[index])


if __name__ == "__main__":
    main()
