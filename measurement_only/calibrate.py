"""保底方案的标定与校验工具：输入现场实测值，输出可用的 config.json。

设计前提（与比赛规则一致）：
  - 规则2.1：桩的位置与间距"一律以现场实际布置为准" —— 所以必须实测；
  - 规则4.3：vx/vy 只与底盘速度成正比，没有 m/s 含义 —— 所以不标定绝对速度；
  - 协议3.2：存在90°轴向交叉，正方向必须逐轴实测 —— 所以给出 --sweep 提示。

本工具只做几何与配置校验，不打开串口、不控制车辆。

关键几何（务必按底座测量，不要按细杆测量）：
  车会撞到的是半球形底座，不是那根细杆。所以 pole_radius 取底座半径。

用法示例：
  python3 calibrate.py --spacing 1.6 --pole-angle-deg 90 \
          --base-diameter 0.36 --parking 0 0 --bounds -3 3 -3 3

  python3 calibrate.py ... --write      # 校验通过后直接写回 config.json

坐标约定（重要）：
  实测值必须落在"雷达里程计起点=停车位"的坐标系里：
  车停在停车位、车头朝向 +x 时启动定位，此刻位姿即 (0,0,0)。
  于是 parking = [0,0]，杆的坐标由 --spacing/--pole-angle-deg 推出。
  若你的实测坐标不是以停车位为原点，用 --origin-x/--origin-y 做平移。
"""
import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CONFIG = ROOT / "config.json"
REAL_CONFIG = ROOT / "config" / "real_vehicle.json"

RECOMMENDED_RADIUS_FALLBACK = 0.29
TIGHT_MARGIN = 0.05


def load_config(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def recommended_radius(config):
    """从随车 xacro 推碰撞包络；模型不可读时回退到已核验的 0.29。"""
    try:
        from robot_model import model_for_config
        model = model_for_config(config)
        if model:
            return float(model["recommended_radius_m"]), "xacro"
    except Exception:
        pass
    return RECOMMENDED_RADIUS_FALLBACK, "fallback"


def build(parking, spacing, pole_angle_deg, pole_radius, robot_radius, margin, bounds):
    """按"停车位为原点、两杆中点在车头正前方"构造坐标。"""
    angle = math.radians(pole_angle_deg)
    half = spacing / 2.0
    mid = (half * math.cos(angle), half * math.sin(angle))
    perp = (-math.sin(angle), math.cos(angle))
    lower = (mid[0] - perp[0] * half, mid[1] - perp[1] * half)
    upper = (mid[0] + perp[0] * half, mid[1] + perp[1] * half)
    return {
        "parking": [round(parking[0], 4), round(parking[1], 4)],
        "lower_pole": [round(lower[0], 4), round(lower[1], 4)],
        "upper_pole": [round(upper[0], 4), round(upper[1], 4)],
        "pole_radius": round(pole_radius, 4),
        "robot_radius": round(robot_radius, 4),
        "margin": round(margin, 4),
        "bounds": [round(v, 4) for v in bounds],
    }


def verify(geo, config):
    """返回 (problems, warnings, facts)。problems 非空时不得使用该配置。"""
    problems, warnings, facts = [], [], {}
    pole_r, robot_r, margin = geo["pole_radius"], geo["robot_radius"], geo["margin"]
    spacing = math.dist(geo["lower_pole"], geo["upper_pole"])
    need = pole_r + robot_r + margin
    arc = spacing / 2.0
    facts.update(spacing_m=round(spacing, 4), arc_radius_m=round(arc, 4),
                 min_safe_radius_m=round(need, 4), spare_m=round(arc - need, 4))

    model_r, source = recommended_radius(config)
    facts["recommended_robot_radius_m"] = model_r
    facts["recommended_radius_source"] = source
    if robot_r < model_r - 1e-9:
        problems.append("robot_radius=%.3f 小于随车模型推荐值 %.3f，会把车体当空气"
                        % (robot_r, model_r))
    if arc <= need:
        problems.append("两杆间距 %.2f m 不足：弧半径 %.3f m ≤ 最小安全半径 %.3f m，"
                        "navigation.make_route() 会直接抛错" % (spacing, arc, need))
    elif arc - need < TIGHT_MARGIN:
        warnings.append("余量仅 %+.3f m（<%.2f），绕行空间很紧，建议加大杆间距或减小margin"
                        % (arc - need, TIGHT_MARGIN))
    if pole_r < 0.1:
        warnings.append("pole_radius=%.3f 偏小：若这是按细杆测量的，实际会撞到更粗的半球底座。"
                        "pole_radius 应取底座最大半径（目测底座直径约0.3~0.4 m）" % pole_r)
    b = geo["bounds"]
    if not (b[0] < b[1] and b[2] < b[3]):
        problems.append("bounds 顺序无效：必须是 [xmin, xmax, ymin, ymax]")
    for name in ("parking", "lower_pole", "upper_pole"):
        x, y = geo[name]
        if not (b[0] <= x <= b[1] and b[2] <= y <= b[3]):
            problems.append("%s=%s 落在 bounds 之外" % (name, geo[name]))
    for name in ("lower_pole", "upper_pole", "parking"):
        x, y = geo[name]
        if abs(x) <= pole_r and abs(y) <= pole_r and name != "parking":
            warnings.append("%s 与原点几乎重合，检查 --spacing/--pole-angle-deg" % name)
    if config.get("calibration_confirmed") is not True:
        warnings.append("config.json 的 calibration_confirmed 仍为 false；"
                        "它不参与实车启动门禁（门禁在 config/real_vehicle.json）")
    return problems, warnings, facts


def dry_run(geo, config):
    """用真实导航代码构造路线，确认不抛错且顺序正确。"""
    merged = dict(config)
    merged.update(geo)
    try:
        from navigation import make_route
        route = make_route(merged)
    except Exception as exc:
        return None, "make_route() 失败：%s" % exc
    order = []
    for point in route:
        if not order or order[-1] != point[2]:
            order.append(point[2])
    return route, order


def main():
    parser = argparse.ArgumentParser(description="保底方案：按实测值生成并校验 config.json")
    parser.add_argument("--spacing", type=float, required=True,
                        help="两杆底座中心的实测间距（米）")
    parser.add_argument("--pole-angle-deg", type=float, default=90.0,
                        help="两杆中点相对车头(+x)的方位角，度；默认90表示正左方")
    parser.add_argument("--base-diameter", type=float,
                        help="底座最大直径（米）。pole_radius 取其一半 —— 推荐用这个")
    parser.add_argument("--pole-radius", type=float,
                        help="直接给绕行半径（米）；与 --base-diameter 二选一")
    parser.add_argument("--parking", type=float, nargs=2, default=[0.0, 0.0],
                        metavar=("X", "Y"), help="停车位坐标，默认原点")
    parser.add_argument("--origin-x", type=float, default=0.0, help="坐标平移量")
    parser.add_argument("--origin-y", type=float, default=0.0, help="坐标平移量")
    parser.add_argument("--bounds", type=float, nargs=4, default=[-3.0, 3.0, -3.0, 3.0],
                        metavar=("XMIN", "XMAX", "YMIN", "YMAX"), help="场地边界")
    parser.add_argument("--margin", type=float, default=0.1, help="额外安全距离")
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--write", action="store_true", help="校验通过后写回配置文件")
    args = parser.parse_args()

    if (args.base_diameter is None) == (args.pole_radius is None):
        parser.error("--base-diameter 与 --pole-radius 必须且只能给一个")
    if args.base_diameter is not None:
        if not 0 < args.base_diameter < 2:
            parser.error("--base-diameter 必须在0到2米之间")
        pole_radius = args.base_diameter / 2.0
    else:
        pole_radius = args.pole_radius
    if not 0 < args.spacing < 20:
        parser.error("--spacing 必须在0到20米之间")
    if not 0 < args.margin < 1:
        parser.error("--margin 必须在0到1米之间")

    config = load_config(args.config)
    robot_radius = float(config.get("robot_radius", RECOMMENDED_RADIUS_FALLBACK))
    parking = (args.parking[0] + args.origin_x, args.parking[1] + args.origin_y)
    geo = build(parking, args.spacing, args.pole_angle_deg, pole_radius,
                robot_radius, args.margin, args.bounds)

    problems, warnings, facts = verify(geo, config)
    route, order = dry_run(geo, config)

    print("=" * 62)
    print("实测输入")
    print("  两杆间距      %.3f m" % args.spacing)
    print("  绕行半径      %.3f m  (%s)" % (
        pole_radius,
        "底座直径 %.3f 的一半" % args.base_diameter if args.base_diameter else "直接指定"))
    print("  机器人半径    %.3f m  (推荐值 %.3f，来源 %s)"
          % (robot_radius, facts["recommended_robot_radius_m"], facts["recommended_radius_source"]))
    print("  额外余量      %.3f m" % args.margin)
    print("-" * 62)
    print("几何校验")
    print("  弧半径        %.3f m" % facts["arc_radius_m"])
    print("  最小安全半径  %.3f m  (= %.3f + %.3f + %.3f)"
          % (facts["min_safe_radius_m"], pole_radius, robot_radius, args.margin))
    print("  余量          %+.3f m" % facts["spare_m"])
    if route is not None:
        print("  路线点数      %d" % len(route))
        print("  路线顺序      %s" % " → ".join(order))
    print("-" * 62)
    print("生成的坐标（雷达里程计起点 = 停车位）")
    for key in ("parking", "lower_pole", "upper_pole", "pole_radius",
                "robot_radius", "margin", "bounds"):
        print("  %-14s %s" % (key, json.dumps(geo[key], ensure_ascii=False)))
    print("-" * 62)

    for item in problems:
        print("[错误] " + item)
    for item in warnings:
        print("[警告] " + item)

    if problems or route is None:
        print("=" * 62)
        print("结论：配置不可用，未写入任何文件。")
        return 1

    print("=" * 62)
    print("结论：几何校验通过。")
    print("仍需完成（本工具无法替代）：")
    print("  1. 雷达外参：量/找雷达中心相对 base_link 的 xyz 与 rpy；")
    print("     sentry_robot_cylinder.xacro 里是 $(arg xyz) 默认全零，URDF 不含它。")
    print("  2. 高度带：min_height 需降到地面噪声之上一点点。半球底座从地面开始，")
    print("     现在的 0.08 会切掉它最宽的部分，导致拟合半径偏小 → 贴底座过。")
    print("  3. 四轴方向：python3 main.py serial --sweep --k 0.15 --fast（只测方向，")
    print("     不测绝对速度），再把结果填进 body_from_command。")
    print("  4. 起步位姿：每次都在同一位置、同一朝向停车启动，否则坐标原点是错的。")

    if args.write:
        merged = dict(config)
        merged.update(geo)
        args.config.write_text(json.dumps(merged, ensure_ascii=False, indent=2) + "\n",
                               encoding="utf-8")
        print("\n已写入 %s" % args.config)
    else:
        print("\n（未写文件；确认无误后加 --write 写回 %s）" % args.config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
