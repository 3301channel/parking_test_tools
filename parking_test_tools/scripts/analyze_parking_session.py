#!/usr/bin/env python3
"""
从 rosbag 中提取泊车轨迹并做质量评估，输出：
  <out>/trajectory_info.json   轨迹点、起点/终点、障碍物、指标（机器可读）
  <out>/trajectory_points.csv  轨迹点表格（Excel 可读）
  <out>/trajectory_plot.png    轨迹平面图
  <out>/evaluation.md          人读的评估报告

用法：
  source ~/autoware/install/setup.bash
  python3 analyze_parking_session.py <bag目录> [--out 输出目录] [--slot 车位配置yaml]

车位的三种来源（优先级从高到低）：
  1. --slot 指定的 yaml
  2. 环境变量/同目录下的 parking_slot.yaml
  3. 未提供时用 Goal 点作为车位中心 + 默认尺寸（报告中会标注）
"""

import argparse
import csv
import json
import math
import os
import sys
from collections import defaultdict

try:
    import numpy as np
    import yaml
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Polygon
    from matplotlib.collections import LineCollection
except ImportError as e:
    print(f"[错误] 缺少 Python 依赖: {e}", file=sys.stderr)
    sys.exit(1)

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:
    print(f"[错误] 未找到 ROS2 环境，请先 source install/setup.bash: {e}", file=sys.stderr)
    sys.exit(1)


# ---------------------------------------------------------------- 基础工具
def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def dist2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def curvature(p1, p2, p3):
    """三点曲率，退化时返回 0"""
    a, b, c = dist2d(p1, p2), dist2d(p2, p3), dist2d(p1, p3)
    if a < 1e-6 or b < 1e-6 or c < 1e-6:
        return 0.0
    area2 = abs((p2[0] - p1[0]) * (p3[1] - p1[1]) -
                (p3[0] - p1[0]) * (p2[1] - p1[1]))
    return 2.0 * area2 / (a * b * c)


# ---------------------------------------------------------------- 读 bag
def read_bag(uri):
    meta_path = os.path.join(uri, "metadata.yaml")
    storage_id = "sqlite3"
    if os.path.isfile(meta_path):
        try:
            with open(meta_path) as f:
                meta = yaml.safe_load(f)
            storage_id = meta["rosbag2_bagfile_information"].get(
                "storage_identifier", "sqlite3")
        except Exception:
            pass

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cache = {}
    warned = set()
    data = defaultdict(list)
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic not in types:
            continue
        try:
            if topic not in cache:
                cache[topic] = get_message(types[topic])
            msg = deserialize_message(raw, cache[topic])
        except Exception as e:
            # 最常见原因：没 source autoware 环境，自定义消息类型解析不了
            if topic not in warned:
                print(f"[警告] 无法解析话题 {topic} ({types.get(topic)}): {e}",
                      file=sys.stderr)
                print("       → 请先 source ~/autoware/install/setup.bash 再运行本脚本",
                      file=sys.stderr)
                warned.add(topic)
            continue
        data[topic].append((stamp, msg))
    return data, types


def pick_trajectories(data):
    """返回 (主轨迹话题, 主轨迹帧, A*泊车轨迹话题, A*泊车帧)

    主轨迹 = 最终送给控制的轨迹，RViz 上显示的就是它（含 behavior_path_planner
    拼接的几何 pull over 段）。A* 泊车轨迹 /planning/scenario_planning/parking/trajectory
    只是其中的单向切分段，点很稀疏，不能代表完整入库路径。
    """
    main_topic, main_msgs = None, []
    for t in ("/planning/trajectory", "/planning/scenario_planning/trajectory"):
        if data.get(t):
            main_topic, main_msgs = t, data[t]
            break

    pk_topic = "/planning/scenario_planning/parking/trajectory"
    pk_msgs = data.get(pk_topic, [])

    if not main_msgs and pk_msgs:  # 没有主轨迹时退回 A* 段
        main_topic, main_msgs = pk_topic, pk_msgs
    return main_topic, main_msgs, pk_topic, pk_msgs


def msg_to_points(msg):
    pts = []
    for p in msg.points:
        # time_from_start 与 heading_rate_rps 由 velocity_smoother 在发布前填充；
        # 老版本 Autoware 或旧 bag 里这两个字段可能全为 0，用 0.0 兜底不报错
        t = getattr(p, "time_from_start", None)
        t_sec = (t.sec + t.nanosec * 1e-9) if t is not None else 0.0
        # 记录 TrajectoryPoint 的全部 8 个字段，供 show_parking_result.py 原样展示。
        # x/y/yaw 是 pose 的拆分；v_lat/a/steer_f/steer_r 是泊车专用字段，
        # 老版本 Autoware 或 A* 原始轨迹里可能全为 0，用 0.0 兜底不报错。
        pts.append({
            "x": p.pose.position.x,
            "y": p.pose.position.y,
            "yaw": quat_to_yaw(p.pose.orientation),
            "v": getattr(p, "longitudinal_velocity_mps", 0.0),
            "t": t_sec,
            "w": getattr(p, "heading_rate_rps", 0.0),
            "v_lat": getattr(p, "lateral_velocity_mps", 0.0),
            "a": getattr(p, "acceleration_mps2", 0.0),
            "steer_f": getattr(p, "front_wheel_angle_rad", 0.0),
            "steer_r": getattr(p, "rear_wheel_angle_rad", 0.0),
        })
    return pts


# ---------------------------------------------------------------- 车位
def load_slot(path, goal_xy, goal_yaw):
    if path and os.path.isfile(path):
        with open(path) as f:
            d = yaml.safe_load(f) or {}
        s = d.get("parking_slot", d)
        return {
            "center": [float(s["center"][0]), float(s["center"][1])],
            "yaw": float(s.get("yaw", goal_yaw)),
            "length": float(s.get("length", 5.0)),
            "width": float(s.get("width", 2.5)),
            "source": f"配置: {os.path.basename(path)}",
        }
    if goal_xy is not None:
        return {
            "center": [goal_xy[0], goal_xy[1]],
            "yaw": goal_yaw if goal_yaw is not None else 0.0,
            "length": 5.0,
            "width": 2.5,
            "source": "未配置车位，回退为 Goal 点 + 默认 5.0x2.5m（尺寸需人工确认）",
        }
    return None


def slot_corners(slot):
    cx, cy = slot["center"]
    yaw = slot["yaw"]
    hl, hw = slot["length"] / 2.0, slot["width"] / 2.0
    c, s = math.cos(yaw), math.sin(yaw)
    local = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
    return [(cx + x * c - y * s, cy + x * s + y * c) for x, y in local]


def point_in_polygon(pt, poly):
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xi = x1 + (y - y1) * (x2 - x1) / (y2 - y1 + 1e-12)
            if x < xi:
                inside = not inside
    return inside


# ---------------------------------------------------------------- 障碍物
def extract_objects(data):
    objs = []
    msgs = data.get("/perception/object_recognition/objects", [])
    if not msgs:
        return objs, None
    stamp, msg = msgs[0]
    for o in msg.objects:
        try:
            p = o.kinematics.initial_pose_with_covariance.pose
            dim = o.shape.dimensions
            objs.append({
                "x": p.position.x, "y": p.position.y,
                "yaw": quat_to_yaw(p.orientation),
                "length": dim.x, "width": dim.y,
                "vx": o.kinematics.initial_twist_with_covariance.twist.linear.x,
            })
        except Exception:
            continue
    return objs, stamp


def dist_point_to_obb(pt, obj):
    """点到有向包围盒的最近距离（含盒内为 0）"""
    dx, dy = pt[0] - obj["x"], pt[1] - obj["y"]
    c, s = math.cos(-obj["yaw"]), math.sin(-obj["yaw"])
    lx, ly = dx * c - dy * s, dx * s + dy * c
    hl, hw = obj["length"] / 2.0, obj["width"] / 2.0
    ox = max(abs(lx) - hl, 0.0)
    oy = max(abs(ly) - hw, 0.0)
    return math.hypot(ox, oy)


# ---------------------------------------------------------------- 指标
def evaluate(points, start_pose, slot, objects):
    m = {}
    if len(points) < 2:
        return {"valid": False, "reason": "轨迹点少于 2 个"}

    xy = [(p["x"], p["y"]) for p in points]

    # 长度 / 方向
    seg = [dist2d(xy[i], xy[i + 1]) for i in range(len(xy) - 1)]
    total_len = sum(seg)
    m["轨迹总长_m"] = round(total_len, 3)
    m["轨迹点数"] = len(points)

    # 前进/倒车判定：位移方向与车头朝向点积
    reverse_flags = []
    for i in range(len(points) - 1):
        dx = points[i + 1]["x"] - points[i]["x"]
        dy = points[i + 1]["y"] - points[i]["y"]
        proj = dx * math.cos(points[i]["yaw"]) + dy * math.sin(points[i]["yaw"])
        reverse_flags.append(proj < 0)

    fwd_len = sum(seg[i] for i in range(len(seg)) if not reverse_flags[i])
    rev_len = sum(seg[i] for i in range(len(seg)) if reverse_flags[i])
    m["前进段长_m"] = round(fwd_len, 3)
    m["倒车段长_m"] = round(rev_len, 3)

    switches = sum(1 for i in range(1, len(reverse_flags))
                   if reverse_flags[i] != reverse_flags[i - 1])
    m["方向切换次数"] = int(switches)
    m["是否含倒车"] = bool(rev_len > 0.05)

    # 曲率 / 平滑度
    ks = [curvature(xy[i - 1], xy[i], xy[i + 1]) for i in range(1, len(xy) - 1)]
    if ks:
        m["最大曲率_1pm"] = round(max(ks), 4)
        m["平均曲率_1pm"] = round(float(np.mean(ks)), 4)
        m["最小转弯半径_m"] = round(1.0 / max(ks), 3) if max(ks) > 1e-6 else float("inf")
        m["曲率抖动_标准差"] = round(float(np.std(ks)), 4)

    # 时间轴与横摆角速度：这两个字段由 velocity_smoother 填充，
    # 老版本 / 旧 bag 里可能全 0，因此额外给出填充率，便于判断数据是否可信
    ts = [p.get("t") or 0.0 for p in points]
    ws = [p.get("w") or 0.0 for p in points]
    n_t = sum(1 for t in ts if t > 1e-9)
    abs_ws = [abs(w) for w in ws if abs(w) > 1e-9]
    m["时间轴填充率_pct"] = round(100.0 * n_t / max(1, len(ts)), 1)
    m["角速度填充率_pct"] = round(100.0 * len(abs_ws) / max(1, len(ws)), 1)
    if n_t >= 2:
        span = max(ts) - min(ts)
        m["轨迹总时长_s"] = round(span, 3)
        if span > 1e-6:
            m["平均速度_mps"] = round(total_len / span, 3)
    if abs_ws:
        m["最大横摆角速度_radps"] = round(max(abs_ws), 4)
        m["平均横摆角速度_radps"] = round(float(np.mean(abs_ws)), 4)

    # 终点 / 起点
    end = xy[-1]
    start = xy[0]
    m["起点_x"] = round(start[0], 3)
    m["起点_y"] = round(start[1], 3)
    m["终点_x"] = round(end[0], 3)
    m["终点_y"] = round(end[1], 3)
    m["终点朝向_deg"] = round(math.degrees(points[-1]["yaw"]), 2)

    if start_pose:
        m["起点与车辆初始位置偏差_m"] = round(dist2d(start, start_pose), 3)
        m["起点朝向_deg"] = round(math.degrees(start_pose[2]), 2)

    # 绕行系数
    if slot:
        straight = dist2d(start, slot["center"])
        m["起点到车位直线距离_m"] = round(straight, 3)
        m["绕行系数"] = round(total_len / straight, 3) if straight > 0.1 else None

        # 终点相对车位的位置（车位坐标系）
        cx, cy = slot["center"]
        c, s = math.cos(-slot["yaw"]), math.sin(-slot["yaw"])
        dx, dy = end[0] - cx, end[1] - cy
        lx, ly = dx * c - dy * s, dx * s + dy * c
        m["终点相对车位中心_纵向_m"] = round(lx, 3)
        m["终点相对车位中心_横向_m"] = round(ly, 3)
        m["终点相对车位角度偏差_deg"] = round(
            math.degrees(wrap_pi(points[-1]["yaw"] - slot["yaw"])), 2)
        m["轨迹终点在车位框内"] = point_in_polygon(end, slot_corners(slot))

    # 障碍物
    if objects:
        dmin, nearest = float("inf"), None
        for obj in objects:
            for pt in xy:
                dd = dist_point_to_obb(pt, obj)
                if dd < dmin:
                    dmin, nearest = dd, obj
        m["最近障碍物距离_m"] = round(dmin, 3)
        m["最近障碍物位置"] = [round(nearest["x"], 2), round(nearest["y"], 2)]
        m["周围车辆数"] = len(objects)
    else:
        m["周围车辆数"] = 0
        m["最近障碍物距离_m"] = None

    # 轨迹首尾位移（执行完整性）
    m["首尾直线距离_m"] = round(dist2d(start, end), 3)
    m["valid"] = True
    return m


def evaluate_final_pose(odo_last, slot):
    if not odo_last or not slot:
        return {}
    x, y, yaw = odo_last
    cx, cy = slot["center"]
    c, s = math.cos(-slot["yaw"]), math.sin(-slot["yaw"])
    dx, dy = x - cx, y - cy
    lx, ly = dx * c - dy * s, dx * s + dy * c
    return {
        "最终停车位姿_x": round(x, 3),
        "最终停车位姿_y": round(y, 3),
        "最终停车位姿_朝向_deg": round(math.degrees(yaw), 2),
        "停车位姿相对车位_纵向_m": round(lx, 3),
        "停车位姿相对车位_横向_m": round(ly, 3),
        "停车位姿相对车位_角度偏差_deg": round(
            math.degrees(wrap_pi(yaw - slot["yaw"])), 2),
        "停车位姿在车位框内": point_in_polygon((x, y), slot_corners(slot)),
    }


def clearance_to_costmap(points, costmap, max_pts=120):
    """轨迹各点到 costmap 硬障碍栅格的最小距离（米）。

    比只看 /perception/objects 更可靠：感知可能漏检，而 costmap 已经把
    wayarea、停车位边界、多边形膨胀都算进去了，是 A* 真正看到的东西。
    costmap 需用 thresh=100 提取（只取硬障碍），否则量到的是膨胀层。
    """
    if not costmap or len(points) < 1:
        return None
    xs, ys, res = costmap
    if len(xs) == 0:
        return None
    occ = np.column_stack([np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)])

    # 轨迹点数多时抽稀，避免 N×M 过大
    step = max(1, len(points) // max_pts)
    dmin = float("inf")
    for p in points[::step]:
        d = np.hypot(occ[:, 0] - p["x"], occ[:, 1] - p["y"])
        m = float(d.min())
        if m < dmin:
            dmin = m
    if math.isinf(dmin):
        return None
    # 栅格中心到边缘再减半格
    return max(0.0, dmin - res / 2.0)


def speed_profile(points):
    """由轨迹点自带的纵向速度反推时间轴，得到速度/加速度/jerk 指标。

    轨迹消息不给时间戳，只能靠 ds / v 积分。v 全为 0 时（速度没填）返回空。
    """
    if len(points) < 3:
        return {}
    vabs = [abs(p.get("v", 0.0)) for p in points]
    if max(vabs) < 1e-3:
        return {}

    ts = [0.0]
    for i in range(len(points) - 1):
        a, b = points[i], points[i + 1]
        d = dist2d((a["x"], a["y"]), (b["x"], b["y"]))
        v_avg = 0.5 * (vabs[i] + vabs[i + 1])
        ts.append(ts[-1] + (d / v_avg if v_avg > 0.05 else 0.1))

    accs = []
    for i in range(1, len(points) - 1):
        dt = ts[i + 1] - ts[i - 1]
        if dt > 1e-3:
            accs.append(((ts[i + 1] + ts[i - 1]) / 2.0, (vabs[i + 1] - vabs[i - 1]) / dt))

    jerks = []
    for i in range(1, len(accs)):
        dt = accs[i][0] - accs[i - 1][0]
        if dt > 1e-3:
            jerks.append(abs(accs[i][1] - accs[i - 1][1]) / dt)

    a_vals = [a for _, a in accs] or [0.0]
    return {
        "速度_最大_mps": round(max(vabs), 3),
        "速度_平均_mps": round(float(np.mean(vabs)), 3),
        "最大加速度_mps2": round(max(a_vals), 3),
        "最大减速度_mps2": round(min(a_vals), 3),
        "最大_jerk_mps3": round(max(jerks) if jerks else 0.0, 3),
        "预估行驶时间_s": round(ts[-1], 2),
    }


def planner_time_stats(data):
    """A* 单次规划耗时，来自节点的 debug 话题（需在录包列表里）。"""
    for key in ("/planning/scenario_planning/parking/freespace_planner/debug/processing_time_ms",
                "/planning/scenario_planning/parking/processing_time"):
        msgs = data.get(key) or []
        vals = []
        for _, m in msgs:
            v = getattr(m, "data", None)
            if v is not None:
                vals.append(float(v))
        if not vals:
            continue
        return {
            "规划次数": len(vals),
            "规划耗时_平均_ms": round(float(np.mean(vals)), 1),
            "规划耗时_最大_ms": round(float(np.max(vals)), 1),
            "规划耗时来源": key.rsplit("/", 1)[-1],
        }
    return {}


def load_pcd_xy(path, bbox, max_pts=60000):
    """读取 PCD（FIELDS x y z ...，binary 格式），裁剪到 bbox 并下采样，用作底图。

    bbox = (xmin, xmax, ymin, ymax)。返回 (N,2) 数组或 None。
    这样画出来的图带上真实点云地图轮廓，视觉上更接近 RViz。
    """
    try:
        with open(path, "rb") as f:
            header = b""
            while True:
                line = f.readline()
                if not line:
                    return None
                header += line
                if line.startswith(b"DATA"):
                    break
            hdr = header.decode("ascii", "ignore")
            fields, npts = [], 0
            for ln in hdr.splitlines():
                if ln.startswith("FIELDS"):
                    fields = ln.split()[1:]
                elif ln.startswith("POINTS"):
                    npts = int(ln.split()[1])
            if "binary" not in hdr.split("DATA")[-1] or len(fields) < 3:
                return None
            nf = len(fields)
            arr = np.frombuffer(f.read(npts * 4 * nf), dtype=np.float32).reshape(-1, nf)
        x, y = arr[:, 0], arr[:, 1]
        m = ((x > bbox[0]) & (x < bbox[1]) & (y > bbox[2]) & (y < bbox[3]))
        x, y = x[m], y[m]
        if len(x) == 0:
            return None
        if len(x) > max_pts:
            idx = np.random.choice(len(x), max_pts, replace=False)
            x, y = x[idx], y[idx]
        return np.column_stack([x, y])
    except Exception:
        return None


def extract_lanelet_markers(data):
    """从 /map/vector_map_marker (visualization_msgs/MarkerArray) 提取车道线。

    这条话题就是 RViz 里画车道线的同一份数据（autoware_lanelet2_map_visualizer
    的输出），坐标已经在 map 系，无需再做 MGRS 投影，因此比对 pcd 点云配准更可靠。

    返回 [(Nx2 数组, (r,g,b,a)), ...]。
    """
    markers = data.get("/map/vector_map_marker") or []
    if not markers:
        return []
    # 地图是静态的，每帧内容相同，取最后一帧即可
    arr = markers[-1][1]
    out = []
    for mk in getattr(arr, "markers", []):
        if mk.type not in (4, 5):  # LINE_STRIP / LINE_LIST
            continue
        if len(mk.points) < 2:
            continue
        pts = np.array([(p.x, p.y) for p in mk.points], dtype=float)
        c = mk.color
        rgba = (float(c.r), float(c.g), float(c.b), float(c.a))
        if sum(rgba[:3]) < 1e-3:  # 全黑说明没设色，给个可见的灰
            rgba = (0.45, 0.50, 0.58, 0.85)
        elif rgba[3] <= 0.0:
            rgba = (rgba[0], rgba[1], rgba[2], 0.85)
        out.append((pts, rgba))
    return out


def extract_costmap(data, thresh=50, latest=True):
    """从泊车 costmap (nav_msgs/OccupancyGrid) 提取占用栅格中心点。

    A* 就是在这张图上搜索的，把它画出来才能直观看到"路被什么挡住了"。
    话题名以 ros2 topic list 实测为准：运行时挂在 costmap_generator 命名空间下，
    而 launch 里 remap 的目标名 .../parking/occupancy_grid 实际并不存在。
    返回 (xs, ys, resolution) 或 None。
    """
    grids = []
    for key in ("/planning/scenario_planning/parking/costmap_generator/occupancy_grid",
                "/planning/scenario_planning/parking/occupancy_grid",
                "/planning/scenario_planning/parking/costmap"):
        grids = data.get(key) or []
        if grids:
            break
    if not grids:
        return None
    _, msg = grids[-1] if latest else grids[0]
    info = msg.info
    w, h, res = int(info.width), int(info.height), float(info.resolution)
    if w <= 0 or h <= 0 or res <= 0:
        return None
    arr = np.asarray(msg.data, dtype=np.int16)
    if arr.size != w * h:
        return None
    occ = arr.reshape(h, w) >= thresh
    if not occ.any():
        return None
    rows, cols = np.nonzero(occ)
    ox, oy = info.origin.position.x, info.origin.position.y
    yaw = quat_to_yaw(info.origin.orientation)
    lx = (cols + 0.5) * res
    ly = (rows + 0.5) * res
    c, s = math.cos(yaw), math.sin(yaw)
    return (ox + lx * c - ly * s, oy + lx * s + ly * c, res)


def footprint_corners(x, y, yaw, length, width, rear_overhang):
    """按车辆真实尺寸生成车身轮廓（base_link 在后轴中心，不是几何中心）。
    Autoware 的 pose 原点在 rear axle，所以车身向后只延伸 rear_overhang。
    """
    hl, hw = length / 2.0, width / 2.0
    x_front, x_rear = hl, hl - length  # 相对几何中心
    # 转到以 rear axle 为原点
    x_front -= (hl - rear_overhang)
    x_rear -= (hl - rear_overhang)
    c, s = math.cos(yaw), math.sin(yaw)
    local = [(x_front, -hw), (x_front, hw), (x_rear, hw), (x_rear, -hw)]
    return [(x + px * c - py * s, y + px * s + py * c) for px, py in local]


def _add_compass_and_scale(ax, view_span):
    """指北针 + 比例尺，让图更像 RViz 的 Grid 显示。"""
    ax.annotate("", xy=(0.955, 0.945), xytext=(0.955, 0.875),
                xycoords="axes fraction",
                arrowprops=dict(arrowstyle="-|>", color="#c8d0d8", lw=1.4),
                zorder=20)
    ax.text(0.955, 0.955, "N", transform=ax.transAxes, ha="center",
            va="bottom", color="#c8d0d8", fontsize=9, zorder=20)

    # 比例尺长度取 1/5 视野并按 1/2/5 序列取整
    raw = max(0.5, view_span / 5.0)
    mag = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 5, 10):
        if raw <= m * mag:
            bar = m * mag
            break
    else:
        bar = 10 * mag
    # 比例尺放右下角，避开左下角图例
    x0 = 0.62
    x1 = x0 + 0.28 * (bar / raw) if raw else x0 + 0.28
    x1 = min(x1, 0.92)
    y = 0.045
    ax.plot([x0, x1], [y, y], transform=ax.transAxes,
            color="#c8d0d8", lw=2.0, solid_capstyle="butt", zorder=20)
    ax.text((x0 + x1) / 2.0, y + 0.012, f"{bar:g} m",
            transform=ax.transAxes, ha="center", va="bottom",
            color="#c8d0d8", fontsize=7.5, zorder=20)


# ---------------------------------------------------------------- 画图
def plot_trajectory(points, slot, objects, start_pose, odo_last, metrics, out_png,
                    title_src, odo_path=None, all_points=None, zoom=None,
                    map_points=None, show_inset=False, global_view=False,
                    lanelet=None, costmap=None, vehicle_dims=(4.9, 1.9, 0.9),
                    goal=None):
    fig, ax = plt.subplots(figsize=(9, 8), dpi=130)
    fig.patch.set_facecolor("#0f1419")
    ax.set_facecolor("#0f1419")

    xy = np.array([(p["x"], p["y"]) for p in points])

    # 1) 点云地图底图：带真实环境轮廓，视觉上接近 RViz
    if map_points is not None and len(map_points) > 0:
        ax.scatter(map_points[:, 0], map_points[:, 1], s=1.1, c="#6e7b8d",
                   alpha=0.85, zorder=0, linewidths=0, label="map (pcd)")

    # 2) 车道线：与 RViz 同源（/map/vector_map_marker），按 Marker 自带颜色分组批量画
    if lanelet:
        groups = defaultdict(list)
        for pts, rgba in lanelet:
            groups[tuple(round(c, 2) for c in rgba)].append(pts)
        for key, plist in groups.items():
            ax.add_collection(LineCollection(
                plist, colors=[key], linewidths=1.1, zorder=1,
                alpha=min(0.95, key[3] if len(key) > 3 else 0.9)))
        ax.plot([], [], color="#9aa7b4", lw=1.2, label="lanelet2 lanes")

    # 3) 泊车 costmap：A* 的搜索栅格，直观体现"哪里被判成障碍"
    if costmap is not None:
        cxs, cys, cres = costmap
        ax.scatter(cxs, cys, s=max(6.0, (cres * 13.0) ** 2), c="#c0392b",
                   alpha=0.55, marker="s", linewidths=0, zorder=2,
                   label="parking costmap")

    # 4) 所有规划帧的轨迹点（淡色），叠加后形成泊车走廊
    if all_points:
        ap = np.array(all_points)
        ax.scatter(ap[:, 0], ap[:, 1], s=4.0, c="#3d7dd8", alpha=0.30,
                   zorder=2.2, label="planned corridor")

    # 5) 车辆实际执行路径：这才是 RViz 里看到的完整轨迹
    if odo_path and len(odo_path) > 1:
        op = np.array(odo_path)
        ax.plot(op[:, 0], op[:, 1], color="#8fa3b8", lw=1.8, alpha=0.95,
                zorder=2.5, label="executed (odom)")

    # ---- 主轨迹：优先按速度着色（RViz 的 trajectory 插件同样是速度配色）----
    if len(points) > 1:
        segs, vels = [], []
        for i in range(len(points) - 1):
            segs.append([(points[i]["x"], points[i]["y"]),
                         (points[i + 1]["x"], points[i + 1]["y"])])
            vels.append(abs(0.5 * (points[i].get("v", 0.0) + points[i + 1].get("v", 0.0))))
        if max(vels) > 1e-3:
            lc = LineCollection(segs, linewidths=2.8, zorder=4, cmap="turbo")
            lc.set_array(np.array(vels))
            lc.set_clim(0.0, max(1.0, max(vels)))  # 固定从 0 起，低速也分得出层次
            ax.add_collection(lc)
            cb = fig.colorbar(lc, ax=ax, fraction=0.032, pad=0.02)
            cb.set_label("|v| [m/s]", color="#d8dee9", fontsize=8)
            cb.ax.tick_params(colors="#7f8c8d", labelsize=7)
            cb.outline.set_edgecolor("#3b4252")
        else:
            # 轨迹消息没填速度时，退回按行进方向着色（绿=前进，橙=倒车）
            seg_colors = []
            for i in range(len(points) - 1):
                dx = points[i + 1]["x"] - points[i]["x"]
                dy = points[i + 1]["y"] - points[i]["y"]
                proj = dx * math.cos(points[i]["yaw"]) + dy * math.sin(points[i]["yaw"])
                seg_colors.append("#2ecc71" if proj >= 0 else "#ff7f0e")
            ax.add_collection(LineCollection(segs, colors=seg_colors,
                                             linewidths=2.8, zorder=4))

    # 朝向箭头：稀疏画几个，模拟 RViz 里路径点的位姿箭头（太多会盖住轨迹配色）
    step = max(1, len(points) // 8)
    sel = list(range(0, len(points), step))
    if sel:
        for i in sel:
            p = points[i]
            ax.annotate("", xy=(p["x"] + 0.4 * math.cos(p["yaw"]),
                                p["y"] + 0.4 * math.sin(p["yaw"])),
                        xytext=(p["x"], p["y"]),
                        arrowprops=dict(arrowstyle="-|>", color="#ffffff",
                                        lw=1.0, shrinkA=0, shrinkB=0),
                        zorder=5, alpha=0.45)

    # 车位框（浅填充+虚线边，避免与最终 footprint 叠成深色块看不清）
    if slot:
        poly = slot_corners(slot)
        ax.add_patch(Polygon(poly, closed=True, fill=True, facecolor="#4a90d9",
                             alpha=0.10, edgecolor="#5aa0e8", lw=1.5, ls="--",
                             zorder=2.8))
        ax.plot(*slot["center"], marker="X", ms=10, color="#5aa0e8", zorder=6,
                label="slot center")

    # 障碍物
    for obj in objects:
        hl, hw = obj["length"] / 2.0, obj["width"] / 2.0
        c, s = math.cos(obj["yaw"]), math.sin(obj["yaw"])
        pts = [(obj["x"] + x * c - y * s, obj["y"] + x * s + y * c)
               for x, y in [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]]
        ax.add_patch(Polygon(pts, closed=True, facecolor="#e74c3c", alpha=0.35,
                             edgecolor="#e74c3c", lw=1.2, zorder=2))

    # 起点 / 终点：同时画出车辆真实轮廓（对应 RViz 的 Ego footprint）
    vl, vw, vro = vehicle_dims
    ax.add_patch(Polygon(footprint_corners(xy[0][0], xy[0][1], points[0]["yaw"],
                                          vl, vw, vro),
                         closed=True, fill=False, edgecolor="#f1c40f", lw=1.5,
                         ls="--", zorder=7))
    ax.plot(*xy[0], marker="o", ms=8, mfc="#f1c40f", mec="k", zorder=7, label="start(traj)")
    ax.plot(*xy[-1], marker="s", ms=8, mfc="#9b59b6", mec="k", zorder=7, label="end(traj)")
    if start_pose:
        ax.plot(start_pose[0], start_pose[1], marker="^", ms=8, mfc="#f39c12",
                mec="k", zorder=7, label="ego init")
    if odo_last:
        fyaw = odo_last[2] if len(odo_last) > 2 else points[-1]["yaw"]
        ax.add_patch(Polygon(footprint_corners(odo_last[0], odo_last[1], fyaw,
                                              vl, vw, vro),
                             closed=True, fill=True, facecolor="#00e5ff",
                             alpha=0.22, edgecolor="#00e5ff", lw=1.8, zorder=8))
        ax.plot(odo_last[0], odo_last[1], marker="*", ms=13, mfc="#00e5ff",
                mec="k", zorder=9, label="ego final")
    if goal:
        gx, gy, gyaw = goal
        ax.annotate("", xy=(gx + 1.2 * math.cos(gyaw), gy + 1.2 * math.sin(gyaw)),
                    xytext=(gx, gy),
                    arrowprops=dict(arrowstyle="-|>", color="#e67e22", lw=2.0,
                                    shrinkA=0, shrinkB=0), zorder=8)
        ax.plot(gx, gy, marker="X", ms=10, mfc="#e67e22", mec="k", zorder=8,
                label="goal")

    info = []
    if "轨迹总长_m" in metrics:
        info.append(f"len={metrics['轨迹总长_m']}m")
    if "倒车段长_m" in metrics:
        info.append(f"rev={metrics['倒车段长_m']}m")
    if "方向切换次数" in metrics:
        info.append(f"shift={metrics['方向切换次数']}")
    if metrics.get("最近障碍物距离_m") is not None:
        info.append(f"clear={metrics['最近障碍物距离_m']}m")
    if "终点相对车位中心_横向_m" in metrics:
        info.append(f"lat_err={metrics['终点相对车位中心_横向_m']}m")
    if "终点相对车位角度偏差_deg" in metrics:
        info.append(f"yaw_err={metrics['终点相对车位角度偏差_deg']}deg")

    ax.set_title(f"Parking Trajectory  [{title_src}]\n" + "   ".join(info),
                 color="#d8dee9", fontsize=10)
    ax.set_xlabel("x [m]", color="#d8dee9")
    ax.set_ylabel("y [m]", color="#d8dee9")
    ax.tick_params(colors="#7f8c8d")
    for sp in ax.spines.values():
        sp.set_color("#3b4252")
    ax.grid(True, color="#2e3440", ls="--", lw=0.6)

    # ---- 视野 ----
    view_span = None
    if global_view:
        # 全图：包住所有轨迹点与执行路径
        xs, ys = [xy[:, 0]], [xy[:, 1]]
        if odo_path and len(odo_path) > 1:
            op = np.array(odo_path)
            xs.append(op[:, 0])
            ys.append(op[:, 1])
        if slot:
            sc = np.array(slot_corners(slot))
            xs.append(sc[:, 0])
            ys.append(sc[:, 1])
        ax_x = np.concatenate(xs)
        ax_y = np.concatenate(ys)
        pad = max(3.0, 0.08 * max(ax_x.max() - ax_x.min(), ax_y.max() - ax_y.min()))
        ax.set_xlim(ax_x.min() - pad, ax_x.max() + pad)
        ax.set_ylim(ax_y.min() - pad, ax_y.max() + pad)
        view_span = max(ax_x.max() - ax_x.min(), ax_y.max() - ax_y.min()) + 2 * pad
    elif slot:
        span = zoom if zoom else max(max(slot["length"], slot["width"]) * 3.2, 12.0)
        cx, cy = slot["center"]
        ax.set_xlim(cx - span / 2, cx + span / 2)
        ax.set_ylim(cy - span / 2, cy + span / 2)
        view_span = span
    ax.set_aspect("equal", adjustable="box")
    if view_span:
        _add_compass_and_scale(ax, view_span)

    leg = ax.legend(loc="lower left", fontsize=7.5, facecolor="#1b2229",
                    edgecolor="#3b4252", framealpha=0.85)
    leg.set_zorder(21)
    for t in leg.get_texts():
        t.set_color("#d8dee9")

    # ---- 右上角全局概览（保留整段行驶路径的空间位置）----
    if show_inset and odo_path and len(odo_path) > 1:
        axi = fig.add_axes([0.655, 0.115, 0.285, 0.285], facecolor="#161b22")
        op = np.array(odo_path)
        axi.plot(op[:, 0], op[:, 1], color="#8fa3b8", lw=1.0, zorder=1)
        if slot:
            axi.add_patch(Polygon(slot_corners(slot), closed=True, fill=True,
                                  facecolor="#4a90d9", alpha=0.6,
                                  edgecolor="#4a90d9", lw=1.0, zorder=2))
        axi.plot(op[0, 0], op[0, 1], marker="^", ms=5, mfc="#f39c12",
                 mec="none", zorder=3)
        axi.plot(xy[-1][0], xy[-1][1], marker="s", ms=5, mfc="#9b59b6",
                 mec="none", zorder=3)
        axi.set_title("overview (full route)", color="#8b949e", fontsize=7)
        axi.tick_params(colors="#5a6673", labelsize=6)
        for sp in axi.spines.values():
            sp.set_color("#3b4252")
        axi.grid(True, color="#232a33", ls=":", lw=0.4)
        axi.set_aspect("equal", adjustable="datalim")

    fig.savefig(out_png, facecolor=fig.get_facecolor())
    plt.close(fig)


# ---------------------------------------------------------------- 报告
def write_timeseries(path, odo):
    """导出 PlotJuggler 可直接加载的时间序列 CSV。

    第一列为相对时间（秒），可直接拖进 PlotJuggler 后选 t_sec 作为时间轴。
    列含义：位置、朝向、速度、偏航率、纵向加速度、曲率（yaw_rate / v）。
    """
    if not odo:
        return 0
    t0 = odo[0][0]
    prev_v = prev_t = None
    rows = []
    for ns, m in odo:
        t = (ns - t0) / 1e9
        p = m.pose.pose
        yaw = quat_to_yaw(p.orientation)
        v = m.twist.twist.linear.x
        wz = m.twist.twist.angular.z
        acc = ((v - prev_v) / (t - prev_t)
               if (prev_v is not None and t - prev_t > 1e-6) else 0.0)
        curv = wz / v if abs(v) > 0.05 else 0.0
        rows.append((t, ns / 1e9, p.position.x, p.position.y,
                     math.degrees(yaw), v, wz, acc, curv))
        prev_v, prev_t = v, t

    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_sec", "unix_time", "x", "y", "yaw_deg",
                    "v_mps", "yaw_rate_radps", "accel_mps2", "curvature_1pm"])
        for r in rows:
            w.writerow([f"{r[0]:.4f}", f"{r[1]:.6f}", f"{r[2]:.4f}", f"{r[3]:.4f}",
                        f"{r[4]:.3f}", f"{r[5]:.5f}", f"{r[6]:.5f}",
                        f"{r[7]:.4f}", f"{r[8]:.5f}"])
    return len(rows)


def write_report(path, metrics, final_metrics, slot, objects, src_topic, n_msgs, sizes,
                 extra_sections=None):
    lines = []
    lines.append("# 泊车轨迹评估报告\n")
    lines.append(f"- 轨迹来源话题: `{src_topic}`")
    lines.append(f"- 轨迹消息帧数: {n_msgs}（取「终点最接近最终停车位姿」的一帧作为入位轨迹）")
    if sizes:
        lines.append(f"- 各帧轨迹长度(前5): {sizes[:5]}")
    lines.append(f"- 车位来源: {slot['source'] if slot else '无'}")
    if slot:
        lines.append(f"- 车位: 中心 ({slot['center'][0]:.2f}, {slot['center'][1]:.2f}), "
                     f"朝向 {math.degrees(slot['yaw']):.1f}°, "
                     f"尺寸 {slot['length']}x{slot['width']} m")
    lines.append(f"- 周围目标数: {len(objects)}\n")

    def table(title, d):
        lines.append(f"## {title}\n")
        lines.append("| 指标 | 值 |")
        lines.append("|---|---|")
        for k, v in d.items():
            lines.append(f"| {k} | {v} |")
        lines.append("")

    table("轨迹指标", metrics)
    if final_metrics:
        table("最终停车位姿评估", final_metrics)

    if extra_sections:
        for title, d, note in extra_sections:
            lines.append(f"## {title}\n")
            if d:
                lines.append("| 指标 | 值 |")
                lines.append("|---|---|")
                for k, v in d.items():
                    lines.append(f"| {k} | {v} |")
                lines.append("")
            lines.append(f"> {note}\n")

    if objects:
        lines.append("## 周围目标\n")
        lines.append("| # | 位置(x,y) | 尺寸(长x宽) | 朝向(deg) | 速度(m/s) |")
        lines.append("|---|---|---|---|---|")
        for i, o in enumerate(objects):
            lines.append(f"| {i} | ({o['x']:.2f}, {o['y']:.2f}) | "
                         f"{o['length']:.2f}x{o['width']:.2f} | "
                         f"{math.degrees(o['yaw']):.1f} | {o['vx']:.2f} |")
        lines.append("")

    with open(path, "w") as f:
        f.write("\n".join(lines))


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="rosbag 目录")
    ap.add_argument("--out", help="输出目录，默认与 bag 同级")
    ap.add_argument("--slot", help="车位配置 yaml")
    ap.add_argument("--zoom", type=float, default=None,
                    help="主图视野跨度（米），默认按车位尺寸自动（约 3.2 倍车位长）")
    ap.add_argument("--map", default=None,
                    help="点云地图 pcd 路径，用作底图（默认 ~/autoware_data/maps/sample-map-planning/pointcloud_map.pcd）")
    ap.add_argument("--vehicle-length", type=float, default=4.9,
                    help="车长（含前后悬），仅用于画 footprint，默认 4.9 m")
    ap.add_argument("--vehicle-width", type=float, default=1.9,
                    help="车宽，默认 1.9 m")
    ap.add_argument("--rear-overhang", type=float, default=0.9,
                    help="后悬：Autoware 的 pose 原点在后轴，到车尾的距离，默认 0.9 m")
    args = ap.parse_args()

    bag = os.path.abspath(args.bag)
    if not os.path.isdir(bag):
        print(f"[错误] bag 目录不存在: {bag}", file=sys.stderr)
        return 1

    out = os.path.abspath(args.out) if args.out else os.path.dirname(bag)
    os.makedirs(out, exist_ok=True)

    print(f"[1/4] 读取 bag: {bag}")
    data, types = read_bag(bag)

    topic, trajs, pk_topic, pk_trajs = pick_trajectories(data)
    if not trajs:
        print("[警告] bag 中没有可用轨迹话题，无法评估", file=sys.stderr)
        print("       期望: /planning/trajectory 或 "
              "/planning/scenario_planning/parking/trajectory", file=sys.stderr)
        return 2

    # --- 先取 odometry，用于挑出最能代表「最终入位」的那一帧轨迹 ---
    start_pose = None
    odo_last = None
    odo_path = []
    odo = data.get("/localization/kinematic_state", [])
    if odo:
        p = odo[0][1].pose.pose
        start_pose = (p.position.x, p.position.y, quat_to_yaw(p.orientation))
        p = odo[-1][1].pose.pose
        odo_last = (p.position.x, p.position.y, quat_to_yaw(p.orientation))
        # 抽稀到 2000 点以内，避免图上糊成一片
        step = max(1, len(odo) // 2000)
        odo_path = [(m.pose.pose.position.x, m.pose.pose.position.y)
                    for _, m in odo[::step]]

    # --- 帧选择策略 ---
    # 取「轨迹终点离最终停车位姿最近」的一帧：这才是真正完成入位的规划段。
    # 之前取最长帧会选到早期车道行驶段（终点还离车位很远），导致图和实际对不上。
    sizes = [len(m.points) for _, m in trajs]
    if odo_last is not None:
        best_idx, best_dist = 0, float("inf")
        for i, (_, m) in enumerate(trajs):
            if not m.points:
                continue
            p = m.points[-1].pose.position
            d = math.hypot(p.x - odo_last[0], p.y - odo_last[1])
            if d < best_dist:
                best_dist, best_idx = d, i
    else:
        best_idx = max(range(len(trajs)), key=lambda i: sizes[i])

    points = msg_to_points(trajs[best_idx][1])

    # 所有帧的轨迹点（抽稀），画成淡色背景，展示规划覆盖范围
    all_pts = []
    stride = max(1, len(trajs) // 120)
    for _, m in trajs[::stride]:
        for p in m.points:
            all_pts.append((p.pose.position.x, p.pose.position.y))

    print(f"[2/4] 轨迹: 第 {best_idx}/{len(trajs)} 帧, {len(points)} 点, "
          f"共 {len(trajs)} 帧, 话题 {topic}")

    goal_xy, goal_yaw = None, None
    routes = data.get("/planning/mission_planning/route", [])
    if routes:
        try:
            goal = routes[-1][1].goal_pose
            goal_xy = (goal.position.x, goal.position.y)
            goal_yaw = quat_to_yaw(goal.orientation)
        except Exception:
            pass
    if goal_xy is None:
        goal_xy = (points[-1]["x"], points[-1]["y"])
        goal_yaw = points[-1]["yaw"]

    slot = load_slot(args.slot, goal_xy, goal_yaw)
    # 只保留车位附近（12 m 内）的规划点，叠加后形成「泊车走廊」，
    # 效果对应 RViz 里那条蓝色带
    if slot and all_pts:
        cx, cy = slot["center"]
        all_pts = [(x, y) for x, y in all_pts
                   if math.hypot(x - cx, y - cy) < 12.0]

    # 点云地图底图（只取视野附近 ±20 m，否则 175 万点画不动）
    map_pts = None
    map_file = args.map or os.path.expanduser(
        "~/autoware_data/maps/sample-map-planning/pointcloud_map.pcd")
    if slot and os.path.isfile(map_file):
        cx, cy = slot["center"]
        map_pts = load_pcd_xy(map_file, (cx - 20, cx + 20, cy - 20, cy + 20))
        if map_pts is not None:
            print(f"       已加载地图点云 {len(map_pts)} 点（视野 ±20 m）")

    objects, _ = extract_objects(data)
    print(f"[3/4] 目标 {len(objects)} 个, 车位来源: {slot['source'] if slot else '无'}")

    # 车道线：与 RViz 同源，坐标已在 map 系
    lanelet = extract_lanelet_markers(data)
    if lanelet:
        print(f"       已加载车道线 {len(lanelet)} 条（/map/vector_map_marker）")
    else:
        print("       未录到 /map/vector_map_marker，图上不显示车道线")

    # 泊车 costmap：A* 的输入栅格
    costmap = extract_costmap(data)
    if costmap is not None:
        print(f"       已加载 costmap 占用栅格 {len(costmap[0])} 格 "
              f"(res={costmap[2]:.2f} m, 话题 parking/occupancy_grid)")
    else:
        print("       未录到 .../parking/costmap_generator/occupancy_grid；"
              "旧 bag 属正常，若要分析 A* 为何找不到路径需重新录制")

    metrics = evaluate(points, start_pose, slot, objects)
    final_metrics = evaluate_final_pose(odo_last, slot)

    # ---- 三项补充指标：安全距离 / 速度剖面 / 规划耗时 ----
    # 安全距离用 thresh=100（只算硬障碍），与 A* 的 obstacle_threshold 一致；
    # 用 50 会把膨胀层也算进去，量出来偏小、误导。
    extra_sections = []

    hard_costmap = extract_costmap(data, thresh=100)
    clr = clearance_to_costmap(points, hard_costmap)
    if clr is not None:
        if clr >= 0.3:
            note = "✅ 安全（>= 0.3 m）"
        elif clr >= 0.1:
            note = "⚠️ 偏紧（0.1~0.3 m）：车体可能贴着障碍擦过"
        else:
            note = "❌ 危险（< 0.1 m）：几乎贴住障碍，实际执行易碰撞"
        extra_sections.append(("安全距离（轨迹 vs 硬障碍栅格）",
                               {"轨迹到硬障碍最小距离_m": round(clr, 3)}, note))
    else:
        extra_sections.append(("安全距离（轨迹 vs 硬障碍栅格）", {},
                               "未录到 parking/occupancy_grid，无法评估"))

    sp = speed_profile(points)
    if sp:
        j = sp["最大_jerk_mps3"]
        note = ("✅ 平顺（jerk < 1.0）" if j < 1.0
                else ("⚠️ 可接受（jerk 1.0~2.0）" if j < 2.0
                      else "❌ 有顿挫（jerk > 2.0），起步/停车会点头"))
        extra_sections.append(("速度与加速度剖面", sp, note))

    pt = planner_time_stats(data)
    if pt:
        avg = pt["规划耗时_平均_ms"]
        note = ("✅ 良好（平均 < 1000 ms）" if avg < 1000
                else "⚠️ 偏慢（平均 > 1000 ms），可调 theta_size / expansion_distance")
        extra_sections.append(("规划耗时（A* 单次）", pt, note))
    else:
        extra_sections.append(("规划耗时（A* 单次）", {},
                               "未录到 freespace_planner/debug/processing_time_ms"))

    # 画图用的车辆尺寸与目标位姿
    veh_dims = (args.vehicle_length, args.vehicle_width, args.rear_overhang)
    goal_tuple = (goal_xy[0], goal_xy[1],
                  goal_yaw if goal_yaw is not None else 0.0)

    # 输出
    info = {
        "bag": bag,
        "trajectory_topic": topic,
        "trajectory_frames": len(trajs),
        "frame_sizes": sizes,
        "parking_trajectory_topic": pk_topic,
        "parking_trajectory_frames": len(pk_trajs),
        "executed_path_points": len(odo_path),
        "start_pose": start_pose,
        "final_pose": odo_last,
        "goal": {"x": goal_xy[0], "y": goal_xy[1], "yaw": goal_yaw},
        "parking_slot": slot,
        "objects": objects,
        "metrics": metrics,
        "final_metrics": final_metrics,
        "extra_metrics": {t: d for t, d, _ in extra_sections},
        "extra_notes": {t: n for t, _, n in extra_sections},
        "trajectory_points": points,
    }
    with open(os.path.join(out, "trajectory_info.json"), "w") as f:
        json.dump(info, f, indent=1, ensure_ascii=False)

    with open(os.path.join(out, "trajectory_points.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["idx", "x", "y", "yaw_deg", "v_mps"])
        for i, p in enumerate(points):
            w.writerow([i, f"{p['x']:.4f}", f"{p['y']:.4f}",
                        f"{math.degrees(p['yaw']):.2f}", f"{p['v']:.3f}"])

    # 1) 车位局部图（无 inset，避免遮挡刻度）
    plot_trajectory(points, slot, objects, start_pose, odo_last, metrics,
                    os.path.join(out, "trajectory_plot.png"), os.path.basename(topic),
                    odo_path=odo_path, all_points=all_pts, zoom=args.zoom,
                    map_points=map_pts, show_inset=False,
                    lanelet=lanelet, costmap=costmap, vehicle_dims=veh_dims,
                    goal=goal_tuple)

    # 2) 全局路线图（能看到泊车发生在整段路线的哪个位置）
    global_map = None
    if slot and os.path.isfile(map_file):
        cx, cy = slot["center"]
        global_map = load_pcd_xy(map_file, (cx - 70, cx + 70, cy - 70, cy + 70),
                                 max_pts=90000)
    plot_trajectory(points, slot, objects, start_pose, odo_last, metrics,
                    os.path.join(out, "plot_global.png"), os.path.basename(topic),
                    odo_path=odo_path, all_points=None, map_points=global_map,
                    show_inset=False, global_view=True,
                    lanelet=lanelet, costmap=None, vehicle_dims=veh_dims,
                    goal=goal_tuple)

    # 3) PlotJuggler 时间序列
    n_ts = write_timeseries(os.path.join(out, "timeseries_plotjuggler.csv"), odo)
    if n_ts:
        print(f"       时间序列 {n_ts} 行 → timeseries_plotjuggler.csv")

    write_report(os.path.join(out, "evaluation.md"), metrics, final_metrics,
                 slot, objects, topic, len(trajs), sizes,
                 extra_sections=extra_sections)

    print(f"[4/4] 已输出到 {out}")
    for name in ("trajectory_info.json", "trajectory_points.csv",
                 "trajectory_plot.png", "plot_global.png",
                 "timeseries_plotjuggler.csv", "evaluation.md"):
        p = os.path.join(out, name)
        if os.path.isfile(p):
            print(f"   - {name}  ({os.path.getsize(p)/1024:.1f} KB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
