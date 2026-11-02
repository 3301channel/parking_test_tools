#!/usr/bin/env python3
"""
实时监控 Autoware 泊车过程，并对"每一次停车"给出唯一、可验证的原因。

设计原则：不猜。
  每条结论都必须能落到具体字段值（速度、距离、点数、布尔量）上，
  输出里同时列出"已排除的检查项"，让你能自己核对结论是否成立。

================================ 输出格式 ================================

行驶中（一行）：
  14:25:23 [行驶] DRIVING   Parking     v= 0.83 前进 段1(前进) 剩6.48m
                    pos(3717.36,73751.17) yaw62° 目标1.20m 障3.2m

停车（原因两行：结论 + 说明）：
  14:25:26 [停车] DRIVING   Parking     v= 0.00 静止 段1(倒车) 剩0.00m
                    pos(3718.69,73754.37) yaw114°
           └ ★ 段末停车：A* 第 1 段（倒车）已走到端点，末端 v=0
              分段轨迹每段末点必须停车：若随后下发反向新段 = 换向（正常揉车）；
              若不再有新段且 is_completed=True = 泊车完成；若都没有 = 异常（2 s 后改报「规划无路径」）

只讲"为什么停"，不罗列命中了哪一项、排除了哪一项——那些是内部推理过程。

退出时会打印"结论变化时间线"，把整场运行里每一次停车的原因按时间列出。

================================ 停车原因分级 ================================
按优先级互斥判定，命中即停，后面的检查不再参与结论：

  ①  任务终态      ARRIVED_GOAL / is_completed=True / FINALIZING      正常结束
  ②  状态机未行驶  WAITING_FOR_ROUTE / PLANNING / WAITING_FOR_ENGAGE  还没到该走的时候
  ③  安全接管      MRM 生效 / 障碍物进入停止距离                      外部强制
  ④  规划无路径    A* 无输出 / goal_planner 无法生成候选 / 轨迹断流    规划给不出路
  ⑤  规划要求停    轨迹起点v=0 / 轨迹耗尽 / 换向点(揉车) / STOP factor  规划本身要求停
  ⑥  控制层停车    vehicle_cmd_gate is_stopped / control_cmd 速度为 0  能走但不让走
  ⑦  执行层        挂 P 档 / 空挡 / 未 engage                          车本身不让走
  ⑧  无结论        以上全部排除，明确说明缺哪一项证据                 不糊弄

关于"揉车端点速度为零"：
  A*（freespace_planner）输出的是**分段轨迹**，每段是一把单向行驶。
  段与段之间必须停车换挡（D↔R），所以每段末点 v=0 是**设计行为**，
  不是故障。脚本会显示 "段N/M 前进/倒车"，并在换向时明确标注。

================================ 用法 ================================
  source ~/autoware/install/setup.bash
  python3 watch_state.py                # 单行原地刷新
  python3 watch_state.py --line         # 每 0.5s 一行（重定向到文件用这个）
  python3 watch_state.py --events       # 只在"状态变化"时打印（最干净的时间线）

退出码：正常退出 0；Ctrl+C 也返回 0。
"""

import argparse
import math
import re
import sys
import time

# ---------------------------------------------------------------- 依赖

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           HistoryPolicy)
    from autoware_system_msgs.msg import AutowareState
    from autoware_internal_planning_msgs.msg import Scenario
    from autoware_adapi_v1_msgs.msg import MrmState
    from autoware_perception_msgs.msg import PredictedObjects
    from autoware_planning_msgs.msg import Trajectory, LaneletRoute
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
except ImportError as e:
    print(f"[错误] 未找到 ROS2 环境，请先 source ~/autoware/install/setup.bash: {e}",
          file=sys.stderr)
    sys.exit(1)


def _try_import(module, name):
    """可选依赖：不同 Autoware 版本包名/字段有差异，缺了只降级，不中断。"""
    try:
        mod = __import__(module, fromlist=[name])
        return getattr(mod, name)
    except Exception:
        return None


Control = _try_import("autoware_control_msgs.msg", "Control")
GearReport = _try_import("autoware_vehicle_msgs.msg", "GearReport")
MotionState = _try_import("autoware_adapi_v1_msgs.msg", "MotionState")
ValidatorStatus = _try_import("autoware_planning_validator.msg", "PlanningValidatorStatus")
PlanningFactorArray = _try_import("autoware_internal_planning_msgs.msg", "PlanningFactorArray")
# path_candidate / path_reference 实际类型是 PathWithLaneId（不是 Path）
PathWithLaneId = _try_import("autoware_internal_planning_msgs.msg", "PathWithLaneId")
DiagnosticArray = _try_import("diagnostic_msgs.msg", "DiagnosticArray")
RosLog = _try_import("rcl_interfaces.msg", "Log")
# vehicle_cmd_gate 的接口状态话题不是 std_msgs/Bool，而是 tier4_control_msgs 的专用消息。
# 用 Bool 去订阅会因类型不匹配永远收不到 —— 之前 "is_stopped 未收到" 就是这个原因。
IsStopped = _try_import("tier4_control_msgs.msg", "IsStopped")
IsStartRequested = _try_import("tier4_control_msgs.msg", "IsStartRequested")


# ---------------------------------------------------------------- 常量表

STATE_NAME = {
    1: "INITIALIZING", 2: "WAITING_FOR_ROUTE", 3: "PLANNING",
    4: "WAITING_FOR_ENGAGE", 5: "DRIVING", 6: "ARRIVED_GOAL", 7: "FINALIZING",
}
MRM_NAME = {0: "UNKNOWN", 1: "NORMAL", 2: "MRM_OPERATING",
            3: "MRM_SUCCEEDED", 4: "MRM_FAILED"}
MRM_BEHAVIOR = {1: "NONE", 2: "EMERGENCY_STOP", 3: "COMFORTABLE_STOP", 4: "PULL_OVER"}
MOTION_NAME = {0: "UNKNOWN", 1: "STOPPED", 2: "STARTING", 3: "MOVING"}
# GearReport.report 的取值（以 autoware_vehicle_msgs/msg/GearReport 为准）
GEAR_NONE, GEAR_NEUTRAL, GEAR_DRIVE = 0, 1, 2
GEAR_REVERSE, GEAR_REVERSE_2, GEAR_PARK = 20, 21, 22
GEAR_NAME = {0: "NONE", 1: "N", 2: "D", 20: "R", 21: "R2", 22: "P", 23: "LOW", 24: "LOW2"}

# AutowareState 里"正常结束"的状态
END_STATES = {"ARRIVED_GOAL", "FINALIZING"}

# ---- 判定阈值（都可调，数值来源见注释） ----
V_STOP = 0.05           # m/s，低于此值认为没动
STOP_CONFIRM_S = 0.4    # s，速度持续低于 V_STOP 这么久才认定"停车"，避开过零抖动
TRAJ_STALE = 1.0        # s，/planning/trajectory 超过这么久没更新 = 断流
PK_TRAJ_STALE = 2.0     # s，A* 泊车轨迹超过这么久没更新 = 无输出
GOAL_TRAJ_STALE = 2.0   # s，goal_planner 候选路径超过这么久没更新 = 无候选
OBSTACLE_STOP = 1.5     # m，最近障碍距离小于此值即认为被挡
TRAJ_TAIL = 0.30        # m，剩余轨迹短于此值认为"轨迹耗尽"
ON_TRACK = 2.0          # m，车离轨迹最近点超过此值，认为车已不在该轨迹上
TURN_CONFIRM_S = 3.0    # s，新段下发后多久内 control_cmd=0 视为换挡等待（非异常）


# ---------------------------------------------------------------- 小工具

def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_deg(a):
    """把角度规整到 (-180, 180]。"""
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def fmt_gear(code):
    if code is None:
        return "?"
    return GEAR_NAME.get(code, f"gear{code}")


# ---------------------------------------------------------------- 证据/结论

class Finding:
    """一次诊断的结论。

    code   : ①..⑧，表示归属哪一级
    title  : 一句话结论，必须具体（带数字或字段名）
    detail : 补充说明（可空）
    checks : [(名称, 是否命中, 具体值)]，仅在 --explain 时列出
    """

    __slots__ = ("code", "title", "detail", "checks", "is_stop")

    def __init__(self, code, title, detail="", checks=None, is_stop=True):
        self.code = code
        self.title = title
        self.detail = detail
        self.checks = checks or []
        self.is_stop = is_stop


class SegmentTracker:
    """跟踪 A* 分段轨迹的段号与方向，用来解释"揉车端点 v=0"。

    段号 = 换向次数 + 1，判据只有一条：**行驶方向反转**。

    不能用"首点位置跳变"当判据：A* 每帧都会重发当前段，首点随车前进
    而移动，位置一直在变，那会把一次完整的倒车拆成几十段（实测涨到 81）。

    方向由"首点→末点位移在首点朝向上的投影"决定，比直接用 v 的符号稳
    （A* 常在段末把 v 填 0）。并且要求连续 2 帧一致才确认换向，
    避免 v 在 0 附近抖动时段号乱涨。
    """

    # 位移短于此值时不判方向（轨迹太短，投影不可信）
    MIN_DISP = 1.0
    # 连续多少帧方向一致才确认换向
    CONFIRM_N = 2

    def __init__(self):
        self.index = 0          # 当前段号，从 1 开始计
        self.direction = None   # "前进" / "倒车" / None
        self.turns = []         # 换向历史 [(新段号, 原方向, 新方向)]
        self.first_seen = None
        self._cand = None       # 待确认的新方向
        self._cand_n = 0        # 该方向连续出现的帧数

    @classmethod
    def _direction_of(cls, pts):
        """用首末位移投影判方向；轨迹太短返回 None（判不了，别乱改状态）。"""
        head, tail = pts[0], pts[-1]
        dx, dy = tail[0] - head[0], tail[1] - head[1]
        if math.hypot(dx, dy) < cls.MIN_DISP:
            return None
        proj = dx * math.cos(head[2]) + dy * math.sin(head[2])
        return "前进" if proj >= 0 else "倒车"

    def update(self, pts):
        """pts: [(x, y, yaw, v), ...]。返回 True 表示段号发生了变化。"""
        if not pts:
            return False
        d = self._direction_of(pts)
        if d is None:
            return False                 # 判不出方向，保持原状态

        if self.first_seen is None:      # 第一段
            self.index = 1
            self.direction = d
            self.first_seen = time.time()
            return True

        if d == self.direction:          # 同向，清掉待确认
            self._cand, self._cand_n = None, 0
            return False

        # 方向变了，需连续 CONFIRM_N 帧一致才认
        if self._cand == d:
            self._cand_n += 1
        else:
            self._cand, self._cand_n = d, 1
        if self._cand_n < self.CONFIRM_N:
            return False

        old = self.direction
        self.index += 1
        self.direction = d
        # first_seen 的语义是"当前这一段是什么时候下发的"，换段时要刷新，
        # 控制器判断"是否还在换挡"要靠它
        self.first_seen = time.time()
        self.turns.append((self.index, old, d))
        self._cand, self._cand_n = None, 0
        return True


# ---------------------------------------------------------------- 主节点

class WatchState(Node):
    def __init__(self):
        super().__init__("watch_state")

        # —— 基础状态 ——
        self.state = None
        self.scenario = None
        self.v = None            # 车速标量（m/s）
        self.v_signed = None     # 带方向：倒车为负
        self.ego = None          # (x, y, yaw)
        self.gear = None
        self.motion = None

        # —— 安全 ——
        self.mrm = None
        self.mrm_behavior = None
        self.diag_bad = []       # [(name, level, message)]
        self.near_obstacle = None   # 距车体表面估算距离（m）
        self.near_obj = None        # (id, class, 距离)

        # —— 规划 ——
        self.traj = None            # 最新 /planning/trajectory
        self.traj_t = None
        self.traj_src = None
        self.pk_traj = None         # 最新 A* 轨迹
        self.pk_traj_t = None
        self.pk_seg = SegmentTracker()
        self.ref_path_t = None      # path_reference/goal_planner 最后更新
        self.ref_path_n = None      # 该路径点数
        self.cand_path_t = None     # path_candidate/goal_planner
        self.cand_path_n = None
        self.gp_factors = None      # goal_planner 的 PlanningFactorArray
        self.validator_bad = []     # 轨迹校验未通过的项

        # —— 控制 ——
        self.ctrl_v = None          # control_cmd 纵向速度指令
        self.ctrl_a = None
        self.is_stopped = None      # vehicle_cmd_gate
        self.is_start_requested = None

        # —— 任务 ——
        self.parking_done = None
        self.goal = None            # (x, y, yaw)

        # —— 日志缓存：用于从 /rosout 抓确定性原因 ——
        self.last_node_log = {}     # node -> (time, msg)

        # —— 停车确认 ——
        self.stop_since = None

        # ================= 订阅 =================
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(AutowareState, "/autoware/state", self.on_state, 10)
        self.create_subscription(Scenario, "/planning/scenario_planning/scenario",
                                 self.on_scenario, 10)
        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 self.on_odo, sensor)
        self.create_subscription(MrmState, "/api/fail_safe/mrm_state", self.on_mrm, 10)
        self.create_subscription(Bool, "/planning/scenario_planning/parking/is_completed",
                                 self.on_parking_done, 10)
        self.create_subscription(PredictedObjects, "/perception/object_recognition/objects",
                                 self.on_objects, 10)
        self.create_subscription(Trajectory, "/planning/trajectory", self.on_traj, 10)
        self.create_subscription(Trajectory, "/planning/scenario_planning/parking/trajectory",
                                 self.on_pk_traj, 10)
        # route 是 transient_local，中途启动也能拿到当前的（含 goal）
        self.create_subscription(LaneletRoute, "/planning/mission_planning/route",
                                 self.on_route, latched)

        # 紫色轨迹：path_reference 是 goal_planner 算出的参考路径（RViz 里 110;110;210）
        # 绿色轨迹：path_candidate 是加了安全校验后的候选路径
        # 两者都是 PathWithLaneId；订阅失败只降级，不影响其他诊断
        if PathWithLaneId is not None:
            for topic, cb in (("/planning/path_reference/goal_planner", self.on_ref_path),
                              ("/planning/path_candidate/goal_planner", self.on_cand_path)):
                try:
                    self.create_subscription(PathWithLaneId, topic, cb, 10)
                except Exception:
                    pass

        if PlanningFactorArray is not None:
            self.create_subscription(PlanningFactorArray,
                                     "/planning/planning_factors/goal_planner",
                                     self.on_gp_factors, 10)
        if ValidatorStatus is not None:
            self.create_subscription(ValidatorStatus,
                                     "/planning/planning_validator/validation_status",
                                     self.on_validator, 10)
        if Control is not None:
            self.create_subscription(Control, "/control/command/control_cmd",
                                     self.on_control_cmd, 10)
        # 这两个话题只在"状态变化"时发布（vehicle_cmd_gate 里是
        # if (prev != cur) publish()），而且消息是 tier4_control_msgs 的专用类型。
        # 用 transient_local 才能拿到订阅前就已确定的当前值；发布端 QoS 不确定，
        # 再挂一个 volatile 订阅兜底，哪个能通就用哪个。
        if IsStopped is not None:
            self._subscribe_latched("/control/vehicle_cmd_gate/is_stopped",
                                    IsStopped, "is_stopped")
        if IsStartRequested is not None:
            self._subscribe_latched("/control/vehicle_cmd_gate/is_start_requested",
                                    IsStartRequested, "is_start_requested")
        if GearReport is not None:
            self.create_subscription(GearReport, "/vehicle/status/gear_status",
                                     self.on_gear, 10)
        if MotionState is not None:
            self.create_subscription(MotionState, "/api/motion/state", self.on_motion, 10)
        if DiagnosticArray is not None:
            self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diag, 10)
        if RosLog is not None:
            self.create_subscription(RosLog, "/rosout", self.on_rosout, 50)

    # ---------------- 回调 ----------------

    def _subscribe_latched(self, topic, msg_type, attr):
        """订阅"仅状态变化时发布"的接口话题，两种 QoS 各挂一个。

        为什么两个都要：
          - 发布端 transient_local 时，只有订阅端也是 transient_local 才能拿到
            订阅之前就已存在的当前值（否则要等下一次状态翻转，可能是几分钟后）；
          - 发布端若是 volatile，transient_local 订阅反而不兼容，此时靠 volatile 那一份。
        两者回调相同、幂等，谁先到都行。
        """
        def cb(msg):
            try:
                setattr(self, attr, bool(msg.data))
            except Exception:
                pass

        for durability in (DurabilityPolicy.TRANSIENT_LOCAL,
                           DurabilityPolicy.VOLATILE):
            qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=durability)
            try:
                self.create_subscription(msg_type, topic, cb, qos)
            except Exception:
                continue

    def on_state(self, msg):
        self.state = self._as_int(msg.state)

    def on_scenario(self, msg):
        self.scenario = msg.current_scenario

    def on_odo(self, msg):
        t = msg.twist.twist.linear
        self.v = (t.x ** 2 + t.y ** 2 + t.z ** 2) ** 0.5
        self.v_signed = t.x            # base_link 系，倒车为负
        q = msg.pose.pose.orientation
        self.ego = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                    quat_to_yaw(q))

    def on_mrm(self, msg):
        self.mrm = self._as_int(msg.state)
        bh = getattr(msg, "behavior", None)
        self.mrm_behavior = None if bh is None else self._as_int(bh)

    def on_parking_done(self, msg):
        self.parking_done = msg.data

    def on_route(self, msg):
        g = msg.goal_pose
        self.goal = (g.position.x, g.position.y, quat_to_yaw(g.orientation))

    def on_traj(self, msg):
        self.traj = msg
        self.traj_t = time.time()

    def on_pk_traj(self, msg):
        self.pk_traj = msg
        self.pk_traj_t = time.time()
        pts = self._points_of(msg)
        self.pk_seg.update(pts)

    def on_ref_path(self, msg):
        self.ref_path_t = time.time()
        self.ref_path_n = len(msg.points)

    def on_cand_path(self, msg):
        self.cand_path_t = time.time()
        self.cand_path_n = len(msg.points)

    def on_gp_factors(self, msg):
        self.gp_factors = msg

    def on_validator(self, msg):
        """收集所有 False 的校验项。字段名以 is_valid_ 开头，逐个比对。"""
        bad = []
        for name in dir(msg):
            if not name.startswith("is_valid_"):
                continue
            try:
                if not getattr(msg, name):
                    bad.append(name[len("is_valid_"):])
            except Exception:
                continue
        self.validator_bad = bad

    def on_control_cmd(self, msg):
        try:
            self.ctrl_v = msg.longitudinal.velocity
            self.ctrl_a = msg.longitudinal.acceleration
        except Exception:
            pass

    def on_gear(self, msg):
        r = getattr(msg, "report", None)
        self.gear = None if r is None else self._as_int(r)

    def on_motion(self, msg):
        s = getattr(msg, "state", None)
        self.motion = None if s is None else self._as_int(s)

    @staticmethod
    def _as_int(x, default=0):
        """把 ROS2 的各种整型标量统一成 int。

        rosidl 里 `byte` 字段映射到 Python 的 bytes（如 b'\\x02'），
        而 `uint8`/`int32` 映射到 int。DiagnosticStatus.level 就是 byte，
        直接拿它和 int 比较会抛 TypeError: '>=' not supported between 'bytes' and 'int'。
        """
        if isinstance(x, (bytes, bytearray)):
            return x[0] if len(x) else default
        try:
            return int(x)
        except (TypeError, ValueError):
            return default

    def on_diag(self, msg):
        """缓存非 OK 的诊断项，用于说明 MRM 是因哪一项触发的。"""
        bad = []
        for st in msg.status:
            # level: 0=OK 1=WARN 2=ERROR 3=STALE（byte 字段，需先转 int）
            lv = self._as_int(getattr(st, "level", 0))
            if lv >= 1:
                lvl = {1: "WARN", 2: "ERROR", 3: "STALE"}.get(lv, f"L{lv}")
                msg_txt = getattr(st, "message", "") or ""
                bad.append((st.name, lvl, msg_txt))
        self.diag_bad = bad[:8]

    def on_rosout(self, msg):
        """缓存每个节点最近一条日志，停车归因时直接引用原文。"""
        try:
            self.last_node_log[msg.name] = (time.time(), msg.msg)
        except Exception:
            pass

    def on_objects(self, msg):
        """估算车体到最近障碍表面的距离与对象身份。"""
        if self.ego is None or not msg.objects:
            return
        ex, ey, eyaw = self.ego
        best = None
        for obj in msg.objects:
            try:
                kin = obj.kinematics
                pose = None
                for attr in ("initial_pose_with_covariance", "pose_with_covariance"):
                    cov = getattr(kin, attr, None)
                    if cov is not None and hasattr(cov, "pose"):
                        pose = cov.pose
                        break
                if pose is None:
                    continue
                d = math.hypot(pose.position.x - ex, pose.position.y - ey)
                dim = obj.shape.dimensions
                # 用外接圆近似；再减去车身半长（4.9/2≈2.45）
                r = 0.5 * max(dim.x, dim.y)
                gap = d - r - 2.45
                if best is None or gap < best[2]:
                    cls = "?"
                    try:
                        cls = _classify(obj.classification)
                    except Exception:
                        pass
                    best = (obj.object_id.uuid[0] if obj.object_id.uuid else -1,
                            cls, gap)
            except Exception:
                continue
        if best is not None:
            self.near_obj = best
            self.near_obstacle = best[2]

    # ---------------- 派生量 ----------------

    @staticmethod
    def _points_of(traj):
        """Trajectory -> [(x, y, yaw, v)]"""
        out = []
        for p in getattr(traj, "points", []):
            out.append((p.pose.position.x, p.pose.position.y,
                        quat_to_yaw(p.pose.orientation),
                        getattr(p, "longitudinal_velocity_mps", None)))
        return out

    def traj_remaining(self):
        """返回 (剩余长度 m, 剩余点数, 末点距车 m, 车是否在轨迹上)。

        on_track 很关键：车离轨迹很远时（例如轨迹是上一段的残留），
        "最近点"会落在末点上，剩余会被算成 0，从而误报"轨迹耗尽"。
        只有车确实贴着轨迹（最近点 < ON_TRACK）时，剩余量才有意义。
        无轨迹返回 (None, 0, None, False)。
        """
        if self.traj is None or self.ego is None:
            return None, 0, None, False
        pts = self._points_of(self.traj)
        if not pts:
            return None, 0, None, False
        ex, ey, _ = self.ego
        # 找离车最近的轨迹点
        idx, best = 0, float("inf")
        for i, p in enumerate(pts):
            d = math.hypot(p[0] - ex, p[1] - ey)
            if d < best:
                best, idx = d, i
        remain = 0.0
        for i in range(idx, len(pts) - 1):
            remain += math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        tail_d = math.hypot(pts[-1][0] - ex, pts[-1][1] - ey)
        return remain, len(pts) - idx, tail_d, best <= ON_TRACK

    def goal_offset(self):
        """返回 (距离 m, 横向偏差 m, 角度偏差 deg)，无目标返回 None。"""
        if self.goal is None or self.ego is None:
            return None
        gx, gy, gyaw = self.goal
        ex, ey, eyaw = self.ego
        dx, dy = gx - ex, gy - ey
        dist = math.hypot(dx, dy)
        # 横向偏差：在目标朝向的法向上的投影
        lateral = -dx * math.sin(gyaw) + dy * math.cos(gyaw)
        ang = wrap_deg(math.degrees(eyaw - gyaw))
        return dist, lateral, ang

    def node_log(self, key):
        """取某节点**最近**的一条日志（key 为名字子串，如 freespace / goal_planner）。"""
        best_t, best_m = None, None
        for name, (t, m) in self.last_node_log.items():
            if key in name and (best_t is None or t > best_t):
                best_t, best_m = t, m
        return best_t, best_m

    # ---------------- 停车确认 ----------------

    def update_stop_state(self):
        """维护"持续静止"计时，返回已静止秒数（未静止返回 0）。"""
        if self.v is None:
            self.stop_since = None
            return 0.0
        now = time.time()
        if self.v < V_STOP:
            if self.stop_since is None:
                self.stop_since = now
            return now - self.stop_since
        self.stop_since = None
        return 0.0

    # ---------------- 归因（核心） ----------------

    def diagnose(self):
        """返回 Finding。"""
        now = time.time()
        v = self.v if self.v is not None else -1.0
        name = STATE_NAME.get(self.state, "?") if self.state else "?"

        if self.state is None:
            return Finding("⑧", "尚未收到 /autoware/state",
                           "确认 Autoware 在运行、且已 source 环境", is_stop=False)

        # ===== ① 任务终态 =====
        if name == "ARRIVED_GOAL":
            off = self.goal_offset()
            extra = ""
            if off:
                extra = f"，距目标 {off[0]:.2f} m、角度偏差 {off[2]:+.1f}°"
            return Finding("①", f"任务完成：状态机进入 ARRIVED_GOAL{extra}",
                           "arrival_check 判定到达，规划停止输出，属于正常收尾")

        if name == "FINALIZING":
            return Finding("①", "任务完成：正在 FINALIZING 收尾",
                           "进程即将退出，无需处理")

        if self.parking_done is True and name == "DRIVING":
            # 泊车完成但状态机还没切走，属于极短暂过渡，仍按终态报
            return Finding("①", "泊车完成：is_completed=True，A* 全部分段已执行完",
                           "分段轨迹跑完最后一段后置位，属于正常结束")

        # ===== ② 状态机未进入行驶 =====
        if name == "WAITING_FOR_ROUTE":
            return Finding("②", "无路线：状态机停在 WAITING_FOR_ROUTE",
                           "还没设目标点，或 route 被清除", is_stop=False)
        if name == "PLANNING":
            return Finding("②", "规划中：状态机处于 PLANNING",
                           "正在算路径，此时不该动", is_stop=False)
        if name == "WAITING_FOR_ENGAGE":
            return Finding("②", "等待 Engage：路径已就绪但未进入自动模式",
                           "点 Autonomous 或发 /autoware/engage", is_stop=False)
        if name == "INITIALIZING":
            return Finding("②", "启动中：INITIALIZING",
                           "节点尚未全部就绪", is_stop=False)

        if name != "DRIVING":
            return Finding("⑧", f"未知状态 {name}", "状态机取值不在已知表内", is_stop=False)

        # 静止计时必须每帧都维护：车一旦动起来就要清零。
        # 只在停车分支里调用的话，车动过之后 stop_since 仍停在最初那一刻，
        # 再次停下时会报出"已静止 213.6s"这种明显错误的数字。
        stop_s = self.update_stop_state()

        # ===== 还在动，说明没停 =====
        if v < 0:
            return Finding("⑧", "还没收到车速", "检查 /localization/kinematic_state", is_stop=False)
        if v >= V_STOP:
            return self._moving_finding(v)

        # ===== 从以下开始：确认为停车，逐级归因 =====
        checks = []      # 内部推理记录，不输出

        # ===== ③ 安全接管 =====
        f = self._check_mrm(now, stop_s, checks)
        if f:
            f.checks = checks + f.checks
            return f

        f = self._check_obstacle(checks)
        if f:
            f.checks = checks + f.checks
            return f

        # ===== ④ 规划无路径 =====
        f = self._check_no_path(now, checks)
        if f:
            f.checks = checks + f.checks
            return f

        # ===== ⑤ 规划要求停 =====
        f = self._check_plan_stop(now, checks)
        if f:
            f.checks = checks + f.checks
            return f

        # ===== ⑥ 控制层 =====
        f = self._check_control(v, stop_s, checks)
        if f:
            f.checks = checks + f.checks
            return f

        # ===== ⑦ 执行层 =====
        f = self._check_execution(checks)
        if f:
            f.checks = checks + f.checks
            return f

        # ===== ⑧ 无结论：明确说缺什么 =====
        # 刚停稳不到 1 s 就报"排除全部原因"会误导（起步前的短暂静止就是这样）
        if stop_s < 1.0:
            return Finding("⑧", f"短暂静止 {stop_s:.2f}s，尚未达到归因时长",
                           "各层检查均未命中，可能刚停稳或正在起步；连续静止 1 s 后再判定",
                           checks)

        missing = []
        if self.is_stopped is None:
            # 该话题只在状态翻转时发布，没翻转就一直没消息，不一定是链路断了
            missing.append("is_stopped 未收到（仅状态变化时发布，可能本次未翻转）")
        if self.ctrl_v is None:
            missing.append("/control/command/control_cmd 未收到")
        if self.traj_t is None:
            missing.append("/planning/trajectory 从未收到")
        if self.gear is None:
            missing.append("/vehicle/status/gear_status 未收到")
        return Finding(
            "⑧",
            f"各层原因均已排除，但仍未动（已静止 {stop_s:.1f}s）",
            ("缺少证据：" + "；".join(missing)) if missing else
            "所有输入正常。请用 --explain 核对各项取值，或查 vehicle_cmd_gate 限幅日志",
            checks)

    # ---- 各级检查 ----

    def _moving_finding(self, v):
        seg = self.pk_seg
        seg_txt = ""
        if self.scenario == "Parking" and seg.index > 0:
            seg_txt = f" 段{seg.index} {seg.direction or ''}"
        return Finding("—", f"行驶中 v={v:.2f} m/s{seg_txt}", "", is_stop=False)

    def _check_mrm(self, now, stop_s, checks):
        mrm = self.mrm
        if mrm is None:
            checks.append(("MRM", False, "未收到"))
            return None
        label = MRM_NAME.get(mrm, str(mrm))
        if mrm == 1:      # NORMAL
            checks.append(("MRM", False, label))
            return None

        bh = MRM_BEHAVIOR.get(self.mrm_behavior, "")
        why = ""
        if self.diag_bad:
            name, lvl, txt = self.diag_bad[0]
            why = f"；首个异常诊断 {name}({lvl})"
            if txt:
                why += f"：{txt[:60]}"
        checks.append(("MRM", True, f"{label}({bh})"))
        return Finding("③", f"安全接管：MRM={label}，行为={bh or '未知'}",
                       f"已静止 {stop_s:.1f}s" + why)

    def _check_obstacle(self, checks):
        d = self.near_obstacle
        if d is None:
            checks.append(("障碍", False, "无对象"))
            return None
        if d >= OBSTACLE_STOP:
            checks.append(("障碍", False, f"{d:.1f}m"))
            return None
        oid, cls, _ = self.near_obj or (-1, "?", d)
        checks.append(("障碍", True, f"{d:.1f}m"))
        if d < 0:
            return Finding("③", f"障碍物压到车体：最近 {cls}(id={oid}) 估算距离 {d:.2f} m",
                           "负值说明车体轮廓与障碍膨胀层已重叠，多为 costmap 膨胀覆盖相邻车位")
        return Finding("③", f"障碍物阻挡：最近 {cls}(id={oid}) 距离 {d:.2f} m < {OBSTACLE_STOP} m",
                       "规划会把速度压到 0；确认是否真的该障碍挡路")

    def _check_no_path(self, now, checks):
        """④ 规划给不出可执行的轨迹。"""
        tr_age = now - self.traj_t if self.traj_t else None

        # 4.1 Parking 场景：A* 是主力
        if self.scenario == "Parking":
            pk_age = now - self.pk_traj_t if self.pk_traj_t else None
            if self.pk_traj_t is None:
                # 从未产出：通常在等起点/终点合法化。优先引 freespace_planner 自己的话
                t, m = self.node_log("freespace")
                if not m:
                    t, m = self.node_log("parking")
                hint = f"；节点日志：{m[:70]}" if m else ""
                checks.append(("A*", True, "从未输出"))
                return Finding("④", "A* 未产出任何泊车轨迹（启动后 0 帧）",
                               "多为起点/终点不在 costmap 可通行区，或 costmap 未就绪" + hint)
            if pk_age is not None and pk_age > PK_TRAJ_STALE:
                # 产出过又停：区分"已完成"与"算不出"
                remain, n_left, tail_d, on_track = self.traj_remaining()
                t, m = self.node_log("freespace")
                hint = f"；节点日志：{m[:70]}" if m else ""
                checks.append(("A*", True, f"停{pk_age:.1f}s"))
                if n_left == 0 or (tail_d is not None and tail_d < TRAJ_TAIL):
                    # 轨迹已走完但没置 is_completed，属于异常
                    return Finding("④", f"A* 停止输出 {pk_age:.1f}s，且轨迹已走完",
                                   "轨迹耗尽却未收到 is_completed，检查 A* 是否在等下一步" + hint)
                return Finding("④", f"A* 停止输出 {pk_age:.1f}s（Parking 场景）",
                               f"当前轨迹仍剩 {remain:.2f} m，但不再有新段下发" + hint)
            checks.append(("A*", False,
                           f"{pk_age:.1f}s" if pk_age is not None else "?"))
        else:
            # 4.2 LaneDriving：看 goal_planner 的两条路径
            ref_age = now - self.ref_path_t if self.ref_path_t else None
            cand_age = now - self.cand_path_t if self.cand_path_t else None
            has_ref = ref_age is not None and ref_age < GOAL_TRAJ_STALE and (self.ref_path_n or 0) > 0
            has_cand = cand_age is not None and cand_age < GOAL_TRAJ_STALE and (self.cand_path_n or 0) > 0

            if has_ref and not has_cand:
                # 紫色有、绿色没有 —— 正是"目标不能规划路径"
                detail = self._gp_factor_reason()
                checks.append(("goal_planner", True, "紫色有/候选无"))
                return Finding("④", "goal_planner 无法生成安全候选路径"
                                    "（紫色参考路径存在，绿色候选为空）",
                               detail or "参考路径已算，但安全校验不通过；"
                                         "查 rosout 的 [DecidingPathStatus]")
            if not has_ref and not has_cand:
                checks.append(("goal_planner", True, "两者皆无"))
                return Finding("④", "goal_planner 未生成任何路径（参考与候选均空）",
                               self._gp_factor_reason() or
                               "检查 goal 是否在 lanelet2 可行驶区域内")
            checks.append(("goal_planner", False,
                           "参考+候选" if (has_ref and has_cand) else "有"))

        # 4.3 通用：最终轨迹断流
        if tr_age is not None and tr_age > TRAJ_STALE:
            checks.append(("轨迹", True, f"断流{tr_age:.1f}s"))
            return Finding("④", f"最终轨迹断流 {tr_age:.1f}s",
                           "规划管道停止发布 /planning/trajectory，控制端会因 topic 超时进入 MRM")

        checks.append(("轨迹", False,
                       f"{tr_age:.1f}s" if tr_age is not None else "未收到"))
        return None

    def _check_plan_stop(self, now, checks):
        """⑤ 规划本身要求停车，或轨迹已耗尽。"""
        remain, n_left, tail_d, on_track = self.traj_remaining()
        # 车不在这条轨迹上（例如拿到的是上一段的残留）时，
        # "剩余多少"没有意义，置 None 让它跳过耗尽判定
        if not on_track:
            remain = None

        # 5.0 车停在轨迹起点，且首点速度就是 0
        # —— "路径已经规划好了，但车不动"最常见的一种：规划要求静止起步
        if remain is not None and self.traj is not None and self.ego is not None:
            pts = self._points_of(self.traj)
            if pts:
                head_v = pts[0][3] if pts[0][3] is not None else 1.0
                d_head = math.hypot(pts[0][0] - self.ego[0], pts[0][1] - self.ego[1])
                if abs(head_v) < 0.05 and d_head < 2.0 and remain > 1.0:
                    checks.append(("轨迹首点", True, "v=0"))
                    return Finding(
                        "⑤",
                        f"轨迹已规划但车停在起点：首点要求 v=0（距首点 {d_head:.2f} m）",
                        f"前方还有 {remain:.2f} m 可走；等控制器速度爬升、"
                        "或检查 is_start_requested / 操作模式")

        # 5.1 换向点（揉车）：A* 分段轨迹末端 v=0 是设计行为
        if self.scenario == "Parking" and self.pk_traj is not None:
            pts = self._points_of(self.pk_traj)
            if pts:
                last_v = pts[-1][3] if pts[-1][3] is not None else 0.0
                seg = self.pk_seg
                # 只需三件事同时成立：末点 v=0、车贴着这条轨迹、车已经在末点附近。
                # 不再要求"首末同号"—— 倒车段首点 v<0、末点 v=0，符号本来就不同。
                if abs(last_v) < 0.05 and on_track and n_left is not None and n_left <= 2:
                    checks.append(("分段", True, f"段{seg.index}{seg.direction or ''}已到端点"))

                    # 把控制层/执行层状态一并列出，避免"只报一层、信息丢失"
                    extra = []
                    if self.ctrl_v is not None and abs(self.ctrl_v) < 1e-3:
                        extra.append("control_cmd 速度=0")
                        checks.append(("ctrl_v", True, "0"))
                    if self.gear == GEAR_PARK:
                        extra.append("档位=P")
                        checks.append(("档位", True, "P"))
                    elif self.gear == GEAR_NEUTRAL:
                        extra.append("档位=N")
                        checks.append(("档位", True, "N"))
                    if self.is_stopped is True:
                        extra.append("is_stopped=True")

                    turns = ""
                    if seg.turns:
                        turns = "；本趟已换向 " + "、".join(
                            f"{t[1]}→{t[2]}(第{t[0]}段)" for t in seg.turns[-3:])

                    if seg.turns:
                        # 已经发生过换向，可以确定这是揉车的换挡点
                        head = (f"揉车换向点：A* 第 {seg.index} 段（{seg.direction or '?'}）"
                                f"到达端点，末端 v=0 等待换挡反向")
                        detail = "分段轨迹每段末点都必须停车换挡，属正常设计" + turns
                    else:
                        # 首次段末：还不能断言是换向还是终点，明确列出两种去向
                        head = (f"段末停车：A* 第 {seg.index} 段（{seg.direction or '?'}）"
                                f"已走到端点，末端 v=0")
                        detail = ("分段轨迹每段末点必须停车："
                                  "若随后下发反向新段 = 换向（正常揉车）；"
                                  "若不再有新段且 is_completed=True = 泊车完成；"
                                  "若两者都没有 = 异常，2 s 后会改报「规划无路径」")
                    if extra:
                        detail += "；当前控制层：" + "、".join(extra)
                    return Finding("⑤", head, detail)

        # 5.2 轨迹耗尽但规划有续段
        # 注意 remain 为 None 表示"根本没有轨迹"，那是 ④ 的事，不能算耗尽
        if remain is not None and (n_left <= 1 or remain < TRAJ_TAIL):
            checks.append(("轨迹耗尽", True, f"剩{remain:.2f}m/{n_left}点"))
            detail = "最后一点就在车下"
            if tail_d is not None:
                detail += f"（末点距车 {tail_d:.2f} m）"
            tr_age = now - self.traj_t if self.traj_t else None
            if tr_age is not None:
                detail += f"；最新轨迹 {tr_age:.1f}s 前更新"
            return Finding("⑤", f"轨迹耗尽：剩余 {remain:.2f} m / {n_left} 点，规划未续", detail)

        # 5.3 planning_factors 明确要求停
        stop_f = self._find_stop_factor()
        if stop_f:
            checks.append(("factor", True, "STOP"))
            return Finding("⑤", f"规划要求停车：{stop_f[0]} 在 {stop_f[1]:.2f} m 处置速度 0",
                           "来自 planning_factors，属于模块主动限速")

        if remain is not None:
            checks.append(("剩余轨迹", False, f"{remain:.2f}m"))
        return None

    def _check_control(self, v, stop_s, checks):
        """⑥ 规划给了可走轨迹，但控制层不让走。"""
        if self.is_stopped is True:
            checks.append(("is_stopped", True, "True"))
            extra = ""
            if self.is_start_requested is False:
                extra = "；且 is_start_requested=False（未请求起步）"
            return Finding("⑥", "vehicle_cmd_gate 判定停稳并保持（is_stopped=True）",
                           "闸门主动抑制指令" + extra)
        if self.is_stopped is False:
            checks.append(("is_stopped", False, "False"))

        if self.ctrl_v is not None and abs(self.ctrl_v) < 1e-3:
            checks.append(("ctrl_v", True, "0"))
            a = f"，加速度指令 {self.ctrl_a:+.2f} m/s²" if self.ctrl_a is not None else ""

            # A* 刚切到新段时，控制器要先完成换挡（D↔R）才会给速度，
            # 这一两秒里 control_cmd=0 是正常的，不能当成控制器异常。
            seg = self.pk_seg
            age = (time.time() - seg.first_seen) if seg.first_seen else None
            if seg.index > 1 and age is not None and age < TURN_CONFIRM_S:
                checks.append(("换挡", True, f"第{seg.index}段{age:.1f}s"))
                return Finding(
                    "⑥",
                    f"换挡等待：A* 第 {seg.index} 段（{seg.direction or '?'}）刚下发 "
                    f"{age:.1f}s，控制器尚未给出速度",
                    f"control_cmd.velocity=0{a}；换挡完成后会自动起步，属正常过程")

            return Finding("⑥", "未请求起步：控制指令纵向速度为 0",
                           f"control_cmd.velocity=0{a}；轨迹可走但控制器输出 0，"
                           "查 trajectory_follower 与车辆接口")
        if self.ctrl_v is not None:
            checks.append(("ctrl_v", False, f"{self.ctrl_v:.2f}"))
            # 指令已经给了速度，车却还停着：起步延迟、换挡未完成、或驱动受限。
            # 这是"但仍未动"最常见的一种，必须给出确定结论而不是"原因未明"。
            if stop_s >= 1.0:
                checks.append(("指令vs车速", True,
                               f"{self.ctrl_v:+.2f}/{v:.2f}"))
                return Finding(
                    "⑥",
                    f"指令已下发但车未动：control_cmd={self.ctrl_v:+.2f} m/s，"
                    f"实际车速 {v:.2f} m/s（已静止 {stop_s:.1f}s）",
                    "控制器给了目标速度但车辆没响应：查换挡是否完成、驻车/手刹状态、"
                    "加速度限制或驱动扭矩")

        if self.is_start_requested is False:
            checks.append(("start_req", True, "False"))
            return Finding("⑥", "未请求起步（is_start_requested=False）",
                           "闸门在等起步信号，检查操作模式与 engage 状态")
        return None

    def _check_execution(self, checks):
        """⑦ 档位/模式层面。"""
        g = fmt_gear(self.gear)
        if self.gear == GEAR_PARK:
            checks.append(("档位", True, "P"))
            return Finding("⑦", "车辆已挂 P 档", "挂 P 后驱动指令不会生效")
        if self.gear == GEAR_NEUTRAL:
            checks.append(("档位", True, "N"))
            return Finding("⑦", "车辆在空挡 N", "空挡无法输出动力")
        checks.append(("档位", False, g))
        return None

    # ---- 原因提取 ----

    def _gp_factor_reason(self):
        """从 goal_planner 的 PlanningFactor 里取 detail（最权威的原因字符串）。"""
        if self.gp_factors is None:
            return ""
        texts = []
        for f in getattr(self.gp_factors, "factors", []):
            d = getattr(f, "detail", "") or ""
            mod = getattr(f, "module", "") or ""
            if d:
                texts.append(f"{mod}: {d}" if mod else d)
        return "；".join(texts[:2])

    def _find_stop_factor(self):
        """找 planning_factors 里速度为 0 且距离很近的控制点。"""
        if self.gp_factors is None:
            return None
        for f in getattr(self.gp_factors, "factors", []):
            mod = getattr(f, "module", "") or "?"
            for cp in getattr(f, "control_points", []):
                if abs(getattr(cp, "velocity", 1.0)) < 1e-3:
                    return (mod, getattr(cp, "distance", 0.0))
        return None


def _classify(classification):
    """把 classification 数组压成一个标签（取置信度最高的）。"""
    best, best_p = "?", -1.0
    label_map = {0: "UNKNOWN", 1: "CAR", 2: "TRUCK", 3: "BUS", 4: "TRAILER",
                 5: "MOTORCYCLE", 6: "BICYCLE", 7: "PEDESTRIAN"}
    for c in classification:
        p = getattr(c, "probability", 0.0)
        if p > best_p:
            best_p = p
            best = label_map.get(getattr(c, "label", 0), str(getattr(c, "label", 0)))
    return best


# ---------------------------------------------------------------- 输出

def status_line(node, finding):
    """主行：把车辆的行驶信息压进一行。"""
    v = node.v if node.v is not None else None
    v_txt = f"{v:5.2f}" if v is not None else "  ?  "
    dirn = "静止"
    if v is None:
        dirn = "无车速"
    elif v >= V_STOP:
        dirn = "倒车" if (node.v_signed or 0) < -0.01 else "前进"

    # 分段信息：方向加括号，与"当前实际行驶方向"区分开
    # （车可能已驶离上一段的末端，两者不一致时不至于看不懂）
    seg_txt = ""
    if node.scenario == "Parking" and node.pk_seg.index > 0:
        remain, n_left, _, on_track = node.traj_remaining()
        if on_track and remain is not None:
            r = f" 剩{remain:.2f}m"
        else:
            r = " 已离轨迹"
        seg_txt = f" 段{node.pk_seg.index}({node.pk_seg.direction or '?'}){r}"

    pose_txt = ""
    if node.ego:
        ex, ey, eyaw = node.ego
        pose_txt = f" pos({ex:.2f},{ey:.2f}) yaw{math.degrees(eyaw):.0f}°"

    goal_txt = ""
    off = node.goal_offset()
    if off:
        goal_txt = f" 目标{off[0]:.2f}m"

    obs_txt = ""
    if node.near_obstacle is not None:
        obs_txt = f" 障{node.near_obstacle:.1f}m"

    name = STATE_NAME.get(node.state, "?") if node.state else "?"
    # 四种运动语义，别把"还没到该走的阶段"也标成[行驶]
    if node.state is None:
        tag = "[等待]"          # 连状态机都没收到
    elif name != "DRIVING":
        tag = "[待命]"          # 初始化/等路线/等 Engage/已到达收尾
    elif v is not None and v < V_STOP:
        tag = "[停车]"
    else:
        tag = "[行驶]"
    return (f"{time.strftime('%H:%M:%S')} {tag} {name:<17} "
            f"{(node.scenario or '?'):<11} v={v_txt} {dirn}{seg_txt}"
            f"{pose_txt}{goal_txt}{obs_txt}")


def print_finding(node, finding):
    """打印停车原因：一行结论 + 一行补充说明。

    只讲"为什么停"。命中了哪一项、排除了哪一项属于内部推理过程，
    不输出 —— 会把真正的原因淹掉，也让整段日志没法快速扫读。
    """
    print(f"        └ ★ {finding.title}", flush=True)
    if finding.detail:
        print(f"           {finding.detail}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--line", action="store_true",
                    help="每次刷新打印一行（重定向到文件用这个）")
    ap.add_argument("--events", action="store_true",
                    help="只在结论变化时打印，形成干净的时间线")
    ap.add_argument("--period", type=float, default=0.5, help="采样周期（秒）")
    # 已废弃：逐项核对不再输出。参数保留只为兼容旧的调用方式
    # （run_watch.sh / watch_after_ready.sh 里还带着 --verbose），传了也不会报错。
    ap.add_argument("--explain", "--verbose", action="store_true", dest="explain",
                    help="已废弃，无效果（逐项核对不再输出）")
    ap.add_argument("--repeat", type=float, default=10.0,
                    help="同一停车原因重复打印的最小间隔（秒），默认 10")
    args = ap.parse_args()

    rclpy.init()
    node = WatchState()

    print("监听 状态机/场景/车速/MRM/障碍/轨迹/A*/goal_planner/控制/档位/诊断/rosout")
    print("归因顺序：任务终态 → 状态机未行驶 → 安全接管 → 规划无路径 "
          "→ 规划要求停 → 控制层 → 执行层 → 无结论")
    print("（Ctrl+C 退出）\n")

    last_print = 0.0
    last_key = ""
    last_emit = 0.0
    events = []      # [(时刻, 分级码, 结论)]，退出时汇总成时间线
    # 回调异常：同类错误 5 s 内只提示一次，避免刷屏掩盖正常输出
    last_err, last_err_t = "", 0.0

    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except Exception as e:
                if not rclpy.ok():
                    break
                # 带上异常类型，方便直接定位；同类错误限流，避免刷屏
                msg = f"{type(e).__name__}: {e}"
                now_e = time.time()
                if msg != last_err or now_e - last_err_t >= 5.0:
                    print(f"\n[警告] 处理消息出错，已跳过: {msg}", file=sys.stderr)
                    last_err, last_err_t = msg, now_e
                continue

            now = time.time()
            if now - last_print < args.period:
                continue
            last_print = now

            finding = node.diagnose()
            line = status_line(node, finding)

            # 去重键：分级码 + 去掉数字的标题
            # （距离从 0.31 变 0.29 属于同一结论，不该重复打印）
            key = finding.code + re.sub(r"[-\d.]+", "", finding.title)
            changed = key != last_key
            if changed:
                events.append((time.strftime("%H:%M:%S"), finding.code, finding.title))

            if args.events:
                if not changed:
                    continue
                print(line, flush=True)
                print_finding(node, finding)
                last_key, last_emit = key, now
                continue

            if args.line:
                print(line, flush=True)
                # 行驶中（"—"）只打一行；其余在结论变化或周期提醒时补原因
                if finding.code != "—" and (changed or now - last_emit >= args.repeat):
                    print_finding(node, finding)
                    last_emit = now
            else:
                sys.stdout.write("\r" + line + "\033[K")
                sys.stdout.flush()
                if changed:
                    # 原地刷新模式下结论变化时换行打印，避免被下一帧覆盖
                    print()
                    print_finding(node, finding)
            last_key = key

    except KeyboardInterrupt:
        pass
    finally:
        if not args.line and not args.events:
            sys.stdout.write("\n")
        # 退出摘要：整场运行里每次结论变化的完整时间线
        # --events 模式下正文已经是时间线了，不再重复
        if events and not args.events:
            print("\n===== 结论变化时间线（每次停车都会在这里） =====")
            for t, code, title in events:
                print(f"  {t}  ★  {title}")
        if node.state is None:
            print("[提示] 没收到 /autoware/state，确认 Autoware 在运行且已 source "
                  "~/autoware/install/setup.bash", file=sys.stderr)
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
