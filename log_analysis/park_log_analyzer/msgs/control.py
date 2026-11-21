"""控制链 / 车辆状态 / ADAPI 状态消息的轻量解码。

只取"判断链路是否贯通"必需的标量，不还原完整消息：

  autoware_control_msgs/Control          -> velocity / acceleration / steering
  autoware_vehicle_msgs/VelocityReport   -> 纵向、横向速度与横摆角速度
  autoware_vehicle_msgs/SteeringReport   -> 前轮转角
  autoware_vehicle_msgs/ActuationCommand -> accel/brake/steer 幅值指令
  autoware_system_msgs/AutowareState     -> uint16 状态机
  autoware_adapi_v1_msgs/{OperationMode,Motion,Routing}State -> uint8 状态

字段数量在不同 Autoware 版本间有差异（Control 是否带 jerk、是否带
is_defined_* 布尔量），因此按"能读几个 8 字节 double"推断布局，而不是写死下标。
"""

from __future__ import annotations

from ..cdr import Cdr, CdrError, payload

# ---- 枚举表（与 autoware 消息定义一致）----
AW_STATE = {
    0: "UNKNOWN", 1: "INITIALIZING", 2: "WAITING_FOR_ROUTE", 3: "PLANNING",
    4: "WAITING_FOR_ENGAGE", 5: "DRIVING", 6: "ARRIVED_GOAL", 7: "FINALIZING",
}
OPERATION_MODE = {0: "UNKNOWN", 1: "STOP", 2: "AUTONOMOUS", 3: "LOCAL", 4: "REMOTE"}
MOTION_STATE = {0: "UNKNOWN", 1: "STOPPED", 2: "STARTING", 3: "MOVING"}
ROUTING_STATE = {0: "UNKNOWN", 1: "UNSET", 2: "ROUTE_SET", 3: "ARRIVED", 4: "CHANGING"}


def _doubles_after_header(raw: bytes, max_n: int = 8):
    """读 header 之后的 double 序列（CDR 对齐会自动跳过 1 字节 bool 与填充）。

    返回 (stamp, frame_id, values)。读到越界就停，不抛异常。
    """
    c = Cdr(payload(raw))
    stamp, frame_id = c.header()
    vals = []
    for _ in range(max_n):
        try:
            vals.append(c.f64())
        except CdrError:
            break
    return stamp, frame_id, vals


def _control_layout(raw: bytes):
    """推断 Control 的 (velocity, acceleration, steering)。

    Control 的纵向/横向各是若干 double，横向前两个分别是
    steering_tire_angle / steering_tire_rotation_rate。所以：
      - 共 5 个 double（带 jerk）→ 取 [3] 作为转角
      - 共 4 个 double（无 jerk）→ 取 [2] 作为转角
    用"读满 n 个后剩余字节 <= 8"校验，避免把尾部 bool 当成 double。
    """
    for n_long in (3, 2):
        try:
            c = Cdr(payload(raw))
            stamp, frame_id = c.header()
            vals = [c.f64() for _ in range(n_long + 2)]
            remain = len(c.d) - c.o
        except CdrError:
            continue
        if remain <= 8:
            v = vals[0]
            a = vals[1] if len(vals) > 1 else 0.0
            steer = vals[n_long] if len(vals) > n_long else None
            return stamp, frame_id, v, a, steer
    return None


def decode_control(raw: bytes, t_bag: float = 0.0):
    got = _control_layout(raw)
    if got is None:
        raise CdrError("Control 字段数无法解释")
    stamp, frame_id, v, a, steer = got
    return {"t_bag": t_bag, "stamp": stamp, "frame_id": frame_id,
            "velocity": v, "acceleration": a, "steering": steer}


def decode_velocity_report(raw: bytes, t_bag: float = 0.0):
    c = Cdr(payload(raw))
    stamp, frame_id = c.header()
    return {"t_bag": t_bag, "stamp": stamp, "frame_id": frame_id,
            "v_long": c.f32(), "v_lat": c.f32(), "yaw_rate": c.f32()}


def decode_steering_report(raw: bytes, t_bag: float = 0.0):
    c = Cdr(payload(raw))
    stamp, frame_id = c.header()
    return {"t_bag": t_bag, "stamp": stamp, "frame_id": frame_id, "steer": c.f32()}


def decode_actuation_command(raw: bytes, t_bag: float = 0.0):
    # autoware_vehicle_msgs/ActuationCommand 没有 header：accel/brake/steer 三个 double。
    c = Cdr(payload(raw))
    return {"t_bag": t_bag, "stamp": t_bag, "frame_id": "",
            "accel": c.f64(), "brake": c.f64(), "steer": c.f64()}


def decode_autoware_state(raw: bytes, t_bag: float = 0.0):
    # autoware_system_msgs/AutowareState：单个 uint16，无 header。
    c = Cdr(payload(raw))
    value = c.u16()
    return {"t_bag": t_bag, "stamp": t_bag, "state": value,
            "state_name": AW_STATE.get(value, "STATE_%d" % value)}


def decode_operation_mode(raw: bytes, t_bag: float = 0.0):
    # builtin_interfaces/Time stamp; uint8 mode; bool×6
    c = Cdr(payload(raw))
    stamp = c.time()
    mode = c.u8()
    flags = {}
    for name in ("is_autoware_control_enabled", "is_in_transition",
                 "is_stop_mode_available", "is_autonomous_mode_available",
                 "is_local_mode_available", "is_remote_mode_available"):
        try:
            flags[name] = c.boolean()
        except CdrError:
            break
    return {"t_bag": t_bag, "stamp": stamp, "mode": mode,
            "mode_name": OPERATION_MODE.get(mode, "MODE_%d" % mode), **flags}


def decode_motion_state(raw: bytes, t_bag: float = 0.0):
    c = Cdr(payload(raw))
    stamp = c.time()
    state = c.u8()
    return {"t_bag": t_bag, "stamp": stamp, "state": state,
            "state_name": MOTION_STATE.get(state, "MOTION_%d" % state)}


def decode_routing_state(raw: bytes, t_bag: float = 0.0):
    c = Cdr(payload(raw))
    stamp = c.time()
    state = c.u8()
    return {"t_bag": t_bag, "stamp": stamp, "state": state,
            "state_name": ROUTING_STATE.get(state, "ROUTING_%d" % state)}
