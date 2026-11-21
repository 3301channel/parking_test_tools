"""标量类消息：std_msgs/Bool、Float64Stamped、PoseStamped。"""

from __future__ import annotations

from ..cdr import Cdr, payload
from ..model import ScalarPoint


def decode_bool(raw: bytes, t_bag: float = 0.0) -> ScalarPoint:
    # std_msgs/Bool：data 在 payload offset 0（消息 offset 4）
    c = Cdr(payload(raw))
    return ScalarPoint(t_bag=t_bag, stamp=t_bag, value=1.0 if c.u8() else 0.0, raw=raw)


def decode_bool_stamped(raw: bytes, t_bag: float = 0.0) -> ScalarPoint:
    # std_msgs/BoolStamped：header + bool
    c = Cdr(payload(raw))
    stamp, _ = c.header()
    return ScalarPoint(t_bag=t_bag, stamp=stamp, value=1.0 if c.u8() else 0.0, raw=raw)


def decode_float64_stamped(raw: bytes, t_bag: float = 0.0) -> ScalarPoint:
    # 没有 header / frame_id：仅 Time(8) + float64(8)
    c = Cdr(payload(raw))
    stamp = c.time()
    return ScalarPoint(t_bag=t_bag, stamp=stamp, value=c.f64(), raw=raw)


def decode_pose_stamped(raw: bytes, t_bag: float = 0.0):
    c = Cdr(payload(raw))
    stamp, frame_id = c.header()
    x, y, z, yaw = c.pose()
    return {"t_bag": t_bag, "stamp": stamp, "frame_id": frame_id,
            "x": x, "y": y, "z": z, "yaw": yaw}


GEAR_NAMES = {0: "NONE", 1: "NEUTRAL", 2: "DRIVE", 20: "REVERSE", 22: "PARK"}


def decode_gear(raw: bytes, t_bag: float = 0.0):
    """autoware_vehicle_msgs/GearReport 与 GearCommand 同构：stamp + uint8。"""
    c = Cdr(payload(raw))
    stamp = c.time()
    value = c.u8()
    return {"t_bag": t_bag, "stamp": stamp, "gear": value,
            "gear_name": GEAR_NAMES.get(value, str(value))}
