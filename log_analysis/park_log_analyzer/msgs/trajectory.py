"""autoware_planning_msgs/Trajectory（开发文档 §3.2.5，每点 88 B）。"""

from __future__ import annotations

from ..cdr import Cdr, payload
from ..model import TrajFrame, TrajPoint


def decode(raw: bytes, t_bag: float = 0.0) -> TrajFrame:
    c = Cdr(payload(raw))
    stamp, frame_id = c.header()
    n = c.u32()
    pts = []
    for _ in range(n):
        t0 = c.time()
        x, y, z, yaw = c.pose()
        pts.append(TrajPoint(
            x=x, y=y, z=z, yaw=yaw,
            v_long=c.f32(), v_lat=c.f32(), w_heading=c.f32(),
            accel=c.f32(), front_steer=c.f32(), rear_steer=c.f32(),
            t_from_start=t0,
        ))
    return TrajFrame(t_bag=t_bag, stamp=stamp, frame_id=frame_id, points=tuple(pts))
