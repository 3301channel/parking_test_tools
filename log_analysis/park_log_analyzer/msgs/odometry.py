"""nav_msgs/Odometry（开发文档 §3.2.4，注意 child_frame_id 不能漏）。"""

from __future__ import annotations

from ..cdr import Cdr, payload
from ..model import OdomFrame


def decode(raw: bytes, t_bag: float = 0.0) -> OdomFrame:
    c = Cdr(payload(raw))
    stamp, _ = c.header()
    c.string()                    # child_frame_id
    x, y, _z, yaw = c.pose()
    for _ in range(36):           # pose covariance
        c.f64()
    vx, vy, _vz = c.f64(), c.f64(), c.f64()
    wx, wy, wz = c.f64(), c.f64(), c.f64()
    for _ in range(36):           # twist covariance
        c.f64()
    return OdomFrame(t_bag=t_bag, stamp=stamp, x=x, y=y, yaw=yaw,
                     vx=vx, vy=vy, wz=wz)
