"""nav_msgs/OccupancyGrid（开发文档 §3.2.2）。

origin 是左下角；data[0] 对应 (origin.x, origin.y)，先 x 后 y。
"""

from __future__ import annotations

from ..cdr import Cdr, payload
from ..model import OccFrame


def decode(raw: bytes, t_bag: float = 0.0) -> OccFrame:
    c = Cdr(payload(raw))
    stamp, frame_id = c.header()
    c.time()                      # map_load_time
    res = c.f32()
    w = c.u32()
    h = c.u32()
    ox, oy, oz, oyaw = c.pose()
    n = c.u32()
    data = c.take(n)
    if n != w * h:
        raise ValueError("OccupancyGrid data %d != %d*%d" % (n, w, h))
    return OccFrame(
        t_bag=t_bag, stamp=stamp, frame_id=frame_id, resolution=res,
        w=w, h=h, origin=(ox, oy, oyaw), data=data,
    )
