"""autoware_planning_msgs/LaneletRoute（开发文档 §3.2.3）。

字段序：header → start_pose → goal_pose；uuid 是定长 uint8[16]。
"""

from __future__ import annotations

from ..cdr import Cdr, payload
from ..model import RouteInfo


def decode(raw: bytes, t_bag: float = 0.0) -> RouteInfo:
    c = Cdr(payload(raw))
    stamp, _ = c.header()
    start = c.pose()
    goal = c.pose()
    n_seg = c.u32()
    segs = []
    n_prim_total = 0
    for _ in range(n_seg):
        pid = c.i64()
        ptype = c.string()
        n_pr = c.u32()
        prims = []
        for _ in range(n_pr):
            prims.append((c.i64(), c.string()))
        n_prim_total += n_pr
        segs.append((pid, ptype, tuple(prims)))
    uuid_bytes = c.take(16)
    allow = c.boolean()
    return RouteInfo(
        t_bag=t_bag, stamp=stamp, start=start, goal=goal,
        segments=tuple(segs), uuid=uuid_bytes.hex(), allow_modification=allow,
        n_primitives=n_prim_total,
    )
