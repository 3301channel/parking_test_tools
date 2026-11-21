"""autoware_internal_planning_msgs/Scenario。

本仓库版本没有 header：
    string current_scenario
    string[] activating_scenarios
（开发文档 §3.2.8 记录的是带 header 的旧版本，此处两种都兼容。）
"""

from __future__ import annotations

from ..cdr import Cdr, payload


def decode(raw: bytes, t_bag: float = 0.0):
    c = Cdr(payload(raw))
    first = c.string()
    if first in ("map", "") or len(first) > 60:
        # 旧版：header(Time + frame_id) 打头
        c2 = Cdr(payload(raw))
        stamp, frame_id = c2.header()
        n = c2.u32()
        names = tuple(c2.string() for _ in range(n))
        return {"t_bag": t_bag, "stamp": stamp, "frame_id": frame_id, "scenarios": names}
    n = c.u32()
    names = tuple(c.string() for _ in range(n))
    return {"t_bag": t_bag, "stamp": t_bag, "frame_id": "", "scenarios": names,
            "current": first}
