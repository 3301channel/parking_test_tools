"""autoware_perception_msgs/PredictedObjects：只读 header + objects_size。

object 数组内容不解析（开发文档 §3.2.9），仅用于判定"感知是否喂料"。
"""

from __future__ import annotations

from ..cdr import Cdr, payload


def decode(raw: bytes, t_bag: float = 0.0):
    c = Cdr(payload(raw))
    stamp, frame_id = c.header()
    n = c.u32()
    return {"t_bag": t_bag, "stamp": stamp, "frame_id": frame_id, "objects_size": n,
            "bytes": len(raw)}
