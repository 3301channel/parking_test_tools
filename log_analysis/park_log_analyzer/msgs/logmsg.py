"""rcl_interfaces/Log 与 diagnostic_msgs/DiagnosticArray 的轻量解码（S-04/S-05）。"""

from __future__ import annotations

from ..cdr import Cdr, payload

LEVELS = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}


def decode_log(raw: bytes, t_bag: float = 0.0):
    # rcl_interfaces/Log 本版本没有 header，只有 stamp：
    #   Time(8) | uint8 level | string name | string msg | string file | string function | uint32 line
    c = Cdr(payload(raw))
    stamp = c.time()
    level = c.u8()
    name = c.string()
    msg = c.string()
    c.string()  # file
    c.string()  # function
    line = c.u32()
    return {"t_bag": t_bag, "stamp": stamp, "level": LEVELS.get(level, str(level)),
            "name": name, "msg": msg, "line": line}


def decode_diagnostics(raw: bytes, t_bag: float = 0.0):
    """只取 header + 每个 status 的 level 与 name，够做分组统计。"""
    c = Cdr(payload(raw))
    stamp, _ = c.header()
    n = c.u32()
    out = []
    for _ in range(n):
        # DiagnosticStatus: level(int8) name(str) message(str) hardware_id(str) values[]
        c.align(1)
        level = c.i8()
        name = c.string()
        c.string()
        c.string()
        nv = c.u32()
        for _ in range(nv):
            c.string()
            c.string()
        out.append((level, name))
    return {"t_bag": t_bag, "stamp": stamp, "statuses": out}
