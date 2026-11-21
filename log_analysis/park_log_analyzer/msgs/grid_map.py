"""grid_map_msgs/GridMap 解码。

本仓库版本 `data` 为 `Float32MultiArray[]`（每层一个 MultiArray），
旧版本为单块 `float32[]`。两种都支持（开发文档 §3.2.1 的旧布局 + 实测新布局）。
"""

from __future__ import annotations

from array import array

from ..cdr import Cdr, payload
from ..model import GridFrame


def decode(raw: bytes, t_bag: float = 0.0) -> GridFrame:
    c = Cdr(payload(raw))
    stamp, frame_id = c.header()
    res = c.f64()
    len_x = c.f64()
    len_y = c.f64()
    pose = c.pose_raw()

    n_layers = c.u32()
    layers = tuple(c.string() for _ in range(n_layers))
    n_basic = c.u32()                      # basic_layers（本仓库恒为空）
    for _ in range(n_basic):
        c.string()

    lx = int(round(len_x / res)) if res else 0
    ly = int(round(len_y / res)) if res else 0
    need = lx * ly

    cells = {}
    # 布局 A：uint32 n_arrays(=层数) + 每层 MultiArrayLayout + float32[]
    # 布局 B：uint32 n_floats + 稠密 float32[]（所有层连续，旧版本）
    n = c.u32()
    if n == n_layers and n_layers != 1:
        for name in layers:
            ndim = c.u32()
            for _ in range(ndim):
                c.string()
                c.u32()
                c.u32()
            c.u32()  # data_offset
            arr = c.f32_seq()
            if len(arr) != need:
                raise ValueError("层 %s 尺寸 %d != %d" % (name, len(arr), need))
            cells[name] = arr
    elif n == need:
        cells[layers[0]] = c.f32_array(need)
    elif n == n_layers * need:
        flat = c.f32_array(n)
        for i, name in enumerate(layers):
            cells[name] = flat[i * need:(i + 1) * need]
    else:
        raise ValueError("GridMap data 长度异常：n=%d layers=%d cells=%d" % (n, n_layers, need))

    return GridFrame(
        t_bag=t_bag,
        stamp=stamp,
        frame_id=frame_id,
        resolution=res,
        lx=lx,
        ly=ly,
        center=(pose[0], pose[1], pose[2]),
        layers=layers,
        cells=cells,
    )


def layer_signature(cells: dict, layers) -> str:
    """快速帧间比较：拼接各层原始字节后取 sha1 前 16 位。"""
    import hashlib

    h = hashlib.sha1()
    for name in layers:
        h.update(memoryview(cells[name]).cast("B"))
    return h.hexdigest()[:16]


def raw_tobytes(cells: dict, layers) -> bytes:
    out = bytearray()
    for name in layers:
        out += memoryview(cells[name]).cast("B")
    return bytes(out)


def _unused():  # pragma: no cover
    return array
