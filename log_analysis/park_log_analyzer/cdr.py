"""CDR 读取原语。

关键约束（开发文档 §3.2 / §10.3）：
- 所有偏移相对 payload 起点（跳过 4 B 封装头），对齐也相对 offset 0；
- string = uint32 len（含结尾 \\0）+ bytes + \\0，len=0 表示空串；
- 数组/序列 = uint32 长度 + 紧接的元素（元素各自按自然对齐）。
"""

from __future__ import annotations

import struct
from array import array


class CdrError(Exception):
    """字节流与期望的 msg 定义不符。"""


class Cdr:
    __slots__ = ("d", "o")

    def __init__(self, data, offset: int = 0):
        self.d = data
        self.o = offset

    # -- 基础 ------------------------------------------------------------
    def align(self, n: int) -> None:
        r = self.o % n
        if r:
            self.o += n - r

    def _read(self, fmt: str, size: int):
        self.align(size)
        if self.o + size > len(self.d):
            raise CdrError("越界：offset=%d size=%d len=%d" % (self.o, size, len(self.d)))
        v = struct.unpack_from(fmt, self.d, self.o)[0]
        self.o += size
        return v

    def u8(self) -> int:
        return self._read("<B", 1)

    def i8(self) -> int:
        return self._read("<b", 1)

    def boolean(self) -> bool:
        return self.u8() != 0

    def u16(self) -> int:
        return self._read("<H", 2)

    def i16(self) -> int:
        return self._read("<h", 2)

    def u32(self) -> int:
        return self._read("<I", 4)

    def i32(self) -> int:
        return self._read("<i", 4)

    def u64(self) -> int:
        return self._read("<Q", 8)

    def i64(self) -> int:
        return self._read("<q", 8)

    def f32(self) -> float:
        return self._read("<f", 4)

    def f64(self) -> float:
        return self._read("<d", 8)

    # -- 复合 ------------------------------------------------------------
    def string(self) -> str:
        n = self.u32()
        if n == 0:
            return ""
        if self.o + n > len(self.d):
            raise CdrError("字符串越界：len=%d offset=%d" % (n, self.o))
        b = bytes(self.d[self.o:self.o + n])
        self.o += n
        return b.rstrip(b"\0").decode("utf-8", "replace")

    def take(self, n: int) -> bytes:
        if self.o + n > len(self.d):
            raise CdrError("take 越界：%d > %d" % (self.o + n, len(self.d)))
        b = bytes(self.d[self.o:self.o + n])
        self.o += n
        return b

    def f32_array(self, n: int) -> array:
        self.align(4)
        if self.o + 4 * n > len(self.d):
            raise CdrError("float32[] 越界：n=%d offset=%d" % (n, self.o))
        a = array("f")
        a.frombytes(self.d[self.o:self.o + 4 * n])
        self.o += 4 * n
        return a

    def i8_array(self, n: int) -> bytes:
        return self.take(n)

    def f32_seq(self) -> array:
        return self.f32_array(self.u32())

    # -- 常用消息片段 ----------------------------------------------------
    def time(self) -> float:
        sec = self.i32()
        nsec = self.u32()
        return sec + nsec * 1e-9

    def header(self):
        stamp = self.time()
        frame_id = self.string()
        return stamp, frame_id

    def pose(self):
        """返回 (x, y, z, yaw)。"""
        self.align(8)
        x, y, z = self.f64(), self.f64(), self.f64()
        qx, qy, qz, qw = self.f64(), self.f64(), self.f64(), self.f64()
        return x, y, z, quat_to_yaw(qx, qy, qz, qw)

    def pose_raw(self):
        """返回 (x, y, z, qx, qy, qz, qw)。"""
        self.align(8)
        x, y, z = self.f64(), self.f64(), self.f64()
        qx, qy, qz, qw = self.f64(), self.f64(), self.f64(), self.f64()
        return x, y, z, qx, qy, qz, qw


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    import math

    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def payload(raw: bytes) -> memoryview:
    """剥掉 4 B CDR 封装头。"""
    if len(raw) < 4:
        raise CdrError("消息长度 %d < 4" % len(raw))
    return memoryview(raw)[4:]
