"""MCAP 直读后端（rosbag2 jazzy 默认格式）。

采用「一次索引 + 按需 seek」：4 万条消息的索引常驻内存，payload 不常驻。
支持未压缩 / zstd / lz4 chunk；压缩 chunk 的 payload 常驻内存（体积远小于原始 bag）。
"""

from __future__ import annotations

import os
import struct
from typing import Dict, Iterator, List, Tuple

from .base import BagError, Reader, TopicMeta

MAGIC = b"\x89MCAP0\r\n"

OP_SCHEMA = 0x03
OP_CHANNEL = 0x04
OP_MESSAGE = 0x05
OP_CHUNK = 0x06
OP_DATA_END = 0x0F


class McapReader(Reader):
    def __init__(self, path: str):
        if os.path.isdir(path):
            files = sorted(f for f in os.listdir(path) if f.endswith(".mcap"))
            if not files:
                raise BagError("目录内无 .mcap：%s" % path)
            path = os.path.join(path, files[0])
        self.path = path
        self._schemas: Dict[int, str] = {}
        self._channels: Dict[int, Tuple[str, int]] = {}          # cid -> (topic, schema_id)
        self._index: Dict[str, List[Tuple[int, int, int]]] = {}  # topic -> [(log_ns, off, len)]
        self._copies: Dict[str, List[Tuple[int, bytes]]] = {}
        self._bytes: Dict[str, int] = {}
        self._start_ns = 0
        self._build_index()

    # ------------------------------------------------------------------
    def _build_index(self) -> None:
        size = os.path.getsize(self.path)
        first = None
        with open(self.path, "rb") as f:
            if f.read(8) != MAGIC:
                raise BagError("不是 MCAP 文件：%s" % self.path)
            pos = 8
            while pos + 9 <= size:
                f.seek(pos)
                head = f.read(9)
                if len(head) < 9:
                    break
                op = head[0]
                ln = struct.unpack_from("<Q", head, 1)[0]
                body_off = pos + 9
                if op == OP_MESSAGE:
                    body = f.read(ln)
                    self._add_message(body, body_off + 22)
                elif op == OP_CHUNK:
                    self._parse_chunk(f.read(ln), body_off)
                elif op == OP_CHANNEL:
                    self._add_channel(f.read(ln))
                elif op == OP_SCHEMA:
                    self._add_schema(f.read(ln))
                elif op == OP_DATA_END:
                    break
                pos = body_off + ln

        for topic, items in self._index.items():
            items.sort()
            self._bytes[topic] = sum(n for _, _, n in items)
            if items:
                first = items[0][0] if first is None else min(first, items[0][0])
        self._start_ns = first or 0

    def _parse_chunk(self, body: bytes, body_off: int) -> None:
        if len(body) < 32:
            return
        off = 28
        slen = struct.unpack_from("<I", body, off)[0]
        off += 4
        comp = body[off:off + slen].decode("ascii", "replace")
        off += slen
        rlen = struct.unpack_from("<Q", body, off)[0]
        off += 8
        rec = body[off:off + rlen]
        if comp:
            rec = _decompress(comp, rec)
            seekable = False
        else:
            seekable = True
        base = body_off + off
        i = 0
        while i + 9 <= len(rec):
            op = rec[i]
            ln = struct.unpack_from("<Q", rec, i + 1)[0]
            s = i + 9
            if s + ln > len(rec):
                break
            if op == OP_MESSAGE:
                if seekable:
                    self._add_message(rec[s:s + ln], base + s + 22)
                else:
                    self._add_message_copy(rec[s:s + ln])
            elif op == OP_CHANNEL:
                self._add_channel(rec[s:s + ln])
            elif op == OP_SCHEMA:
                self._add_schema(rec[s:s + ln])
            i = s + ln

    def _add_schema(self, body: bytes) -> None:
        if len(body) < 6:
            return
        sid = struct.unpack_from("<H", body, 0)[0]
        nlen = struct.unpack_from("<I", body, 2)[0]
        self._schemas[sid] = body[6:6 + nlen].decode("utf-8", "replace")

    def _add_channel(self, body: bytes) -> None:
        if len(body) < 8:
            return
        cid, sid = struct.unpack_from("<HH", body, 0)
        tlen = struct.unpack_from("<I", body, 4)[0]
        topic = body[8:8 + tlen].decode("utf-8", "replace")
        self._channels[cid] = (topic, sid)

    def _add_message(self, body: bytes, data_off: int) -> None:
        if len(body) < 22:
            return
        cid = struct.unpack_from("<H", body, 0)[0]
        log_ns = struct.unpack_from("<Q", body, 6)[0]
        ch = self._channels.get(cid)
        if ch is None:
            return
        self._index.setdefault(ch[0], []).append((log_ns, data_off, len(body) - 22))

    def _add_message_copy(self, body: bytes) -> None:
        if len(body) < 22:
            return
        cid = struct.unpack_from("<H", body, 0)[0]
        log_ns = struct.unpack_from("<Q", body, 6)[0]
        ch = self._channels.get(cid)
        if ch is None:
            return
        self._copies.setdefault(ch[0], []).append((log_ns, body[22:]))

    # ------------------------------------------------------------------
    def _type_of(self, schema_id: int) -> str:
        return self._schemas.get(schema_id, "")

    def topics(self) -> Dict[str, TopicMeta]:
        out: Dict[str, TopicMeta] = {}
        for cid, (topic, sid) in self._channels.items():
            items = self._index.get(topic, [])
            out[topic] = TopicMeta(topic, self._type_of(sid), len(items), self._bytes.get(topic, 0))
        # 同一话题可能出现在多个 channel（QoS 变化）→ 合并
        merged: Dict[str, TopicMeta] = {}
        for meta in out.values():
            old = merged.get(meta.name)
            if old is None:
                merged[meta.name] = meta
            else:
                merged[meta.name] = TopicMeta(
                    meta.name, old.type or meta.type, old.count + meta.count, old.nbytes + meta.nbytes
                )
        return merged

    def start_ns(self) -> int:
        return self._start_ns

    def end_ns(self) -> int:
        end = self._start_ns
        for items in self._index.values():
            if items:
                end = max(end, items[-1][0])
        for items in self._copies.values():
            if items:
                end = max(end, items[-1][0])
        return end

    def span(self, topic: str):
        items = self._index.get(topic, [])
        if not items:
            return (0, 0, 0)
        return (len(items), items[0][0], items[-1][0])

    def last_message(self, topic: str):
        copies = self._copies.get(topic)
        if copies:
            return copies[-1]
        items = self._index.get(topic)
        if not items:
            return None
        log_ns, off, n = items[-1]
        with open(self.path, "rb") as f:
            f.seek(off)
            return log_ns, f.read(n)

    def messages(self, topic: str) -> Iterator[Tuple[int, bytes]]:
        items = list(self._index.get(topic, []))
        if topic in self._copies:
            items = [(ns, -1, 0) for ns, _ in self._copies[topic]] + items
        items.sort()
        copies = dict((ns, data) for ns, data in self._copies.get(topic, []))
        with open(self.path, "rb") as f:
            for log_ns, off, n in items:
                if off < 0:
                    yield log_ns, copies[log_ns]
                else:
                    f.seek(off)
                    yield log_ns, f.read(n)


def _decompress(comp: str, data: bytes) -> bytes:
    if comp == "zstd":
        try:
            import zstandard  # type: ignore
        except ImportError as e:  # pragma: no cover
            raise BagError("chunk 为 zstd 压缩，请安装 zstandard") from e
        return zstandard.ZstdDecompressor().decompress(data)
    if comp == "lz4":
        try:
            import lz4.frame  # type: ignore
        except ImportError as e:  # pragma: no cover
            raise BagError("chunk 为 lz4 压缩，请安装 lz4") from e
        return lz4.frame.decompress(data)
    raise BagError("未知 chunk 压缩格式：%s" % comp)
