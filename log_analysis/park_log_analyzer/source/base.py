"""Bag 读取后端抽象（开发文档 §5.2）。"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, Iterator, Tuple


@dataclass(frozen=True)
class TopicMeta:
    name: str
    type: str
    count: int
    nbytes: int

    @property
    def short_type(self) -> str:
        return self.type.replace("/msg/", "/")


class BagError(Exception):
    pass


class Reader:
    """所有后端统一接口。"""

    def topics(self) -> Dict[str, TopicMeta]:  # pragma: no cover - 接口
        raise NotImplementedError

    def messages(self, topic: str) -> Iterator[Tuple[int, bytes]]:  # pragma: no cover
        raise NotImplementedError

    def start_ns(self) -> int:  # pragma: no cover
        raise NotImplementedError

    def end_ns(self) -> int:  # pragma: no cover
        raise NotImplementedError

    def span(self, topic: str):  # pragma: no cover - (count, first_ns, last_ns)
        raise NotImplementedError

    def last_message(self, topic: str) -> Tuple[int, bytes] | None:  # pragma: no cover
        raise NotImplementedError


def open_bag(path: str):
    """按目录内容自动选择后端：*.mcap → mcap，*.db3 → sqlite3。"""
    if os.path.isdir(path):
        names = os.listdir(path)
        if any(n.endswith(".mcap") for n in names):
            from .mcap import McapReader

            return McapReader(path)
        if any(n.endswith(".db3") for n in names):
            from .sqlite import SqliteReader

            return SqliteReader(path)
        raise BagError("目录里既没有 *.mcap 也没有 *.db3：%s" % path)
    if path.endswith(".mcap"):
        from .mcap import McapReader

        return McapReader(path)
    if path.endswith(".db3"):
        from .sqlite import SqliteReader

        return SqliteReader(path)
    raise BagError("无法识别的 bag 路径：%s" % path)
