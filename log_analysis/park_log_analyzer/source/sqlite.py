"""rosbag2 SQLite3 直读后端（零依赖默认路径，开发文档 §10.1）。"""

from __future__ import annotations

import os
import sqlite3
from typing import Dict, Iterator, Tuple

from .base import BagError, Reader, TopicMeta


class SqliteReader(Reader):
    def __init__(self, path: str):
        if os.path.isdir(path):
            files = sorted(f for f in os.listdir(path) if f.endswith(".db3"))
            if not files:
                raise BagError("目录内无 .db3：%s" % path)
            path = os.path.join(path, files[0])
        self.path = path
        self._conn = sqlite3.connect("file:%s?mode=ro" % path, uri=True)
        self._topics = self._load_topics()

    def _load_topics(self) -> Dict[str, TopicMeta]:
        cur = self._conn.cursor()
        out: Dict[str, TopicMeta] = {}
        try:
            rows = cur.execute(
                "SELECT t.name, t.type, COUNT(m.id), COALESCE(SUM(m.size),0) "
                "FROM topics t LEFT JOIN messages m ON m.topic_id = t.id GROUP BY t.id"
            ).fetchall()
        except sqlite3.Error as e:  # pragma: no cover
            raise BagError("读取 sqlite 失败：%s" % e) from e
        for name, mtype, count, nbytes in rows:
            out[name] = TopicMeta(name, mtype or "", int(count), int(nbytes))
        return out

    def topics(self) -> Dict[str, TopicMeta]:
        return dict(self._topics)

    def start_ns(self) -> int:
        row = self._conn.execute("SELECT MIN(timestamp) FROM messages").fetchone()
        return int(row[0] or 0)

    def end_ns(self) -> int:
        row = self._conn.execute("SELECT MAX(timestamp) FROM messages").fetchone()
        return int(row[0] or 0)

    def span(self, topic: str):
        row = self._conn.execute(
            "SELECT COUNT(m.id), MIN(m.timestamp), MAX(m.timestamp) "
            "FROM messages m JOIN topics t ON t.id = m.topic_id WHERE t.name = ?",
            (topic,),
        ).fetchone()
        return (int(row[0] or 0), int(row[1] or 0), int(row[2] or 0))

    def last_message(self, topic: str):
        row = self._conn.execute(
            "SELECT m.timestamp, m.data FROM messages m JOIN topics t ON t.id = m.topic_id "
            "WHERE t.name = ? ORDER BY m.timestamp DESC LIMIT 1",
            (topic,),
        ).fetchone()
        if row is None:
            return None
        return int(row[0]), bytes(row[1])

    def messages(self, topic: str) -> Iterator[Tuple[int, bytes]]:
        cur = self._conn.cursor()
        cur.execute(
            "SELECT m.timestamp, m.data FROM messages m JOIN topics t ON t.id = m.topic_id "
            "WHERE t.name = ? ORDER BY m.timestamp",
            (topic,),
        )
        for ts, data in cur:
            yield int(ts), bytes(data)
