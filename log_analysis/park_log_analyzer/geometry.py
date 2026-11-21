"""几何与栅格算法（开发文档 §7）。纯标准库。"""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

INF = 1e9


class CalibrationError(Exception):
    pass


# ----------------------------------------------------------------------
# 轨迹几何
# ----------------------------------------------------------------------
def arclength(points: Sequence) -> List[float]:
    s = [0.0]
    for a, b in zip(points, points[1:]):
        s.append(s[-1] + math.hypot(b.x - a.x, b.y - a.y))
    return s


def steps(points: Sequence) -> List[float]:
    return [math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(points, points[1:])]


def curvature_series(points: Sequence) -> List[float]:
    """三点外接圆曲率 κ = 2·sinθ / |AC| 的等价式（§7.4）。"""
    out = []
    for a, b, c in zip(points, points[1:], points[2:]):
        v1 = (b.x - a.x, b.y - a.y)
        v2 = (c.x - b.x, c.y - b.y)
        d1 = math.hypot(*v1)
        d2 = math.hypot(*v2)
        if d1 < 1e-6 or d2 < 1e-6:
            out.append(0.0)
            continue
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        out.append(cross / (d1 * d2 * (d1 + d2) / 2.0))
    return out


def curvature_at_step(points: Sequence, min_step: float = 0.3) -> List[float]:
    """稠密轨迹先抽稀到 >= min_step 再算曲率（§6.3 T-05 的做法）。"""
    thin = thin_by_distance(points, min_step)
    return curvature_series(thin)


def thin_by_distance(points: Sequence, min_step: float) -> List:
    if not points:
        return []
    out = [points[0]]
    for p in points[1:]:
        if math.hypot(p.x - out[-1].x, p.y - out[-1].y) >= min_step:
            out.append(p)
    if out[-1] is not points[-1]:
        out.append(points[-1])
    return out


def dist_point_to_polyline(px: float, py: float, pts: Sequence) -> float:
    best = INF
    for a, b in zip(pts, pts[1:]):
        dx, dy = b.x - a.x, b.y - a.y
        L2 = dx * dx + dy * dy
        if L2 < 1e-12:
            d = math.hypot(px - a.x, py - a.y)
        else:
            t = max(0.0, min(1.0, ((px - a.x) * dx + (py - a.y) * dy) / L2))
            d = math.hypot(px - (a.x + t * dx), py - (a.y + t * dy))
        best = min(best, d)
    return best


def align_by_arclength(traj_a: Sequence, traj_b: Sequence) -> List[float]:
    """A 的每个点 → 到 B 折线的最近距离（§7.1）。"""
    return [dist_point_to_polyline(p.x, p.y, traj_b) for p in traj_a]


def total_length(points: Sequence) -> float:
    return sum(steps(points))


# ----------------------------------------------------------------------
# 占据栅格
# ----------------------------------------------------------------------
def occ_index(occ, x: float, y: float):
    """世界坐标 → (col, row)，越界返回 None。"""
    ox, oy = occ.origin[0], occ.origin[1]
    res = occ.resolution
    col = int((x - ox) / res)
    row = int((y - oy) / res)
    if 0 <= col < occ.w and 0 <= row < occ.h:
        return col, row
    return None


def occ_value(occ, col: int, row: int) -> int:
    return occ.data[row * occ.w + col] - 256 if occ.data[row * occ.w + col] > 127 else occ.data[row * occ.w + col]


def occ_at(occ, x: float, y: float):
    idx = occ_index(occ, x, y)
    if idx is None:
        return None
    return occ_value(occ, idx[0], idx[1])


def occ_stats(occ) -> dict:
    n = len(occ.data)
    free = blocked = unknown = other = 0
    for b in occ.data:
        v = b - 256 if b > 127 else b
        if v == 0:
            free += 1
        elif v == 100:
            blocked += 1
        elif v == -1:
            unknown += 1
        else:
            other += 1
    return {"cells": n, "free": free, "occupied": blocked, "unknown": unknown, "other": other}


def edt_from_occupied(occ, occupied_threshold: int = 100) -> List[float]:
    """倒角距离变换（§7.2），返回每格的米制距离。"""
    w, h, res = occ.w, occ.h, occ.resolution
    d = [0.0 if occ_value(occ, i % w, i // w) >= occupied_threshold else INF
         for i in range(w * h)]
    s2 = 1.41421356
    for r in range(h):
        base = r * w
        for c in range(w):
            i = base + c
            if d[i] == 0:
                continue
            best = d[i]
            if r:
                best = min(best, d[i - w] + 1)
                if c:
                    best = min(best, d[i - w - 1] + s2)
                if c + 1 < w:
                    best = min(best, d[i - w + 1] + s2)
            if c:
                best = min(best, d[i - 1] + 1)
            d[i] = best
    for r in range(h - 1, -1, -1):
        base = r * w
        for c in range(w - 1, -1, -1):
            i = base + c
            if d[i] == 0:
                continue
            best = d[i]
            if r + 1 < h:
                best = min(best, d[i + w] + 1)
                if c + 1 < w:
                    best = min(best, d[i + w + 1] + s2)
                if c:
                    best = min(best, d[i + w - 1] + s2)
            if c + 1 < w:
                best = min(best, d[i + 1] + 1)
            d[i] = best
    return [(v * res if v < INF else INF) for v in d]


def footprint_samples(spec, step: float = 0.1) -> List[Tuple[float, float]]:
    x_min, x_max, y_half = spec.footprint()
    xs = _frange(x_min, x_max, step)
    ys = _frange(-y_half, y_half, step)
    return [(x, y) for x in xs for y in ys]


def _frange(a: float, b: float, step: float) -> List[float]:
    n = max(1, int(math.ceil((b - a) / step)))
    vals = [a + (b - a) * i / n for i in range(n + 1)]
    return vals


def min_clearance(traj, occ, spec, edt, step: float = 0.1):
    """轨迹 footprint 到最近占据格的距离（§7.3），返回 (最小距离, 点序号) 或 (-1, k)。"""
    samples = footprint_samples(spec, step)
    worst = (INF, -1)
    for k, p in enumerate(traj.points):
        cos_y, sin_y = math.cos(p.yaw), math.sin(p.yaw)
        for dx, dy in samples:
            wx = p.x + dx * cos_y - dy * sin_y
            wy = p.y + dx * sin_y + dy * cos_y
            idx = occ_index(occ, wx, wy)
            if idx is None:
                continue
            col, row = idx
            i = row * occ.w + col
            if occ_value(occ, col, row) >= 100:
                return -1.0, k
            dist = edt[i]
            if dist < worst[0]:
                worst = (dist, k)
    return worst


def bfs_connected(occ, start_xy, goal_xy, threshold: int = 100) -> bool:
    """4 邻域 BFS：起点到终点是否在自由格内连通（§6.2 P-09）。"""
    s = occ_index(occ, *start_xy)
    g = occ_index(occ, *goal_xy)
    if s is None or g is None:
        return False
    w, h, data = occ.w, occ.h, occ.data

    def free(i):
        v = data[i] - 256 if data[i] > 127 else data[i]
        return v >= 0 and v < threshold

    si = s[1] * w + s[0]
    gi = g[1] * w + g[0]
    if not free(si) or not free(gi):
        return False
    seen = bytearray(w * h)
    stack = [si]
    seen[si] = 1
    while stack:
        i = stack.pop()
        if i == gi:
            return True
        c, r = i % w, i // w
        for j, ok in ((i - 1, c > 0), (i + 1, c + 1 < w), (i - w, r > 0), (i + w, r + 1 < h)):
            if ok and not seen[j] and free(j):
                seen[j] = 1
                stack.append(j)
    return False


# ----------------------------------------------------------------------
# 栅格布局标定（§3.3 / §7.5）
# ----------------------------------------------------------------------
_TRANSFORMS = ("id", "flipud", "fliplr", "rot180", "T", "T+flipud", "T+fliplr", "T+rot180")


def reshape(flat: Sequence[float], n: int, layout: str) -> List[List[float]]:
    if layout == "row":
        return [list(flat[r * n:(r + 1) * n]) for r in range(n)]
    return [[flat[c * n + r] for c in range(n)] for r in range(n)]


def apply_transform(m: List[List[float]], t: str) -> List[List[float]]:
    n = len(m)
    if t == "id":
        return m
    if t == "flipud":
        return m[::-1]
    if t == "fliplr":
        return [row[::-1] for row in m]
    if t == "rot180":
        return [row[::-1] for row in m[::-1]]
    # 转置族
    tr = [[m[c][r] for c in range(n)] for r in range(n)]
    if t == "T":
        return tr
    if t == "T+flipud":
        return tr[::-1]
    if t == "T+fliplr":
        return [row[::-1] for row in tr]
    return [row[::-1] for row in tr[::-1]]


def calibrate_layout(combined, occ, min_agreement: float = 0.95):
    """穷举 2 种布局 × 8 种变换，返回 (layout, transform, agreement, mismatch)。"""
    n = occ.w
    occ_vals = [b - 256 if b > 127 else b for b in occ.data]
    total = 0
    valid = []
    for i, v in enumerate(occ_vals):
        valid.append(i)
        if v >= 0:
            total += 1
    best = None
    for layout in ("row", "col"):
        base = reshape(combined, n, layout)
        for t in _TRANSFORMS:
            m = apply_transform(base, t)
            flat = [v for row in m for v in row]
            match = 0
            for i in valid:
                ov = occ_vals[i]
                if ov < 0:
                    continue
                cv = flat[i]
                if not (cv == cv):        # NaN
                    continue
                c = int(round(max(0.0, min(1.0, cv)) * 100))
                if abs(c - ov) <= 1:
                    match += 1
            agree = match / total if total else 0.0
            if best is None or agree > best[2]:
                best = (layout, t, agree, total - match)
    if best is None or best[2] < min_agreement:
        raise CalibrationError(
            "栅格布局标定失败：最佳一致率 %.2f%%（阈值 %.0f%%），请检查两个话题是否同帧"
            % (best[2] * 100 if best else 0.0, min_agreement * 100)
        )
    return best


def grid_index_in_calibrated(cx: int, cy: int, n: int, layout: str, transform: str):
    """把标定后的 (col,row) 反查回原始 flat 下标。仅在需要时使用。"""
    idx = cy * n + cx
    return idx
