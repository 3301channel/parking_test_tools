"""纯字符串拼 SVG（开发文档 §9.3，不依赖 matplotlib）。"""

from __future__ import annotations

import math
import os

from .. import geometry

_FONT = "font-family='Noto Sans CJK SC, Source Han Sans SC, sans-serif'"


def _speed_color(v: float, vmax: float) -> str:
    if vmax <= 0:
        return "#388e3c"
    t = max(0.0, min(1.0, abs(v) / vmax))
    if t < 0.5:
        a = t / 0.5
        c1, c2 = (0x7B, 0x1F, 0xA2), (0x00, 0x89, 0x7B)
    else:
        a = (t - 0.5) / 0.5
        c1, c2 = (0x00, 0x89, 0x7B), (0x38, 0x8E, 0x3C)
    rgb = tuple(int(c1[i] + (c2[i] - c1[i]) * a) for i in range(3))
    return "#%02x%02x%02x" % rgb


def _grid_plot(ctx, path, pad=5.0):
    occ = ctx.occ_last()
    ctx.load("traj")
    if occ is None:
        return None
    res = occ.resolution
    ox, oy = occ.origin[0], occ.origin[1]
    x1, y1 = ox + occ.w * res, oy + occ.h * res

    # ROI：有轨迹时围绕轨迹，否则整个栅格
    if ctx.traj:
        xs = [p.x for t in ctx.traj for p in t.points]
        ys = [p.y for t in ctx.traj for p in t.points]
        x0 = max(ox, min(xs) - pad)
        y0 = max(oy, min(ys) - pad)
        x1 = min(x1, max(xs) + pad)
        y1 = min(y1, max(ys) + pad)
    else:
        x0, y0 = ox, oy
    W = 900
    scale = W / max(1e-6, (x1 - x0))
    H = max(120, int((y1 - y0) * scale))

    def px(x):
        return (x - x0) * scale

    def py(y):
        return H - (y - y0) * scale

    out = ["<svg xmlns='http://www.w3.org/2000/svg' width='%d' height='%d' viewBox='0 0 %d %d'>"
           % (W, H, W, H)]
    out.append("<rect width='%d' height='%d' fill='#ffffff'/>" % (W, H))
    # 占据格（2x2 合并降采样）
    cell = res * scale
    rects = []
    step = 2 if cell < 3 else 1
    col0 = max(0, int((x0 - ox) / res))
    col1 = min(occ.w, int((x1 - ox) / res) + 1)
    row0 = max(0, int((y0 - oy) / res))
    row1 = min(occ.h, int((y1 - oy) / res) + 1)
    for row in range(row0, row1, step):
        for col in range(col0, col1, step):
            v = geometry.occ_value(occ, col, row)
            if v >= 50:
                x = ox + col * res
                y = oy + row * res
                rx, ry = px(x), py(y + step * res)
                if rx > W or ry > H or rx + cell * step < 0 or ry + cell * step < 0:
                    continue
                rects.append("<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='#9e9e9e'/>"
                             % (rx, ry, cell * step + 0.4, cell * step + 0.4))
    out.extend(rects)
    # 轨迹
    if ctx.traj:
        t = ctx.traj[-1]
        vmax = max((abs(p.v_long) for p in t.points), default=1.0)
        for a, b in zip(t.points, t.points[1:]):
            out.append("<line x1='%.2f' y1='%.2f' x2='%.2f' y2='%.2f' stroke='%s' stroke-width='3'/>"
                       % (px(a.x), py(a.y), px(b.x), py(b.y), _speed_color(b.v_long, vmax)))
        p0 = t.points[0]
        out.append("<circle cx='%.2f' cy='%.2f' r='6' fill='#1976d2'/>" % (px(p0.x), py(p0.y)))
        out.append("<text x='%.2f' y='%.2f' font-size='12' fill='#1976d2' %s>起点</text>"
                   % (px(p0.x) + 8, py(p0.y), _FONT))
        p1 = t.points[-1]
        out.append("<circle cx='%.2f' cy='%.2f' r='5' fill='#d32f2f'/>" % (px(p1.x), py(p1.y)))
        out.append("<text x='%.2f' y='%.2f' font-size='12' fill='#d32f2f' %s>轨迹终点</text>"
                   % (px(p1.x) + 8, py(p1.y), _FONT))
        # 最小余量点
        edt = ctx.edt()
        if edt is not None:
            dist, k = geometry.min_clearance(t, occ, ctx.vehicle, edt)
            if 0 <= k < len(t.points):
                p = t.points[k]
                out.append("<circle cx='%.2f' cy='%.2f' r='7' fill='none' stroke='#f57c00' stroke-width='2'/>"
                           % (px(p.x), py(p.y)))
                out.append("<text x='%.2f' y='%.2f' font-size='12' fill='#f57c00' %s>最小余量 %.2f m</text>"
                           % (px(p.x) + 9, py(p.y) - 6, _FONT, max(0.0, dist)))
    # goal
    g = None
    if ctx.route is not None:
        g = (ctx.route.goal[0], ctx.route.goal[1])
    elif ctx.goal is not None:
        g = (ctx.goal["x"], ctx.goal["y"])
    if g:
        out.append("<circle cx='%.2f' cy='%.2f' r='6' fill='none' stroke='#7b1fa2' stroke-width='2'/>"
                   % (px(g[0]), py(g[1])))
        out.append("<text x='%.2f' y='%.2f' font-size='12' fill='#7b1fa2' %s>goal</text>"
                   % (px(g[0]) + 8, py(g[1]) - 6, _FONT))
    out.append("<text x='10' y='18' font-size='12' fill='#555' %s>occupancy_grid 末帧 + parking/trajectory 末帧"
               "（灰=占据，颜色=速度）</text>" % _FONT)
    out.append("</svg>")
    _write(path, out)
    return path


def _speed_plot(ctx, path):
    ctx.load("traj")
    ctx.load("final_traj")
    if not ctx.traj:
        return None
    src = ctx.traj[-1]
    sa = geometry.arclength(src.points)
    series = [("源 trajectory", list(zip(sa, [p.v_long for p in src.points])), "#1976d2")]
    if ctx.final_traj:
        fin = min(ctx.final_traj, key=lambda f: abs(f.stamp - src.stamp))
        fa = geometry.arclength(fin.points)
        series.append(("最终 /planning/trajectory", list(zip(fa, [p.v_long for p in fin.points])), "#f57c00"))
    W, H, m = 760, 320, 50
    xs = [x for _, pts, _ in series for x, _ in pts] or [0, 1]
    ys = [y for _, pts, _ in series for _, y in pts] or [0, 1]
    xmax = max(xs) or 1
    ymin, ymax = min(min(ys), 0.0), max(max(ys), 0.1)

    def px(x):
        return m + (x / xmax) * (W - 2 * m)

    def py(y):
        return H - m - ((y - ymin) / (ymax - ymin)) * (H - 2 * m)

    out = ["<svg xmlns='http://www.w3.org/2000/svg' width='%d' height='%d'>" % (W, H)]
    out.append("<rect width='100%' height='100%' fill='#fff'/>")
    out.append("<line x1='%d' y1='%d' x2='%d' y2='%d' stroke='#999'/>" % (m, py(0), W - m, py(0)))
    out.append("<line x1='%d' y1='%d' x2='%d' y2='%d' stroke='#999'/>" % (m, m, m, H - m))
    for name, pts, color in series:
        d = " ".join("%s%.2f,%.2f" % ("M" if i == 0 else "L", px(x), py(y)) for i, (x, y) in enumerate(pts))
        out.append("<path d='%s' fill='none' stroke='%s' stroke-width='2.5'/>" % (d, color))
    for i, (name, _, color) in enumerate(series):
        out.append("<rect x='%d' y='%d' width='12' height='12' fill='%s'/>" % (W - 260, 20 + 18 * i, color))
        out.append("<text x='%d' y='%d' font-size='12' fill='#333' %s>%s</text>"
                   % (W - 242, 30 + 18 * i, _FONT, name))
    out.append("<text x='%d' y='%d' font-size='12' fill='#555' %s>s [m]</text>" % (W - 70, H - 20, _FONT))
    out.append("<text x='10' y='24' font-size='12' fill='#555' %s>v [m/s]</text>" % _FONT)
    out.append("<text x='%d' y='%d' font-size='12' fill='#555' %s>%.2f</text>" % (m - 30, py(ymax) + 4, _FONT, ymax))
    out.append("<text x='%d' y='%d' font-size='12' fill='#555' %s>%.2f</text>" % (m - 30, py(ymin) + 4, _FONT, ymin))
    out.append("<text x='%d' y='%d' font-size='12' fill='#555' %s>%.2f</text>" % (W - m - 12, H - m + 16, _FONT, xmax))
    out.append("</svg>")
    _write(path, out)
    return path


def _timeline(ctx, path):
    rows = []
    keys = ["scenario", "occupancy_grid", "grid_map", "trajectory", "final_trajectory",
            "is_completed", "odometry"]
    dur = max(1e-6, ctx.bag_duration)
    for key in keys:
        meta = ctx.topic_meta(key)
        if meta is None or meta.count == 0:
            continue
        count, first, last = ctx.reader.span(ctx.topic(key))
        rows.append((key, ctx.rel(first), ctx.rel(last), count))
    W, H, left = 900, 60 + 26 * max(1, len(rows)), 150

    def px(t):
        return left + (t / dur) * (W - left - 20)

    out = ["<svg xmlns='http://www.w3.org/2000/svg' width='%d' height='%d'>" % (W, H)]
    out.append("<rect width='100%' height='100%' fill='#fff'/>")
    for i, (name, a, b, count) in enumerate(rows):
        y = 30 + i * 26
        out.append("<text x='10' y='%d' font-size='12' fill='#333' %s>%s</text>" % (y + 12, _FONT, name))
        out.append("<rect x='%.1f' y='%d' width='%.1f' height='14' fill='#90caf9'/>"
                   % (px(a), y, max(1.5, px(b) - px(a))))
        out.append("<text x='%.1f' y='%d' font-size='10' fill='#555' %s>%d</text>"
                   % (px(b) + 4, y + 11, _FONT, count))
    # Parking 窗口
    for a, b in ctx.parking_windows():
        out.append("<rect x='%.1f' y='%d' width='%.1f' height='%d' fill='#a5d6a7' opacity='0.35'/>"
                   % (px(a), 20, max(1.5, px(b) - px(a)), H - 40))
    out.append("<text x='10' y='14' font-size='12' fill='#555' %s>绿底=Parking 激活窗口；横轴 = bag 相对时间 %.1f s</text>"
               % (_FONT, dur))
    out.append("</svg>")
    _write(path, out)
    return path


def _write(path, lines):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def write_plots(ctx, outdir: str):
    pdir = os.path.join(outdir, "plots")
    os.makedirs(pdir, exist_ok=True)
    out = {}
    try:
        out["grid_traj"] = _grid_plot(ctx, os.path.join(pdir, "grid_traj.svg"))
    except Exception as e:  # 图失败不影响报告
        ctx.warnings.append("grid_traj.svg 生成失败：%s" % e)
    try:
        out["speed_profile"] = _speed_plot(ctx, os.path.join(pdir, "speed_profile.svg"))
    except Exception as e:
        ctx.warnings.append("speed_profile.svg 生成失败：%s" % e)
    try:
        out["timeline"] = _timeline(ctx, os.path.join(pdir, "timeline.svg"))
    except Exception as e:
        ctx.warnings.append("timeline.svg 生成失败：%s" % e)
    return out
