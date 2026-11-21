"""costmap 类规则 C-01 ~ C-12（开发文档 §6.1）。"""

from __future__ import annotations

from .. import geometry
from .base import FAIL, INFO, PASS, SKIP, WARN, CheckResult, check


def _layer_ratios(gf):
    out = {}
    n = gf.n_cells
    for name in gf.layers:
        d = gf.digest
        nz = d.nonzero.get(name, 0)
        out[name] = {
            "nonzero": nz,
            "nonzero_ratio": round(nz / n, 4) if n else 0.0,
            "nan": d.nan.get(name, 0),
            "dirty": d.dirty.get(name, 0),
            "max": round(d.vmax.get(name, 0.0), 4),
        }
    return out


def _ego_at(ctx, t):
    ctx.load("odom")
    o = ctx.nearest_odom(t)
    return (o.x, o.y) if o else None


@check("C-01", "输出话题存在且非空", requires=("grid_map", "occupancy_grid"))
def c01(ctx):
    gm = ctx.topic_meta("grid_map")
    og = ctx.topic_meta("occupancy_grid")
    detail = {"grid_map": {"count": gm.count, "bytes": gm.nbytes},
              "occupancy_grid": {"count": og.count, "bytes": og.nbytes}}
    bad = [k for k, m in (("grid_map", gm), ("occupancy_grid", og)) if m.count == 0]
    if bad:
        return CheckResult("C-01", "输出话题存在且非空", FAIL,
                           "costmap 输出缺失：%s 为 0 条" % "、".join(bad), detail)
    return CheckResult("C-01", "输出话题存在且非空", PASS,
                       "grid_map %d 帧 / occupancy_grid %d 帧" % (gm.count, og.count), detail,
                       ["%s: %d 条" % (gm.name, gm.count)])


@check("C-02", "发布频率", requires=("grid_map", "occupancy_grid", "trajectory"))
def c02(ctx):
    target = ctx.threshold("update_rate")
    tol = ctx.threshold("topic_rate_tolerance")
    rows = {}
    warn = []
    notes = []
    for key in ("grid_map", "occupancy_grid", "trajectory", "odometry", "final_trajectory"):
        meta = ctx.topic_meta(key)
        if meta is None or meta.count < 2:
            continue
        rate_bag = ctx.topic_rate(key)
        rate_stamp = ctx.stamp_rate(key)
        count, first, last = ctx.reader.span(ctx.topic(key))
        span = (last - first) * 1e-9
        exp = target if key in ("grid_map", "occupancy_grid", "trajectory") else None
        # 算法节拍由 header.stamp（仿真时钟）决定；bag 时间是录制时钟
        effective = rate_stamp if (exp and rate_stamp) else rate_bag
        dev = abs(effective - exp) / exp if exp and effective is not None else None
        clock_scale = (rate_bag / rate_stamp) if (rate_bag and rate_stamp) else None
        rows[key] = {
            "count": count,
            "bytes": meta.nbytes,
            "first_s": round(ctx.rel(first), 3),
            "last_s": round(ctx.rel(last), 3),
            "span_s": round(span, 3),
            "rate_bag_hz": round(rate_bag, 3) if rate_bag is not None else None,
            "rate_stamp_hz": round(rate_stamp, 3) if rate_stamp is not None else None,
            "rate_hz": round(effective, 3) if effective is not None else None,
            "expected_hz": exp,
            "clock_scale": round(clock_scale, 3) if clock_scale else None,
            "deviation": round(dev, 3) if dev is not None else None,
        }
        if dev is not None and dev > tol:
            warn.append("%s %.2f Hz（按 stamp 计，期望 %.1f）" % (key, effective, exp))
        elif clock_scale and abs(clock_scale - 1.0) > 0.1 and exp:
            notes.append(round(clock_scale, 2))
    detail = {"targets_hz": target, "tolerance": tol, "topics": rows}
    if warn:
        return CheckResult("C-02", "发布频率", WARN,
                           "频率偏离超过 %.0f%%：%s" % (tol * 100, "；".join(warn)), detail)
    msg = "各话题频率在容差内（按 header.stamp 跨度计算）"
    if notes:
        scale = sorted(set(notes))[0]
        msg += "；bag 时钟/仿真时钟 ≈ %.2f，bag 时间下频率偏低属正常" % scale
    return CheckResult("C-02", "发布频率", PASS, msg, detail)


@check("C-03", "栅格几何合法", requires=("grid_map", "occupancy_grid"))
def c03(ctx):
    ctx.load("grids")
    ctx.load("occs")
    gf = ctx.grid_last()
    occ = ctx.occ_last()
    detail, bad = {}, []
    if gf is not None:
        cell_len = len(gf.cells.get(gf.layers[-1], [])) if gf.cells else gf.n_cells
        detail["grid_map"] = {
            "resolution": gf.resolution, "lx": gf.lx, "ly": gf.ly,
            "cells": cell_len, "expected": gf.lx * gf.ly,
            "layers": list(gf.layers),
        }
        if gf.resolution <= 0 or gf.lx * gf.ly != cell_len:
            bad.append("grid_map 几何不一致")
    if occ is not None:
        detail["occupancy_grid"] = {
            "resolution": occ.resolution, "w": occ.w, "h": occ.h,
            "cells": len(occ.data), "expected": occ.w * occ.h,
        }
        if occ.resolution <= 0 or occ.w * occ.h != len(occ.data):
            bad.append("occupancy_grid 几何不一致")
    if gf and occ and (gf.lx != occ.w or gf.ly != occ.h):
        bad.append("两个栅格尺寸不一致 %dx%d vs %dx%d" % (gf.lx, gf.ly, occ.w, occ.h))
    if bad:
        return CheckResult("C-03", "栅格几何合法", FAIL, "；".join(bad), detail)
    return CheckResult("C-03", "栅格几何合法", PASS,
                       "%dx%d @ %.2f m，格数与数据长度一致" % (occ.w, occ.h, occ.resolution), detail)


@check("C-04", "occupancy 与 grid_map.combined 一致", requires=("grid_map", "occupancy_grid"))
def c04(ctx):
    cal = ctx.calibrate()
    if cal is None:
        return CheckResult("C-04", "occupancy 与 grid_map.combined 一致", FAIL,
                           ctx.calibration_error or "标定失败", {})
    layout, transform, agree, mismatch = cal
    gf = ctx.grid_last()
    occ = ctx.occ_last()
    detail = {
        "layout": layout, "transform": transform,
        "agreement": round(agree, 4), "mismatch_cells": mismatch, "cells": occ.w * occ.h,
    }
    ev = ["tables/05_occupancy_grid_last_frame_cells.csv:1-%d" % (occ.w * occ.h)]
    if agree < ctx.threshold("grid_calibration_min"):
        return CheckResult("C-04", "occupancy 与 grid_map.combined 一致", FAIL,
                           "逐格一致率 %.2f%% 低于阈值" % (agree * 100), detail, ev)
    return CheckResult("C-04", "occupancy 与 grid_map.combined 一致", PASS,
                       "逐格一致率 %.2f%%（%s + %s，%d 格不一致）"
                       % (agree * 100, layout, transform, mismatch), detail, ev)


@check("C-05", "层活跃度", requires=("grid_map",))
def c05(ctx):
    ctx.load("grids")
    gf = ctx.grid_last()
    ratios = _layer_ratios(gf) if gf else {}
    detail = {"frame_t": round(gf.t_bag, 3) if gf else None, "layers": ratios,
              "filter_rule": "isfinite(v) and |v| <= 100"}
    empty = [k for k, v in ratios.items() if v["nonzero"] == 0]
    msg = "；".join("%s 非零 %d 格" % (k, v["nonzero"]) for k, v in ratios.items())
    if len(empty) >= len(ratios) - 1 and "combined" not in empty:
        msg += "（除 combined 外均为空）"
    return CheckResult("C-05", "层活跃度", INFO, msg, detail)


@check("C-06", "帧间变化率", requires=("grid_map",))
def c06(ctx):
    ctx.load("grids")
    if not ctx.grid_changes:
        return CheckResult("C-06", "帧间变化率", INFO, "只有 1 帧，无法比较", {})
    n = ctx.grids[0].n_cells * len(ctx.grids[0].layers)
    ratios = [d / n for _, d, _ in ctx.grid_changes]
    identical = sum(1 for _, _, same in ctx.grid_changes if same)
    detail = {
        "frames": len(ctx.grids),
        "pairs": len(ratios),
        "identical_pairs": identical,
        "max_change_ratio": round(max(ratios), 4),
        "mean_change_ratio": round(sum(ratios) / len(ratios), 4),
    }
    if max(ratios) > ctx.threshold("frame_change_ratio_warn"):
        return CheckResult("C-06", "帧间变化率", WARN,
                           "最大帧间变化 %.1f%% 超过 %.0f%%"
                           % (max(ratios) * 100, ctx.threshold("frame_change_ratio_warn") * 100), detail)
    return CheckResult("C-06", "帧间变化率", INFO,
                       "%d/%d 对相邻帧完全相同，最大变化率 %.2f%%"
                       % (identical, len(ratios), max(ratios) * 100), detail)


@check("C-07", "未知格比例", requires=("occupancy_grid",))
def c07(ctx):
    ctx.load("occs")
    occ = ctx.occ_last()
    st = geometry.occ_stats(occ)
    ratio = st["unknown"] / st["cells"] if st["cells"] else 0.0
    detail = dict(st)
    detail["unknown_ratio"] = round(ratio, 4)
    detail["frame_t"] = round(occ.t_bag, 3)
    if ratio > 0.10:
        return CheckResult("C-07", "未知格比例", WARN,
                           "未知格 %.1f%%（>10%%），栅格覆盖不足" % (ratio * 100), detail)
    return CheckResult("C-07", "未知格比例", INFO,
                       "未知格 %.2f%%，自由 %.1f%% / 占据 %.1f%%"
                       % (ratio * 100, st["free"] / st["cells"] * 100,
                          st["occupied"] / st["cells"] * 100), detail)


@check("C-08", "栅格中心跟随自车", requires=("grid_map", "odometry"))
def c08(ctx):
    ctx.load("grids")
    gf = ctx.grid_last()
    ego = _ego_at(ctx, gf.t_bag)
    if ego is None:
        return CheckResult("C-08", "栅格中心跟随自车", SKIP, "无 odom", {})
    dx = gf.center[0] - ego[0]
    dy = gf.center[1] - ego[1]
    dist = (dx * dx + dy * dy) ** 0.5
    detail = {"center": [round(gf.center[0], 3), round(gf.center[1], 3)],
              "ego": [round(ego[0], 3), round(ego[1], 3)],
              "dx": round(dx, 3), "dy": round(dy, 3), "distance_m": round(dist, 3),
              "resolution": gf.resolution}
    if dist > gf.resolution:
        return CheckResult("C-08", "栅格中心跟随自车", WARN,
                           "栅格中心与自车相距 %.2f m（> %.2f）" % (dist, gf.resolution), detail)
    return CheckResult("C-08", "栅格中心跟随自车", PASS,
                       "偏差 %.3f m（小于 %.2f m）" % (dist, gf.resolution), detail)


@check("C-09", "自车所在格可通行", requires=("grid_map", "occupancy_grid", "odometry"))
def c09(ctx):
    ctx.load("grids")
    ctx.load("occs")
    gf = ctx.grid_last()
    occ = ctx.occ_last()
    ego = _ego_at(ctx, gf.t_bag)
    if ego is None:
        return CheckResult("C-09", "自车所在格可通行", SKIP, "无 odom", {})
    cost = geometry.occ_at(occ, *ego)
    rows = ctx.calibrated_rows(gf)
    idx = geometry.occ_index(occ, *ego)
    gcost = None
    if idx is not None:
        gcost = rows[idx[1]][idx[0]] * 100
    detail = {"ego": [round(ego[0], 3), round(ego[1], 3)],
              "occupancy_cost": cost,
              "grid_combined_cost": round(gcost, 1) if gcost is not None else None}
    if cost is None or cost >= 100:
        return CheckResult("C-09", "自车所在格可通行", FAIL,
                           "自车落在占据格/栅格外（cost=%s）" % cost, detail)
    return CheckResult("C-09", "自车所在格可通行", PASS,
                       "自车格 cost=%d，combined=%.0f" % (cost, gcost if gcost is not None else -1), detail)


@check("C-10", "车位区填充比例", requires=("grid_map",))
def c10(ctx):
    ctx.load("grids")
    gf = ctx.grid_last()
    rows = ctx.calibrated_rows(gf)
    n = gf.n_cells
    free = sum(1 for row in rows for v in row if abs(v) <= 100 and v < 0.5)
    detail = {"free_cells": free, "cells": n, "free_ratio": round(free / n, 4),
              "occupied_ratio": round(1 - free / n, 4)}
    return CheckResult("C-10", "车位区填充比例", INFO,
                       "自由格 %.2f%% / 占据 %.2f%%" % (free / n * 100, (1 - free / n) * 100), detail)


@check("C-11", "脏值格检测", requires=("grid_map",))
def c11(ctx):
    ctx.load("grids")
    bad = {}
    for gf in ctx.grids:
        d = gf.digest
        for name in gf.layers:
            k = d.dirty.get(name, 0) + d.nan.get(name, 0)
            if k:
                bad[name] = max(bad.get(name, 0), k)
    detail = {"dirty_by_layer": bad, "frames_checked": len(ctx.grids),
              "rule": "not isfinite(v) or abs(v) > 100"}
    if bad:
        return CheckResult("C-11", "脏值格检测", WARN,
                           "存在未初始化脏值：%s" % "、".join("%s %d 格" % kv for kv in bad.items()),
                           detail)
    return CheckResult("C-11", "脏值格检测", PASS, "未发现 NaN / abs(v)>100 的脏值格", detail)


@check("C-12", "层间数据来源可解释性", requires=("grid_map", "objects"))
def c12(ctx):
    ctx.load("grids")
    ctx.load("objects")
    gf = ctx.grid_last()
    obj_layer = gf.digest.nonzero.get("objects", 0)
    nonzero_msgs = sum(1 for o in ctx.objects if o["objects_size"] > 0)
    detail = {
        "objects_layer_nonzero": obj_layer,
        "objects_msgs": len(ctx.objects),
        "objects_msgs_nonzero": nonzero_msgs,
        "frame_t": round(gf.t_bag, 3),
    }
    if obj_layer > 0 and nonzero_msgs == 0:
        return CheckResult("C-12", "层间数据来源可解释性", WARN,
                           "objects 层 %d 格非零，但 %d 条 objects 消息全为空 → 来源无法由 bag 解释"
                           % (obj_layer, len(ctx.objects)), detail)
    if nonzero_msgs > 0:
        return CheckResult("C-12", "层间数据来源可解释性", PASS,
                           "objects 消息有 %d 条非空，层数据可解释" % nonzero_msgs, detail)
    return CheckResult("C-12", "层间数据来源可解释性", INFO,
                       "objects 层与 objects 消息均为空，二者一致", detail)
