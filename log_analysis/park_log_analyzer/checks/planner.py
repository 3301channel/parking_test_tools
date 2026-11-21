"""planner 类规则 P-01 ~ P-12（开发文档 §6.2）。"""

from __future__ import annotations

import math

from .. import geometry
from .base import FAIL, INFO, PASS, SKIP, WARN, CheckResult, check


def _stats(vals):
    if not vals:
        return {}
    s = sorted(vals)
    n = len(s)

    def q(p):
        return s[min(n - 1, max(0, int(round(p * (n - 1)))))]

    return {"n": n, "min": round(s[0], 2), "p50": round(q(0.5), 2),
            "p95": round(q(0.95), 2), "max": round(s[-1], 2)}


@check("P-01", "三路输入齐备", requires=("route", "occupancy_grid", "odometry"))
def p01(ctx):
    detail = {}
    missing = []
    for key in ("route", "occupancy_grid", "odometry"):
        meta = ctx.topic_meta(key)
        count, first, _ = ctx.reader.span(ctx.topic(key))
        detail[key] = {"count": meta.count, "first_s": round(ctx.rel(first), 3) if count else None}
        if meta.count == 0:
            missing.append(key)
    if missing:
        return CheckResult("P-01", "三路输入齐备", FAIL, "缺少输入：%s" % "、".join(missing), detail)
    return CheckResult("P-01", "三路输入齐备", PASS,
                       "route %.3f s / occupancy %.3f s / odom %.3f s 均已到达"
                       % (detail["route"]["first_s"], detail["occupancy_grid"]["first_s"],
                          detail["odometry"]["first_s"]), detail)


@check("P-02", "激活信号", requires=("scenario",))
def p02(ctx):
    ctx.load("scenario")
    windows = ctx.parking_windows()
    first = ctx.first_parking_time()
    names = sorted({n for s in ctx.scenario for n in s["scenarios"]})
    detail = {"scenarios_seen": names, "parking_windows": [[round(a, 3), round(b, 3)] for a, b in windows],
              "first_parking_s": round(first, 3) if first is not None else None,
              "messages": len(ctx.scenario)}
    if first is None:
        return CheckResult("P-02", "激活信号", FAIL,
                           "scenario 中没有 Parking（出现过的场景：%s）" % ("、".join(names) or "无"), detail)
    return CheckResult("P-02", "激活信号", PASS,
                       "Parking 激活，首条 %.3f s，共 %d 段" % (first, len(windows)), detail)


@check("P-03", "启动时延", requires=("scenario", "trajectory"))
def p03(ctx):
    ctx.load("scenario")
    ctx.load("traj")
    first = ctx.first_parking_time()
    if first is None or not ctx.traj:
        return CheckResult("P-03", "启动时延", SKIP, "无 Parking 段或无轨迹", {})
    t0 = ctx.traj[0].t_bag
    delay = t0 - first
    detail = {"first_parking_s": round(first, 3), "first_traj_s": round(t0, 3),
              "delay_s": round(delay, 3)}
    if delay > 3.0:
        return CheckResult("P-03", "启动时延", WARN, "激活到首帧轨迹 %.2f s，偏慢" % delay, detail)
    if delay < -0.5:
        return CheckResult("P-03", "启动时延", INFO,
                           "首帧轨迹早于 Parking 激活 %.2f s（轨迹话题可能被 latch 复用）" % -delay, detail)
    return CheckResult("P-03", "启动时延", PASS, "启动时延 %.3f s" % delay, detail)


@check("P-04", "规划成功率", requires=("trajectory", "scenario"))
def p04(ctx):
    ctx.load("traj")
    windows = ctx.parking_windows()
    # bag 时间与算法节拍可能不同（仿真时钟缩放），用轨迹话题自身的 bag 频率折算
    rate = ctx.topic_rate("trajectory") or ctx.threshold("update_rate")
    active = sum(b - a for a, b in windows)
    expected = max(0.0, active * rate)
    n = len(ctx.traj)
    ratio = n / expected if expected > 0 else None
    detail = {"traj_frames": n, "parking_active_s": round(active, 3),
              "expected_cycles": round(expected, 1),
              "success_ratio": round(ratio, 3) if ratio is not None else None,
              "parking_windows": [[round(a, 3), round(b, 3)] for a, b in windows]}
    if expected > 0 and n == 0:
        return CheckResult("P-04", "规划成功率", FAIL, "Parking 激活期间没有任何轨迹输出", detail)
    if ratio is not None and ratio < 0.5:
        return CheckResult("P-04", "规划成功率", WARN,
                           "轨迹帧数 %d 仅为期望周期数的 %.0f%%" % (n, ratio * 100), detail)
    return CheckResult("P-04", "规划成功率", PASS,
                       "%d 帧轨迹 / 期望 %.0f 周期%s"
                       % (n, expected, "" if ratio is None else "（%.0f%%）" % (ratio * 100)), detail)


@check("P-05", "规划耗时", requires=("processing_time",))
def p05(ctx):
    ctx.load("proc")
    vals = [p.value for p in ctx.proc_ms]
    st = _stats(vals)
    st["threshold_p95"] = ctx.threshold("max_planning_ms_p95")
    if not vals:
        return CheckResult("P-05", "规划耗时", WARN, "没有 processing_time_ms 数据", st)
    if st["p95"] > st["threshold_p95"]:
        return CheckResult("P-05", "规划耗时", WARN,
                           "p95 %.2f ms 超过阈值 %.0f ms" % (st["p95"], st["threshold_p95"]), st)
    return CheckResult("P-05", "规划耗时", PASS,
                       "p50 %.2f ms / p95 %.2f ms / max %.2f ms"
                       % (st["p50"], st["p95"], st["max"]), st)


@check("P-06", "首帧冷启动", requires=("processing_time",))
def p06(ctx):
    ctx.load("proc")
    vals = [p.value for p in ctx.proc_ms]
    if not vals:
        return CheckResult("P-06", "首帧冷启动", SKIP, "无耗时数据", {})
    st = _stats(vals)
    detail = {"first_ms": round(vals[0], 2), "p50_ms": st["p50"],
              "ratio": round(vals[0] / st["p50"], 2) if st["p50"] else None}
    if st["p50"] and vals[0] > 3 * st["p50"]:
        return CheckResult("P-06", "首帧冷启动", INFO,
                           "首帧 %.2f ms 是 p50 的 %.1f 倍（首次构建碰撞表）"
                           % (vals[0], vals[0] / st["p50"]), detail)
    return CheckResult("P-06", "首帧冷启动", INFO,
                       "首帧 %.2f ms，p50 %.2f ms" % (vals[0], st["p50"]), detail)


@check("P-07", "goal 落在自由格", requires=("occupancy_grid", "goal"))
def p07(ctx):
    ctx.load("occs")
    ctx.load("goal")
    g = ctx.goal
    occ = ctx.occ_last()
    if g is None:
        return CheckResult("P-07", "goal 落在自由格", SKIP, "没有 goal 消息", {})
    cost = geometry.occ_at(occ, g["x"], g["y"])
    detail = {"goal": [round(g["x"], 3), round(g["y"], 3)], "cost": cost}
    if cost is None:
        return CheckResult("P-07", "goal 落在自由格", FAIL, "goal 在栅格范围外", detail)
    if cost >= 100:
        return CheckResult("P-07", "goal 落在自由格", FAIL, "goal 落在占据格（cost=%d）" % cost, detail)
    return CheckResult("P-07", "goal 落在自由格", PASS, "goal 格 cost=%d" % cost, detail)


@check("P-08", "goal 在栅格内", requires=("occupancy_grid", "goal"))
def p08(ctx):
    ctx.load("occs")
    ctx.load("goal")
    g = ctx.goal
    occ = ctx.occ_last()
    if g is None:
        return CheckResult("P-08", "goal 在栅格内", SKIP, "没有 goal 消息", {})
    x0, y0 = occ.origin[0], occ.origin[1]
    x1, y1 = x0 + occ.w * occ.resolution, y0 + occ.h * occ.resolution
    inside = x0 <= g["x"] <= x1 and y0 <= g["y"] <= y1
    detail = {"goal": [round(g["x"], 3), round(g["y"], 3)],
              "bounds": [round(x0, 3), round(y0, 3), round(x1, 3), round(y1, 3)]}
    if not inside:
        return CheckResult("P-08", "goal 在栅格内", FAIL, "goal 超出栅格边界", detail)
    return CheckResult("P-08", "goal 在栅格内", PASS, "goal 落在栅格范围内", detail)


@check("P-09", "起终点连通", requires=("occupancy_grid", "goal", "odometry"))
def p09(ctx):
    ctx.load("occs")
    ctx.load("goal")
    ctx.load("odom")
    g = ctx.goal
    occ = ctx.occ_last()
    if g is None or not ctx.odom:
        return CheckResult("P-09", "起终点连通", SKIP, "缺少 goal 或 odom", {})
    o = ctx.odom_before(g["stamp"]) or ctx.odom[-1]
    ok = geometry.bfs_connected(occ, (o.x, o.y), (g["x"], g["y"]))
    detail = {"start": [round(o.x, 3), round(o.y, 3)], "goal": [round(g["x"], 3), round(g["y"], 3)],
              "connected": ok}
    if not ok:
        return CheckResult("P-09", "起终点连通", FAIL, "占用栅格上起终点不连通", detail)
    return CheckResult("P-09", "起终点连通", PASS, "4 邻域 BFS 连通", detail)


@check("P-10", "is_completed 语义", requires=("is_completed", "scenario"))
def p10(ctx):
    ctx.load("completed")
    ctx.load("scenario")
    first = ctx.first_parking_time()
    n = len(ctx.completed)
    true_pts = [p for p in ctx.completed if p.value > 0.5]
    false_pts = [p for p in ctx.completed if p.value <= 0.5]
    before = sum(1 for p in false_pts if first is not None and p.t_bag < first)
    rate = ctx.topic_rate("is_completed") or ctx.threshold("update_rate")
    first_msg_t = ctx.completed[0].t_bag if ctx.completed else None
    expected = max(0.0, (first - first_msg_t) * rate) if first is not None and first_msg_t else None
    detail = {
        "messages": n, "false": len(false_pts), "true": len(true_pts),
        "false_before_activation": before,
        "expected_false_before_activation": round(expected, 1) if expected is not None else None,
        "true_times": [round(p.t_bag, 3) for p in true_pts][:8],
        "window": [round(ctx.completed[0].t_bag, 3), round(ctx.completed[-1].t_bag, 3)] if n else None,
    }
    if n and len(true_pts) == 0 and before == len(false_pts):
        return CheckResult("P-10", "is_completed 语义", INFO,
                           "%d 条全为 false，且 %d 条落在激活前空转期（%s），非故障"
                           % (n, before, "窗口 %.2f~%.2f s" % tuple(detail["window"])), detail)
    if true_pts:
        return CheckResult("P-10", "is_completed 语义", INFO,
                           "出现 %d 条 is_completed=true，首个 %.3f s"
                           % (len(true_pts), true_pts[0].t_bag), detail)
    return CheckResult("P-10", "is_completed 语义", INFO,
                       "%d 条 false / %d 条 true，其中 %d 条在激活前" % (len(false_pts), len(true_pts), before),
                       detail)


@check("P-11", "重规划次数", requires=("trajectory",))
def p11(ctx):
    ctx.load("traj")
    if not ctx.traj:
        return CheckResult("P-11", "重规划次数", SKIP, "无轨迹", {})
    counts = [len(t.points) for t in ctx.traj]
    changes = sum(1 for a, b in zip(counts, counts[1:]) if abs(a - b) > 0)
    gaps = 0
    rate = ctx.threshold("update_rate")
    for a, b in zip(ctx.traj, ctx.traj[1:]):
        if b.t_bag - a.t_bag > 2.5 / rate:
            gaps += 1
    detail = {"frames": len(counts), "point_count_changes": changes,
              "time_gaps": gaps, "point_counts": sorted(set(counts))}
    return CheckResult("P-11", "重规划次数", INFO,
                       "点数变化 %d 次，时间断档 %d 次（推断重规划/重启）" % (changes, gaps), detail)


@check("P-12", "停车轨迹注入", requires=("trajectory",))
def p12(ctx):
    ctx.load("traj")
    hits = [t.t_bag for t in ctx.traj if len(t.points) == 1 and abs(t.points[0].v_long) < 1e-6]
    detail = {"frames": len(ctx.traj), "single_point_stop_frames": len(hits),
              "times": [round(t, 3) for t in hits][:8]}
    if hits:
        return CheckResult("P-12", "停车轨迹注入", INFO,
                           "存在 %d 帧单点 v=0 停车轨迹（reset 注入）" % len(hits), detail)
    return CheckResult("P-12", "停车轨迹注入", INFO, "无单点停车轨迹帧", detail)
