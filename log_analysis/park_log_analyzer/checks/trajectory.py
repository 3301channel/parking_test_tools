"""trajectory 类规则 T-01 ~ T-13（开发文档 §6.3）。"""

from __future__ import annotations

import math

from .. import geometry
from .base import FAIL, INFO, PASS, SKIP, WARN, CheckResult, check


def _last_traj(ctx):
    ctx.load("traj")
    return ctx.traj[-1] if ctx.traj else None


def _frame_key(t):
    return tuple((round(p.x, 6), round(p.y, 6), round(p.v_long, 6)) for p in t.points)


@check("T-01", "轨迹非空且点数合理", requires=("trajectory",))
def t01(ctx):
    ctx.load("traj")
    if not ctx.traj:
        return CheckResult("T-01", "轨迹非空且点数合理", FAIL, "没有任何轨迹", {})
    counts = [len(t.points) for t in ctx.traj]
    detail = {"frames": len(counts), "min": min(counts), "max": max(counts),
              "median": sorted(counts)[len(counts) // 2], "distinct": sorted(set(counts))}
    if min(counts) < 2:
        n_bad = sum(1 for c in counts if c < 2)
        return CheckResult("T-01", "轨迹非空且点数合理", WARN,
                           "%d 帧点数 < 2（可能是停车轨迹注入）" % n_bad, detail)
    return CheckResult("T-01", "轨迹非空且点数合理", PASS,
                       "%d 帧 × %d~%d 点" % (len(counts), min(counts), max(counts)), detail)


@check("T-02", "弧长与步长", requires=("trajectory",))
def t02(ctx):
    ctx.load("traj")
    t = _last_traj(ctx)
    if t is None or len(t.points) < 2:
        return CheckResult("T-02", "弧长与步长", SKIP, "轨迹点不足", {})
    lens = [geometry.total_length(f.points) for f in ctx.traj if len(f.points) > 1]
    st = geometry.steps(t.points)
    detail = {
        "arclength_last": round(geometry.total_length(t.points), 4),
        "arclength_min": round(min(lens), 4) if lens else None,
        "arclength_max": round(max(lens), 4) if lens else None,
        "step_min": round(min(st), 4), "step_max": round(max(st), 4),
        "step_mean": round(sum(st) / len(st), 4),
        "step_bounds": [ctx.threshold("min_step_m"), round(max(0.5 * ctx.vehicle.wheel_base, 0.5), 3)],
    }
    lo, hi = detail["step_bounds"]
    if min(st) < lo or max(st) > hi:
        return CheckResult("T-02", "弧长与步长", WARN,
                           "步长 %.3f~%.3f m 超出 [%.2f, %.2f]" % (min(st), max(st), lo, hi), detail)
    return CheckResult("T-02", "弧长与步长", PASS,
                       "弧长 %.3f m，步长 %.3f~%.3f m（均值 %.3f）"
                       % (detail["arclength_last"], min(st), max(st), detail["step_mean"]), detail)


@check("T-03", "速度剖面合法", requires=("trajectory",))
def t03(ctx):
    ctx.load("traj")
    t = _last_traj(ctx)
    if t is None:
        return CheckResult("T-03", "速度剖面合法", SKIP, "无轨迹", {})
    vs = [p.v_long for p in t.points]
    uniq = sorted({round(v, 4) for v in vs})
    has_neg = any(v < -1e-6 for f in ctx.traj for p in f.points for v in (p.v_long,))
    detail = {"last_frame_v": [round(v, 4) for v in vs][:10],
              "unique_v": uniq[:8], "has_reverse": has_neg,
              "last_point_v": round(vs[-1], 4)}
    if abs(vs[-1]) > 1e-6 and len(vs) > 1:
        return CheckResult("T-03", "速度剖面合法", WARN,
                           "末点速度 %.3f 未强制归零" % vs[-1], detail)
    return CheckResult("T-03", "速度剖面合法", PASS,
                       "速度取值 %d 种，末点 %.2f，%s倒车段"
                       % (len(uniq), vs[-1], "含" if has_neg else "无"), detail)


@check("T-04", "time_from_start 可用性", requires=("trajectory",))
def t04(ctx):
    ctx.load("traj")
    total = sum(len(t.points) for t in ctx.traj)
    nonzero = sum(1 for t in ctx.traj for p in t.points if abs(p.t_from_start) > 1e-9)
    detail = {"points": total, "nonzero": nonzero, "usable": nonzero > 0}
    if nonzero == 0:
        return CheckResult("T-04", "time_from_start 可用性", INFO,
                           "time_from_start 全部为 0 → 禁用该字段，到达时刻须用弧长/速度积分", detail)
    return CheckResult("T-04", "time_from_start 可用性", PASS,
                       "%d/%d 点带有效 time_from_start" % (nonzero, total), detail)


@check("T-05", "曲率 vs 车辆极限", requires=("trajectory",))
def t05(ctx):
    ctx.load("traj")
    kmax = 0.0
    worst_t = None
    for t in ctx.traj:
        if len(t.points) < 3:
            continue
        ks = geometry.curvature_at_step(t.points, 0.3)
        if ks:
            m = max(abs(k) for k in ks)
            if m > kmax:
                kmax, worst_t = m, t.t_bag
    limit = ctx.vehicle.kappa_limit
    detail = {"kappa_max": round(kmax, 4), "r_min_m": round(1 / kmax, 3) if kmax else None,
              "kappa_limit": round(limit, 4), "frame_t": round(worst_t, 3) if worst_t else None,
              "ratio": round(kmax / limit, 3) if limit else None}
    if kmax > limit:
        return CheckResult("T-05", "曲率 vs 车辆极限", FAIL,
                           "最大曲率 %.4f（R=%.2f m）超过极限 %.4f" % (kmax, 1 / kmax, limit), detail)
    return CheckResult("T-05", "曲率 vs 车辆极限", PASS,
                       "最大曲率 %.4f（R_min %.2f m），极限 %.3f，占 %.0f%%"
                       % (kmax, 1 / kmax if kmax else float("inf"), limit, kmax / limit * 100), detail)


@check("T-06", "几何余量", requires=("trajectory", "occupancy_grid"))
def t06(ctx):
    ctx.load("traj")
    ctx.load("occs")
    t = _last_traj(ctx)
    occ = ctx.occ_last()
    edt = ctx.edt()
    if t is None or edt is None:
        return CheckResult("T-06", "几何余量", SKIP, "缺少轨迹或栅格", {})
    dist, k = geometry.min_clearance(t, occ, ctx.vehicle, edt)
    detail = {"min_clearance_m": None if dist == float("inf") else round(dist, 4),
              "worst_point_index": k, "threshold_m": ctx.threshold("min_clearance_m"),
              "footprint": list(ctx.vehicle.footprint())}
    if dist < 0:
        return CheckResult("T-06", "几何余量", FAIL,
                           "轨迹第 %d 点的 footprint 压到占据格" % k, detail)
    if dist < ctx.threshold("min_clearance_m"):
        return CheckResult("T-06", "几何余量", WARN,
                           "最小余量 %.3f m 低于 %.2f m" % (dist, ctx.threshold("min_clearance_m")), detail)
    return CheckResult("T-06", "几何余量", PASS,
                       "最小余量 %.3f m（第 %d 点）" % (dist, k), detail)


@check("T-07", "轨迹不越出可泊区", requires=("trajectory", "occupancy_grid"))
def t07(ctx):
    ctx.load("traj")
    ctx.load("occs")
    occ = ctx.occ_last()
    bad = []
    for t in ctx.traj:
        for k, p in enumerate(t.points):
            c = geometry.occ_at(occ, p.x, p.y)
            if c is not None and c >= 100:
                bad.append((round(t.t_bag, 3), k))
                break
    detail = {"frames_checked": len(ctx.traj), "frames_with_occupied_point": len(bad),
              "samples": bad[:5]}
    if bad:
        return CheckResult("T-07", "轨迹不越出可泊区", FAIL,
                           "%d 帧轨迹经过占据格" % len(bad), detail)
    return CheckResult("T-07", "轨迹不越出可泊区", PASS,
                       "%d 帧轨迹全部落在自由格" % len(ctx.traj), detail)


@check("T-08", "分片连续性", requires=("trajectory",))
def t08(ctx):
    ctx.load("traj")
    if len(ctx.traj) < 2:
        return CheckResult("T-08", "分片连续性", SKIP, "轨迹帧不足", {})
    counts = [len(t.points) for t in ctx.traj]
    partial = counts[len(counts) // 2] <= 10
    if partial:
        ctx.load("odom")
        worst = 0.0
        for t in ctx.traj:
            o = ctx.nearest_odom(t.t_bag)
            if o and t.points:
                worst = max(worst, math.hypot(t.points[0].x - o.x, t.points[0].y - o.y))
        detail = {"partial": True, "points_median": counts[len(counts) // 2],
                  "max_first_point_to_ego_m": round(worst, 3), "frames": len(ctx.traj)}
        if worst > 1.0:
            return CheckResult("T-08", "分片连续性", WARN,
                               "分段轨迹首点与自车位姿最大相距 %.2f m" % worst, detail)
        return CheckResult("T-08", "分片连续性", PASS,
                           "分段轨迹每周期从自车重规划，首点距自车 ≤ %.2f m" % worst, detail)
    gaps = []
    for a, b in zip(ctx.traj, ctx.traj[1:]):
        if not a.points or not b.points:
            continue
        d = math.hypot(b.points[0].x - a.points[-1].x, b.points[0].y - a.points[-1].y)
        if d > 0.5:
            gaps.append((round(b.t_bag, 3), round(d, 3)))
    detail = {"partial": False, "pairs": len(ctx.traj) - 1,
              "discontinuities": len(gaps), "samples": gaps[:5]}
    if len(gaps) > max(3, 0.1 * len(ctx.traj)):
        return CheckResult("T-08", "分片连续性", WARN,
                           "%d/%d 对相邻帧首末点相距 > 0.5 m" % (len(gaps), len(ctx.traj) - 1), detail)
    return CheckResult("T-08", "分片连续性", PASS,
                       "相邻帧首末点衔接正常（%d 处 > 0.5 m）" % len(gaps), detail)


@check("T-09", "原地空转检测", requires=("trajectory", "odometry"))
def t09(ctx):
    ctx.load("traj")
    ctx.load("odom")
    if len(ctx.traj) < 2:
        return CheckResult("T-09", "原地空转检测", SKIP, "轨迹帧不足", {})
    max_run = run = 1
    prev = _frame_key(ctx.traj[0])
    for t in ctx.traj[1:]:
        k = _frame_key(t)
        if k == prev:
            run += 1
            max_run = max(max_run, run)
        else:
            run = 1
        prev = k
    xs = [o.x for o in ctx.odom]
    ys = [o.y for o in ctx.odom]
    span = math.hypot(max(xs) - min(xs), max(ys) - min(ys)) if xs else 0.0
    vmax = max((math.hypot(o.vx, o.vy) for o in ctx.odom), default=0.0)
    detail = {"max_identical_run": max_run, "frames": len(ctx.traj),
              "odom_xy_span_m": round(span, 3), "odom_v_max": round(vmax, 3),
              "threshold_frames": ctx.threshold("stall_frames")}
    if max_run >= ctx.threshold("stall_frames") and (span < 1.0 or vmax < 0.1):
        return CheckResult("T-09", "原地空转检测", WARN,
                           "连续 %d 帧轨迹完全相同且车辆静止（未推进）" % max_run, detail)
    return CheckResult("T-09", "原地空转检测", PASS,
                       "最长相同轨迹连续 %d 帧，车辆位移 %.2f m" % (max_run, span), detail)


@check("T-10", "终点误差", requires=("trajectory", "route"))
def t10(ctx):
    ctx.load("traj")
    ctx.load("route")
    t = _last_traj(ctx)
    route = ctx.route
    if t is None or route is None or not t.points:
        return CheckResult("T-10", "终点误差", SKIP, "缺少轨迹或 route", {})
    gx, gy, _gz, gyaw = route.goal
    ex, ey = t.points[-1].x, t.points[-1].y
    d = math.hypot(ex - gx, ey - gy)
    dyaw = math.degrees(abs(_wrap(t.points[-1].yaw - gyaw)))
    counts = [len(f.points) for f in ctx.traj]
    arc = geometry.total_length(t.points)
    # 分段发布：轨迹只是全程的一段（点数少且远短于到 goal 的直线距离）
    partial = (len(t.points) < max(counts) * 0.9) or (len(t.points) <= 10 and arc < d * 0.6)
    detail = {"end": [round(ex, 3), round(ey, 3)], "goal": [round(gx, 3), round(gy, 3)],
              "delta_xy_m": round(d, 3), "delta_yaw_deg": round(dyaw, 2),
              "arclength_m": round(arc, 3),
              "n_points": len(t.points), "max_points": max(counts), "partial": partial,
              "threshold_xy_m": ctx.threshold("max_goal_xy_error_m"),
              "threshold_yaw_deg": ctx.threshold("max_goal_yaw_error_deg")}
    if partial and d > ctx.threshold("max_goal_xy_error_m"):
        return CheckResult("T-10", "终点误差", INFO,
                           "分段轨迹 %d 点 / %.2f m，距 goal %.2f m —— 只发了第一段（分段规划的正常表现）"
                           % (len(t.points), arc, d), detail)
    if d > ctx.threshold("max_goal_xy_error_m") or dyaw > ctx.threshold("max_goal_yaw_error_deg"):
        return CheckResult("T-10", "终点误差", WARN,
                           "终点距 goal %.2f m / %.1f°" % (d, dyaw), detail)
    return CheckResult("T-10", "终点误差", PASS,
                       "终点距 goal %.2f m / %.1f°" % (d, dyaw), detail)


@check("T-11", "源轨迹 vs 最终轨迹", requires=("trajectory", "final_trajectory"))
def t11(ctx):
    ctx.load("traj")
    ctx.load("final_traj")
    if not ctx.traj or not ctx.final_traj:
        return CheckResult("T-11", "源轨迹 vs 最终轨迹", SKIP, "缺少某条轨迹", {})
    src = _last_traj(ctx)
    fin = min(ctx.final_traj, key=lambda f: abs(f.stamp - src.stamp))
    if len(src.points) < 2 or len(fin.points) < 2:
        return CheckResult("T-11", "源轨迹 vs 最终轨迹", SKIP, "轨迹点不足", {})
    la = geometry.total_length(src.points)
    lb = geometry.total_length(fin.points)
    devs = geometry.align_by_arclength(src.points, fin.points)
    dev = max(devs)
    detail = {"arc_source_m": round(la, 4), "arc_final_m": round(lb, 4),
              "arc_diff_m": round(abs(la - lb), 4),
              "max_lateral_dev_m": round(dev, 4),
              "threshold_m": ctx.threshold("max_arc_deviation_m"),
              "source_points": len(src.points), "final_points": len(fin.points)}
    if dev > ctx.threshold("max_arc_deviation_m"):
        return CheckResult("T-11", "源轨迹 vs 最终轨迹", WARN,
                           "按弧长对齐后最大横向偏差 %.3f m" % dev, detail)
    return CheckResult("T-11", "源轨迹 vs 最终轨迹", PASS,
                       "弧长 %.3f vs %.3f m，最大横向偏差 %.3f m" % (la, lb, dev), detail)


@check("T-12", "最终轨迹速度单调性", requires=("final_trajectory",))
def t12(ctx):
    ctx.load("final_traj")
    if not ctx.final_traj:
        return CheckResult("T-12", "最终轨迹速度单调性", SKIP, "无 /planning/trajectory", {})
    t = ctx.final_traj[-1]
    vs = [p.v_long for p in t.points]
    # 首点从 0 起步（|v0| <= |v1|）或与第二点相同；倒车段为负值属正常
    head_ok = len(vs) < 2 or abs(vs[0]) <= abs(vs[1]) + 1e-6
    tail_ok = abs(vs[-1]) < 1e-6
    detail = {"first_v": round(vs[0], 4), "second_v": round(vs[1], 4) if len(vs) > 1 else None,
              "last_v": round(vs[-1], 4), "points": len(vs),
              "v_abs_max": round(max(abs(v) for v in vs), 4)}
    if not (head_ok and tail_ok):
        return CheckResult("T-12", "最终轨迹速度单调性", WARN,
                           "首点 %.2f / 末点 %.2f，未满足起停约束" % (vs[0], vs[-1]), detail)
    return CheckResult("T-12", "最终轨迹速度单调性", PASS,
                       "首点 %.2f → 峰值(abs) %.2f → 末点 %.2f" % (vs[0], detail["v_abs_max"], vs[-1]), detail)


@check("T-13", "帧时间戳单调", requires=("trajectory",))
def t13(ctx):
    ctx.load("traj")
    ctx.load("final_traj")
    bad = []
    for name, seq in (("trajectory", ctx.traj), ("final_trajectory", ctx.final_traj)):
        for a, b in zip(seq, seq[1:]):
            if b.stamp <= a.stamp:
                bad.append((name, round(b.t_bag, 3)))
    detail = {"violations": len(bad), "samples": bad[:5],
              "traj_frames": len(ctx.traj), "final_frames": len(ctx.final_traj)}
    if bad:
        return CheckResult("T-13", "帧时间戳单调", WARN, "%d 处时间戳非严格递增" % len(bad), detail)
    return CheckResult("T-13", "帧时间戳单调", PASS,
                       "两条轨迹的 header.stamp 严格递增", detail)


def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a
