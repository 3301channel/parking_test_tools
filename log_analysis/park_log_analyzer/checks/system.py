"""system 类规则 S-01 ~ S-08（开发文档 §6.4）。"""

from __future__ import annotations

import math
from collections import Counter

from ..cdr import Cdr, payload
from .base import FAIL, INFO, PASS, SKIP, WARN, CheckResult, check

_NO_HEADER_TYPES = ("std_msgs/Bool", "autoware_internal_debug_msgs/Float64Stamped")
_LEVEL = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}


def _median(vals):
    s = sorted(vals)
    n = len(s)
    if not n:
        return None
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def _header_times(ctx, key, limit=200):
    """只解 header.stamp，避免整条消息解码。"""
    meta = ctx.topic_meta(key)
    if meta is None:
        return []
    out = []
    for i, (ns, raw) in enumerate(ctx.reader.messages(ctx.topic(key))):
        if i >= limit:
            break
        if meta.type in _NO_HEADER_TYPES or "/msg/Bool" in meta.type:
            continue
        try:
            c = Cdr(payload(raw))
            out.append((ctx.rel(ns), c.time()))
        except Exception:
            continue
    return out


@check("S-01", "tf 与 tf_static", requires=())
def s01(ctx):
    meta_tf = ctx.topics.get(ctx.topic("tf"))
    meta_static = ctx.topics.get(ctx.topic("tf_static"))
    detail = {"tf": meta_tf.count if meta_tf else 0, "tf_static": meta_static.count if meta_static else 0}
    if not detail["tf"] and not detail["tf_static"]:
        return CheckResult("S-01", "tf 与 tf_static", SKIP,
                           "bag 内无 /tf 与 /tf_static → 所有需要坐标变换的规则降级为 SKIP", detail)
    return CheckResult("S-01", "tf 与 tf_static", PASS,
                       "/tf %d 条 / /tf_static %d 条" % (detail["tf"], detail["tf_static"]), detail)


@check("S-02", "时间戳一致性", requires=("trajectory",))
def s02(ctx):
    rows = {}
    offset_topics = []
    ratios = []
    bad = []
    for key in ("grid_map", "occupancy_grid", "trajectory", "odometry"):
        samples = _header_times(ctx, key)
        if len(samples) < 2:
            continue
        lags = [bag - stamp for bag, stamp in samples]
        med = _median(lags)
        drift = lags[-1] - lags[0]
        bag_span = samples[-1][0] - samples[0][0]
        stamp_span = samples[-1][1] - samples[0][1]
        ratio = stamp_span / bag_span if bag_span else None
        rows[key] = {"samples": len(lags), "median_s": round(med, 4),
                     "min_s": round(min(lags), 4), "max_s": round(max(lags), 4),
                     "drift_over_window_s": round(drift, 4),
                     "clock_ratio": round(ratio, 3) if ratio else None}
        if abs(med) > 5.0:
            offset_topics.append(key)
            if ratio:
                ratios.append(ratio)
        elif med > ctx.threshold("max_latency_s"):
            bad.append("%s 录制延迟 %.2f s" % (key, med))
    detail = {"topics": rows, "time_base_offset": bool(offset_topics)}
    if bad:
        return CheckResult("S-02", "时间戳一致性", WARN, "；".join(bad), detail)
    if offset_topics:
        return CheckResult("S-02", "时间戳一致性", INFO,
                           "%s 的 header.stamp 与 bag 时间存在固定零时刻差，时钟比约 %.2f"
                           "（仿真时钟 vs 录制时钟，不是真实延迟；频率判定已按 stamp 计算）"
                           % ("、".join(offset_topics), sum(ratios) / len(ratios) if ratios else 0.0),
                           detail)
    return CheckResult("S-02", "时间戳一致性", PASS, "bag 时间与 header.stamp 延迟正常", detail)


@check("S-03", "乱序检测", requires=("trajectory",))
def s03(ctx):
    ctx.load("traj")
    ctx.load("final_traj")
    bad = []
    for name, seq in (("trajectory", ctx.traj), ("final_trajectory", ctx.final_traj)):
        for a, b in zip(seq, seq[1:]):
            if b.stamp < a.stamp:
                bad.append((name, round(b.t_bag, 3)))
    detail = {"violations": len(bad), "samples": bad[:5]}
    if bad:
        return CheckResult("S-03", "乱序检测", WARN, "%d 处 header.stamp 递减" % len(bad), detail)
    return CheckResult("S-03", "乱序检测", PASS, "轨迹话题 header.stamp 单调不减", detail)


@check("S-04", "rosout 错误聚类", requires=("rosout",))
def s04(ctx):
    ctx.load("rosout")
    levels = Counter(m["level"] for m in ctx.rosout)
    errs = [m for m in ctx.rosout if m["level"] in ("ERROR", "FATAL")]
    by_node = Counter(m["name"] for m in errs)
    msgs = Counter(m["msg"][:80] for m in errs)
    detail = {"messages": len(ctx.rosout), "by_level": dict(levels),
              "error_by_node": dict(by_node.most_common(8)),
              "top_errors": [{"msg": k, "count": v} for k, v in msgs.most_common(5)]}
    if errs:
        return CheckResult("S-04", "rosout 错误聚类", WARN,
                           "%d 条 ERROR/FATAL，涉及 %d 个节点" % (len(errs), len(by_node)), detail)
    return CheckResult("S-04", "rosout 错误聚类", PASS,
                       "%d 条日志，无 ERROR/FATAL" % len(ctx.rosout), detail)


@check("S-05", "diagnostics 汇总", requires=("diagnostics",))
def s05(ctx):
    ctx.load("diag")
    by_name = {}
    by_level = {}
    level_name = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}
    for m in ctx.diagnostics:
        for level, name in m["statuses"]:
            nm = level_name.get(level, str(level))
            key = (name, nm)
            by_name[key] = by_name.get(key, 0) + 1
            by_level[nm] = by_level.get(nm, 0) + 1
    bad = [(k, v) for k, v in by_name.items() if k[1] != "OK"]
    bad.sort(key=lambda kv: -kv[1])
    detail = {"messages": len(ctx.diagnostics), "statuses": sum(by_name.values()),
              "by_level": by_level,
              "top_not_ok": [{"name": k[0], "level": k[1], "count": v} for k, v in bad[:12]]}
    total_bad = sum(v for _, v in bad)
    if total_bad:
        names = "、".join("%s(%s)×%d" % (k[0].split(":")[0], k[1], v) for k, v in bad[:3])
        return CheckResult("S-05", "diagnostics 汇总", WARN,
                           "%d/%d 条诊断非 OK：%s" % (total_bad, sum(by_name.values()), names), detail)
    return CheckResult("S-05", "diagnostics 汇总", PASS,
                       "%d 条诊断消息全部 OK" % len(ctx.diagnostics), detail)


@check("S-06", "车辆是否运动", requires=("odometry",))
def s06(ctx):
    ctx.load("odom")
    ctx.load("traj")
    xs = [o.x for o in ctx.odom]
    ys = [o.y for o in ctx.odom]
    vs = [math.hypot(o.vx, o.vy) for o in ctx.odom]
    span = math.hypot(max(xs) - min(xs), max(ys) - min(ys)) if xs else 0.0
    target = max((abs(p.v_long) for t in ctx.traj for p in t.points), default=0.0)
    gears = []
    try:
        ctx.load("gear")
        prev = None
        for g in ctx.gear_status:
            if g["gear_name"] != prev:
                gears.append({"t_s": round(g["t_bag"], 2), "gear": g["gear_name"]})
                prev = g["gear_name"]
    except Exception:
        pass
    detail = {"samples": len(ctx.odom), "xy_span_m": round(span, 3),
              "v_max": round(max(vs), 3) if vs else 0.0,
              "v_mean": round(sum(vs) / len(vs), 3) if vs else 0.0,
              "traj_target_v_max": round(target, 3),
              "gear_changes": gears,
              "start": [round(xs[0], 3), round(ys[0], 3)] if xs else None,
              "end": [round(xs[-1], 3), round(ys[-1], 3)] if xs else None}
    if span < 0.1 and (not vs or max(vs) < 0.01):
        return CheckResult("S-06", "车辆是否运动", WARN,
                           "odom 位置跨度 %.2f m、速度全 0 → 车辆静止" % span, detail)
    if vs and max(vs) < 0.5:
        extra = "" if target <= 0 else "（轨迹目标 %.2f m/s，执行未跟随）" % target
        return CheckResult("S-06", "车辆是否运动", WARN,
                           "车辆几乎未运动：%.2f m / %.1f s，最高速度仅 %.2f m/s%s"
                           % (span, ctx.bag_duration, max(vs), extra), detail)
    return CheckResult("S-06", "车辆是否运动", PASS,
                       "车辆位移 %.2f m，最高速度 %.2f m/s（目标 %.2f）"
                       % (span, max(vs) if vs else 0, target), detail)


@check("S-07", "车辆模式", requires=())
def s07(ctx):
    out = {}
    for topic, meta in ctx.topics.items():
        if any(k in topic for k in ("motion/state", "route_state", "routing/state",
                                    "operation_mode", "is_autonomous_available")) \
                or topic == "/autoware/state":
            out[topic] = {"type": meta.short_type, "count": meta.count}
    if not out:
        return CheckResult("S-07", "车辆模式", SKIP, "未录制状态话题", {})
    unknown = [t for t, v in out.items() if v["count"] == 0]
    detail = {"topics": out, "empty": unknown}
    return CheckResult("S-07", "车辆模式", INFO,
                       "；".join("%s=%d" % (t, v["count"]) for t, v in sorted(out.items())), detail)


@check("S-08", "空话题归因", requires=())
def s08(ctx):
    empty = []
    for name, meta in sorted(ctx.topics.items()):
        if meta.count == 0:
            empty.append({"name": name, "type": meta.short_type,
                          "reason": "record_rejected_or_no_publisher"})
    detail = {"total_topics": len(ctx.topics), "empty_topics": empty}
    if empty:
        return CheckResult("S-08", "空话题归因", INFO,
                           "%d 个话题消息数为 0（录制拒绝 / 无发布者 / 未触发）" % len(empty), detail)
    return CheckResult("S-08", "空话题归因", INFO, "没有空话题", detail)
