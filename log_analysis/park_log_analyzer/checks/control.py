"""执行链规则 D-01 ~ D-08。

回答的核心问题是"现象在 planning，原因在哪一层"：

  规划 → 跟随器 → 控制指令 → 门控 → 执行指令 → 车辆上报

C/P/T 规则只看规划输出，本组规则把控制链与状态机一起摆出来，
所以能直接区分"规划给不出路"和"规划给了但车不走"。
"""

from __future__ import annotations

from ..context import total_seconds, transitions
from ..msgs.control import AW_STATE, OPERATION_MODE, ROUTING_STATE
from .base import FAIL, INFO, PASS, SKIP, WARN, CheckResult, check

GEAR = {0: "NONE", 1: "NEUTRAL", 2: "DRIVE", 20: "REVERSE", 21: "REVERSE_2", 22: "PARK"}
_FORWARD = {2}
_REVERSE = {20, 21}


def _p95(vals):
    if not vals:
        return None
    s = sorted(vals)
    return s[min(len(s) - 1, max(0, int(round(0.95 * (len(s) - 1)))))]


def _last(seq, key):
    if not seq:
        return None
    item = seq[-1]
    return item.get(key) if isinstance(item, dict) else getattr(item, key, None)


@check("D-01", "状态机时间线", requires=("autoware_state",))
def d01(ctx):
    ctx.load("aw_state")
    tr = ctx.states("aw")
    dur = total_seconds(ctx.aw_state, "state", AW_STATE, until=ctx.bag_duration)
    driving = dur.get("DRIVING", 0.0)
    final = _last(ctx.aw_state, "state_name")
    detail = {"transitions": tr[:16], "durations_s": dur, "final": final,
              "bag_duration_s": round(ctx.bag_duration, 3),
              "messages": len(ctx.aw_state)}

    if not ctx.aw_state:
        return CheckResult("D-01", "状态机时间线", SKIP, "没有 /autoware/state", detail)
    if driving > 0.0:
        first = next((t["t_s"] for t in tr if t["name"] == "DRIVING"), None)
        return CheckResult("D-01", "状态机时间线", PASS,
                           "%.2f s 进入 DRIVING，累计行驶 %.2f s，终态 %s"
                           % (first or 0.0, driving, final), detail)
    if ctx.bag_duration < 20.0:
        return CheckResult("D-01", "状态机时间线", INFO,
                           "bag 仅 %.1f s，状态机停在 %s（不足以判定）"
                           % (ctx.bag_duration, final), detail)
    return CheckResult("D-01", "状态机时间线", WARN,
                       "全程未进入 DRIVING，终态 %s（累计 %s）"
                       % (final, "、".join("%s %.1fs" % kv for kv in dur.items()) or "无"), detail)


@check("D-02", "操作模式与起步许可", requires=("operation_mode",))
def d02(ctx):
    ctx.load("modes")
    tr = ctx.states("operation")
    dur = total_seconds(ctx.operation_mode, "mode", OPERATION_MODE, until=ctx.bag_duration)
    auto_s = dur.get("AUTONOMOUS", 0.0)
    first_auto = next((t["t_s"] for t in tr if t["name"] == "AUTONOMOUS"), None)
    motion_tr = ctx.states("motion")

    avail = None
    if ctx.is_auto_available:
        avail = all(p.value > 0.5 for p in ctx.is_auto_available)
    detail = {"transitions": tr[:12], "durations_s": dur,
              "first_autonomous_s": first_auto,
              "motion_transitions": motion_tr[:12],
              "is_autonomous_available_all_true": avail}

    if auto_s <= 0.0:
        hint = ""
        if avail is False:
            hint = "；/control/is_autonomous_available 全程为 false（AUTO 按钮不可用是前置阻塞）"
        return CheckResult("D-02", "操作模式与起步许可", WARN,
                           "全程未进入 AUTONOMOUS（停留 %s）%s"
                           % ("、".join("%s %.1fs" % kv for kv in dur.items()) or "未知", hint), detail)
    return CheckResult("D-02", "操作模式与起步许可", PASS,
                       "%.2f s 切到 AUTONOMOUS，累计 %.2f s" % (first_auto or 0.0, auto_s), detail)


@check("D-03", "档位指令一致性", requires=("gear_command", "gear_status"))
def d03(ctx):
    ctx.load("gear")
    ctx.load("shift")
    tol = ctx.threshold("gear_confirm_s")
    cmds = [g for g in ctx.gear_command if g["gear"] in (_FORWARD | _REVERSE)]
    mism, unconfirmed = [], []
    for cmd in cmds:
        rep = next((g for g in ctx.gear_status
                    if cmd["t_bag"] <= g["t_bag"] <= cmd["t_bag"] + tol), None)
        if rep is None:
            unconfirmed.append(round(cmd["t_bag"], 3))
        elif rep["gear"] != cmd["gear"]:
            mism.append({"t_s": round(cmd["t_bag"], 3),
                         "cmd": GEAR.get(cmd["gear"], str(cmd["gear"])),
                         "report": GEAR.get(rep["gear"], str(rep["gear"])),
                         "delay_s": round(rep["t_bag"] - cmd["t_bag"], 3)})
    gear_tr = transitions(ctx.gear_status, "gear_name")
    shift_first = next((g for g in ctx.shift_gear if g["gear"] in _REVERSE), None)
    detail = {"commands": len(cmds), "mismatch": len(mism), "unconfirmed": len(unconfirmed),
              "samples": mism[:6], "unconfirmed_samples": unconfirmed[:6],
              "gear_timeline": gear_tr[:12],
              "shift_decider_first_reverse_s": round(shift_first["t_bag"], 3) if shift_first else None,
              "threshold_s": tol}
    if not cmds:
        return CheckResult("D-03", "档位指令一致性", SKIP,
                           "没有 D/R 档位指令（全程未请求行驶）", detail)
    if mism and len(mism) >= max(3, 0.2 * len(cmds)):
        return CheckResult("D-03", "档位指令一致性", WARN,
                           "%d/%d 条档位指令未被如实上报(例：指令 %s → 上报 %s)"
                           % (len(mism), len(cmds), mism[0]["cmd"], mism[0]["report"]), detail)
    return CheckResult("D-03", "档位指令一致性", PASS,
                       "%d 条 D/R 指令，%d 条 %.1fs 内换挡到位"
                       % (len(cmds), len(cmds) - len(mism) - len(unconfirmed), tol), detail)


@check("D-04", "控制链贯通", requires=("trajectory", "control_cmd"))
def d04(ctx):
    ctx.load("control")
    ctx.load("traj")
    ctx.load("final_traj")
    ctx.load("actuation")
    ctx.load("odom")
    eps = ctx.threshold("cmd_speed_eps")
    latency = ctx.threshold("cmd_latency_s")
    traj = ctx.final_traj or ctx.traj
    moving = [f for f in traj if f.points and max(abs(p.v_long) for p in f.points) > 0.3]
    covered = 0
    for f in moving:
        if any(abs(c["t_bag"] - f.t_bag) <= latency for c in ctx.control_cmd):
            covered += 1
    zeroed = 0
    lags = []
    for c in ctx.control_cmd:
        if not traj:
            break
        near = min(traj, key=lambda f: abs(f.t_bag - c["t_bag"]))
        if abs(near.t_bag - c["t_bag"]) > latency:
            continue
        lags.append(abs(near.t_bag - c["t_bag"]))
        target = max((abs(p.v_long) for p in near.points), default=0.0)
        if target > 0.3 and abs(c["velocity"]) < eps:
            zeroed += 1
    ratio = covered / len(moving) if moving else None
    detail = {"control_cmd": len(ctx.control_cmd), "follower_cmd": len(ctx.follower_cmd),
              "actuation_cmd": len(ctx.actuation),
              "moving_traj_frames": len(moving), "covered_frames": covered,
              "coverage": round(ratio, 3) if ratio is not None else None,
              "zeroed_while_target_moving": zeroed,
              "latency_p95_s": round(_p95(lags), 3) if lags else None,
              "threshold_latency_s": latency}
    if ratio is not None and ratio < 0.5:
        return CheckResult("D-04", "控制链贯通", WARN,
                           "规划有速度的 %d 帧里仅 %d 帧存在控制指令（覆盖 %.0f%%）"
                           % (len(moving), covered, ratio * 100), detail)
    if zeroed and len(ctx.control_cmd) and zeroed / len(ctx.control_cmd) > 0.5:
        return CheckResult("D-04", "控制链贯通", WARN,
                           "%d/%d 条控制指令速度为 0，而规划目标非零 → 跟随器/门控在抑制"
                           % (zeroed, len(ctx.control_cmd)), detail)
    chain = "control_cmd %d 条" % len(ctx.control_cmd)
    if ctx.follower_cmd:
        chain = "follower %d → " % len(ctx.follower_cmd) + chain
    if ctx.actuation:
        chain += " → actuation %d 条" % len(ctx.actuation)
    return CheckResult("D-04", "控制链贯通", PASS, "链路连续：%s" % chain, detail)


@check("D-05", "指令跟随误差", requires=("control_cmd", "odometry"))
def d05(ctx):
    ctx.load("control")
    ctx.load("odom")
    if not ctx.control_cmd or not ctx.odom:
        return CheckResult("D-05", "指令跟随误差", SKIP, "缺少控制指令或 odom", {})
    step = max(1, len(ctx.control_cmd) // 3000)
    errs, stuck, run_start = [], [], None
    hold = ctx.threshold("stuck_hold_s")
    for c in ctx.control_cmd[::step]:
        actual = ctx.signed_speed_at(c["t_bag"])
        if actual is None:
            continue
        errs.append(abs(actual - c["velocity"]))
        if abs(c["velocity"]) > 0.2 and abs(actual) < ctx.threshold("stuck_speed"):
            run_start = c["t_bag"] if run_start is None else run_start
        elif run_start is not None:
            if c["t_bag"] - run_start >= hold:
                stuck.append([round(run_start, 2), round(c["t_bag"], 2)])
            run_start = None
    if run_start is not None and ctx.control_cmd:
        end = ctx.control_cmd[-1]["t_bag"]
        if end - run_start >= hold:
            stuck.append([round(run_start, 2), round(end, 2)])
    p95 = _p95(errs)
    detail = {"samples": len(errs), "abs_error_p95": round(p95, 4) if p95 is not None else None,
              "abs_error_max": round(max(errs), 4) if errs else None,
              "stuck_windows": stuck[:6], "stuck_count": len(stuck),
              "threshold_error": ctx.threshold("follow_error_warn"),
              "threshold_hold_s": hold}
    if stuck:
        return CheckResult("D-05", "指令跟随误差", WARN,
                           "%d 段「有指令但车不动」：%s"
                           % (len(stuck), "、".join("%.2f~%.2f s" % tuple(w) for w in stuck[:3])),
                           detail)
    if p95 is not None and p95 > ctx.threshold("follow_error_warn"):
        return CheckResult("D-05", "指令跟随误差", WARN,
                           "指令与实际速度偏差 p95 %.2f m/s（阈值 %.2f）"
                           % (p95, ctx.threshold("follow_error_warn")), detail)
    return CheckResult("D-05", "指令跟随误差", PASS,
                       "跟随偏差 p95 %.3f m/s（%d 个采样）" % (p95 or 0.0, len(errs)), detail)


@check("D-06", "停车闸门", requires=("is_stopped",))
def d06(ctx):
    ctx.load("gate")
    tr = transitions(ctx.is_stopped, "value")
    stopped_s = total_seconds(ctx.is_stopped, "value", {1.0: "STOPPED", 0.0: "RELEASED"},
                              until=ctx.bag_duration).get("STOPPED", 0.0)
    start_tr = transitions(ctx.is_start_requested, "value")
    target = None
    try:
        ctx.load("final_traj")
        target = max((max(abs(p.v_long) for p in f.points)
                      for f in ctx.final_traj if f.points), default=None)
    except Exception:
        pass
    detail = {"transitions": tr[:12], "stopped_s": stopped_s,
              "bag_duration_s": round(ctx.bag_duration, 3),
              "start_requested_transitions": start_tr[:12],
              "traj_target_v_max": round(target, 3) if target else None}
    if stopped_s > 0.5 * max(ctx.bag_duration, 1e-9) and (target or 0.0) > 0.3:
        return CheckResult("D-06", "停车闸门", WARN,
                           "is_stopped 累计 %.1f s / %.1f s（%.0f%%），而规划目标 %.2f m/s"
                           % (stopped_s, ctx.bag_duration,
                              stopped_s / max(ctx.bag_duration, 1e-9) * 100, target), detail)
    if stopped_s > 0.0:
        return CheckResult("D-06", "停车闸门", INFO,
                           "is_stopped 累计 %.1f s（%d 次翻转）" % (stopped_s, len(tr)), detail)
    return CheckResult("D-06", "停车闸门", INFO, "is_stopped 全程未置位", detail)


@check("D-07", "路由与场景前置", requires=("routing_state", "scenario"))
def d07(ctx):
    ctx.load("modes")
    ctx.load("scenario")
    routing_tr = ctx.states("routing")
    windows = ctx.parking_windows()
    names = sorted({n for s in ctx.scenario for n in s["scenarios"]})
    route_set = next((t["t_s"] for t in routing_tr if t["name"] == "ROUTE_SET"), None)
    first_pk = ctx.first_parking_time()
    detail = {"routing_transitions": routing_tr[:12], "scenarios_seen": names,
              "first_parking_s": round(first_pk, 3) if first_pk is not None else None,
              "parking_windows": [[round(a, 3), round(b, 3)] for a, b in windows],
              "route_set_s": route_set}

    if route_set is None:
        return CheckResult("D-07", "路由与场景前置", WARN,
                           "routing_state 全程未到 ROUTE_SET（%s）"
                           % ("、".join("%s %.1fs" % kv for kv in
                                       total_seconds(ctx.routing, "state", ROUTING_STATE,
                                                     until=ctx.bag_duration).items()) or "无数据"),
                           detail)
    if first_pk is None:
        return CheckResult("D-07", "路由与场景前置", WARN,
                           "路由已建立（%.2f s）但场景未进入 Parking（出现过：%s）"
                           % (route_set, "、".join(names) or "无"), detail)
    if first_pk < route_set:
        return CheckResult("D-07", "路由与场景前置", INFO,
                           "Parking(%.2f s) 早于 ROUTE_SET(%.2f s)：首帧可能是残留状态"
                           % (first_pk, route_set), detail)
    return CheckResult("D-07", "路由与场景前置", PASS,
                       "路由 %.2f s → Parking %.2f s，共 %d 段"
                       % (route_set, first_pk, len(windows)), detail)


@check("D-08", "倒车方向一致性", requires=("gear_status", "velocity_status"))
def d08(ctx):
    ctx.load("gear")
    ctx.load("vehicle")
    ctx.load("control")
    tol = 0.3
    wrong = []
    for g in ctx.gear_status:
        if g["gear"] not in (_FORWARD | _REVERSE):
            continue
        v = next((x for x in ctx.velocity if abs(x["t_bag"] - g["t_bag"]) <= tol), None)
        if v is None:
            continue
        if g["gear"] in _REVERSE and v["v_long"] > 0.1:
            wrong.append({"t_s": round(g["t_bag"], 3), "gear": "REVERSE",
                          "v_long": round(v["v_long"], 3), "expect": "负"})
        elif g["gear"] in _FORWARD and v["v_long"] < -0.1:
            wrong.append({"t_s": round(g["t_bag"], 3), "gear": "DRIVE",
                          "v_long": round(v["v_long"], 3), "expect": "正"})
    steer_err = None
    if ctx.control_cmd and ctx.steering:
        diffs = []
        for c in ctx.control_cmd[:: max(1, len(ctx.control_cmd) // 1000)]:
            s = next((x for x in ctx.steering if abs(x["t_bag"] - c["t_bag"]) <= 0.2), None)
            if s is not None and c.get("steering") is not None:
                diffs.append(abs(s["steer"] - c["steering"]))
        steer_err = _p95(diffs)
    detail = {"wrong_direction_samples": len(wrong), "samples": wrong[:6],
              "velocity_msgs": len(ctx.velocity), "gear_msgs": len(ctx.gear_status),
              "steer_error_p95_rad": round(steer_err, 4) if steer_err is not None else None}
    if len(wrong) >= 3:
        return CheckResult("D-08", "倒车方向一致性", FAIL,
                           "%d 个采样挡位与实测速度反号(例：%s 档 v=%+.2f m/s) → "
                           "油门/挡位符号接反（Carla 接口经典坑）"
                           % (len(wrong), wrong[0]["gear"], wrong[0]["v_long"]), detail)
    if wrong:
        return CheckResult("D-08", "倒车方向一致性", INFO,
                           "%d 个采样挡位与速度反号（少量，可能是换挡瞬间）" % len(wrong), detail)
    return CheckResult("D-08", "倒车方向一致性", PASS,
                       "挡位与实测速度符号一致（%d/%d 帧可对齐）"
                       % (len(ctx.velocity), len(ctx.gear_status)), detail)
