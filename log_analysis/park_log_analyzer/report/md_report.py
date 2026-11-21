"""report.md（开发文档 §9.2 固定章节）。"""

from __future__ import annotations

import os
from typing import List

from ..checks.base import FAIL, WARN

_ICON = {"PASS": "PASS", "WARN": "WARN", "FAIL": "FAIL", "INFO": "INFO", "SKIP": "SKIP"}


def _table(results) -> str:
    lines = ["| 规则 | 结论 | 说明 |", "|---|---|---|"]
    for r in results:
        lines.append("| %s %s | %s | %s |" % (r.id, r.title, _ICON.get(r.severity, r.severity),
                                              r.summary.replace("|", "/")))
    return "\n".join(lines)


def _by_prefix(results, prefix):
    return [r for r in results if r.id.startswith(prefix)]


def _find(results, cid):
    for r in results:
        if r.id == cid:
            return r
    return None


def build(ctx, report: dict, results, outdir: str) -> str:
    meta = report["meta"]
    verdict = report["verdict"]
    fails = [r for r in results if r.severity == FAIL]
    warns = [r for r in results if r.severity == WARN]

    L: List[str] = []
    L.append("# 泊车日志分析报告 —— %s" % meta["bag"])
    L.append("")
    L.append("- bag 路径：`%s`" % meta["bag_path"])
    L.append("- 时长：%.2f s，话题 %d 个，消息 %d 条，payload %.2f MB"
             % (meta["bag_duration_s"], meta["topics"], meta["messages"], meta["payload_bytes"] / 1048576))
    cal = meta.get("grid_calibration")
    if cal:
        L.append("- 栅格标定：`%s` + `%s`，一致率 %.2f%%（%d 格不一致）"
                 % (cal["layout"], cal["transform"], cal["agreement"] * 100, cal["mismatch_cells"]))
    elif meta.get("grid_calibration_error"):
        L.append("- 栅格标定：**失败** —— %s" % meta["grid_calibration_error"])
    L.append("")

    L.append("## 0. 一句话结论")
    L.append("")
    L.append("**%s** —— %s" % (verdict["level"], verdict["headline"]))
    L.append("")
    L.append("失败 %d 条、告警 %d 条；详见下方分组。" % (len(fails), len(warns)))
    L.append("")
    L.extend(_chain(results))

    # 1 数据清单
    L.append("## 1. 数据清单（含缺失/空话题归因）")
    L.append("")
    L.append("| 话题 | 类型 | 条数 | 大小 | 角色 |")
    L.append("|---|---|---|---|---|")
    for t in report["inventory"]["topics"]:
        L.append("| `%s` | %s | %d | %s | %s |"
                 % (t["name"], t["type"], t["count"], _size(t["bytes"]), t["role"]))
    L.append("")
    if report["inventory"]["missing_required"]:
        L.append("**缺失的必需话题**：%s" % "、".join("`%s`" % x for x in report["inventory"]["missing_required"]))
        L.append("")
    empties = report["inventory"]["empty_topics"]
    if empties:
        L.append("**空话题（0 条）**：%s" % "、".join("`%s`" % e["name"] for e in empties))
        L.append("")

    # 2 costmap
    L.append("## 2. costmap 诊断")
    L.append("")
    L.append(_table(_by_prefix(results, "C-")))
    L.append("")
    L.extend(_costmap_extra(ctx, results))

    L.append("## 3. 规划器诊断（输入 → 输出）")
    L.append("")
    L.append(_table(_by_prefix(results, "P-")))
    L.append("")

    L.append("## 4. 轨迹体检")
    L.append("")
    L.append(_table(_by_prefix(results, "T-")))
    L.append("")

    L.append("## 5. 执行链与状态机（规划 → 控制 → 执行）")
    L.append("")
    L.append(_table(_by_prefix(results, "D-")))
    L.append("")

    L.append("## 6. 时序与系统")
    L.append("")
    L.append(_table(_by_prefix(results, "S-")))
    L.append("")

    L.append("## 7. 未验证项（这份 bag 证明不了什么）")
    L.append("")
    L.extend(_unverified(ctx, results))
    L.append("")

    L.append("## 8. 建议动作（按优先级）")
    L.append("")
    for i, line in enumerate(_actions(results), 1):
        L.append("%d. %s" % (i, line))
    L.append("")
    L.append("---")
    L.append("")
    L.append("*由 `park_log_analyzer %s` 生成；所有阈值见 `report.json` 的 `meta.config_snapshot`。*"
             % meta["tool_version"])

    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, "report.md")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
    return path


def _size(b: int) -> str:
    if b > 1048576:
        return "%.1f MB" % (b / 1048576)
    if b > 1024:
        return "%.1f KB" % (b / 1024)
    return "%d B" % b


def _chain(results):
    """把互相印证的规则拼成因果链，避免只报现象不给结论。"""
    s06, s05, s04 = _find(results, "S-06"), _find(results, "S-05"), _find(results, "S-04")
    p04, p05, t05, t06 = (_find(results, "P-04"), _find(results, "P-05"),
                          _find(results, "T-05"), _find(results, "T-06"))
    t09, t10 = _find(results, "T-09"), _find(results, "T-10")
    d04, d05 = _find(results, "D-04"), _find(results, "D-05")
    d08, d02 = _find(results, "D-08"), _find(results, "D-02")
    if not (s06 and s06.severity == WARN):
        return []
    L = ["### 0.1 关联观察（自动推断的因果链）", ""]
    n = 0

    def add(text):
        nonlocal n
        n += 1
        L.append("%d. %s" % (n, text))

    plan_ok = []
    if p04 and p04.severity == "PASS":
        plan_ok.append("规划成功率 %s" % p04.summary.split("（")[0].strip())
    if p05 and p05.severity == "PASS":
        plan_ok.append("耗时 p95 %.2f ms" % p05.detail.get("p95", 0.0))
    if t05 and t05.severity == "PASS":
        plan_ok.append("曲率占极限 %.0f%%" % (t05.detail.get("ratio", 0) * 100))
    if t06 and t06.detail.get("min_clearance_m") is not None:
        plan_ok.append("最小余量 %.2f m" % t06.detail["min_clearance_m"])
    add("**规划侧没有明显异常**：%s。" % ("，".join(plan_ok) if plan_ok else "规则未见 FAIL"))
    add("**执行侧没有跟随**：%s。" % s06.summary)

    # 用 D 组规则把"哪一层断了"钉死，而不是笼统地说"执行没跟上"
    if d08 and d08.severity in ("FAIL", "WARN") and d08.detail.get("wrong_direction_samples"):
        add("**挡位与速度反号**：%s；这是接口层油门/挡位符号问题，不是规划或控制参数问题。"
            % d08.summary)
    elif d05 and d05.severity == "WARN" and d05.detail.get("stuck_windows"):
        add("**控制指令已下发但车不动**：%s；瓶颈在执行层（换挡/驻车/驱动），"
            "规划侧可排除。" % d05.summary)
    elif d04 and d04.severity == "WARN":
        add("**链路断在规划→控制**：%s。" % d04.summary)
    elif d02 and d02.severity == "WARN":
        add("**车根本没进入自动驾驶模式**：%s；先解决模式，不要调泊车参数。" % d02.summary)

    if t09 and t09.severity == "WARN":
        add("**规划输出被冻结**：%s，说明每周期都在算同一条第一段轨迹，而非重规划震荡。"
            % t09.summary)
    if s05 and s05.severity == "WARN":
        names = [x["name"] for x in s05.detail.get("top_not_ok", [])]
        ctrl = [n for n in names if "control" in n.lower()]
        add("**控制指令侧报错**：diagnostics 中 %s；`control_command` 类监控为 ERROR 表示"
            "下游没有收到控制指令。" % ("、".join("`%s`" % n for n in ctrl[:3]) if ctrl
                                         else "存在非 OK 状态"))
    if s04 and s04.severity == "WARN":
        msgs = [x["msg"] for x in s04.detail.get("top_errors", [])][:2]
        add("**模式/门控**：rosout 顶部错误 %s；若同时存在 `/tf` 缺失或变换失败，"
            "应先判断为自动驾驶模式未进入或输入不完整，而不是泊车算法本身失败。"
            % ("、".join("“%s”" % m for m in msgs) if msgs else "见 §6"))
    if t10 and t10.detail.get("partial"):
        add("**注意**：`parking/trajectory` 只发分段的第一段（%d 点 / %.2f m），"
            "不能据此判断全程可达性。" % (t10.detail["n_points"], t10.detail["arclength_m"]))
    L.append("")
    return L


def _costmap_extra(ctx, results):
    out = []
    c02 = _find(results, "C-02")
    if c02 and "topics" in c02.detail:
        out.append("**频率（按各话题自身跨度）**")
        out.append("")
        out.append("| 话题 | 条数 | 首个 (s) | 末个 (s) | 跨度 (s) | bag 频率 | stamp 频率 | 期望 | 时钟比 |")
        out.append("|---|---|---|---|---|---|---|---|---|")
        for k, v in c02.detail["topics"].items():
            out.append("| %s | %d | %.3f | %.3f | %.3f | %s | %s | %s | %s |"
                       % (k, v["count"], v["first_s"], v["last_s"], v["span_s"],
                          "%.2f" % v["rate_bag_hz"] if v.get("rate_bag_hz") is not None else "-",
                          "%.2f" % v["rate_stamp_hz"] if v.get("rate_stamp_hz") is not None else "-",
                          "%.1f" % v["expected_hz"] if v.get("expected_hz") else "-",
                          "%.2f" % v["clock_scale"] if v.get("clock_scale") else "-"))
        out.append("")
    c05 = _find(results, "C-05")
    if c05:
        out.append("**末帧层活跃度**：%s" % c05.summary)
        out.append("")
    return out


def _unverified(ctx, results):
    out = []
    pts_empty = True
    obj_nonzero = False
    c05 = _find(results, "C-05")
    if c05:
        layers = c05.detail.get("layers", {})
        pts_empty = layers.get("points", {}).get("nonzero", 0) == 0
        obj_nonzero = layers.get("objects", {}).get("nonzero", 0) > 0
    if pts_empty:
        out.append("- **避障能力**：`points` 层全空%s，轨迹是在\"停车场空无一物\"的假设下算出的，"
                   "本报告不构成避障能力证据。" % ("、`objects` 层仅有少量格" if obj_nonzero else ""))
    t09 = _find(results, "T-09")
    if t09 and t09.severity == WARN:
        out.append("- **轨迹推进**：%s。" % t09.summary)
    s01 = _find(results, "S-01")
    if s01 and s01.severity == "SKIP":
        out.append("- **坐标链**：bag 内无 `/tf`、`/tf_static`，无法验证 map→base_link 的变换正确性。")
    p12 = _find(results, "P-12")
    if p12 and p12.detail.get("single_point_stop_frames"):
        out.append("- **停车注入路径**：存在单点停车轨迹，但没有 `is_completed=true` 的完整收尾样本（若适用）。")
    if not any(r.id.startswith("D-") and r.severity != "SKIP" for r in results):
        out.append("- **控制执行**：bag 里没有 `/control/*`、`/api/*` 状态话题，"
                   "无法判断链路断在哪一层；按 `config/bag_topics.yaml` 补录后重跑。")
    else:
        out.append("- **控制执行**：D-* 只判断链路是否贯通、挡位与速度方向是否一致，"
                   "不评价控制律参数（MPC 增益、限幅值等）。")
    return out


_D_IDS = ("D-01", "D-02", "D-03", "D-04", "D-05", "D-06", "D-07", "D-08")


def _actions(results):
    acts = []
    s06, s05, s04 = _find(results, "S-06"), _find(results, "S-05"), _find(results, "S-04")
    if s06 and s06.severity == WARN:
        d = s06.detail
        acts.append("**先查执行链，不是规划**：规划下发 %.2f m/s，实际最高 %.2f m/s、位移 %.2f m。"
                    "依次确认 `/autoware/state` 是否 ENGAGED、`/control/vehicle_cmd_gate/is_start_requested` 与"
                    " `is_filter_activated`、`/control/command/control_cmd` 是否连续发布。"
                    % (d.get("traj_target_v_max", 0.0), d.get("v_max", 0.0), d.get("xy_span_m", 0.0)))
    d02, d03 = _find(results, "D-02"), _find(results, "D-03")
    d04, d05, d08 = _find(results, "D-04"), _find(results, "D-05"), _find(results, "D-08")
    if d02 and d02.severity == WARN:
        acts.append("**先解决操作模式**：%s。模式没进 AUTONOMOUS 时，后面的规划/控制参数怎么调都不会动。"
                    % d02.summary)
    if d08 and d08.severity == "FAIL":
        acts.append("**修接口的油门/挡位符号**：%s。检查 `autoware_carla_interface` 的 "
                    "`throttle=abs(accel_cmd)` 与 `gear=-1（倒挡）` 处理。" % d08.summary)
    if d05 and d05.severity == "WARN":
        acts.append("**查执行层（有指令但车不动）**：%s。依次确认换挡是否完成、"
                    "驻车/手刹、驱动扭矩与 `acceleration` 限幅。" % d05.summary)
    if d04 and d04.severity == "WARN":
        acts.append("**查规划→控制的发布**：%s。确认 `trajectory_follower` 与 "
                    "`vehicle_cmd_gate` 的输入话题名与实际不一致的问题。" % d04.summary)
    if d03 and d03.severity == "WARN":
        acts.append("**查换挡链路**：%s。对比 `/control/command/gear_cmd`、"
                    "`/control/shift_decider/gear_cmd` 与 `/vehicle/status/gear_status`。" % d03.summary)
    if s05 and s05.severity == WARN:
        names = [x["name"] for x in s05.detail.get("top_not_ok", []) if x["level"] != "OK"]
        ctrl = [n for n in names if "control" in n.lower()]
        if ctrl:
            acts.append("**修复控制话题监控 ERROR**：%s；说明控制指令没有到达执行层，"
                        "与 `topic_state_monitor` 的 ERROR 一致。" % "、".join("`%s`" % n for n in ctrl[:3]))
    if s04 and s04.severity == WARN:
        acts.append("**处理 rosout 顶部错误**：%s。"
                    % "；".join("“%s”×%d" % (x["msg"][:60], x["count"])
                               for x in s04.detail.get("top_errors", [])[:2]))
    for r in results:
        if r.severity == FAIL and r.id != "D-08":
            acts.append("**修复** %s（%s）：%s" % (r.id, r.title, r.summary))
    for r in results:
        if r.severity == WARN and r.id not in ("S-04", "S-05", "S-06") + _D_IDS:
            acts.append("**复核** %s（%s）：%s" % (r.id, r.title, r.summary))
    if not acts:
        acts.append("无阻断项；如需回归门禁，把本次 `report.json` 固化进 `baseline/`。")
    acts.append("补齐 `/tf`、`/tf_static` 与点云/有效 `objects` 后重跑，让 S-01、避障与重规划分支可判定。")
    return acts
