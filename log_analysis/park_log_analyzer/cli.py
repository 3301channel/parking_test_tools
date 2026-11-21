"""唯一入口：argparse 子命令（开发文档 §5.3）。"""

from __future__ import annotations

import argparse
import json
import os
import sys

from . import __version__
from .checks import CHECKS, exit_code, run_checks, worst
from .checks.base import FAIL, WARN
from .context import AnalysisContext
from .report import json_report, md_report, svg_plot
from .source import open_bag
from . import tables

DEFAULT_CFG = os.path.join(os.path.dirname(__file__), "config", "default.yaml")


def load_config(path: str | None) -> dict:
    path = path or DEFAULT_CFG
    if not os.path.exists(path):
        alt = path.rsplit(".", 1)[0] + ".json"
        if os.path.exists(alt):
            path = alt
        else:
            raise SystemExit("配置文件不存在：%s" % path)
    if path.endswith((".yaml", ".yml")):
        try:
            import yaml

            with open(path, encoding="utf-8") as f:
                cfg = yaml.safe_load(f)
        except ImportError:
            alt = path.rsplit(".", 1)[0] + ".json"
            print("[warn] PyYAML 不可用，回落到 %s" % alt, file=sys.stderr)
            with open(alt, encoding="utf-8") as f:
                cfg = json.load(f)
    else:
        with open(path, encoding="utf-8") as f:
            cfg = json.load(f)
    cfg.setdefault("topics", {})
    cfg.setdefault("thresholds", {})
    cfg.setdefault("vehicle", {})
    cfg.setdefault("checks", {})
    return cfg


def _headline(ctx, results) -> str:
    from .checks.base import _find_result

    fails = [r for r in results if r.severity == FAIL]
    warns = [r for r in results if r.severity == WARN]
    s06, t09, t06 = _find_result(results, "S-06"), _find_result(results, "T-09"), _find_result(results, "T-06")
    s04, s05 = _find_result(results, "S-04"), _find_result(results, "S-05")

    if fails:
        head = "存在 %d 项阻断问题：%s" % (len(fails), "；".join("%s %s" % (r.id, r.title) for r in fails[:3]))
    elif t09 and t09.severity == WARN and s06 and s06.severity == WARN:
        d = s06.detail
        head = ("泊车未完成：车辆几乎未运动（%.2f m / %.0f s，最高 %.2f m/s，轨迹目标 %.2f m/s），"
                "%d 帧轨迹完全相同" % (d["xy_span_m"], ctx.bag_duration, d["v_max"],
                                       d.get("traj_target_v_max", 0.0),
                                       t09.detail.get("max_identical_run", 0)))
    else:
        head = "%d 项需复核：%s" % (len(warns), "；".join("%s %s" % (r.id, r.title) for r in warns[:3])) \
            if warns else "全部规则通过"
    extra = []
    if s04 and s04.detail.get("by_level", {}).get("ERROR"):
        extra.append("rosout ERROR %d 条" % s04.detail["by_level"]["ERROR"])
    if s05 and s05.detail.get("by_level"):
        n = sum(v for k, v in s05.detail["by_level"].items() if k != "OK")
        if n:
            extra.append("diagnostics 非 OK %d 条" % n)
    if t06 and t06.detail.get("min_clearance_m") is not None:
        extra.append("最小余量 %.2f m" % t06.detail["min_clearance_m"])
    # 执行链结论：直接告诉人"断在哪一层"，比只报现象有用。
    # 有 FAIL 时 head 只列 FAIL，这里补 WARN 级的 D 结论；否则 head 已含 WARN，不重复。
    if fails:
        for cid in ("D-02", "D-08", "D-05", "D-04"):
            r = _find_result(results, cid)
            if r is not None and r.severity == WARN:
                extra.append("%s %s" % (r.id, r.title))
    return head + ("（%s）" % "，".join(extra) if extra else "")


def cmd_inspect(args) -> int:
    reader = open_bag(args.bag)
    topics = reader.topics()
    total = sum(m.count for m in topics.values())
    nbytes = sum(m.nbytes for m in topics.values())
    print("bag: %s" % getattr(reader, "path", args.bag))
    print("topics: %d, messages: %d, payload: %.2f MB, duration: %.2f s"
          % (len(topics), total, nbytes / 1048576, (reader.end_ns() - reader.start_ns()) * 1e-9))
    print()
    print("%-72s %-46s %8s %12s" % ("topic", "type", "count", "bytes"))
    for name, m in sorted(topics.items()):
        print("%-72s %-46s %8d %12d" % (name, m.type, m.count, m.nbytes))
    return 0


def _build_ctx(args):
    cfg = load_config(args.config)
    reader = open_bag(args.bag)
    ctx = AnalysisContext(reader, cfg, keep_all_grids=getattr(args, "keep_all_grids", False))
    return ctx


def cmd_decode(args) -> int:
    ctx = _build_ctx(args)
    outdir = args.out or os.path.join(os.path.dirname(os.path.abspath(args.bag)), "extracted")
    tables.write_tables(ctx, outdir)
    print("tables -> %s" % os.path.join(outdir, "tables"))
    return 0


def cmd_analyze(args) -> int:
    ctx = _build_ctx(args)
    outdir = args.out or "report"
    only = args.only.split(",") if args.only else None
    results = run_checks(ctx, only)
    level = worst(results)
    report = json_report.build(ctx, results, level, _headline(ctx, results))
    if args.tables:
        tables.write_tables(ctx, outdir)
    json_path = json_report.write(report, outdir)
    md_path = md_report.build(ctx, report, results, outdir)
    plots = {}
    if not args.no_plots:
        plots = svg_plot.write_plots(ctx, outdir)

    n_fail = sum(1 for r in results if r.severity == FAIL)
    n_warn = sum(1 for r in results if r.severity == WARN)
    n_skip = sum(1 for r in results if r.severity == "SKIP")
    print("%-6s %-8s %s" % ("ID", "LEVEL", "SUMMARY"))
    for r in results:
        print("%-6s %-8s %s" % (r.id, r.severity, r.summary))
    print()
    print("verdict: %s（FAIL %d / WARN %d / SKIP %d）" % (level, n_fail, n_warn, n_skip))
    print("report: %s" % json_path)
    print("        %s" % md_path)
    for k, v in plots.items():
        if v:
            print("plot:   %s" % v)
    for w in ctx.warnings:
        print("[warn] %s" % w, file=sys.stderr)
    return exit_code(results)


def cmd_timeline(args) -> int:
    """把状态机 / 控制链的关键翻转压成一条时间线。

    现象（车停着、车往反方向冲、AUTO 点不动）先在这里定位到时刻，
    再回 report.md 看对应规则的解释。
    """
    from .context import transitions

    ctx = _build_ctx(args)
    rows = []
    for what, label in (("aw", "状态机"), ("operation", "操作模式"),
                        ("motion", "运动"), ("routing", "路由")):
        for t in ctx.states(what):
            rows.append((t["t_s"], label, t["name"]))

    ctx.load("gear")
    for t in transitions(ctx.gear_status, "gear_name"):
        rows.append((t["t_s"], "档位", str(t["name"])))
    ctx.load("shift")
    for t in transitions(ctx.shift_gear, "gear_name"):
        rows.append((t["t_s"], "换挡指令", str(t["name"])))
    ctx.load("gate")
    for t in transitions(ctx.is_stopped, "value"):
        rows.append((t["t_s"], "stopped", "STOPPED" if t["value"] else "RELEASED"))
    ctx.load("scenario")
    for a, b in ctx.parking_windows():
        rows.append((round(a, 3), "场景", "Parking 开始"))
        rows.append((round(b, 3), "场景", "Parking 结束"))

    # 链路里程碑：用来判断"哪一层先断"
    ctx.load("final_traj")
    if ctx.final_traj:
        rows.append((round(ctx.final_traj[0].t_bag, 3), "里程碑", "首个 /planning/trajectory"))
    ctx.load("control")
    if ctx.control_cmd:
        rows.append((round(ctx.control_cmd[0]["t_bag"], 3), "里程碑", "首条 control_cmd"))
        eps = ctx.threshold("cmd_speed_eps")
        for c in ctx.control_cmd:
            if abs(c["velocity"]) > eps:
                rows.append((round(c["t_bag"], 3), "里程碑",
                             "首条非零 control_cmd（%.2f m/s）" % c["velocity"]))
                break
    if ctx.actuation:
        rows.append((round(ctx.actuation[0]["t_bag"], 3), "里程碑", "首条 actuation_cmd"))
    ctx.load("completed")
    for p in ctx.completed:
        if p.value > 0.5:
            rows.append((round(p.t_bag, 3), "里程碑", "is_completed=true"))
            break

    rows.sort(key=lambda r: (r[0], r[1]))
    print("bag: %s  时长 %.2f s  事件 %d 条" % (getattr(ctx.reader, "path", args.bag),
                                              ctx.bag_duration, len(rows)))
    print()
    print("%9s  %-10s %s" % ("t(s)", "类别", "事件"))
    for t_s, label, text in rows:
        print("%9.3f  %-10s %s" % (t_s, label, text))
    for w in ctx.warnings:
        print("[warn] %s" % w, file=sys.stderr)
    return 0


def cmd_compare(args) -> int:
    with open(args.report, encoding="utf-8") as f:
        cur = json.load(f)
    with open(args.baseline, encoding="utf-8") as f:
        base = json.load(f)
    diffs = []
    bmap = {r["id"]: r for r in base.get("results", [])}
    for r in cur.get("results", []):
        b = bmap.get(r["id"])
        if b is None:
            diffs.append(("new", r["id"], r["severity"], None))
            continue
        if b["severity"] != r["severity"]:
            diffs.append(("severity", r["id"], r["severity"], b["severity"]))
    for d in diffs:
        print("%-9s %-6s %s (baseline=%s)" % d)
    if any(d[0] == "severity" and d[2] == FAIL for d in diffs):
        return 1
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="park_log_analyzer",
                                 description="Autoware 泊车日志分析（只读 rosbag，输出诊断报告）")
    ap.add_argument("--version", action="version", version=__version__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--bag", required=True, help="bag 目录或 .mcap/.db3 文件")
        p.add_argument("--config", default=None, help="阈值/话题配置（yaml 或 json）")

    p = sub.add_parser("inspect", help="列出话题清单（不解析 payload）")
    common(p)
    p.set_defaults(func=cmd_inspect)

    p = sub.add_parser("decode", help="解码为规范化 CSV")
    common(p)
    p.add_argument("--out", default=None)
    p.add_argument("--keep-all-grids", action="store_true")
    p.set_defaults(func=cmd_decode)

    p = sub.add_parser("analyze", help="全量分析，出 json + md + svg")
    common(p)
    p.add_argument("--out", default="report")
    p.add_argument("--only", default=None, help="只跑指定规则，如 C-*,T-01")
    p.add_argument("--tables", action="store_true", help="同时导出 tables/*.csv")
    p.add_argument("--no-plots", action="store_true")
    p.add_argument("--keep-all-grids", action="store_true")
    p.set_defaults(func=cmd_analyze)

    p = sub.add_parser("timeline", help="状态机/控制链关键翻转的时间线")
    common(p)
    p.set_defaults(func=cmd_timeline)

    p = sub.add_parser("compare", help="与基线比对")
    p.add_argument("--report", required=True)
    p.add_argument("--baseline", required=True)
    p.set_defaults(func=cmd_compare)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
