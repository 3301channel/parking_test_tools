#!/usr/bin/env python3
"""
泊车结果展示：一次跑完后，在终端直接看到"规划成什么样 + 停得准不准"。

数据来源是 analyze_parking_session.py 生成的 trajectory_info.json，
所以本脚本不连 ROS、不读 bag，纯解析，秒出。

================================ 输出内容 ================================

  【规划结果】  轨迹长度、前进/倒车比例、换向次数、最小转弯半径、终点位置
  【停车精度】  规划终点 vs 实际停车，两列并排——这是本脚本的重点
  【偏差归因】  把总偏差拆成"规划偏差"和"执行偏差"两部分并给建议
  【判定】      按阈值给出 ✔/✘
  【产物】      图片与报告路径，可选直接打开

为什么要把"规划终点"和"实际停车"并排：
  车位里停歪时，可能是规划就没对正，也可能是控制没跟上。
  两者差得越多，说明控制环节吃掉的精度越多，调参方向完全不同：
    规划偏差大 → 调 freespace_planner（theta_size / angle_goal_range）
    执行偏差大 → 调 mpc（extend_trajectory_for_end_yaw_control）

================================ 用法 ================================
  python3 show_parking_result.py                 # 默认看最近一次会话（按目录名取最新）
  python3 show_parking_result.py <会话目录>       # 指定某次
  python3 show_parking_result.py --open          # 同时用系统看图器打开两张图
  python3 show_parking_result.py --list          # 列出最近几次有结果的会话
"""

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

LOG_ROOT = Path.home() / "autoware_logs"

# 判定阈值：线性量单位 m，角度单位 deg。
# 车位 5.0 x 2.5 m，车 4.9 x 1.9 m，余量本就不多，这里是"合格线"不是"理想线"。
THRESHOLDS = {
    "longitudinal": 0.50,
    "lateral": 0.30,
    "angle": 2.0,
}

# 执行偏差超过这个值，就认为控制环节是主要来源
EXEC_BIAS_WARN = 2.0      # deg


# ---------------------------------------------------------------- 排版工具

_ANSI_RE = re.compile(r"\033\[[0-9;]*m")


# 这些符号的码位虽然大于 0x2E80，但终端按 1 列渲染（East Asian Width = Ambiguous），
# 统一按 2 列算会让判定列的 ✔/✘ 比表头多偏 1 列。
_NARROW_SYMBOLS = set("✔✘✓✗×√")


def _dw(s):
    """字符串在终端里的实际显示宽度。

    两件事必须处理，否则判定列对不齐：
    1) 先剥掉 ANSI 颜色码。mark() 返回的是 "\\033[32m✔\\033[0m"，
       共 10 个字符，算进宽度会让 pad 认为已经超宽而不再补空格，
       于是 ✔/✘ 顶格贴左，表头却按 8 列右对齐。
    2) ✔/✘ 这类符号终端只占 1 列，不能按宽字符算。
    """
    s = _ANSI_RE.sub("", str(s))
    w = 0
    for c in s:
        if c in _NARROW_SYMBOLS:
            w += 1
        else:
            w += 2 if ord(c) > 0x2E80 else 1
    return w


def pad(s, width, align="left"):
    """按显示宽度补空格，保证中英混排时表格能对齐。"""
    s = str(s)
    gap = max(0, width - _dw(s))
    if align == "right":
        return " " * gap + s
    if align == "center":
        left = gap // 2
        return " " * left + s + " " * (gap - left)
    return s + " " * gap


class C:
    """ANSI 颜色，非 tty 时自动降级为纯文本。"""
    _on = sys.stdout.isatty()

    @classmethod
    def _c(cls, code, s):
        return f"\033[{code}m{s}\033[0m" if cls._on else str(s)

    @classmethod
    def ok(cls, s):      return cls._c("32", s)
    @classmethod
    def bad(cls, s):     return cls._c("31", s)
    @classmethod
    def warn(cls, s):    return cls._c("33", s)
    @classmethod
    def dim(cls, s):     return cls._c("90", s)
    @classmethod
    def bold(cls, s):    return cls._c("1", s)


def mark(ok):
    return C.ok("✔") if ok else C.bad("✘")


def num(v, fmt="{:.2f}", unit=""):
    """None / 缺失统一显示为 —，避免整表被一个空值打断。"""
    if v is None:
        return "—"
    try:
        # 米这类英文单位前留空格更易读，度数符号则紧贴数字
        sep = "" if unit == "°" else (" " if unit else "")
        return fmt.format(float(v)) + sep + unit
    except (TypeError, ValueError):
        return str(v)


# ---------------------------------------------------------------- 会话定位

def has_result(d):
    return (d / "trajectory_info.json").is_file()


def _session_sort_key(d):
    """会话排序键：目录名是 YYYYMMDD_HHMMSS 时按名字，否则按 mtime。

    用目录名而不是 st_mtime，因为 mtime 会被后续写文件、cp、touch 打乱，
    而目录名就是启动时刻，字典序天然等于时间序。
    """
    ts = d.name.replace("_", "")
    if len(ts) == 14 and ts.isdigit():
        return ts
    return f"{int(d.stat().st_mtime):013d}"


def resolve_session(arg):
    """把命令行参数解析成一个含有 trajectory_info.json 的目录。"""
    if arg:
        p = Path(arg).expanduser().resolve()
        # 允许直接传 bag 目录，自动跳到父目录
        if p.name == "bag" and has_result(p.parent):
            return p.parent
        if has_result(p):
            return p
        print(f"[错误] {p} 下没有 trajectory_info.json", file=sys.stderr)
        print("       先跑一次分析：", file=sys.stderr)
        print(f"       python3 analyze_parking_session.py {p}/bag --out {p}", file=sys.stderr)
        return None

    # 直接按会话目录名取最新，不再读 ~/autoware_logs/latest。原因有两个：
    #   1) latest 只由 start_planning_sim_log.sh / run_parking.sh 更新，
    #      用裸 ros2 launch 跑出来的会话根本不会动它，于是永远停在某个旧会话上；
    #   2) 它还可能指向一个分析失败（没有 trajectory_info.json）的目录。
    # 目录名就是 YYYYMMDD_HHMMSS，字典序即时间序，比 st_mtime 可靠得多
    # （mtime 会被后续写文件、cp、touch 打乱）。
    cands = [d for d in LOG_ROOT.glob("*")
             if d.is_dir() and d.name != "latest" and has_result(d)]
    if not cands:
        return None

    return max(cands, key=_session_sort_key)


def list_sessions(n=10):
    rows = []
    cands = [d for d in LOG_ROOT.glob("*")
             if d.is_dir() and d.name != "latest" and has_result(d)]
    for d in sorted(cands, key=_session_sort_key, reverse=True):
        try:
            info = json.loads((d / "trajectory_info.json").read_text(errors="ignore"))
            fm = info.get("final_metrics") or {}
            rows.append((d.name,
                         fm.get("停车位姿相对车位_角度偏差_deg"),
                         fm.get("停车位姿相对车位_横向_m"),
                         fm.get("停车位姿在车位框内")))
        except Exception:
            rows.append((d.name, None, None, None))
        if len(rows) >= n:
            break
    if not rows:
        print("没有找到已完成的评估结果。")
        return 0
    print(f"{pad('会话', 20)}{pad('角度偏差(°)', 14, 'right')}"
          f"{pad('横向偏差(m)', 14, 'right')}{pad('在框内', 10, 'right')}")
    print("-" * 58)
    for name, ang, lat, inside in rows:
        print(f"{pad(name, 20)}{pad(num(ang), 14, 'right')}"
              f"{pad(num(lat), 14, 'right')}{pad('是' if inside else '否', 10, 'right')}")
    return 0


# ---------------------------------------------------------------- 展示

def show(d):
    info = json.loads((d / "trajectory_info.json").read_text(errors="ignore"))
    m = info.get("metrics") or {}
    fm = info.get("final_metrics") or {}
    slot = info.get("parking_slot") or {}

    print()
    print(C.bold("=" * 68))
    print(C.bold(f" 泊车结果报告   {d.name}"))
    print(C.bold("=" * 68))

    # ---- 车位信息 ----
    if slot:
        c = slot.get("center") or [0, 0]
        yaw = slot.get("yaw")
        print(f" 车位  : 中心 ({c[0]:.2f}, {c[1]:.2f})  "
              f"朝向 {yaw * 57.29578:.1f}°  "
              f"尺寸 {slot.get('length', 0):.1f}x{slot.get('width', 0):.1f} m"
              f"  {C.dim('(' + str(slot.get('source', '?')) + ')')}")

    # ---- 规划结果 ----
    print()
    print(C.bold("【规划结果】"))
    total = m.get("轨迹总长_m") or 0
    fwd = m.get("前进段长_m") or 0
    rev = m.get("倒车段长_m") or 0
    traj_topic = info.get("trajectory_topic", "?")
    traj_frames = info.get("trajectory_frames")
    frames_txt = C.dim(f"{traj_frames} 帧") if traj_frames is not None else ""
    print(f"  轨迹来源 : {traj_topic}  {frames_txt}")
    tp = info.get("trajectory_points") or []
    # 点数优先用 metrics；老 json 缺这个键时按实际点数组长度兜底，避免显示成 "?"
    n_pts = m.get("轨迹点数")
    if n_pts is None:
        n_pts = len(tp) if tp else "?"

    print(f"  轨迹长度 : {total:.2f} m   前进 {fwd:.2f} m / 倒车 {rev:.2f} m")
    print(f"  轨迹点数 : {n_pts}   换向 {m.get('方向切换次数', '?')} 次")

    # 时间轴与横摆角速度：由 velocity_smoother 在发布前填充
    dur = m.get("轨迹总时长_s")
    avg_v = m.get("平均速度_mps")
    if dur is not None:
        tail = f"   平均 {avg_v:.2f} m/s" if avg_v is not None else ""
        print(f"  轨迹时长 : {dur:.2f} s{tail}")
    w_max = m.get("最大横摆角速度_radps")
    w_avg = m.get("平均横摆角速度_radps")
    if w_max is not None:
        print(f"  横摆角速度: 最大 {w_max:.3f} rad/s   平均 {w_avg:.3f} rad/s")

    print(f"  转弯半径 : 最小 {num(m.get('最小转弯半径_m'))} m"
          f"   曲率抖动 {num(m.get('曲率抖动_标准差'), '{:.4f}')}")

    # 填充率：低于 90% 说明规划器没填这两个字段（老版本或旧 bag）
    t_pct = m.get("时间轴填充率_pct")
    w_pct = m.get("角速度填充率_pct")
    if t_pct is not None:
        txt = f"time_from_start {t_pct:.0f}% / heading_rate {w_pct or 0:.0f}%"
        if t_pct >= 90:
            print(f"  字段填充 : {txt}  {C.ok('✔')}")
        else:
            print(f"  字段填充 : {txt}  {C.warn('← 规划器未填充，升级后重录才有效')}")

    if m.get("valid") is not None:
        print(f"  轨迹有效 : {mark(bool(m.get('valid')))}")

    # ---- 轨迹点字段：TrajectoryPoint 的 8 个字段，取轨迹末点 ----
    print()
    print(C.bold("【轨迹点字段】")
          + C.dim(f"  共 {len(tp)} 点 · TrajectoryPoint 8 个字段 · 下表取末点（规划停车点）"))
    last_pt = tp[-1] if tp else None
    if not last_pt:
        print("  " + C.dim("json 里没有轨迹点数据"))
    else:
        def fv(v, fmt="{:.3f}", unit=""):
            """数值格式化；None 一律显示 —，避免一个空值打乱整表。"""
            if v is None:
                return "—"
            try:
                s = fmt.format(float(v))
            except (TypeError, ValueError):
                return "—"
            # -0.000 只是浮点误差（例如 a=-2.7e-4），显示成 0.000 更直观
            if s.startswith("-") and float(s) == 0:
                s = s[1:]
            return f"{s} {unit}" if unit else s

        yaw = last_pt.get("yaw")
        x, y = last_pt.get("x", 0.0), last_pt.get("y", 0.0)
        pose_txt = (f"({x:.2f}, {y:.2f}, {yaw * 57.29578:.1f}°)"
                    if yaw is not None else f"({x:.2f}, {y:.2f})")

        rows8 = [
            ("time_from_start", fv(last_pt.get("t"), "{:.3f}", "s"),
             "相对 header.stamp 的时间偏移，用于时序对齐"),
            ("pose", pose_txt,
             "map 系目标位姿：position(x, y) + orientation(yaw)"),
            ("longitudinal_velocity_mps", fv(last_pt.get("v"), "{:.3f}", "m/s"),
             "目标纵向速度，控制侧最核心的跟踪量"),
            ("lateral_velocity_mps", fv(last_pt.get("v_lat"), "{:.3f}", "m/s"),
             "目标横向速度，正常行驶为 0，仅泊车等场景非零"),
            ("acceleration_mps2", fv(last_pt.get("a"), "{:.3f}", "m/s²"),
             "纵向加速度，按绝对运动方向增减，不区分前进/倒车"),
            ("heading_rate_rps", fv(last_pt.get("w"), "{:.3f}", "rad/s"),
             "目标横摆角速度，作为横向控制前馈"),
            ("front_wheel_angle_rad", fv(last_pt.get("steer_f"), "{:.4f}", "rad"),
             "前轮转角，MPC 与纯跟踪的前馈量"),
            ("rear_wheel_angle_rad", fv(last_pt.get("steer_r"), "{:.4f}", "rad"),
             "后轮转角，普通乘用车为 0，后轮转向车型使用"),
        ]
        # 值列 28 列：pose 写成 (x, y, yaw°) 大约 26 宽，留余量免得坐标位数一多就溢出
        print("  " + C.dim(pad("字段", 26) + pad("末点值", 28, "right") + "  含义"))
        print(C.dim("  " + "-" * 26 + "  " + "-" * 28 + "  " + "-" * 44))
        for name, val, desc in rows8:
            print("  " + pad(name, 26) + pad(val, 28, "right") + "  " + C.dim(desc))

        if last_pt.get("v_lat") is None:
            print("  " + C.warn("← 这份 json 由旧版分析生成，缺横向速度/加速度/前后轮转角"))
            print("  " + C.warn("   重新分析可补全："
                               "python3 analyze_parking_session.py <bag> --out <dir>"))

    # ---- 精度对比（核心） ----
    print()
    print(C.bold("【停车精度】"))
    rows = [
        ("纵向偏差", "终点相对车位中心_纵向_m", "停车位姿相对车位_纵向_m", "longitudinal", "m"),
        ("横向偏差", "终点相对车位中心_横向_m", "停车位姿相对车位_横向_m", "lateral", "m"),
        ("角度偏差", "终点相对车位角度偏差_deg", "停车位姿相对车位_角度偏差_deg", "angle", "°"),
    ]
    # 判定列用居中：✔/✘ 只有 1 列宽，右对齐会贴在"判定"两个字的右边缘，
    # 看起来像没对齐；居中后正好落在"判定"下方。
    hdr = (pad("  指标", 12) + pad("规划终点", 12, "right")
           + pad("实际停车", 12, "right") + pad("阈值", 10, "right")
           + pad("判定", 8, "center"))
    print(C.dim(hdr))
    print(C.dim("  " + "-" * 52))

    details = {}
    for label, k_plan, k_real, th_key, unit in rows:
        plan_v = m.get(k_plan)
        real_v = fm.get(k_real)
        lim = THRESHOLDS[th_key]
        # 角度是带符号的偏转，比较时取绝对值
        passed = real_v is not None and abs(float(real_v)) <= lim
        details[th_key] = (plan_v, real_v)
        print("  " + pad(label, 10)
              + pad(num(plan_v, unit=unit), 12, "right")
              + pad(num(real_v, unit=unit), 12, "right")
              + pad(f"±{lim:g}", 10, "right")
              + pad(mark(passed), 8, "center"))

    inside = fm.get("停车位姿在车位框内")
    plan_inside = m.get("轨迹终点在车位框内")
    print("  " + pad("在车位框内", 10)
          + pad("是" if plan_inside else "否", 12, "right")
          + pad("是" if inside else "否", 12, "right")
          + pad("是", 10, "right")
          + pad(mark(bool(inside)), 8, "center"))

    # ---- 偏差归因 ----
    plan_ang = details["angle"][0]
    real_ang = details["angle"][1]
    print()
    print(C.bold("【偏差归因】"))
    if plan_ang is None or real_ang is None:
        print("  " + C.dim("数据不足，无法拆分"))
    else:
        exec_ang = abs(float(real_ang)) - abs(float(plan_ang))
        print(f"  规划偏差 {abs(float(plan_ang)):.2f}° + 执行偏差 {exec_ang:+.2f}°"
              f" = 实际 {abs(float(real_ang)):.2f}°")

        if exec_ang > EXEC_BIAS_WARN:
            print("  " + C.warn("执行偏差偏大 —— 主要是控制没跟上规划，而不是规划算歪。"))
            print("     建议：mpc.param.yaml 里把")
            print("           extend_trajectory_for_end_yaw_control 改为 true")
        elif abs(float(plan_ang)) > THRESHOLDS["angle"]:
            print("  " + C.warn("规划偏差本身就偏大 —— A* 给出的终点角度就没对正。"))
            print("     建议：freespace_planner.param.yaml 里提高 theta_size")
            print("           （120 → 240，角度分辨率 3° → 1.5°）")
        else:
            print("  " + C.ok("规划与执行都在合理范围。"))

    # ---- 产物 ----
    print()
    print(C.bold("【产物】"))
    for name, desc in (("trajectory_plot.png", "车位局部轨迹图（车道线+成本地图+车辆轮廓）"),
                       ("plot_global.png", "整段行驶路线全局图"),
                       ("evaluation.md", "完整评估报告"),
                       ("timeseries_plotjuggler.csv", "时序数据（可直接拖进 PlotJuggler）")):
        f = d / name
        if f.is_file():
            size = f.stat().st_size / 1024
            print(f"  {pad(name, 30)}{pad(f'{size:.0f} KB', 10, 'right')}  {C.dim(desc)}")
    print(f"  {C.dim('目录: ' + str(d))}")
    print()
    return d


def open_images(d):
    images = [d / "trajectory_plot.png", d / "plot_global.png"]
    images = [p for p in images if p.is_file()]
    if not images:
        print("[提示] 没有可打开的图片", file=sys.stderr)
        return
    opener = None
    for cmd in ("xdg-open", "eog", "feh", "display"):
        if subprocess.run(["which", cmd], capture_output=True).returncode == 0:
            opener = cmd
            break
    if not opener:
        print("[提示] 没找到图片查看器，手动打开：")
        for p in images:
            print(f"       {p}")
        return
    for p in images:
        subprocess.Popen([opener, str(p)],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"[已打开] {len(images)} 张图（{opener}）")


def main():
    ap = argparse.ArgumentParser(
        description="展示泊车规划结果与评估结果（读取已生成的 trajectory_info.json）")
    ap.add_argument("session", nargs="?", default=None,
                    help="会话目录，省略时取 ~/autoware_logs 下最新的一次会话")
    ap.add_argument("--open", action="store_true", help="用系统看图器打开轨迹图")
    ap.add_argument("--list", action="store_true", help="列出最近有结果的会话")
    args = ap.parse_args()

    if args.list:
        return list_sessions()

    d = resolve_session(args.session)
    if d is None:
        return 1

    show(d)
    if args.open:
        open_images(d)
    return 0


if __name__ == "__main__":
    sys.exit(main())
