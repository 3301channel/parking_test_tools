#!/usr/bin/env python3
"""
实时检查泊车规划依赖的 TF 链状态。

================================ 为什么需要它 ================================

freespace_planner 的 A* 只在 costmap 坐标系里搜索，车和 Goal 都要靠 TF 转进去：

    current_pose_in_costmap = transform_pose(odom.pose, getTransform(costmap, odom))
    goal_pose_in_costmap    = transform_pose(goal.pose, getTransform(costmap, map))
    algo_->makePlan(current_pose_in_costmap, goal_pose_in_costmap)

而 getTransform() 失败时**不返回错误标志**，只是把默认构造的 TransformStamped
（位置全 0、四元数 w=0）原样返回。于是 A* 拿着原点附近的假位姿去搜索，
最后只报一句 "Can't find goal"，真正的原因被埋在上面那行 RCLCPP_ERROR 里。

本脚本直接盯 TF 本身，把断链、延迟、频率不足提前暴露出来。

================================ 不写死链路结构 ================================

不同配置的 TF 树并不一样，实测 planning_simulator 是**直连**的：

    map → base_link → sensor_kit_base_link → 各传感器

并没有 odom 这一层。所以本脚本不预设 "map → odom → base_link"，
而是从 TF 树里**回溯 base_link 的父链**，实际是什么就查什么。

================================ 用法 ================================

  python3 tf_monitor.py                    # 实时刷新（默认 2 Hz）
  python3 tf_monitor.py --once             # 采样 2 秒后输出一次
  python3 tf_monitor.py --line             # 每次一行，适合重定向到日志
  python3 tf_monitor.py --leaf base_footprint   # 换叶节点名
  python3 tf_monitor.py --chain map:odom   # 额外检查一条指定链路

退出码：0 = 核心链路正常，1 = 断开
"""

import argparse
import os
import re
import sys
import time
from collections import deque
from pathlib import Path


def _prepare_dds_env():
    """让本进程和 Autoware 用同一套 DDS —— 必须在 import rclpy 之前完成。

    RMW 实现是在 import 时确定的，之后再改环境变量无效。
    两边 RMW 不同（如本进程 fastrtps、Autoware cyclonedds）时，
    互相之间**任何话题都收不到**，现象就是"脚本跑起来了但一个 TF 都没有"。
    """
    notes = []

    rmw = os.environ.get("RMW_IMPLEMENTATION")
    if not rmw:
        os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
        notes.append("RMW_IMPLEMENTATION 未设置 → 已设为 rmw_cyclonedds_cpp（与 Autoware 一致）")
    elif "cyclonedds" not in rmw:
        notes.append(f"当前 RMW={rmw}，而 Autoware 一般用 cyclonedds；"
                     "两边不一致时会收不到任何话题")

    if not os.environ.get("CYCLONEDDS_URI") and \
            "cyclonedds" in os.environ.get("RMW_IMPLEMENTATION", ""):
        for cand in (Path.home() / "zzz/parking/parking_test_tools/config/cyclonedds.xml",
                     Path.home() / "autoware/docker/files/cyclonedds.xml"):
            if cand.is_file():
                os.environ["CYCLONEDDS_URI"] = f"file://{cand}"
                notes.append(f"CYCLONEDDS_URI 未设置 → 已启用 {cand}")
                break

    return notes


# 必须放在 import rclpy 之前
_DDS_NOTES = _prepare_dds_env()

try:
    import rclpy
    from rclpy.node import Node
    import tf2_ros
except ImportError:
    print("[错误] 需要先 source ROS 与 Autoware 环境：", file=sys.stderr)
    print("       source ~/autoware/install/setup.bash", file=sys.stderr)
    sys.exit(2)


# 核心终点：规划真正需要的那个 frame
DEFAULT_LEAF = "base_link"
# 期望的树根；不强制，只用于判断"连到了 map 之外"
EXPECTED_ROOT = "map"

RATE_WARN = 5.0          # Hz，低于此值认为数据不新鲜
LATENCY_WARN = 100.0     # ms
RATE_WINDOW = 2.0        # s，频率统计窗口
TREE_REFRESH = 1.0       # s，TF 树结构刷新间隔（树为空时不受此限制）


# ---------------------------------------------------------------- 显示工具

def _dw(s):
    return sum(2 if ord(c) > 0x2E80 else 1 for c in str(s))


def pad(s, width, align="left"):
    s = str(s)
    gap = max(0, width - _dw(s))
    if align == "right":
        return " " * gap + s
    if align == "center":
        left = gap // 2
        return " " * left + s + " " * (gap - left)
    return s + " " * gap


class C:
    _on = sys.stdout.isatty()

    @classmethod
    def _c(cls, code, s):
        return f"\033[{code}m{s}\033[0m" if cls._on else str(s)

    @classmethod
    def ok(cls, s):   return cls._c("32", s)
    @classmethod
    def bad(cls, s):  return cls._c("31", s)
    @classmethod
    def warn(cls, s): return cls._c("33", s)
    @classmethod
    def dim(cls, s):  return cls._c("90", s)
    @classmethod
    def bold(cls, s): return cls._c("1", s)


# ---------------------------------------------------------------- 单链路统计

class ChainStat:
    def __init__(self):
        self.ok = 0
        self.fail = 0
        self.state = "INIT"
        self.err = ""
        self.latency = deque(maxlen=20)
        self.last_stamp = None
        self.updates = deque(maxlen=300)

    def record_ok(self, latency_ms, stamp_ns):
        self.ok += 1
        self.state = "OK"
        self.err = ""
        self.latency.append(latency_ms)
        now = time.time()
        # stamp 变了才算链路真的在更新（缓冲区命中不算）
        if self.last_stamp is None or stamp_ns != self.last_stamp:
            self.updates.append(now)
            self.last_stamp = stamp_ns
        while self.updates and now - self.updates[0] > RATE_WINDOW:
            self.updates.popleft()

    def record_fail(self, state, err):
        self.fail += 1
        self.state = state
        self.err = err

    @property
    def rate(self):
        if not self.updates:
            return 0.0
        now = time.time()
        recent = [t for t in self.updates if now - t <= RATE_WINDOW]
        span = min(RATE_WINDOW, max(1e-6, now - self.updates[0]))
        return len(recent) / span

    @property
    def avg_latency(self):
        return sum(self.latency) / len(self.latency) if self.latency else None

    @property
    def success_pct(self):
        n = self.ok + self.fail
        return 100.0 * self.ok / n if n else 0.0


class TfMonitor(Node):
    def __init__(self, leaf=DEFAULT_LEAF, extra_chains=()):
        super().__init__("tf_monitor")
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.leaf = leaf
        self.extra = list(extra_chains)

        self.tree = {}                 # {frame: parent}
        self.frames = set()
        self._tree_t = 0.0

        self.stats = {}
        self.chain = []                # [(parent, child), ...] 从根到叶

    # ---- TF 树 ----

    def refresh_tree(self, force=False):
        now = time.time()
        # tree 为空时必须每次都试：启动初期 TF 还没到，
        # 若被节流卡住，会一直认为"没有 frame"而彻底不探测。
        if not force and self.tree and now - self._tree_t < TREE_REFRESH:
            return
        self._tree_t = now
        try:
            text = self.buffer.all_frames_as_yaml()
        except Exception:
            return
        pairs = re.findall(r"^\s*(\S+):\s*\n\s+parent:\s*'?([^'\n]*)", text, re.M)
        self.tree = {f: p for f, p in pairs if p and p != f}
        self.frames = set(self.tree) | set(self.tree.values())

    def discover_chain(self):
        """从 leaf 回溯到根，返回从根到叶的段列表。

        不假设中间有 odom：实测 planning_simulator 是 map → base_link 直连。
        """
        chain, cur, seen = [], self.leaf, set()
        while cur in self.tree and cur not in seen:
            seen.add(cur)
            parent = self.tree[cur]
            if not parent:
                break
            chain.append((parent, cur))
            cur = parent
        chain.reverse()
        return chain

    def rebuild_chains(self, force=False):
        self.refresh_tree(force=force)
        chain = self.discover_chain()
        wanted = list(chain)
        for pair in self.extra:
            if pair not in wanted:
                wanted.append(pair)
        # 新增的段才建统计对象，已有的保留历史
        for pair in wanted:
            if pair not in self.stats:
                self.stats[pair] = ChainStat()
        self.chain = wanted

    # ---- 探测 ----

    def probe(self, parent, child):
        stat = self.stats.setdefault((parent, child), ChainStat())
        try:
            t0 = time.perf_counter()
            # Time() == Time(0) 取最新可用，不做时间同步
            tf = self.buffer.lookup_transform(parent, child, rclpy.time.Time())
            latency = (time.perf_counter() - t0) * 1000.0
            stamp_ns = rclpy.time.Time.from_msg(tf.header.stamp).nanoseconds
            stat.record_ok(latency, stamp_ns)
        except tf2_ros.ExtrapolationException as e:
            stat.record_fail("EXTRAP", str(e)[:70])
        except tf2_ros.ConnectivityException as e:
            stat.record_fail("DISCONN", str(e)[:70])
        except tf2_ros.LookupException as e:
            stat.record_fail("MISSING", str(e)[:70])
        except Exception as e:
            stat.record_fail("ERROR", f"{type(e).__name__}: {str(e)[:60]}")

    def probe_all(self):
        self.rebuild_chains()
        for pair in self.chain:
            self.probe(*pair)

    # ---- 诊断 ----

    def leaf_missing(self):
        return self.leaf not in self.frames

    def actual_root(self):
        """leaf 所在树的根（沿父链走到底）。"""
        cur, seen = self.leaf, set()
        while cur in self.tree and cur not in seen:
            seen.add(cur)
            cur = self.tree[cur]
        return cur

    def children_of(self, frame):
        return [f for f, p in self.tree.items() if p == frame]


# ---------------------------------------------------------------- 渲染

def _state_cell(stat):
    if stat.state == "OK":
        if stat.rate and stat.rate < RATE_WARN:
            return C.warn("偏慢"), f"{stat.rate:.1f} Hz 过低"
        return C.ok("正常"), ""
    if stat.state == "INIT":
        return C.dim("等待"), "还没收到 TF"
    if stat.state == "MISSING":
        return C.bad("缺 frame"), "该 frame 未发布"
    if stat.state == "DISCONN":
        return C.bad("断开"), "TF 树不连通"
    if stat.state == "EXTRAP":
        return C.bad("外推"), "时间戳不连续"
    return C.bad(stat.state), ""


def render(node):
    lines = []
    lines.append(C.bold("=" * 78))
    lines.append(C.bold(f" TF 状态监控   {time.strftime('%H:%M:%S')}"
                        f"   叶节点 {node.leaf}"))
    lines.append(C.bold("=" * 78))

    # 一个 frame 都没有 → 不是 TF 的问题，是根本没收到消息（DDS 隔离）
    if not node.frames:
        lines.append(" " + C.bad("结论：一个 TF 都没收到，DDS 大概率没连通。"))
        lines.append(" " + C.dim(f"      本进程 RMW={os.environ.get('RMW_IMPLEMENTATION')}"
                                 f"  ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}"))
        lines.append(" " + C.dim("      Autoware 用 cyclonedds 时，本脚本也必须是 cyclonedds，"))
        lines.append(" " + C.dim("      否则两边是两套互不可见的 DDS，任何话题都收不到。"))
        lines.append(" " + C.dim("      手动对齐：export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"))
        lines.append(" " + C.dim("      并 source ~/autoware/install/setup.bash"))
        return "\n".join(lines), False

    if node.leaf_missing():
        lines.append(" " + C.bad(f"结论：TF 树里找不到 {node.leaf}。"))
        lines.append(" " + C.dim("      车辆模型可能没启动，或 frame 名不同（如 base_footprint）。"))
        lines.append(" " + C.dim("      已知 frame 共 "
                                 f"{len(node.frames)} 个，用 --leaf 指定实际的叶节点。"))
        if node.frames:
            sample = ", ".join(sorted(node.frames)[:8])
            lines.append(" " + C.dim(f"      示例：{sample} ..."))
        return "\n".join(lines), False

    if not node.chain:
        # base_link 存在但没有任何父节点 —— 它自己就是树的根。
        # 这正是"定位还没起来"的特征：map 这一层还没发布。
        if node.leaf not in node.tree:
            lines.append(" " + C.bad(f"结论：{node.leaf} 就是 TF 树的根，"
                                     f"上面没有 {EXPECTED_ROOT}。"))
            lines.append(" " + C.dim(f"      说明还没有 {EXPECTED_ROOT} → {node.leaf} 这条变换，"
                                     "规划拿不到全局位姿。"))
            lines.append(" " + C.dim("      操作：RViz → 2D Pose Estimate，点车所在位置并拖出朝向"))
            lines.append(" " + C.dim("      或：ros2 topic pub --once /initialpose "
                                     "geometry_msgs/msg/PoseWithCovarianceStamped \\"))
            lines.append(" " + C.dim("          \"{header: {frame_id: map}, pose: {pose: "
                                     "{position: {x: 3715.0, y: 73728.0}, "
                                     "orientation: {z: 0.47, w: 0.88}}}}\""))
        else:
            lines.append(" " + C.warn("等待 TF 树成形..."))
        return "\n".join(lines), False

    lines.append(C.dim(" " + pad("链路", 30) + pad("状态", 10)
                       + pad("延迟", 10, "right") + pad("频率", 11, "right")
                       + pad("成功率", 9, "right") + "  备注"))

    all_ok = True
    for pair in node.chain:
        parent, child = pair
        stat = node.stats.get(pair, ChainStat())
        txt, note = _state_cell(stat)

        lat = stat.avg_latency
        lat_s = f"{lat:.1f} ms" if lat is not None else "—"
        if lat is not None and lat > LATENCY_WARN:
            lat_s = C.warn(lat_s)

        rate = stat.rate
        rate_s = f"{rate:.1f} Hz" if rate > 0 else "—"

        n = stat.ok + stat.fail
        pct_s = f"{stat.success_pct:.0f}%" if n else "—"
        if n and stat.success_pct < 90:
            pct_s = C.bad(pct_s)

        if not note:
            note = "规划实际使用" if child == node.leaf else ""
        if stat.state != "OK":
            all_ok = False

        lines.append(" " + pad(f"{parent} → {child}", 30) + pad(txt, 10)
                     + pad(lat_s, 10, "right") + pad(rate_s, 11, "right")
                     + pad(pct_s, 9, "right") + "  " + C.dim(note))

    lines.append(C.dim(" " + "-" * 76))

    # ---- 树结构 ----
    root = node.actual_root()
    kids = node.children_of(node.leaf)
    chain_txt = " → ".join([node.chain[0][0]] + [c for _, c in node.chain])
    if kids:
        chain_txt += f" → ({len(kids)} 个静态帧)"
    lines.append(" " + C.dim(f"TF 树：{chain_txt}"))
    lines.append(C.dim(" " + "-" * 76))

    # ---- 结论 ----
    if all_ok:
        if root != EXPECTED_ROOT:
            lines.append(" " + C.warn(f"结论：链路通，但根节点是 {root} 而不是 {EXPECTED_ROOT}。"))
            lines.append(" " + C.dim("      若 costmap 在 map 系下，A* 仍会因跨树拿不到变换。"))
        else:
            lines.append(" " + C.ok("结论：TF 链路完整，A* 可以拿到正确的起点和终点。"))
    else:
        bad = [p for p in node.chain if node.stats.get(p) and node.stats[p].state != "OK"]
        if root != EXPECTED_ROOT:
            lines.append(" " + C.bad(f"结论：{node.leaf} 挂在 {root} 下，不在 {EXPECTED_ROOT} 下。"))
            lines.append(" " + C.warn("      两棵树不连通，freespace_planner 的 getTransform 会失败。"))
            lines.append(" " + C.dim("      仿真里最常见原因：还没设初始位姿。"))
            lines.append(" " + C.dim("      操作：RViz → 2D Pose Estimate，点车所在位置并拖出朝向"))
        else:
            lines.append(" " + C.bad("结论：链路断开，A* 会拿到全零变换去规划。"))
            lines.append(" " + C.warn("      此时日志只会显示 \"Can't find goal\"，真因在 TF。"))
        lines.append(" " + C.dim("      排查：ros2 run tf2_tools view_frames"))

    return "\n".join(lines), all_ok


def fatal_hint(node):
    out = []
    for pair in node.chain:
        stat = node.stats.get(pair)
        if stat and stat.state not in ("OK", "INIT") and stat.err:
            out.append(f"  {pair[0]} → {pair[1]}: [{stat.state}] {stat.err}")
    return "\n".join(out)


# ---------------------------------------------------------------- 主流程

def main():
    ap = argparse.ArgumentParser(
        description="实时检查泊车规划依赖的 TF 链（自动发现 base_link 的父链）")
    ap.add_argument("--once", action="store_true", help="采样 2 秒后输出一次就退出")
    ap.add_argument("--line", action="store_true", help="每次输出一行，适合写日志")
    ap.add_argument("--rate", type=float, default=2.0, help="刷新率 Hz，默认 2")
    ap.add_argument("--duration", type=float, default=0, help="持续秒数，0 表示一直跑")
    ap.add_argument("--leaf", default=DEFAULT_LEAF,
                    help=f"叶节点 frame，默认 {DEFAULT_LEAF}")
    ap.add_argument("--chain", action="append", default=[],
                    metavar="PARENT:CHILD", help="额外检查的链路，可重复")
    args = ap.parse_args()

    extra = []
    for spec in args.chain:
        if ":" in spec:
            a, b = spec.split(":", 1)
            extra.append((a.strip(), b.strip()))

    for note in _DDS_NOTES:
        print(f"[DDS] {note}", file=sys.stderr)

    rclpy.init()
    node = TfMonitor(leaf=args.leaf, extra_chains=extra)

    period = 1.0 / max(args.rate, 0.1)
    start = time.time()

    def sample(seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            node.probe_all()
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        if args.once:
            sample(2.0)
            text, ok = render(node)
            print(text)
            hint = fatal_hint(node)
            if hint:
                print()
                print(C.dim(hint))
            return 0 if ok else 1

        # 实时模式：先攒 2 秒，频率统计才有意义
        sample(2.0)
        last_render = 0.0
        while rclpy.ok():
            node.probe_all()
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.time()

            if now - last_render >= period:
                if args.line:
                    if node.chain:
                        parts = []
                        for pair in node.chain:
                            st = node.stats.get(pair)
                            tag = st.state if st else "?"
                            if st and st.rate:
                                tag += f" {st.rate:.0f}Hz"
                            parts.append(f"{pair[0]} → {pair[1]}={tag}")
                        line = "  ".join(parts)
                    elif not node.frames:
                        # 收不到任何 TF 时也要给出可读状态，不能只剩一个时间戳
                        line = f"NO_TF (DDS 不通? RMW={os.environ.get('RMW_IMPLEMENTATION')})"
                    elif node.leaf_missing():
                        line = f"{node.leaf}=NOT_FOUND"
                    else:
                        line = f"{node.leaf}=ROOT_NO_{EXPECTED_ROOT}"
                    print(f"{time.strftime('%H:%M:%S')} {line}", flush=True)
                else:
                    text, _ = render(node)
                    sys.stdout.write("\033[H\033[J" + text + "\n")
                    sys.stdout.flush()
                last_render = now

            if args.duration and now - start >= args.duration:
                break
            time.sleep(0.02)

        text, ok = render(node)
        if args.line:
            print()
            print(text)
        return 0 if ok else 1

    except KeyboardInterrupt:
        if not args.line:
            print()
        text, ok = render(node)
        print(text)
        return 0 if ok else 1
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
