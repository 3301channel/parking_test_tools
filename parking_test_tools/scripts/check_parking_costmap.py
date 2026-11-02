#!/usr/bin/env python3
"""
检查泊车 costmap 在车位附近是否被判定为障碍 —— 用来回答
「A* 为什么报 Invalid start or goal pose / 为什么只规划到车位入口」。

用法：
  source ~/autoware/install/setup.bash
  python3 check_parking_costmap.py                       # 用 /tmp 下默认车位
  python3 check_parking_costmap.py --slot parking_slot.yaml --out /tmp/costmap.png

输出：
  终端打印车位中心及周围 3 m 内的栅格统计（占据 / 空闲 / 未知）
  同时把该区域画成图片（占据=红、空闲=深灰、未知=黑）
"""

import argparse
import math
import os
import sys
import time

import numpy as np

try:
    import yaml
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
except ImportError as e:
    print(f"[错误] 缺少依赖: {e}", file=sys.stderr)
    sys.exit(1)

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid
except ImportError as e:
    print(f"[错误] 未找到 ROS2 环境，请先 source install/setup.bash: {e}", file=sys.stderr)
    sys.exit(1)

TOPIC = "/planning/scenario_planning/parking/costmap_generator/occupancy_grid"


class Grab(Node):
    def __init__(self):
        super().__init__("grab_parking_costmap")
        self.msg = None
        self.create_subscription(OccupancyGrid, TOPIC, self.cb, 10)

    def cb(self, msg):
        if self.msg is None:
            self.msg = msg


def load_slot(path, fallback=(3719.82, 73749.04, 2.039)):
    if path and os.path.isfile(path):
        with open(path) as f:
            d = yaml.safe_load(f) or {}
        s = d.get("parking_slot", d)
        return (float(s["center"][0]), float(s["center"][1]),
                float(s.get("yaw", 0.0)),
                float(s.get("length", 5.0)), float(s.get("width", 2.5)))
    print(f"[提示] 未提供车位配置，使用内置值 {fallback}")
    return (fallback[0], fallback[1], fallback[2], 5.0, 2.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--slot",
                    default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         "..", "config", "parking_slot.yaml"))
    ap.add_argument("--out", default="/tmp/parking_costmap.png")
    ap.add_argument("--radius", type=float, default=4.0, help="统计半径（米）")
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("--gradient", action="store_true",
                    help="按栅格值 0~100 连续着色，能明显看出膨胀层（低值）与硬障碍（100）")
    args = ap.parse_args()

    # 自动创建输出目录，避免 --out 指到不存在的路径时 savefig 报 FileNotFoundError
    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    cx, cy, yaw, length, width = load_slot(args.slot)

    rclpy.init()
    node = Grab()
    t0 = time.time()
    while node.msg is None and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.3)

    if node.msg is None:
        print(f"[失败] {args.timeout:.0f}s 内没收到 {TOPIC}", file=sys.stderr)
        print("       可能原因：Autoware 没启动 / 没进入 PARKING 场景 / 话题名不同", file=sys.stderr)
        print("       先执行：ros2 topic list | grep -iE 'grid|costmap'", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown()
        return 1

    msg = node.msg
    info = msg.info
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    w, h = info.width, info.height
    data = np.array(msg.data, dtype=np.int16).reshape(h, w)

    print(f"costmap: {w}x{h} 格, 分辨率 {res:.3f} m, 原点 ({ox:.2f}, {oy:.2f})")
    print(f"        覆盖范围 x[{ox:.1f}, {ox + w*res:.1f}]  y[{oy:.1f}, {oy + h*res:.1f}]")

    col = int((cx - ox) / res)
    row = int((cy - oy) / res)
    inside = 0 <= col < w and 0 <= row < h
    print(f"\n车位中心 ({cx:.2f}, {cy:.2f}) → 栅格 (col={col}, row={row})"
          f"  {'在范围内' if inside else '★ 超出 costmap 范围 ★'}")

    def stat(name, c, r, radius):
        half = int(radius / res)
        c0, c1 = max(0, c - half), min(w, c + half + 1)
        r0, r1 = max(0, r - half), min(h, r + half + 1)
        block = data[r0:r1, c0:c1]
        if block.size == 0:
            print(f"  {name}: 无数据（超出范围）")
            return
        hard = int((block >= 50).sum())
        infl = int(((block > 0) & (block < 50)).sum())
        free = int((block == 0).sum())
        unk = int((block < 0).sum())
        val = int(data[r, c]) if (0 <= r < h and 0 <= c < w) else -999
        print(f"  {name}: 中心栅格值={val}  (100/高值=硬障碍, 1~49=膨胀层, 0=空闲, <0=未知)")
        print(f"      {radius:.0f}m 内: 硬障碍 {hard} 格 / 膨胀层 {infl} 格 / "
              f"空闲 {free} 格 / 未知 {unk} 格"
              f"  (共 {block.size} 格, 障碍占比 {100.0*(hard+infl)/block.size:.1f}%)")
        if infl:
            print(f"      膨胀层占障碍面积 {100.0*infl/(hard+infl):.0f}%，"
                  f"约相当于障碍向外扩了 {infl/max(hard,1):.1f} 倍格数")

    print("\n=== 栅格统计 ===")
    stat("车位中心", col, row, 1.0)
    stat("车位中心", col, row, args.radius)

    # 车辆当前位置（从 odometry 不方便拿，这里用 costmap 原点附近提示）
    print("\n判读建议：")
    print("  车位中心栅格值 >= 50  → 车位被判定为障碍，A* 必然报 Invalid goal")
    print("  中心 1m 内占据率 > 20% → 车位过窄，需减小 vehicle_shape_margin_m")

    # 画图
    half = int(args.radius / res)
    c0, c1 = max(0, col - half * 2), min(w, col + half * 2)
    r0, r1 = max(0, row - half * 2), min(h, row + half * 2)
    if c1 > c0 and r1 > r0:
        block = data[r0:r1, c0:c1]
        fig, ax = plt.subplots(figsize=(8, 7), dpi=120)
        fig.patch.set_facecolor("#0f1419")
        ax.set_facecolor("#0f1419")
        extent = [ox + c0 * res, ox + c1 * res, oy + r0 * res, oy + r1 * res]

        if args.gradient:
            # 连续色标：0=空闲(暗) → 中间低值=膨胀层 → 100=硬障碍(亮黄)
            # 未知(<0) 用 NaN 遮掉，单独涂深灰
            disp = np.ma.masked_array(block.astype(float), mask=(block < 0))
            cmap = plt.get_cmap("inferno").copy()
            cmap.set_bad("#20262e")
            im = ax.imshow(disp, cmap=cmap, extent=extent, origin="lower",
                           vmin=0, vmax=100)
            cb = fig.colorbar(im, ax=ax, fraction=0.035, pad=0.02)
            cb.set_label("occupancy 0~100", color="#d8dee9", fontsize=8)
            cb.ax.tick_params(colors="#7f8c8d", labelsize=7)
            cb.outline.set_edgecolor("#3b4252")
            title = ("parking costmap  (gradient: dark=free, bright=obstacle, "
                     "grey=unknown)")
        else:
            disp = np.zeros_like(block, dtype=float)
            disp[block < 0] = 0.15          # 未知
            disp[(block >= 0) & (block < 50)] = 0.55   # 空闲
            disp[block >= 50] = 1.0         # 占据
            ax.imshow(disp, cmap=ListedColormap(["#20262e", "#6b7684", "#e74c3c"]),
                      extent=extent, origin="lower", vmin=0, vmax=1)
            title = "parking costmap around slot  (red=occupied, grey=free, dark=unknown)"
        # 车位框
        hl, hw = length / 2, width / 2
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(cx + x * c - y * s, cy + x * s + y * c)
                   for x, y in [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw), (-hl, -hw)]]
        ax.plot([p[0] for p in corners], [p[1] for p in corners],
                color="#4a90d9", lw=2, label="parking slot")
        ax.plot(cx, cy, marker="X", ms=12, color="#4a90d9")
        ax.set_title(title, color="#d8dee9", fontsize=9)
        ax.tick_params(colors="#7f8c8d")
        for sp in ax.spines.values():
            sp.set_color("#3b4252")
        ax.legend(loc="upper right", fontsize=8, facecolor="#1b2229", edgecolor="#3b4252")
        ax.set_aspect("equal")
        fig.tight_layout()
        fig.savefig(args.out, facecolor=fig.get_facecolor())
        plt.close(fig)
        print(f"\n已保存 costmap 局部图: {args.out}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
