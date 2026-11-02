#!/usr/bin/env python3
"""
等待 Autoware 真正开始出轨迹，用于决定什么时候弹状态监控窗口。

判断条件（满足任一即返回）：
  1. /planning/trajectory 收到一条点数 > 1 的轨迹 —— 说明规划器已工作
  2. /autoware/state 进入 PLANNING / WAITING_FOR_ENGAGE / DRIVING —— 说明已过初始化

用法：
  python3 wait_planning.py [超时秒数]     # 默认 300
退出码：
  0 = 已就绪   1 = 超时

为什么要用 python 而不是 `ros2 topic list`：
  ros2 CLI 每次调用都要连 DDS daemon，在刚启动、节点众多的场景下经常延迟返回，
  之前等 180 秒都等不到就是卡在这里。直接订阅消息最可靠。
"""

import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from autoware_planning_msgs.msg import Trajectory
    from autoware_system_msgs.msg import AutowareState
except ImportError as e:
    print(f"[wait_planning] 缺少 ROS 环境: {e}", file=sys.stderr)
    sys.exit(1)

READY_STATES = {3, 4, 5}   # PLANNING / WAITING_FOR_ENGAGE / DRIVING


class Waiter(Node):
    def __init__(self):
        super().__init__("wait_planning_ready")
        self.ready = False
        self.reason = ""
        self.create_subscription(Trajectory, "/planning/trajectory", self.on_traj, 10)
        self.create_subscription(AutowareState, "/autoware/state", self.on_state, 10)

    def on_traj(self, msg):
        if not self.ready and len(msg.points) > 1:
            self.ready = True
            self.reason = f"/planning/trajectory 已输出轨迹（{len(msg.points)} 点）"

    def on_state(self, msg):
        if not self.ready and msg.state in READY_STATES:
            self.ready = True
            self.reason = f"/autoware/state 进入 state={msg.state}"


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0

    rclpy.init()
    node = Waiter()
    t0 = time.time()
    try:
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.5)
            if node.ready:
                print(f"[wait_planning] 规划已就绪（用时 {time.time()-t0:.0f}s）：{node.reason}",
                      flush=True)
                return 0
        # 区分"真的等满了"和"被 Ctrl+C / SIGTERM 打断"，
        # 否则中断时会打印"等待 300s 未就绪"，与实际的几秒钟完全对不上。
        if not rclpy.ok():
            print("[wait_planning] 等待被中断，跳过就绪检查", flush=True)
        else:
            print(f"[wait_planning] 等待 {timeout:.0f}s 仍未就绪，直接启动监控", flush=True)
        return 1
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
