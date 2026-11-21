"""Autoware 泊车日志分析器。

纯标准库实现（numpy 可选加速），只读 rosbag（sqlite3 / mcap），
输出规范化 CSV、report.json、report.md 与 SVG 图。

v1.1 起增加执行链分析（D-01~D-08）：把"状态机 / 操作模式 / 档位 / 控制指令 /
执行上报"一起解码，用于区分"规划给不出路"和"规划给了但车不走"。
"""

__version__ = "1.1.0"
