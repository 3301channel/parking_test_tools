"""规则库注册：导入各模块即完成注册。"""

from .base import CHECKS, CheckResult, exit_code, run_checks, worst  # noqa: F401
# 导入顺序 = CLI 打印顺序：costmap → 规划 → 执行链 → 轨迹 → 时序
from . import costmap, planner, control, trajectory, system  # noqa: F401,E402
