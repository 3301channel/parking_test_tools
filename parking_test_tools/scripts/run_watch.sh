#!/usr/bin/env bash
#
# 由 start_planning_sim_log.sh 在新终端窗口里调用：
#   加载 Autoware 环境 → 运行 watch_state.py → 把自身 PID 写进 pid 文件（便于主脚本收尾时关闭）
#
# 用法：
#   ./run_watch.sh [日志目录]
#
# 单独手动运行也可以，等价于 source 环境后跑 watch_state.py --line。

set +u
# shellcheck disable=SC1091
source "$HOME/autoware/install/setup.bash" 2>/dev/null
set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${1:-}"

if [[ -n "$LOG_DIR" ]]; then
    mkdir -p "$LOG_DIR" 2>/dev/null || true
    # exec 之前写 PID：exec 后本进程被 python 替换，PID 保持不变，
    # 主脚本收尾时 kill 这个 PID 就能关掉监控
    echo $$ >"$LOG_DIR/watch_state.pid" 2>/dev/null || true
fi

exec python3 "$DIR/watch_state.py" --line
