#!/usr/bin/env bash
#
# 等规划真正出轨迹后，再启动状态监控（供 launch 调用）。
#
# 行为：
#   有图形环境  -> 新开一个终端窗口显示，本脚本立即退出（不占 launch 输出）
#   无图形环境  -> 在当前终端前台显示（会混在 launch 输出里）
#
# 为什么单独开窗口：
#   launch 会给每个进程加 [名称-序号] 前缀，且 watch 每秒刷新，
#   和 Autoware 上百个节点的日志混在一起很难看。

set -u
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WAIT_SEC="${WATCH_WAIT_SEC:-300}"

# 等就绪：输出带 [监控] 前缀，避免和 launch 输出混淆
python3 "$DIR/wait_planning.py" "$WAIT_SEC" 2>&1 | sed -u 's/^/[监控] /' || true

RUN_CMD="python3 '$DIR/watch_state.py' --line; echo; printf '[监控已退出] 按回车关闭窗口'; read -r _"

if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    for term in gnome-terminal x-terminal-emulator xfce4-terminal konsole xterm; do
        if command -v "$term" >/dev/null 2>&1; then
            case "$term" in
                gnome-terminal)
                    setsid gnome-terminal --title="Autoware 状态监控" -- bash -c "$RUN_CMD" \
                        >/dev/null 2>&1 & ;;
                x-terminal-emulator|xterm)
                    setsid "$term" -title "Autoware 状态监控" -e bash -c "$RUN_CMD" \
                        >/dev/null 2>&1 & ;;
                *)
                    setsid "$term" --title="Autoware 状态监控" -e "bash -c \"$RUN_CMD\"" \
                        >/dev/null 2>&1 & ;;
            esac
            echo "[监控] 已在新终端窗口打开（$term）"
            exit 0
        fi
    done
    echo "[监控] 有图形环境但没找到可用终端程序，改为前台显示"
else
    echo "[监控] 无图形环境，改为前台显示"
fi

exec python3 "$DIR/watch_state.py" --line
