#!/usr/bin/env bash
#
# 查看当前规划场景（Parking / LaneDriving / Empty），自动加载 Autoware 环境。
#
# 用法：
#   ./show_scenario.sh            # 只在场景变化时打印一行（推荐）
#   ./show_scenario.sh --once     # 只取当前一帧就退出
#   ./show_scenario.sh --raw      # 原样输出完整消息（含 activating_scenarios）
#
# 为什么需要它：autoware_internal_planning_msgs 是 Autoware 自定义消息包，
# 不在 /opt/ros/humble 里。新开终端若没 source ~/autoware/install/setup.bash，
# ros2 topic echo 就会报 "The message type ... is invalid"。

set +u
# shellcheck disable=SC1091
source "$HOME/autoware/install/setup.bash"
set -u

TOPIC=/planning/scenario_planning/scenario

case "${1:-}" in
    --raw)
        exec ros2 topic echo "$TOPIC"
        ;;
    --once)
        exec ros2 topic echo "$TOPIC" --field current_scenario --once
        ;;
    *)
        echo "监听 $TOPIC （只在场景变化时打印，Ctrl+C 退出）"
        prev=""
        ros2 topic echo "$TOPIC" --field current_scenario 2>/dev/null |
            while IFS= read -r line; do
                [[ -z "$line" || "$line" == "---" ]] && continue
                if [[ "$line" != "$prev" ]]; then
                    printf '%s  %s\n' "$(date +%H:%M:%S)" "$line"
                    prev="$line"
                fi
            done
        ;;
esac
