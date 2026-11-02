#!/usr/bin/env bash
#
# 从一次 Autoware 会话日志中挑出「泊车演示」相关的部分。
#
# 背景：planning_simulator 一次会拉起上百个节点，终端日志混在一起。
# 泊车路径规划真正干活的节点是：
#   /planning/scenario_planning/parking/freespace_planner  (pkg: autoware_freespace_planner)
# 它在终端里的行首前缀是 [freespace_planner-N]，launch.log 里 logger 名是 freespace_planner。
#
# 用法：
#   ./pick_parking_log.sh                                   # 处理最近一次会话
#   ./pick_parking_log.sh ~/autoware_logs/20261026_143012/session_plain.log

set -uo pipefail

LOG="${1:-$HOME/autoware_logs/latest/session_plain.log}"

if [[ ! -f "$LOG" ]]; then
    echo "找不到日志: $LOG" >&2
    echo "请先用 start_planning_sim_log.sh 启动并跑完一次演示" >&2
    exit 1
fi

OUTDIR="$(dirname "$LOG")"
BASE="$(basename "$LOG")"
PREFIX="${BASE%.log}"

PARKING="$OUTDIR/${PREFIX}.parking.log"
ERRORS="$OUTDIR/${PREFIX}.errors.log"
TIMELINE="$OUTDIR/${PREFIX}.timeline.log"

# 泊车链路节点：freespace_planner 本体 + 场景选择器（决定何时切到 parking 场景）
NODE_RE='freespace_planner|freespace_planning_algorithms|scenario_selector'

# 1) 泊车规划节点的全部输出
grep -E "$NODE_RE" "$LOG" >"$PARKING" || true

# 2) 全量 ERROR / WARN（跨所有节点，用于发现被淹掉的问题）
grep -E '\[(ERROR|WARN|FATAL)\]' "$LOG" >"$ERRORS" || true

# 3) 泊车关键事件时间线（源码里真实存在的日志文案）
grep -E 'Freespace planning completed|Can.t find goal|Course out|initialize planning algorithm|Waiting for (route|occupancy|odometry)' \
    "$LOG" >"$TIMELINE" || true

echo "会话日志: $LOG"
echo
printf '  %-42s %8s 行\n' "$(basename "$PARKING")" "$(wc -l <"$PARKING")"
printf '  %-42s %8s 行\n' "$(basename "$ERRORS")" "$(wc -l <"$ERRORS")"
printf '  %-42s %8s 行\n' "$(basename "$TIMELINE")" "$(wc -l <"$TIMELINE")"
echo
echo "建议先看：less -R \"$TIMELINE\""
echo "再看全貌：less -R \"$PARKING\""
