#!/usr/bin/env bash
#
# 录包（供 launch 调用）。话题列表集中在这里，改这里即可，不用动 launch 文件。
#
# 用法：record_bag.sh <输出目录>
# 环境变量：
#   RECORD_TF=1     额外录 /tf /tf_static（体积会涨到几百 MB）
#   RECORD_ROSOUT=0 不录 /rosout
#
# 与 scripts/start_planning_sim_log.sh 里的 BAG_TOPICS 保持同步；
# 两处都改，或者只用其中一种启动方式。

set -u

# 用法：
#   record_bag.sh <输出目录>    直接写到指定位置（run_parking.sh 会传，保证和 session.log 同目录）
#   record_bag.sh               哪个参数都没给时，自己按时间戳建 ~/autoware_logs/<stamp>/bag
if [[ -n "${1:-}" ]]; then
    OUT="$1"
    mkdir -p "$(dirname "$OUT")"
else
    OUT="$HOME/autoware_logs/$(date +%Y%m%d_%H%M%S)/bag"
    mkdir -p "$(dirname "$OUT")"
fi

TOPICS=(
    # —— 泊车规划核心：A* 的输入与输出 ——
    /planning/scenario_planning/parking/costmap_generator/occupancy_grid
    /planning/scenario_planning/parking/costmap_generator/grid_map
    /planning/scenario_planning/parking/trajectory
    /planning/scenario_planning/parking/is_completed
    /planning/scenario_planning/parking/freespace_planner/debug/processing_time_ms
    # —— 场景与目标 ——
    /planning/scenario_planning/scenario
    /planning/mission_planning/route
    /planning/route_state
    /planning/mission_planning/goal
    /planning/scenario_planning/modified_goal
    # —— 最终轨迹 ——
    /planning/trajectory
    /planning/scenario_planning/trajectory
    # —— 定位与感知 ——
    /localization/kinematic_state
    /perception/object_recognition/objects
    # —— 诊断与控制 ——
    /diagnostics
    /control/command/control_cmd
    /vehicle/status/velocity_status
    # —— 停车归因证据：watch_state.py 的诊断全靠这些话题 ——
    # 缺了它们，事后只能看到"车停了"，看不到"为什么停"。
    /control/vehicle_cmd_gate/is_stopped              # 闸门判定停稳并保持
    /control/vehicle_cmd_gate/is_start_requested      # 是否请求起步
    /control/vehicle_cmd_gate/is_filter_activated     # 限幅是否在起作用
    /vehicle/status/gear_status                       # D/R/N/P，区分揉车换挡
    /api/motion/state                                 # STOPPED/STARTING/MOVING
    # —— goal_planner 无路径证据（RViz 里的紫色/绿色轨迹）——
    /planning/path_reference/goal_planner             # 紫色：参考路径
    /planning/path_candidate/goal_planner             # 绿色：安全候选路径
    /planning/planning_factors/goal_planner           # detail 字段写明拒绝原因
    /planning/planning_validator/validation_status    # 轨迹校验哪一项不通过
    # —— 地图可视化（绘图用）——
    /map/vector_map_marker
)

[[ "${RECORD_TF:-0}" == "1" ]] && TOPICS+=(/tf /tf_static)
[[ "${RECORD_ROSOUT:-1}" == "1" ]] && TOPICS+=(/rosout)

echo "[录包] ${#TOPICS[@]} 个话题 -> $OUT"
exec ros2 bag record -o "$OUT" "${TOPICS[@]}"
