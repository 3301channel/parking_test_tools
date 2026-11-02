#!/usr/bin/env bash
#
# 一键启动泊车测试：ros2 launch + 全量日志落盘 + 退出后自动分析。
#
# 用法：
#   ./run_parking.sh                                   # launch 启动，日志自动保存
#   ./run_parking.sh map_path:=$HOME/autoware_data/maps/xxx
#   ./run_parking.sh record_bag:=false watch_state:=false
#   ./run_parking.sh --log                             # 改用 shell 采集版（等价旧脚本）
#
# 相比直接 ros2 launch，它多做四件事：
#   1. 自动 source Autoware 与 parking_test_tools 两个环境
#   2. ros2 launch 的全部输出 tee 到 session.log（launch 自身没有重定向机制）
#   3. ROS_LOG_DIR 指向本次会话目录，节点日志不再散落在 ~/.ros/log
#   4. launch 退出后自动跑轨迹分析，生成 evaluation.md 与图

set -uo pipefail

# ---- 1. 加载环境 ----
set +u
# shellcheck disable=SC1091
source "$HOME/autoware/install/setup.bash"
if [[ -f "$HOME/parking_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$HOME/parking_ws/install/setup.bash"
fi
set -u

# ---- DDS 配置：消除 participant 索引上限 ----
# CycloneDDS 默认按 participant 索引分配端口（0-9），Autoware 全量启动几十个进程
# 再加手动单起的模块就会耗尽，表现为大面积 exit code -6：
#   Failed to find a free participant index for domain 0
# 用 Autoware 自带的 cyclonedds.xml（ParticipantIndex=none）可以彻底避开。
if [[ -z "${CYCLONEDDS_URI:-}" && "${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" == *cyclonedds* ]]; then
    # 优先包内配置：官方那份要求 10 MB 缓冲，本机 rmem_max 只有 208 KB 会直接失败
    dds_cfg="$HOME/zzz/parking/parking_test_tools/config/cyclonedds.xml"
    [[ -f "$dds_cfg" ]] || dds_cfg="$HOME/autoware/docker/files/cyclonedds.xml"
    if [[ -f "$dds_cfg" ]]; then
        export CYCLONEDDS_URI="file://$dds_cfg"
        echo "[DDS] 已启用 $dds_cfg"
    fi
fi

# ---- 残留进程自动清理 ----
# 多份 Autoware 并存会让旧进程占住 DDS participant，新节点创建 domain 直接失败，
# 表现为节点大面积 exit code -6、RViz 里设了目标没反应。这里直接清理。
# 需要保留时设 KEEP_ROS=1。
if [[ "${KEEP_ROS:-0}" != "1" ]]; then
    # 必须用完整命令行匹配：robot_state_publisher 的路径在 /opt/ros/humble/lib/ 下，
    # 不含 autoware_ 前缀，只按节点名前缀过滤会一直漏掉它（累积后 AUTO 不可用）。
    # awk 的"或"是 ||，不是 |
    PAT='/install\/autoware_/ || /rclcpp_components\/component_container/ || /lib\/robot_state_publisher\/robot_state_publisher/ || /lib\/rviz2\// || /bin\/ros2 launch autoware/ || /simple_planning_simulator/'
    stale_pids="$(ps -eo pid,args --no-headers 2>/dev/null \
                  | awk -v me="$$" "($PAT) && !/awk / && \$1 != me {print \$1}")"
    if [[ -n "$stale_pids" ]]; then
        n_stale=$(echo "$stale_pids" | wc -l)
        echo "[清理] 发现 $n_stale 个残留 Autoware 进程，先清理再启动"
        # shellcheck disable=SC2086
        kill -9 $stale_pids 2>/dev/null
        sleep 2
        rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/cyclonedds_* 2>/dev/null
        echo "[清理] 完成"
    fi
fi

if ! ros2 pkg prefix parking_test_tools >/dev/null 2>&1; then
    echo "[错误] 找不到 parking_test_tools 包" >&2
    echo "       请先构建：" >&2
    echo "         mkdir -p ~/parking_ws/src" >&2
    echo "         ln -sfn $HOME/zzz/parking/parking_test_tools ~/parking_ws/src/" >&2
    echo "         cd ~/parking_ws && colcon build --symlink-install" >&2
    exit 1
fi

INST_PKG="$(ros2 pkg prefix parking_test_tools)/share/parking_test_tools"

# ---- 源码与 install 同步 ----
# colcon 对 ament_cmake 的 install(DIRECTORY ...) 是**拷贝**，不是软链接：
# 改了 parking_test_tools/scripts 里的脚本，install 里还是旧副本，
# 于是 run_parking.sh 起的 watch_state 和直接跑源码目录的那个不是同一个版本。
# 这里在启动前检测一次，源码更新就自动重建（约 0.1 s）。
SRC_PKG="$HOME/zzz/parking/parking_test_tools"
if [[ -d "$SRC_PKG/scripts" && -d "$INST_PKG/scripts" ]]; then
    newer="$(find "$SRC_PKG/scripts" "$SRC_PKG/launch" "$SRC_PKG/config" \
              -type f -newer "$INST_PKG/scripts" 2>/dev/null | head -1)"
    if [[ -n "$newer" ]]; then
        echo "[同步] 源码比 install 新（$(basename "$newer")），重新构建 parking_test_tools ..."
        if ( cd "$HOME/parking_ws" && colcon build --packages-select parking_test_tools ) \
                >/tmp/parking_tools_build.log 2>&1; then
            echo "[同步] 完成，install 已与源码一致"
        else
            echo "[同步] 构建失败，本次仍会用 install 里的旧副本" >&2
            echo "       详见 /tmp/parking_tools_build.log" >&2
        fi
    fi
fi

TOOLS="$INST_PKG/scripts"

# ---- --log 走旧的 shell 采集版 ----
if [[ "${1:-}" == "--log" ]]; then
    shift
    exec bash "$TOOLS/start_planning_sim_log.sh" "$@"
fi

# ---- 2. 准备日志目录 ----
LOG_ROOT="${LOG_ROOT:-$HOME/autoware_logs}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/$STAMP"
mkdir -p "$LOG_DIR/ros_log"
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"

export ROS_LOG_DIR="$LOG_DIR/ros_log"

{
    echo "start_time: $(date -Is)"
    echo "command: ros2 launch parking_test_tools parking_test.launch.xml $*"
    echo "ros_log_dir: $ROS_LOG_DIR"
    echo "bag_dir: $LOG_DIR/bag"
} >"$LOG_DIR/meta.txt"

echo "日志目录: $LOG_DIR"
echo

# ---- 3. 前台跑 launch，全量 tee ----
set -o pipefail
ros2 launch parking_test_tools parking_test.launch.xml \
    "bag_dir:=$LOG_DIR/bag" "$@" 2>&1 | tee "$LOG_DIR/session.log"
rc=${PIPESTATUS[0]}

# ---- 4. 收尾：把 launch 自己的日志也并进来 ----
# ros2 launch 会另写一份 <ROS_LOG_DIR>/<session>/launch.log，内容比 tee 捕获的更全
# （节点内部 rcutils 输出都在里面）。两者合并，保证 session.log 完整。
LAUNCH_LOG="$(ls -1t "$ROS_LOG_DIR"/*/launch.log 2>/dev/null | head -1)"
if [[ -f "$LAUNCH_LOG" ]]; then
    {
        echo ""
        echo "===== 以下为 launch 自身日志（$LAUNCH_LOG）====="
        cat "$LAUNCH_LOG"
    } >>"$LOG_DIR/session.log"
fi

if [[ -s "$LOG_DIR/session.log" ]]; then
    sed -E 's/\x1B\[[0-9;?]*[A-Za-z]//g; s/\r+\n/\n/g; s/\r/ /g' \
        "$LOG_DIR/session.log" >"$LOG_DIR/session_plain.log" 2>/dev/null || \
        cp "$LOG_DIR/session.log" "$LOG_DIR/session_plain.log"
fi

{
    echo "end_time: $(date -Is)"
    echo "exit_code: $rc"
} >>"$LOG_DIR/meta.txt"

# ---- 5. 自动分析 ----
if [[ -d "$LOG_DIR/bag" && -f "$TOOLS/analyze_parking_session.py" ]]; then
    echo
    echo "[分析] 正在生成轨迹图与评估报告..."
    SLOT="$(dirname "$TOOLS")/config/parking_slot.yaml"
    slot_arg=()
    [[ -f "$SLOT" ]] && slot_arg=(--slot "$SLOT")
    if timeout 180 python3 "$TOOLS/analyze_parking_session.py" "$LOG_DIR/bag" \
        --out "$LOG_DIR" "${slot_arg[@]}" >"$LOG_DIR/analyze.log" 2>&1; then
        echo "[分析] 完成: $LOG_DIR/evaluation.md"
    else
        echo "[分析] 失败或被跳过，详见 $LOG_DIR/analyze.log"
    fi
fi

echo
echo "============ 会话结束 ============"
echo "日志: $LOG_DIR/session_plain.log"
echo "评估: $LOG_DIR/evaluation.md"
echo "入口: $LOG_ROOT/latest/"
echo "=================================="
exit "$rc"
