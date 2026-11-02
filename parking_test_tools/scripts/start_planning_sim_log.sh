#!/usr/bin/env bash
#
# 启动 Autoware planning_simulator（用于测试泊车规划路径），并把终端里滚动的
# 全部日志完整落盘，事后可 grep / less -R / 直接贴给 AI 分析。
#
# 用法：
#   ./start_planning_sim_log.sh
#   MAP_PATH=/path/to/map ./start_planning_sim_log.sh
#   VEHICLE_MODEL=xxx SENSOR_MODEL=yyy ./start_planning_sim_log.sh
#
# 日志目录：~/autoware_logs/<时间戳>/
#   session.log        终端原样输出（含 ANSI 颜色码，用 less -R 查看）
#   session_plain.log  清洗后的纯文本（无颜色码/回车符，适合 grep、贴给 AI）
#   ros_log/           ROS2 各节点自己的日志（launch.log 里是最完整的节点输出）
#   meta.txt           本次启动的参数、起止时间、退出码
#   bag/               录制的关键话题（含 A* 的 costmap、scenario、轨迹、诊断）
#   trajectory_plot.png      车位局部轨迹图（车道线 + costmap + 车辆轮廓）
#   plot_global.png          整段行驶路线的全局图
#   evaluation.md            泊车质量评估（位置/角度偏差、是否入位）
#   trajectory_info.json / trajectory_points.csv / timeseries_plotjuggler.csv
#   ~/autoware_logs/latest -> 指向最近一次的软链接
#
# 退出方式：Ctrl+C（ros2 launch 会正常收尾，脚本随后录包收尾并出分析图）
#
# 配置：所有开关都在同目录的 parking_test.conf 里，改完直接重跑本脚本。
#   换一份配置：CONF_FILE=/path/other.conf ./start_planning_sim_log.sh
# 常用临时覆盖（命令行优先级最高）：
#   RECORD_BAG=0      关闭录包与轨迹分析
#   RECORD_TF=1       额外录 /tf /tf_static（体积大，几十~几百 MB）
#   RECORD_ROSOUT=0   不录 /rosout（全节点日志汇总话题）
#   PARKING_DEBUG=0   关闭自动调日志级别的尝试
#   CHECK_ARRIVAL=0   关闭到达判定阈值检查
#   WATCH_STATE=0     不新开车辆状态监控窗口

set -uo pipefail

# DEBUG_TRACE=1 时输出每条执行的命令（排查卡在哪个环节用）
if [[ "${DEBUG_TRACE:-0}" == "1" ]]; then
    set -x
fi

# 带时间戳的进度提示：卡住时能一眼看出停在哪一步
step() {
    printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

# 防止误执行：本脚本被 source 或误敲时也会直接启动 Autoware。
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    # 打印文件头的说明，直到第一行可执行语句为止
    sed -n '2,/^set -uo pipefail$/p' "$0" | sed '$d' | sed -E 's/^# ?//'
    return 0 2>/dev/null || exit 0
fi

# 脚本自身目录：配置文件、车位定义都放这里
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 包根目录（scripts/ 的上一级）：config/ 在这里
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# 加载脚本级配置（只控制脚本行为，不改动 Autoware 的任何文件）
CONF_FILE="${CONF_FILE:-$PKG_DIR/config/parking_test.conf}"
if [[ -f "$CONF_FILE" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "$CONF_FILE"
    set -u
fi

AUTOWARE_DIR="${AUTOWARE_DIR:-$HOME/autoware}"
MAP_PATH="${MAP_PATH:-$HOME/autoware_data/maps/sample-map-planning}"
VEHICLE_MODEL="${VEHICLE_MODEL:-sample_vehicle}"
SENSOR_MODEL="${SENSOR_MODEL:-sample_sensor_kit}"
LOG_ROOT="${LOG_ROOT:-$HOME/autoware_logs}"

# 录 /rosout（全节点日志汇总话题）
RECORD_ROSOUT="${RECORD_ROSOUT:-1}"
# 1=新开一个终端窗口实时显示车辆状态（watch_state.py），0=不显示
WATCH_STATE="${WATCH_STATE:-1}"
WATCH_PID=""
# 到达判定检查：只读参数 + 提示，不修改 Autoware
CHECK_ARRIVAL="${CHECK_ARRIVAL:-1}"
ARRIVAL_LATERAL_WARN="${ARRIVAL_LATERAL_WARN:-0.5}"
ARRIVAL_OVERSHOOT_WARN="${ARRIVAL_OVERSHOOT_WARN:-0.5}"

# 启动后自动把泊车相关节点的日志级别调到 debug（PARKING_DEBUG=0 可关闭）
# 格式：节点全名|logger 名（logger 名取自日志里的 [xxx] 字段）
# 为什么要自动化：日志级别是节点运行时状态，每次重启都会回到 INFO，
# 所以必须在本次启动后重新设置一遍，否则拿不到 Freespace planning 耗时之类的细节。
PARKING_DEBUG="${PARKING_DEBUG:-1}"
DEBUG_TARGETS=(
    "/planning/scenario_planning/parking/freespace_planner|planning.scenario_planning.parking.freespace_planner"
    "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner|planning.scenario_planning.lane_driving.behavior_planning.behavior_path_planner.goal_planner"
)
LEVEL_PID=""

# ===== 轨迹记录与分析 =====
# RECORD_BAG=1 时后台录制关键话题，退出后自动生成轨迹图与评估报告
RECORD_BAG="${RECORD_BAG:-1}"
# 话题分五组，越靠前越关键。ros2 bag record 对"此刻还不存在"的话题会先挂着等，
# 一旦发布就自动开始录，所以不用关心 launch 起来的先后顺序。
# 体积参考：本套配置一次 1~3 分钟的演示约 30~120 MB（不含 /tf）。
BAG_TOPICS=(
    # —— 泊车规划核心：A* 的输入与输出 ——
    # 注意：话题名以 ros2 topic list 实测为准。launch 里写的 remap 目标是
    # .../parking/occupancy_grid，但实际运行时话题挂在 costmap_generator/ 下，
    # 写错名字 bag 里就只有空目录、事后无从分析。
    /planning/scenario_planning/parking/costmap_generator/occupancy_grid  # A* 输入栅格
    /planning/scenario_planning/parking/costmap_generator/grid_map        # 同数据的 GridMap 版
    /planning/scenario_planning/parking/trajectory     # A* 分段输出（点稀疏，约 grid_resolution 间距）
    /planning/scenario_planning/parking/is_completed   # 泊车完成标志，True 只应出现一次
    /planning/scenario_planning/parking/freespace_planner/debug/processing_time_ms  # A* 单次规划耗时
    # —— 场景与目标 ——
    /planning/scenario_planning/scenario               # 当前是否 Parking；决定用 A* 还是几何 pull over
    /planning/mission_planning/route                   # 全局路由（LaneletRoute），route 重建会打断泊车
    /planning/route_state                              # 路由状态：UNSET/SET/REROUTE/CHANGED/ARRIVED
    /planning/mission_planning/goal                    # 你设的目标点原值
    /planning/scenario_planning/modified_goal          # goal_planner 修正后的实际目标（泊车真正对齐的是它）
    # —— 最终轨迹（发给控制的那条）——
    /planning/trajectory
    /planning/scenario_planning/trajectory
    # —— 定位与感知 ——
    /localization/kinematic_state
    /perception/object_recognition/objects
    # —— 诊断：MRM / 紧急停车归因（哪条 diag 变 STALE/ERROR）——
    /diagnostics
    # —— 控制侧：区分"规划让它停"和"控制自己刹住"——
    /control/command/control_cmd
    /vehicle/status/velocity_status
    # —— 停车归因证据：watch_state.py 靠这些区分"谁让它停"——
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
    # —— 地图可视化：绘图时画车道线，效果与 RViz 一致（MarkerArray）——
    /map/vector_map_marker
)

# /tf 单个话题就是几十 Hz × 上百 frame，会让 bag 从几十 MB 涨到几百 MB，默认关闭。
# 需要复算坐标变换（map->base_link）时再开：RECORD_TF=1 ./start_planning_sim_log.sh
RECORD_TF="${RECORD_TF:-0}"
if [[ "$RECORD_TF" == "1" ]]; then
    BAG_TOPICS+=(/tf /tf_static)
fi

# /rosout：ROS2 所有节点日志的汇总话题，每条带节点名 + 精确时间戳，
# 适合按时间窗离线复盘（比纯文本 session.log 更适合对齐事件先后）
if [[ "$RECORD_ROSOUT" == "1" ]]; then
    BAG_TOPICS+=(/rosout)
fi
BAG_PID=""

# 车位定义 / 分析脚本
SLOT_FILE="${SLOT_FILE:-$PKG_DIR/config/parking_slot.yaml}"
ANALYZER="${ANALYZER:-$SCRIPT_DIR/analyze_parking_session.py}"


STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/$STAMP"
SESSION_LOG="$LOG_DIR/session.log"
PLAIN_LOG="$LOG_DIR/session_plain.log"
META_LOG="$LOG_DIR/meta.txt"
ROS_LOG_DIR="$LOG_DIR/ros_log"

mkdir -p "$ROS_LOG_DIR"
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"

start_epoch="$(date +%s)"

# 停止后台任务：发信号 → 轮询等待 → 超时强制 KILL。
# 绝不能直接 wait：若目标进程忽略了该信号，wait 会永久阻塞，
# 导致收尾后续的日志生成、轨迹分析全部执行不到。
stop_bg() {
    local pid="$1" name="$2" sig="${3:-TERM}" tries="${4:-25}" i
    [[ -z "$pid" ]] && return 0
    kill -0 "$pid" 2>/dev/null || return 0
    kill -"$sig" "$pid" 2>/dev/null
    for ((i = 0; i < tries; i++)); do
        kill -0 "$pid" 2>/dev/null || return 0
        sleep 0.2 2>/dev/null || true
    done
    echo "[收尾] $name 未在 $((tries / 5))s 内退出，强制结束" >&2
    kill -KILL "$pid" 2>/dev/null
    return 0
}

# 在新终端窗口里执行一条命令；返回非 0 表示本机没有可用的图形终端
open_terminal() {
    local title="$1" cmd="$2"
    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --title="$title" -- bash -c "$cmd" >/dev/null 2>&1 &
    elif command -v x-terminal-emulator >/dev/null 2>&1; then
        x-terminal-emulator -title "$title" -e bash -c "$cmd" >/dev/null 2>&1 &
    elif command -v xfce4-terminal >/dev/null 2>&1; then
        xfce4-terminal --title="$title" -e "bash -c \"$cmd\"" >/dev/null 2>&1 &
    elif command -v konsole >/dev/null 2>&1; then
        konsole --title "$title" -e bash -c "$cmd" >/dev/null 2>&1 &
    elif command -v xterm >/dev/null 2>&1; then
        xterm -title "$title" -e bash -c "$cmd" >/dev/null 2>&1 &
    else
        return 1
    fi
    return 0
}

finalize() {
    # 收尾期间不再响应 Ctrl+C，避免用户第二次中断导致日志/轨迹丢失
    trap '' INT TERM
    local rc="${1:-0}"
    local end_epoch
    end_epoch="$(date +%s)"

    echo
    echo "[收尾] 正在结束后台任务（录制），随后生成轨迹分析..."

    # 1) 停日志级别轮询
    stop_bg "$LEVEL_PID" "日志级别轮询" TERM 5

    # 1.2) 停"等节点就绪再开监控窗口"的后台任务，避免脚本退出后才弹出窗口
    stop_bg "$WATCH_PID" "状态监控启动器" TERM 5

    # 1.5) 关掉状态监控窗口里跑的 watch_state.py
    if [[ -f "$LOG_DIR/watch_state.pid" ]]; then
        local wp
        wp="$(cat "$LOG_DIR/watch_state.pid" 2>/dev/null)"
        [[ -n "$wp" ]] && kill "$wp" 2>/dev/null
        rm -f "$LOG_DIR/watch_state.pid"
    fi

    # 2) 停 rosbag：必须 SIGINT 才能写完 metadata；最多等 20s，超时才强杀
    if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
        echo
        echo "[轨迹] 正在停止录制..."
        stop_bg "$BAG_PID" "rosbag 录制" INT 100
    fi

    # 录制完成后自动分析：生成轨迹图、CSV、JSON 指标与评估报告
    TRAJ_RC=""
    if [[ "$RECORD_BAG" == "1" && -d "$LOG_DIR/bag" ]]; then
        echo "[轨迹] 正在分析轨迹..."
        local slot_arg=()
        [[ -f "$SLOT_FILE" ]] && slot_arg=(--slot "$SLOT_FILE")
        if [[ -f "$ANALYZER" ]]; then
            # 加超时，避免 bag 异常时分析脚本挂住整个收尾流程
            timeout 180 python3 "$ANALYZER" "$LOG_DIR/bag" --out "$LOG_DIR" \
                "${slot_arg[@]}" >"$LOG_DIR/analyze.log" 2>&1
            TRAJ_RC=$?
        fi
        # 分析成功后直接把结果打到终端：
        # 不用再去翻 evaluation.md，也不用记着怎么找 latest
        if [[ -f "$LOG_DIR/trajectory_info.json" && -f "$SCRIPT_DIR/show_parking_result.py" ]]; then
            python3 "$SCRIPT_DIR/show_parking_result.py" "$LOG_DIR" 2>&1 || true
        fi
    fi

    # 生成清理过 ANSI 控制符与 \r 的纯文本版本
    if [[ -s "$SESSION_LOG" ]]; then
        sed -E 's/\x1B\[[0-9;?]*[A-Za-z]//g; s/\r+\n/\n/g; s/\r/ /g' \
            "$SESSION_LOG" >"$PLAIN_LOG" 2>/dev/null || cp "$SESSION_LOG" "$PLAIN_LOG"
    fi

    {
        echo "end_time: $(date -Is)"
        echo "duration_s: $((end_epoch - start_epoch))"
        echo "exit_code: $rc"
        echo "session_log: $SESSION_LOG"
        echo "plain_log: $PLAIN_LOG"
        echo "ros_log_dir: $ROS_LOG_DIR"
    } >>"$META_LOG"

    echo
    if [[ -f "$SESSION_LOG" ]]; then
        echo "================ 日志已保存 ================"
        echo "原始日志 : $SESSION_LOG  ($(du -h "$SESSION_LOG" 2>/dev/null | cut -f1))"
        echo "纯文本   : $PLAIN_LOG    ($(du -h "$PLAIN_LOG" 2>/dev/null | cut -f1))"
        echo "节点日志 : $ROS_LOG_DIR"
        echo "级别设置 : $LOG_DIR/set_logger_level.log"
        [[ -f "$LOG_DIR/arrival_check.log" ]] && echo "到达判定 : $LOG_DIR/arrival_check.log"
        if [[ -d "$LOG_DIR/bag" ]]; then
            echo "录包数据 : $LOG_DIR/bag  ($(du -sh "$LOG_DIR/bag" 2>/dev/null | cut -f1))"
        fi
        if [[ -f "$LOG_DIR/trajectory_plot.png" ]]; then
            echo "轨迹图   : $LOG_DIR/trajectory_plot.png"
            echo "轨迹数据 : $LOG_DIR/trajectory_info.json + trajectory_points.csv"
            echo "轨迹评估 : $LOG_DIR/evaluation.md"
        elif [[ -n "$TRAJ_RC" ]]; then
            echo "轨迹分析 : 未生成（退出码 $TRAJ_RC），详见 $LOG_DIR/analyze.log"
        fi
        echo "快捷入口 : $LOG_ROOT/latest/"
        echo "查看:  less -R $PLAIN_LOG"
        echo "跟踪:  tail -f $SESSION_LOG"
        echo "检索:  grep -niE 'error|warn|fail|planner' $PLAIN_LOG"
        echo "============================================"
    else
        echo "================ 启动失败 ================"
        echo "本次没有产生终端日志，ros2 launch 未被启动（退出码 $rc）。"
        echo "诊断信息: $META_LOG"
        echo "========================================="
    fi
}
trap 'finalize "$?"' EXIT

# 等节点起来之后，把它的 logger 调到 debug。
# 用 ros2 node list 轮询而不是固定 sleep，避免节点启动慢时调不到。
raise_logger_level() {
    local node="$1" logger="$2" short="$1" i
    short="$(basename "$node")"
    local out="$LOG_DIR/set_logger_level.log"

    for i in $(seq 1 90); do
        if ros2 node list 2>/dev/null | grep -qx -- "$node"; then
            sleep 2 # 等节点内部初始化完、logger 注册好
            # 本机 rcl_interfaces 未提供 SetLoggerLevel.srv，节点可能压根没有这个服务，
            # 先查一下，避免每次都白调一遍还报 "The passed service type is invalid"
            if ! ros2 service list 2>/dev/null | grep -qx -- "${node}/set_logger_level"; then
                {
                    echo "=== $node @ $(date -Is) ==="
                    echo "跳过：该节点未提供 set_logger_level 服务。"
                    echo "原因：本机 rcl_interfaces 只有 6 个 srv，缺少 SetLoggerLevel 定义，"
                    echo "      运行时无法调整日志级别。需要 DEBUG 只能改 launch 传 --ros-args --log-level debug。"
                } >>"$out"
                return 0
            fi
            {
                echo "=== $node @ $(date -Is) ==="
                ros2 service call "${node}/set_logger_level" \
                    rcl_interfaces/srv/SetLoggerLevel \
                    "{logger: '$logger', level: 'debug'}" 2>&1
                # 有些节点 logger 名就是节点名本身，兜底再试一次（失败无害）
                ros2 service call "${node}/set_logger_level" \
                    rcl_interfaces/srv/SetLoggerLevel \
                    "{logger: '$short', level: 'debug'}" 2>&1
            } >>"$out"
            return 0
        fi
        sleep 2
    done

    echo "$(date -Is) 超时: 180s 内未发现节点 $node" >>"$out"
    return 1
}

# 读取 mission_planner 的到达判定阈值并在过宽时提示。
# 为什么只读不改：arrival_check_* 在 MissionPlanner 构造函数初始化列表里读一次
#   就存进成员变量（mission_planner.cpp:76），之后 ros2 param set 改参数值
#   不会更新成员变量，对判定无效；launch 也没把它暴露成可覆盖的 launch 参数。
# 所以在"不改 Autoware 文件"的前提下，脚本只能把当前值读出来告诉你。
check_arrival_thresholds() {
    local node="/planning/mission_planning/mission_planner"
    local out="$LOG_DIR/arrival_check.log"
    local i lat over angle
    local -a items=(
        arrival_check_lateral_distance
        arrival_check_longitudinal_overshoot_distance
        arrival_check_longitudinal_undershoot_distance
        arrival_check_angle_deg
        arrival_check_duration
    )

    for i in $(seq 1 90); do
        if ros2 node list 2>/dev/null | grep -qx -- "$node"; then
            sleep 1
            {
                echo "=== $node @ $(date -Is) ==="
                for p in "${items[@]}"; do
                    ros2 param get "$node" "$p" 2>&1
                done
            } >>"$out"

            lat=$(ros2 param get "$node" arrival_check_lateral_distance 2>/dev/null \
                  | grep -oE '[-0-9.]+$' | tail -1)
            over=$(ros2 param get "$node" arrival_check_longitudinal_overshoot_distance 2>/dev/null \
                   | grep -oE '[-0-9.]+$' | tail -1)
            angle=$(ros2 param get "$node" arrival_check_angle_deg 2>/dev/null \
                    | grep -oE '[-0-9.]+$' | tail -1)

            echo "[到达判定] 横向=${lat:-?} m, 纵向过冲=${over:-?} m, 角度=${angle:-?} deg"
            if awk -v a="${lat:-0}" -v b="$ARRIVAL_LATERAL_WARN" 'BEGIN{exit !(a>b)}' 2>/dev/null ||
               awk -v a="${over:-0}" -v b="$ARRIVAL_OVERSHOOT_WARN" 'BEGIN{exit !(a>b)}' 2>/dev/null; then
                echo "[提示] 到达判定偏宽：车开到目标附近就会被判 ArrivedGoal，泊车轨迹可能来不及执行。"
                echo "       本脚本不会去改 Autoware 文件。要收紧请手动改 planning.launch.xml 中"
                echo "       mission_planner 的 arrival_check_lateral_distance（1.0→0.3）和"
                echo "       arrival_check_longitudinal_overshoot_distance（2.0→0.3），然后重启 launch。"
            else
                echo "[到达判定] 阈值看起来已收紧，无需处理"
            fi
            return 0
        fi
        sleep 2
    done
    echo "$(date -Is) 超时: 180s 内未发现 $node，跳过到达判定检查" >>"$out"
    return 1
}

cd "$AUTOWARE_DIR" || {
    echo "[启动失败] 目录不存在: $AUTOWARE_DIR" >&2
    exit 1
}

if [[ ! -f install/setup.bash ]]; then
    echo "[启动失败] 未找到 $AUTOWARE_DIR/install/setup.bash，请先 colcon build" >&2
    exit 1
fi

CMD=(ros2 launch autoware_launch planning_simulator.launch.xml
    map_path:="$MAP_PATH"
    vehicle_model:="$VEHICLE_MODEL"
    sensor_model:="$SENSOR_MODEL")

# 先落盘启动参数：即使后面环境初始化失败，也留有可诊断的记录
{
    echo "start_time: $(date -Is)"
    echo "autoware_dir: $AUTOWARE_DIR"
    echo "map_path: $MAP_PATH"
    echo "vehicle_model: $VEHICLE_MODEL"
    echo "sensor_model: $SENSOR_MODEL"
    echo "ros_distro: ${ROS_DISTRO:-unknown}"
    echo "command: ${CMD[*]}"
} >"$META_LOG"

echo "命令: ${CMD[*]}"
echo "日志: $SESSION_LOG"
echo

# 必须临时关闭 set -u：autoware 的 install/setup.bash（colcon 模板）内部有
#   if [ -n "$COLCON_TRACE" ]
# 这类对未定义变量的引用，在 set -u 下会以 "未绑定的变量" 直接终止整个脚本，
# 导致 ros2 launch 根本没被执行（曾经的 0 秒退出就是这个原因）
# 这一行会 source 519 个包的 local_setup.sh。
# 缓存热时 <1 s，但系统刚重启、磁盘冷缓存时要 15~20 s，期间没有任何输出，
# 很容易被当成卡死。这里把预期耗时说清楚，并统计实际用时。
step "加载 ROS 环境（source install/setup.bash）..."
echo "       519 个包，热缓存 <1 秒；刚开机首次约 15~20 秒，无输出属正常，请稍候..."
_t_src0=$(date +%s)
set +u
# shellcheck disable=SC1091
source install/setup.bash
setup_rc=$?
set -u
_t_src1=$(date +%s)
step "环境加载完成 (rc=$setup_rc, ROS_DISTRO=${ROS_DISTRO:-空}, 耗时 $((_t_src1 - _t_src0))s)"
if (( _t_src1 - _t_src0 >= 10 )); then
    echo "       本次加载较慢（$((_t_src1 - _t_src0))s）：多为系统刚启动、磁盘缓存为空所致，"
    echo "       下次启动通常只需 1 秒左右，不影响后续运行。"
fi

if [[ "$setup_rc" -ne 0 || -z "${ROS_DISTRO:-}" ]]; then
    echo "[启动失败] source install/setup.bash 出错 (rc=$setup_rc, ROS_DISTRO=${ROS_DISTRO:-空})" >&2
    echo "setup_failed: rc=$setup_rc" >>"$META_LOG"
    exit 1
fi

# ---- DDS 配置：消除 participant 索引上限 ----
# CycloneDDS 默认按 participant 索引分配端口（索引 0-9），而 Autoware 全量启动
# 有几十个独立进程，再加上手动单起的模块，索引很快耗尽，表现为大面积崩溃：
#   Failed to find a free participant index for domain 0
#   rmw_create_node: failed to create domain, error Error
# Autoware 自带的 docker/files/cyclonedds.xml 里 ParticipantIndex=none 正是为了绕开它，
# 但本机没有设置 CYCLONEDDS_URI，一直没生效。这里显式启用。
if [[ -z "${CYCLONEDDS_URI:-}" && "${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" == *cyclonedds* ]]; then
    # 优先用包内自带的配置：官方那份要求 10 MB socket 缓冲，而本机
    # net.core.rmem_max 只有 208 KB，会导致 rmw_create_node 直接失败。
    dds_cfg="$SCRIPT_DIR/../config/cyclonedds.xml"
    [[ -f "$dds_cfg" ]] || dds_cfg="$AUTOWARE_DIR/docker/files/cyclonedds.xml"
    if [[ -f "$dds_cfg" ]]; then
        export CYCLONEDDS_URI="file://$dds_cfg"
        step "DDS 配置：$dds_cfg（索引上限 120；缓冲按系统上限）"
    fi
fi

# ---- 统一 RMW，并停掉用 FastDDS 的 ros2 daemon ----
# 实测本机的 ros2 CLI 守护进程是 rmw_fastrtps_cpp：
#   ros2-daemon ... --ros-domain-id 0 --rmw-implementation rmw_fastrtps_cpp
# FastDDS 与 Autoware 的 CycloneDDS 端口范围都从 7400 起，同占 domain 0 会互相抢。
# 表现就是 rmw_create_node 随机失败、节点大面积 exit code -6，
# 而且时好时坏（取决于 daemon 当时活不活跃），极难定位。
# 这里把 RMW 统一成 cyclonedds，并停掉那个 FastDDS daemon。
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ "$RMW_IMPLEMENTATION" == *cyclonedds* ]]; then
    if ros2 daemon stop >/dev/null 2>&1; then
        step "已停止 ros2 daemon（避免其 FastDDS 占用 domain 0 端口）"
    fi
fi

# ---- 残留进程自动清理 ----
# 多份 Autoware 并存时，旧进程一直占着 DDS participant 索引，新启动的
# mission_planner / route_selector 创建 domain 直接失败（exit code -6 或
# RCLError: rmw handle is invalid），表现就是"地图上有自车，但设了目标没反应"。
#
# 之前这里只打印提示、等 5 秒继续，实际很容易被忽略，然后又是一次全崩。
# 这些进程名足够明确（autoware_* / component_container / rviz2），
# 直接清理不会误伤别的程序。确实需要保留时用 KEEP_ROS=1 跳过。
if [[ "${KEEP_ROS:-0}" != "1" ]]; then
    # 匹配必须用完整命令行（args），不能用 comm：
    #   comm 只有 15 个字符，robot_state_publisher 会显示成 robot_state_pub，
    #   而它的路径是 /opt/ros/humble/lib/robot_state_publisher/..., 不含 "autoware_",
    #   只用 autoware_ 前缀过滤会一直漏掉它，每启动一次就多留一个。
    #   累积之后 duplicated_node_checker 报 ERROR，
    #   /autoware/modes/autonomous 就不可用，AUTO 按钮点不动 —— 正是这个链条。
    # 注意：awk 的"或"是 ||，不是 |（用 | 会直接语法错误）
    PAT='/install\/autoware_/ || /rclcpp_components\/component_container/ || /lib\/robot_state_publisher\/robot_state_publisher/ || /lib\/rviz2\// || /bin\/ros2 launch autoware/ || /simple_planning_simulator/'
    stale_pids="$(ps -eo pid,args --no-headers 2>/dev/null \
                  | awk -v me="$$" "($PAT) && !/awk / && \$1 != me {print \$1}")"
    if [[ -n "$stale_pids" ]]; then
        n_stale=$(echo "$stale_pids" | wc -l)
        echo
        echo "[清理] 检测到 $n_stale 个残留的 Autoware 进程（会抢占 DDS 端口、并触发节点重名告警）："
        ps -eo pid,args --no-headers 2>/dev/null \
            | awk -v me="$$" "($PAT) && !/awk / && \$1 != me {printf \"         %-8s %s\n\", \$1, \$2}" \
            | head -8
        # shellcheck disable=SC2086
        kill -9 $stale_pids 2>/dev/null
        sleep 2
        # DDS 的共享内存与端口锁文件也一并清掉，否则新进程可能沿用旧索引
        rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/cyclonedds_* 2>/dev/null
        step "残留进程已清理（如需保留请设 KEEP_ROS=1）"
    fi
fi

# ROS2 节点级日志也写进本次目录，便于对照终端输出
export ROS_LOG_DIR
# 关键：否则 stdout 接管道后 Python 端会全缓冲，日志会严重滞后甚至丢失
export PYTHONUNBUFFERED=1
# 让子进程输出不因管道而变成块缓冲（部分节点是 C++/Python 混用）
export RCUTILS_LOGGING_BUFFERED_STREAM=0

# 后台任务：调日志级别 + 检查到达判定阈值，均不阻塞前台日志输出
if [[ "$PARKING_DEBUG" == "1" || "$CHECK_ARRIVAL" == "1" ]]; then
    (
        if [[ "$PARKING_DEBUG" == "1" ]]; then
            for item in "${DEBUG_TARGETS[@]}"; do
                raise_logger_level "${item%%|*}" "${item#*|}"
            done
        fi
        if [[ "$CHECK_ARRIVAL" == "1" ]]; then
            check_arrival_thresholds
        fi
    ) &
    LEVEL_PID=$!
    if [[ "$PARKING_DEBUG" == "1" ]]; then
        echo "已启用自动 debug：泊车相关 logger 将在节点就绪后尝试调至 debug 级（本机缺 srv 会跳过）"
    fi
fi

# 后台录制关键话题，退出时自动分析出轨迹图与评估报告（RECORD_BAG=0 可关闭）
if [[ "$RECORD_BAG" == "1" ]]; then
    step "启动录包：${#BAG_TOPICS[@]} 个话题 → $LOG_DIR/bag"
    ros2 bag record -o "$LOG_DIR/bag" "${BAG_TOPICS[@]}" \
        >"$LOG_DIR/record.log" 2>&1 &
    BAG_PID=$!
fi

# 后台：等规划真正出轨迹后，再新开一个终端窗口显示车辆状态
if [[ "$WATCH_STATE" == "1" ]]; then
    (
        WAIT_SEC="${WATCH_WAIT_SEC:-300}"
        mon_log="$LOG_DIR/watch_state_launcher.log"

        # 用 python 直接订阅判断"规划已就绪"。
        # 之前用 ros2 topic list 轮询，CLI 每次都要连 DDS daemon，节点多的时候
        # 经常迟迟返回不了，等满 180 s 窗口才弹，看起来就像"没启动"。
        python3 "$SCRIPT_DIR/wait_planning.py" "$WAIT_SEC" >"$mon_log" 2>&1
        wait_rc=$?

        if [[ "$wait_rc" -eq 0 ]]; then
            mon_msg="规划已就绪，启动状态监控"
        else
            mon_msg="等待 ${WAIT_SEC}s 未检测到轨迹，仍然启动状态监控"
        fi
        {
            echo "[监控] $mon_msg"
            tail -1 "$mon_log" 2>/dev/null | sed 's/^/        /'
        } | tee -a "$SESSION_LOG" 2>/dev/null

        if open_terminal "Autoware 状态监控" \
            "bash '$SCRIPT_DIR/run_watch.sh' '$LOG_DIR'; echo; printf '[监控已退出] 按回车关闭窗口'; read -r _"; then
            echo "[监控] 已在新的终端窗口打开（标题：Autoware 状态监控）" | tee -a "$SESSION_LOG" 2>/dev/null
        else
            echo "[监控] 没有可用的图形终端，改为后台写入 $LOG_DIR/watch_state.log" | tee -a "$SESSION_LOG" 2>/dev/null
            bash "$SCRIPT_DIR/run_watch.sh" "$LOG_DIR" >"$LOG_DIR/watch_state.log" 2>&1 &
        fi
    ) &
    WATCH_PID=$!
fi

# 注意：Autoware 要拉起上百个节点，等第一条日志出来通常需要 10~30 秒，
# 这段时间终端没有任何输出属于正常现象，不是卡死。
step "启动 Autoware（ros2 launch），首条日志约需 10~30 秒..."
set -o pipefail
stdbuf -oL -eL "${CMD[@]}" 2>&1 | tee -a "$SESSION_LOG"
launch_rc=${PIPESTATUS[0]}

exit "$launch_rc"
