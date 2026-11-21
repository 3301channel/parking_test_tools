# 泊车日志分析报告 —— 20261113_102432

- bag 路径：`/home/cn/autoware_logs/20261113_102432/bag/bag_0.mcap`
- 时长：134.41 s，话题 27 个，消息 40020 条，payload 450.61 MB
- 栅格标定：`row` + `rot180`，一致率 100.00%（0 格不一致）

## 0. 一句话结论

**WARN** —— 泊车未完成：车辆几乎未运动（0.63 m / 134 s，最高 0.09 m/s，轨迹目标 1.39 m/s），435 帧轨迹完全相同（rosout ERROR 31 条，diagnostics 非 OK 2027 条，最小余量 1.80 m）

失败 0 条、告警 4 条；详见下方分组。

### 0.1 关联观察（自动推断的因果链）

1. **规划侧没有明显异常**：规划成功率 435 帧轨迹 / 期望 438 周期，耗时 p95 25.51 ms，曲率占极限 43%，最小余量 1.80 m。
2. **执行侧没有跟随**：车辆几乎未运动：0.63 m / 134.4 s，最高速度仅 0.09 m/s（轨迹目标 1.39 m/s，执行未跟随）。
3. **规划输出被冻结**：连续 435 帧轨迹完全相同且车辆静止（未推进），说明每周期都在算同一条第一段轨迹，而非重规划震荡。
4. **控制指令侧报错**：diagnostics 中 `topic_state_monitor_control_command_control_cmd: control_topic_status`、`topic_state_monitor_trajectory_follower_control_cmd: control_topic_status`；`control_command` 类监控为 ERROR 表示下游没有收到控制指令。
5. **模式/门控**：rosout 顶部错误 “Failed transform to map.”、“[VehicleCmdFilter] Acceleration change limit by longitudinal jerk: constraint(de”；`vehicle_cmd_gate` 的限幅日志说明指令被门控裁剪，结合 S-01 无 `/tf`、`object_lanelet_filter` 的 “Failed transform to map”，判断为自动驾驶模式未进入或输入不完整，而不是泊车算法本身失败。
6. **注意**：`parking/trajectory` 只发分段的第一段（4 点 / 2.83 m），不能据此判断全程可达性。

## 1. 数据清单（含缺失/空话题归因）

| 话题 | 类型 | 条数 | 大小 | 角色 |
|---|---|---|---|---|
| `/api/motion/state` | autoware_adapi_v1_msgs/MotionState | 6 | 96 B | other |
| `/autoware/state` | autoware_system_msgs/AutowareState | 576 | 9.0 KB | other |
| `/clock` | rosgraph_msgs/Clock | 1152 | 13.5 KB | other |
| `/control/command/control_cmd` | autoware_control_msgs/Control | 865 | 67.6 KB | other |
| `/control/command/gear_cmd` | autoware_vehicle_msgs/GearCommand | 574 | 9.0 KB | other |
| `/control/vehicle_cmd_gate/is_filter_activated` | autoware_vehicle_cmd_gate/IsFilterActivated | 865 | 16.9 KB | other |
| `/control/vehicle_cmd_gate/is_start_requested` | tier4_control_msgs/IsStartRequested | 5 | 80 B | other |
| `/control/vehicle_cmd_gate/is_stopped` | tier4_control_msgs/IsStopped | 1 | 20 B | other |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | 27238 | 11.0 MB | diagnostic |
| `/localization/kinematic_state` | nav_msgs/Odometry | 981 | 685.9 KB | planner_input |
| `/map/vector_map_marker` | visualization_msgs/MarkerArray | 1 | 394.9 KB | other |
| `/perception/object_recognition/objects` | autoware_perception_msgs/PredictedObjects | 538 | 5.4 MB | costmap_input |
| `/planning/mission_planning/goal` | geometry_msgs/PoseStamped | 1 | 76 B | upstream |
| `/planning/mission_planning/route` | autoware_planning_msgs/LaneletRoute | 1 | 296 B | planner_input |
| `/planning/planning_validator/validation_status` | autoware_planning_validator/PlanningValidatorStatus | 435 | 86.7 KB | other |
| `/planning/route_state` | autoware_planning_msgs/RouteState | 4 | 64 B | other |
| `/planning/scenario_planning/parking/costmap_generator/grid_map` | grid_map_msgs/GridMap | 490 | 406.1 MB | costmap_output |
| `/planning/scenario_planning/parking/costmap_generator/occupancy_grid` | nav_msgs/OccupancyGrid | 490 | 25.4 MB | costmap_output |
| `/planning/scenario_planning/parking/freespace_planner/debug/processing_time_ms` | autoware_internal_debug_msgs/Float64Stamped | 435 | 8.5 KB | planner_output |
| `/planning/scenario_planning/parking/is_completed` | std_msgs/Bool | 141 | 1.1 KB | planner_output |
| `/planning/scenario_planning/parking/trajectory` | autoware_planning_msgs/Trajectory | 435 | 161.4 KB | planner_output |
| `/planning/scenario_planning/scenario` | autoware_internal_planning_msgs/Scenario | 438 | 13.7 KB | planner_input |
| `/planning/trajectory` | autoware_planning_msgs/Trajectory | 435 | 1.1 MB | final_output |
| `/rosout` | rcl_interfaces/Log | 460 | 122.7 KB | diagnostic |
| `/vehicle/status/gear_status` | autoware_vehicle_msgs/GearReport | 1151 | 18.0 KB | other |
| `/vehicle/status/steering_status` | autoware_vehicle_msgs/SteeringReport | 1151 | 18.0 KB | other |
| `/vehicle/status/velocity_status` | autoware_vehicle_msgs/VelocityReport | 1151 | 45.0 KB | other |

## 2. costmap 诊断

| 规则 | 结论 | 说明 |
|---|---|---|
| C-01 输出话题存在且非空 | PASS | grid_map 490 帧 / occupancy_grid 490 帧 |
| C-02 发布频率 | PASS | 各话题频率在容差内（按 header.stamp 跨度计算）；bag 时钟/仿真时钟 ≈ 0.55，bag 时间下频率偏低属正常 |
| C-03 栅格几何合法 | PASS | 233x233 @ 0.30 m，格数与数据长度一致 |
| C-04 occupancy 与 grid_map.combined 一致 | PASS | 逐格一致率 100.00%（row + rot180，0 格不一致） |
| C-05 层活跃度 | INFO | points 非零 251 格；objects 非零 768 格；primitives 非零 26011 格；combined 非零 26866 格 |
| C-06 帧间变化率 | INFO | 0/489 对相邻帧完全相同，最大变化率 2.12% |
| C-07 未知格比例 | INFO | 未知格 0.00%，自由 50.5% / 占据 48.2% |
| C-08 栅格中心跟随自车 | PASS | 偏差 0.139 m（小于 0.30 m） |
| C-09 自车所在格可通行 | PASS | 自车格 cost=0，combined=0 |
| C-10 车位区填充比例 | INFO | 自由格 51.45% / 占据 48.55% |
| C-11 脏值格检测 | PASS | 未发现 NaN / abs(v)>100 的脏值格 |
| C-12 层间数据来源可解释性 | PASS | objects 消息有 471 条非空，层数据可解释 |

**频率（按各话题自身跨度）**

| 话题 | 条数 | 首个 (s) | 末个 (s) | 跨度 (s) | bag 频率 | stamp 频率 | 期望 | 时钟比 |
|---|---|---|---|---|---|---|---|---|
| grid_map | 490 | 46.140 | 134.332 | 88.192 | 5.54 | 10.00 | 10.0 | 0.55 |
| occupancy_grid | 490 | 46.139 | 134.330 | 88.191 | 5.54 | 10.00 | 10.0 | 0.55 |
| trajectory | 435 | 55.479 | 134.348 | 78.869 | 5.50 | 10.02 | 10.0 | 0.55 |
| odometry | 981 | 45.970 | 134.326 | 88.356 | 11.09 | 20.00 | - | 0.56 |
| final_trajectory | 435 | 55.510 | 134.350 | 78.839 | 5.50 | 10.02 | - | 0.55 |

**末帧层活跃度**：points 非零 251 格；objects 非零 768 格；primitives 非零 26011 格；combined 非零 26866 格

## 3. 规划器诊断（输入 → 输出）

| 规则 | 结论 | 说明 |
|---|---|---|
| P-01 三路输入齐备 | PASS | route 54.510 s / occupancy 46.139 s / odom 45.970 s 均已到达 |
| P-02 激活信号 | PASS | Parking 激活，首条 54.678 s，共 1 段 |
| P-03 启动时延 | PASS | 启动时延 0.802 s |
| P-04 规划成功率 | PASS | 435 帧轨迹 / 期望 438 周期（99%） |
| P-05 规划耗时 | PASS | p50 21.64 ms / p95 25.51 ms / max 635.47 ms |
| P-06 首帧冷启动 | INFO | 首帧 635.47 ms 是 p50 的 29.4 倍（首次构建碰撞表） |
| P-07 goal 落在自由格 | PASS | goal 格 cost=0 |
| P-08 goal 在栅格内 | PASS | goal 落在栅格范围内 |
| P-09 起终点连通 | PASS | 4 邻域 BFS 连通 |
| P-10 is_completed 语义 | INFO | 141 条 false / 0 条 true，其中 139 条在激活前 |
| P-11 重规划次数 | INFO | 点数变化 0 次，时间断档 0 次（推断重规划/重启） |
| P-12 停车轨迹注入 | INFO | 无单点停车轨迹帧 |

## 4. 轨迹体检

| 规则 | 结论 | 说明 |
|---|---|---|
| T-01 轨迹非空且点数合理 | PASS | 435 帧 × 4~4 点 |
| T-02 弧长与步长 | PASS | 弧长 2.833 m，步长 0.904~0.990 m（均值 0.944） |
| T-03 速度剖面合法 | PASS | 速度取值 2 种，末点 0.00，含倒车段 |
| T-04 time_from_start 可用性 | INFO | time_from_start 全部为 0 → 禁用该字段，到达时刻须用弧长/速度积分 |
| T-05 曲率 vs 车辆极限 | PASS | 最大曲率 0.1306（R_min 7.66 m），极限 0.302，占 43% |
| T-06 几何余量 | PASS | 最小余量 1.800 m（第 0 点） |
| T-07 轨迹不越出可泊区 | PASS | 435 帧轨迹全部落在自由格 |
| T-08 分片连续性 | PASS | 分段轨迹每周期从自车重规划，首点距自车 ≤ 0.62 m |
| T-09 原地空转检测 | WARN | 连续 435 帧轨迹完全相同且车辆静止（未推进） |
| T-10 终点误差 | INFO | 分段轨迹 4 点 / 2.83 m，距 goal 10.58 m —— 只发了第一段（分段规划的正常表现） |
| T-11 源轨迹 vs 最终轨迹 | PASS | 弧长 2.833 vs 2.830 m，最大横向偏差 0.044 m |
| T-12 最终轨迹速度单调性 | PASS | 首点 -0.25 → 峰值(abs) 0.81 → 末点 -0.00 |
| T-13 帧时间戳单调 | PASS | 两条轨迹的 header.stamp 严格递增 |

## 5. 时序与系统

| 规则 | 结论 | 说明 |
|---|---|---|
| S-01 tf 与 tf_static | SKIP | bag 内无 /tf 与 /tf_static → 所有需要坐标变换的规则降级为 SKIP |
| S-02 时间戳一致性 | INFO | grid_map、occupancy_grid、trajectory、odometry 的 header.stamp 与 bag 时间存在固定零时刻差，时钟比约 0.56（仿真时钟 vs 录制时钟，不是真实延迟；频率判定已按 stamp 计算） |
| S-03 乱序检测 | PASS | 轨迹话题 header.stamp 单调不减 |
| S-04 rosout 错误聚类 | WARN | 31 条 ERROR/FATAL，涉及 4 个节点 |
| S-05 diagnostics 汇总 | WARN | 2027/47103 条诊断非 OK：gyro_bias_scale_validator(WARN)×576、topic_state_monitor_control_command_control_cmd(ERROR)×144、topic_state_monitor_trajectory_follower_control_cmd(ERROR)×144 |
| S-06 车辆是否运动 | WARN | 车辆几乎未运动：0.63 m / 134.4 s，最高速度仅 0.09 m/s（轨迹目标 1.39 m/s，执行未跟随） |
| S-07 车辆模式 | INFO | /api/motion/state=6；/autoware/state=576；/planning/route_state=4 |
| S-08 空话题归因 | INFO | 没有空话题 |

## 6. 未验证项（这份 bag 证明不了什么）

- **轨迹推进**：连续 435 帧轨迹完全相同且车辆静止（未推进）。
- **坐标链**：bag 内无 `/tf`、`/tf_static`，无法验证 map→base_link 的变换正确性。
- **控制执行**：`/control/*` 仅作时间线旁证，未对控制律与底层执行做评价。

## 7. 建议动作（按优先级）

1. **先查执行链，不是规划**：规划下发 1.39 m/s，实际最高 0.09 m/s、位移 0.63 m。依次确认 `/autoware/state` 是否 ENGAGED、`/control/vehicle_cmd_gate/is_start_requested` 与 `is_filter_activated`、`/control/command/control_cmd` 是否连续发布。
2. **修复控制话题监控 ERROR**：`topic_state_monitor_control_command_control_cmd: control_topic_status`、`topic_state_monitor_trajectory_follower_control_cmd: control_topic_status`；说明控制指令没有到达执行层，与 `topic_state_monitor` 的 ERROR 一致。
3. **处理 rosout 顶部错误**：“Failed transform to map.”×18；“[VehicleCmdFilter] Acceleration change limit by longitudinal”×4。
4. **复核** T-09（原地空转检测）：连续 435 帧轨迹完全相同且车辆静止（未推进）
5. 补齐 `/tf`、`/tf_static` 与点云/有效 `objects` 后重跑，让 S-01、避障与重规划分支可判定。

---

*由 `park_log_analyzer 1.0.0` 生成；所有阈值见 `report.json` 的 `meta.config_snapshot`。*
