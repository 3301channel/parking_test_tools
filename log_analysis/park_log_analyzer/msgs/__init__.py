"""msg 解码注册表：按 `包名/msg/类型` 或 `包名/类型` 分派。"""

from __future__ import annotations

from typing import Callable, Dict

from . import (control, grid_map, lanelet_route, logmsg, objects, occupancy_grid,
               odometry, scalar, scenario, trajectory)


def _norm(t: str) -> str:
    return t.replace("/msg/", "/")


DECODERS: Dict[str, Callable[..., object]] = {
    "grid_map_msgs/GridMap": grid_map.decode,
    "nav_msgs/OccupancyGrid": occupancy_grid.decode,
    "nav_msgs/Odometry": odometry.decode,
    "autoware_planning_msgs/Trajectory": trajectory.decode,
    "autoware_planning_msgs/LaneletRoute": lanelet_route.decode,
    "std_msgs/Bool": scalar.decode_bool,
    "autoware_internal_debug_msgs/Float64Stamped": scalar.decode_float64_stamped,
    "geometry_msgs/PoseStamped": scalar.decode_pose_stamped,
    "autoware_internal_planning_msgs/Scenario": scenario.decode,
    "autoware_perception_msgs/PredictedObjects": objects.decode,
    "rcl_interfaces/Log": logmsg.decode_log,
    "diagnostic_msgs/DiagnosticArray": logmsg.decode_diagnostics,
    "autoware_vehicle_msgs/GearReport": scalar.decode_gear,
    "autoware_vehicle_msgs/GearCommand": scalar.decode_gear,
    # ---- 控制链 / 执行层（D-* 规则的数据来源）----
    "autoware_control_msgs/Control": control.decode_control,
    "autoware_vehicle_msgs/VelocityReport": control.decode_velocity_report,
    "autoware_vehicle_msgs/SteeringReport": control.decode_steering_report,
    "autoware_vehicle_msgs/ActuationCommand": control.decode_actuation_command,
    "autoware_system_msgs/AutowareState": control.decode_autoware_state,
    "autoware_adapi_v1_msgs/OperationModeState": control.decode_operation_mode,
    "autoware_adapi_v1_msgs/MotionState": control.decode_motion_state,
    "autoware_adapi_v1_msgs/RoutingState": control.decode_routing_state,
    # 以下三个在不同版本里都是"首个字段 bool data"，可直接复用 Bool 解码
    "tier4_control_msgs/IsStopped": scalar.decode_bool,
    "tier4_control_msgs/IsStartRequested": scalar.decode_bool,
    "tier4_control_msgs/IsAutonomousAvailable": scalar.decode_bool,
    "autoware_control_msgs/IsAutonomousAvailable": scalar.decode_bool,
    "std_msgs/BoolStamped": scalar.decode_bool_stamped,
}

NO_HEADER = {"std_msgs/Bool", "autoware_internal_debug_msgs/Float64Stamped",
             "autoware_system_msgs/AutowareState",
             "autoware_vehicle_msgs/ActuationCommand"}


def get_decoder(msgtype: str) -> Callable[..., object]:
    key = _norm(msgtype)
    try:
        return DECODERS[key]
    except KeyError:
        raise KeyError("未支持的 msg 类型：%s" % msgtype) from None


def has_decoder(msgtype: str) -> bool:
    return _norm(msgtype) in DECODERS
