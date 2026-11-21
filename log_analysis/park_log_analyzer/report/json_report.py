"""report.json（开发文档 §9.1）。"""

from __future__ import annotations

import json
import os

from .. import __version__

ROLE = {
    "grid_map": "costmap_output",
    "occupancy_grid": "costmap_output",
    "trajectory": "planner_output",
    "final_trajectory": "final_output",
    "is_completed": "planner_output",
    "processing_time": "planner_output",
    "scenario": "planner_input",
    "route": "planner_input",
    "goal": "upstream",
    "odometry": "planner_input",
    "objects": "costmap_input",
    "rosout": "diagnostic",
    "diagnostics": "diagnostic",
    "tf": "transform",
    "tf_static": "transform",
    # 执行链
    "control_cmd": "control_output",
    "follower_cmd": "control_output",
    "actuation_cmd": "actuation_command",
    "shift_gear_cmd": "control_output",
    "gear_status": "vehicle_state",
    "gear_command": "control_output",
    "is_stopped": "gate_state",
    "is_start_requested": "gate_state",
    "is_autonomous_available": "gate_state",
    "operation_mode": "state_machine",
    "motion_state": "state_machine",
    "routing_state": "state_machine",
    "autoware_state": "state_machine",
    "velocity_status": "vehicle_state",
    "steering_status": "vehicle_state",
}

REQUIRED = ("grid_map", "occupancy_grid", "trajectory", "is_completed", "scenario", "route", "goal", "odometry")


def build(ctx, results, verdict_level: str, headline: str) -> dict:
    roles = {v: k for k, v in ctx.cfg["topics"].items()}
    topics = []
    for name, meta in sorted(ctx.topics.items()):
        key = roles.get(name)
        topics.append({
            "name": name,
            "type": meta.short_type,
            "count": meta.count,
            "bytes": meta.nbytes,
            "role": ROLE.get(key, "other") if key else "other",
        })
    missing = [ctx.topic(k) for k in REQUIRED if not ctx.has_topic(k)]
    empty = [{"name": m["name"], "reason": m["reason"]} for m in
             ([{"name": n, "reason": "record_rejected_or_no_publisher"}
               for n, meta in sorted(ctx.topics.items()) if meta.count == 0])]
    cal = ctx.calibration
    bag_path = getattr(ctx.reader, "path", "bag")
    name = os.path.basename(os.path.normpath(bag_path))
    if name.endswith(".mcap") or name.endswith(".db3"):
        # 用 session 目录名（如 20261113_102432）而不是 bag_0.mcap
        parent = os.path.dirname(os.path.normpath(bag_path))
        name = os.path.basename(parent)
        if name in ("bag", "bags"):
            name = os.path.basename(os.path.dirname(parent))
    return {
        "meta": {
            "tool_version": __version__,
            "bag": name,
            "bag_path": getattr(ctx.reader, "path", ""),
            "bag_duration_s": round(ctx.bag_duration, 3),
            "topics": len(ctx.topics),
            "messages": sum(m.count for m in ctx.topics.values()),
            "payload_bytes": sum(m.nbytes for m in ctx.topics.values()),
            "config_snapshot": {
                "thresholds": ctx.cfg.get("thresholds", {}),
                "vehicle": ctx.cfg.get("vehicle", {}),
            },
            "grid_calibration": None if cal is None else {
                "layout": cal[0], "transform": cal[1],
                "agreement": round(cal[2], 4), "mismatch_cells": cal[3],
            },
            "grid_calibration_error": ctx.calibration_error,
        },
        "inventory": {"topics": topics, "missing_required": missing, "empty_topics": empty},
        "results": [r.as_dict() for r in results],
        "verdict": {"level": verdict_level, "headline": headline},
    }


def write(report: dict, outdir: str) -> str:
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, "report.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
        f.write("\n")
    return path
