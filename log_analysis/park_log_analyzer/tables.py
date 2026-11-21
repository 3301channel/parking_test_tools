"""规范化 CSV 输出（开发文档 §4.3）。"""

from __future__ import annotations

import csv
import os
from typing import Sequence

from . import geometry


def _w(path, header, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def write_tables(ctx, outdir: str) -> None:
    tdir = os.path.join(outdir, "tables")
    os.makedirs(tdir, exist_ok=True)
    ctx.load("grids")
    ctx.load("occs")
    ctx.load("odom")
    ctx.load("traj")
    ctx.load("final_traj")
    ctx.load("completed")
    ctx.load("proc")
    ctx.load("route")
    ctx.load("goal")

    _grid_spec(ctx, tdir)
    _grid_stats(ctx, tdir)
    _grid_cells(ctx, tdir)
    _occ_frames(ctx, tdir)
    _occ_cells(ctx, tdir)
    _goal(ctx, tdir)
    _route(ctx, tdir)
    _odom(ctx, tdir)
    _traj(ctx, tdir)
    _scalars(ctx, tdir)
    _arc_compare(ctx, tdir)
    _control_chain(ctx, tdir)
    _state_timeline(ctx, tdir)


def _grid_spec(ctx, tdir):
    rows = []
    for tag, gf in (("first", ctx.grid_first()), ("last", ctx.grid_last())):
        if gf is None:
            continue
        rows.append([tag, round(gf.t_bag, 6), round(gf.stamp, 6), gf.frame_id, gf.resolution,
                     gf.lx, gf.ly, gf.n_cells, round(gf.center[0], 4), round(gf.center[1], 4),
                     round(gf.center[2], 4), "|".join(gf.layers)])
    _w(os.path.join(tdir, "01_grid_map_spec.csv"),
       ["frame", "bag_time_s", "stamp_s", "frame_id", "resolution", "lx", "ly", "cells",
        "center_x", "center_y", "center_z", "layers"], rows)


def _grid_stats(ctx, tdir):
    rows = []
    for gf in ctx.grids:
        d = gf.digest
        for name in gf.layers:
            rows.append([round(gf.t_bag, 4), round(gf.stamp, 4), name,
                         d.nonzero.get(name, 0), d.nan.get(name, 0), d.dirty.get(name, 0),
                         round(d.vmax.get(name, 0.0), 6)])
    _w(os.path.join(tdir, "02_grid_map_layers_stats.csv"),
       ["bag_time_s", "stamp_s", "layer", "nonzero", "nan", "dirty", "max"], rows)


def _grid_cells(ctx, tdir):
    gf = ctx.grid_last()
    occ = ctx.occ_last()
    if gf is None or occ is None:
        return
    cal = ctx.calibrate()
    res = occ.resolution
    ox, oy = occ.origin[0], occ.origin[1]
    layers = [n for n in gf.layers if n in gf.cells]
    grids = {}
    for name in layers:
        if cal is None:
            m = geometry.reshape(gf.cells[name], gf.lx, "row")
        else:
            m = geometry.apply_transform(geometry.reshape(gf.cells[name], gf.lx, cal[0]), cal[1])
        grids[name] = m
    rows = []
    for row in range(occ.h):
        for col in range(occ.w):
            vals = [round(grids[n][row][col], 6) for n in layers]
            rows.append([col, row, round(ox + (col + 0.5) * res, 3),
                         round(oy + (row + 0.5) * res, 3), *vals])
    _w(os.path.join(tdir, "03_grid_map_last_frame_cells.csv"),
       ["col", "row", "x_m", "y_m", *layers], rows)


def _occ_frames(ctx, tdir):
    rows = []
    for occ in ctx.occs:
        st = geometry.occ_stats(occ)
        rows.append([round(occ.t_bag, 4), round(occ.stamp, 4), occ.w, occ.h, round(occ.resolution, 3),
                     round(occ.origin[0], 4), round(occ.origin[1], 4),
                     st["free"], st["occupied"], st["unknown"], st["other"]])
    _w(os.path.join(tdir, "04_occupancy_grid_frames.csv"),
       ["bag_time_s", "stamp_s", "width", "height", "resolution", "origin_x", "origin_y",
        "free", "occupied", "unknown", "other"], rows)


def _occ_cells(ctx, tdir):
    occ = ctx.occ_last()
    if occ is None:
        return
    res, (ox, oy) = occ.resolution, (occ.origin[0], occ.origin[1])
    rows = []
    for row in range(occ.h):
        for col in range(occ.w):
            rows.append([col, row, round(ox + (col + 0.5) * res, 3),
                         round(oy + (row + 0.5) * res, 3), geometry.occ_value(occ, col, row)])
    _w(os.path.join(tdir, "05_occupancy_grid_last_frame_cells.csv"),
       ["col", "row", "x_m", "y_m", "cost"], rows)


def _goal(ctx, tdir):
    g = ctx.goal
    rows = []
    if g:
        rows.append([round(g["t_bag"], 4), round(g["stamp"], 4), g["frame_id"],
                     round(g["x"], 6), round(g["y"], 6), round(g["z"], 6),
                     round(g["yaw"], 6), round(g["yaw"] * 57.29577951308232, 4)])
    _w(os.path.join(tdir, "06_mission_goal.csv"),
       ["bag_time_s", "stamp_s", "frame_id", "x", "y", "z", "yaw_rad", "yaw_deg"], rows)


def _route(ctx, tdir):
    r = ctx.route
    rows = []
    if r:
        rows.append([round(r.t_bag, 4), round(r.stamp, 4),
                     round(r.start[0], 6), round(r.start[1], 6), round(r.start[3], 6),
                     round(r.goal[0], 6), round(r.goal[1], 6), round(r.goal[2], 6),
                     round(r.goal[3], 6), len(r.segments), r.n_primitives, r.uuid,
                     int(r.allow_modification)])
    _w(os.path.join(tdir, "07_lanelet_route.csv"),
       ["bag_time_s", "stamp_s", "start_x", "start_y", "start_yaw", "goal_x", "goal_y",
        "goal_z", "goal_yaw", "segments", "primitives", "uuid", "allow_modification"], rows)
    seg_rows = []
    if r:
        for i, (pid, ptype, prims) in enumerate(r.segments):
            seg_rows.append([i, pid, ptype, len(prims),
                             "|".join(str(p[0]) for p in prims),
                             "|".join(p[1] for p in prims)])
    _w(os.path.join(tdir, "08_lanelet_route_segments.csv"),
       ["index", "preferred_id", "preferred_type", "primitives", "primitive_ids",
        "primitive_types"], seg_rows)


def _odom(ctx, tdir):
    rows = [[round(o.t_bag, 4), round(o.stamp, 4), round(o.x, 6), round(o.y, 6),
             round(o.yaw, 6), round(o.vx, 6), round(o.vy, 6), round(o.wz, 6)] for o in ctx.odom]
    _w(os.path.join(tdir, "09_kinematic_state.csv"),
       ["bag_time_s", "stamp_s", "x", "y", "yaw", "vx", "vy", "wz"], rows)


def _traj(ctx, tdir):
    def dump(seq, prefix):
        rows, frames = [], []
        for fi, t in enumerate(seq):
            arc = geometry.arclength(t.points)
            vs = [p.v_long for p in t.points]
            for k, p in enumerate(t.points):
                rows.append([fi, round(t.t_bag, 4), round(t.stamp, 4), k, round(p.x, 6),
                             round(p.y, 6), round(p.z, 6), round(p.yaw, 6), round(arc[k], 5),
                             round(p.v_long, 5), round(p.v_lat, 5), round(p.w_heading, 6),
                             round(p.accel, 5), round(p.front_steer, 6), round(p.rear_steer, 6),
                             round(p.t_from_start, 6)])
            frames.append([fi, round(t.t_bag, 4), round(t.stamp, 4), len(t.points),
                           round(arc[-1], 5) if arc else 0.0,
                           round(min(vs), 4) if vs else 0.0,
                           round(max(vs), 4) if vs else 0.0])
        _w(os.path.join(tdir, "%s.csv" % prefix),
           ["frame", "bag_time_s", "stamp_s", "index", "x", "y", "z", "yaw", "s",
            "v_long", "v_lat", "w_heading", "accel", "front_steer", "rear_steer",
            "t_from_start"], rows)
        _w(os.path.join(tdir, "%s_frames.csv" % prefix),
           ["frame", "bag_time_s", "stamp_s", "points", "arclength", "v_min", "v_max"], frames)

    dump(ctx.traj, "10_freespace_trajectory")
    dump(ctx.final_traj, "11_planning_trajectory")


def _scalars(ctx, tdir):
    _w(os.path.join(tdir, "13_freespace_is_completed.csv"),
       ["bag_time_s", "data"],
       [[round(p.t_bag, 4), int(p.value)] for p in ctx.completed])
    _w(os.path.join(tdir, "14_freespace_processing_time_ms.csv"),
       ["bag_time_s", "stamp_s", "data_ms"],
       [[round(p.t_bag, 4), round(p.stamp, 4), round(p.value, 4)] for p in ctx.proc_ms])


def _control_chain(ctx, tdir):
    """规划目标 → 控制指令 → 实际速度，同一时间轴上并排。

    这张表是"车为什么不动"最直接的证据：target 有值、cmd 为 0 是抑制；
    cmd 有值、actual 为 0 是执行层问题；三者都有但方向反了是接口符号问题。
    """
    ctx.load("control")
    ctx.load("final_traj")
    ctx.load("odom")
    traj = ctx.final_traj or []
    rows = []
    for c in ctx.control_cmd:
        target = None
        if traj:
            near = min(traj, key=lambda f: abs(f.t_bag - c["t_bag"]))
            if abs(near.t_bag - c["t_bag"]) <= 1.0 and near.points:
                target = max(abs(p.v_long) for p in near.points)
        actual = ctx.signed_speed_at(c["t_bag"])
        rows.append([round(c["t_bag"], 4), round(c["stamp"], 4),
                     None if target is None else round(target, 4),
                     round(c["velocity"], 4), round(c["acceleration"], 4),
                     None if c.get("steering") is None else round(c["steering"], 5),
                     None if actual is None else round(actual, 4)])
    _w(os.path.join(tdir, "16_control_chain.csv"),
       ["bag_time_s", "stamp_s", "planner_target_v", "control_cmd_v",
        "control_cmd_accel", "control_cmd_steer", "actual_v"], rows)


def _state_timeline(ctx, tdir):
    """状态机 / 操作模式 / 运动状态 / 路由 / 档位 / 闸门 的翻转表。"""
    from .context import transitions

    ctx.load("aw_state")
    ctx.load("modes")
    ctx.load("gear")
    ctx.load("gate")
    rows = []
    for label, seq, key, names in (
        ("autoware_state", ctx.aw_state, "state_name", None),
        ("operation_mode", ctx.operation_mode, "mode_name", None),
        ("motion_state", ctx.motion, "state_name", None),
        ("routing_state", ctx.routing, "state_name", None),
        ("gear", ctx.gear_status, "gear_name", None),
        ("is_stopped", ctx.is_stopped, "value", {1.0: "STOPPED", 0.0: "RELEASED"}),
        ("is_start_requested", ctx.is_start_requested, "value",
         {1.0: "REQUESTED", 0.0: "IDLE"}),
    ):
        for t in transitions(seq, key, names):
            rows.append([round(t["t_s"], 4), label, str(t["name"])])
    rows.sort(key=lambda r: (r[0], r[1]))
    _w(os.path.join(tdir, "17_state_timeline.csv"),
       ["bag_time_s", "channel", "state"], rows)


def _arc_compare(ctx, tdir):
    if not ctx.traj or not ctx.final_traj:
        _w(os.path.join(tdir, "15_trajectory_arc_compare.csv"),
           ["index", "s", "source_x", "source_y", "deviation_m"], [])
        return
    src = ctx.traj[-1]
    fin = min(ctx.final_traj, key=lambda f: abs(f.stamp - src.stamp))
    sa = geometry.arclength(src.points)
    devs = geometry.align_by_arclength(src.points, fin.points)
    rows = [[k, round(sa[k], 5), round(p.x, 6), round(p.y, 6), round(d, 5)]
            for k, (p, d) in enumerate(zip(src.points, devs))]
    _w(os.path.join(tdir, "15_trajectory_arc_compare.csv"),
       ["index", "s", "source_x", "source_y", "deviation_m"], rows)
