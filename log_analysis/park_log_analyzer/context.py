"""AnalysisContext：惰性解码 + 派生量（开发文档 §5.2）。

内存策略（§4.2）：grid_map 只保留首帧、末帧与变化帧的 cells，
其余帧只留 digest；`--keep-all-grids` 可关闭该优化。
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from . import geometry
from .msgs import get_decoder, has_decoder
from .model import GridDigest, GridFrame, OccFrame, OdomFrame, RouteInfo, ScalarPoint, TrajFrame, VehicleSpec

try:  # 可选加速；企业 DLP 环境可能损坏 numpy，故整体 try
    import numpy as _np
except Exception:  # pragma: no cover
    _np = None


MAX_CHANGED_GRIDS = 12


def _attr(item, key):
    if isinstance(item, dict):
        return item.get(key)
    return getattr(item, key, None)


def _t_of(item) -> float:
    return item["t_bag"] if isinstance(item, dict) else float(item.t_bag)


def transitions(seq, key: str, names: Optional[Dict[int, str]] = None) -> List[dict]:
    """把一串带状态字段的消息压成状态变化表。

    返回 [{"t_s", "value", "name"}]，只保留"值发生变化"的点，
    第一条一定在表里（用于回答"一开始是什么状态"）。
    """
    out: List[dict] = []
    prev = object()
    for it in seq:
        value = _attr(it, key)
        name = names.get(value, str(value)) if names else value
        if name != prev:
            out.append({"t_s": round(_t_of(it), 3), "value": value, "name": name})
            prev = name
    return out


def value_at(seq, t: float, key: str = "value"):
    """取 t 时刻（含）之前最后一条的值；没有则返回 None。"""
    found = None
    for it in seq:
        if _t_of(it) <= t:
            found = _attr(it, key)
        else:
            break
    return found


def total_seconds(seq, key: str, names: Optional[Dict[int, str]] = None,
                  until: Optional[float] = None) -> Dict[str, float]:
    """统计每个状态累计持续了多久（按消息时间戳切分）。"""
    out: Dict[str, float] = {}
    for cur, nxt in zip(seq, seq[1:]):
        v = _attr(cur, key)
        name = names.get(v, str(v)) if names else str(v)
        out[name] = out.get(name, 0.0) + max(0.0, _t_of(nxt) - _t_of(cur))
    if seq:
        end = until if until is not None else _t_of(seq[-1])
        v = _attr(seq[-1], key)
        name = names.get(v, str(v)) if names else str(v)
        out[name] = out.get(name, 0.0) + max(0.0, end - _t_of(seq[-1]))
    return {k: round(v, 3) for k, v in out.items()}


def _digest_np(gf: GridFrame) -> GridDigest:
    d = GridDigest(t_bag=gf.t_bag, stamp=gf.stamp)
    for name, arr in gf.cells.items():
        a = _np.frombuffer(arr, dtype=_np.float32)
        finite = _np.isfinite(a)
        dirty = int((~finite | (_np.abs(a) > 100)).sum())
        ok = finite & (_np.abs(a) <= 100)
        d.nonzero[name] = int((_np.abs(a[ok]) > 1e-9).sum())
        d.nan[name] = int((~finite).sum())
        d.dirty[name] = dirty
        d.vmax[name] = float(a[finite].max()) if bool(finite.any()) else 0.0
    d.signature = "|".join("%d:%d" % (d.nonzero.get(k, 0), d.dirty.get(k, 0)) for k in gf.layers)
    return d


def _digest_py(gf: GridFrame) -> GridDigest:
    d = GridDigest(t_bag=gf.t_bag, stamp=gf.stamp)
    for name, arr in gf.cells.items():
        nan = dirty = nz = 0
        vmax = 0.0
        for v in arr:
            if v != v:
                nan += 1
                continue
            av = abs(v)
            if av > 100:
                dirty += 1
                continue
            if av > 1e-9:
                nz += 1
            if v > vmax:
                vmax = v
        d.nonzero[name] = nz
        d.nan[name] = nan
        d.dirty[name] = dirty
        d.vmax[name] = vmax
    d.signature = "|".join("%d:%d" % (d.nonzero.get(k, 0), d.dirty.get(k, 0)) for k in gf.layers)
    return d


def _count_diff(a: Dict[str, object], b: Dict[str, object], layers) -> int:
    total = 0
    if _np is not None:
        for name in layers:
            if name not in a or name not in b:
                continue
            x = _np.frombuffer(a[name], dtype=_np.float32)
            y = _np.frombuffer(b[name], dtype=_np.float32)
            if len(x) != len(y):
                return len(x)
            total += int((x != y).sum())
        return total
    for name in layers:
        if name not in a or name not in b:
            continue
        ba = bytes(memoryview(a[name]).cast("B"))
        bb = bytes(memoryview(b[name]).cast("B"))
        if ba == bb:
            continue
        total += sum(1 for i in range(0, len(ba), 4) if ba[i:i + 4] != bb[i:i + 4])
    return total


class AnalysisContext:
    def __init__(self, reader, cfg: dict, keep_all_grids: bool = False):
        self.reader = reader
        self.cfg = cfg
        self.keep_all_grids = keep_all_grids
        self.topics = reader.topics()
        self.start_ns = reader.start_ns()
        self.vehicle = VehicleSpec(**cfg.get("vehicle", {}))

        self.grids: List[GridFrame] = []
        self.grid_digests: List[GridDigest] = []
        self.grid_changes: List[Tuple[float, int, bool]] = []
        self.occs: List[OccFrame] = []
        self.odom: List[OdomFrame] = []
        self.traj: List[TrajFrame] = []
        self.final_traj: List[TrajFrame] = []
        self.completed: List[ScalarPoint] = []
        self.proc_ms: List[ScalarPoint] = []
        self.scenario: List[dict] = []
        self.objects: List[dict] = []
        self.rosout: List[dict] = []
        self.diagnostics: List[dict] = []
        self.gear_status: List[dict] = []
        self.gear_command: List[dict] = []
        # —— 控制链 / 状态机（D-* 规则）——
        self.control_cmd: List[dict] = []
        self.follower_cmd: List[dict] = []
        self.actuation: List[dict] = []
        self.is_stopped: List[ScalarPoint] = []
        self.is_start_requested: List[ScalarPoint] = []
        self.is_auto_available: List[ScalarPoint] = []
        self.operation_mode: List[dict] = []
        self.motion: List[dict] = []
        self.routing: List[dict] = []
        self.aw_state: List[dict] = []
        self.velocity: List[dict] = []
        self.steering: List[dict] = []
        self.shift_gear: List[dict] = []
        self.goal: Optional[dict] = None
        self.route: Optional[RouteInfo] = None

        self.calibration: Optional[Tuple[str, str, float, int]] = None
        self.calibration_error: Optional[str] = None
        self._edt = None
        self._edt_key = None
        self.warnings: List[str] = []

    # -- 时间 ------------------------------------------------------------
    def rel(self, ns: int) -> float:
        return (ns - self.start_ns) * 1e-9

    @property
    def bag_duration(self) -> float:
        end = self.reader.end_ns()
        return max(0.0, (end - self.start_ns) * 1e-9)

    # -- 配置 ------------------------------------------------------------
    def topic(self, key: str) -> str:
        # 自定义配置可能只写了部分话题：缺的返回空串，让规则走 SKIP 而不是 KeyError
        return self.cfg["topics"].get(key, "")

    def threshold(self, key: str):
        return self.cfg["thresholds"][key]

    def has_topic(self, key: str) -> bool:
        name = self.topic(key)
        if not name:
            return False
        meta = self.topics.get(name)
        return meta is not None and meta.count > 0

    def topic_meta(self, key: str):
        name = self.topic(key)
        return self.topics.get(name) if name else None

    # -- 惰性加载 --------------------------------------------------------
    def load(self, what: str) -> None:
        flag = "_loaded_" + what
        if getattr(self, flag, False):
            return
        setattr(self, flag, True)
        getattr(self, "_load_" + what)()

    def _msgs(self, key: str):
        topic = self.topic(key)
        meta = self.topics.get(topic)
        if meta is None or not has_decoder(meta.type):
            return
        fn = get_decoder(meta.type)
        bad = 0
        for ns, raw in self.reader.messages(topic):
            try:
                yield fn(raw, self.rel(ns))
            except Exception as e:
                # 版本差异导致的个别字段不兼容不应该让整条规则 WARN "执行异常"：
                # 跳过这条消息，把原因记进 warnings 里（报告末尾统一打印）。
                bad += 1
                if bad <= 3:
                    self.warnings.append(
                        "%s 有 %d 条消息解码失败（%s: %s）"
                        % (topic, bad, type(e).__name__, e))

    # -- grid_map --------------------------------------------------------
    def _load_grids(self) -> None:
        from .msgs import grid_map as gm

        prev = None
        kept = 0
        last_raw = None
        last_ns = None
        for ns, raw in self.reader.messages(self.topic("grid_map")):
            gf = gm.decode(raw, self.rel(ns))
            gf.digest = _digest_np(gf) if _np is not None else _digest_py(gf)
            changed = True
            if prev is not None:
                diff = _count_diff(prev, gf.cells, gf.layers)
                self.grid_changes.append((gf.t_bag, diff, diff == 0))
                changed = diff > 0
            keep = self.keep_all_grids or (changed and kept < MAX_CHANGED_GRIDS)
            prev = dict(gf.cells)
            if keep:
                kept += 1
            else:
                gf.cells = {}
            self.grids.append(gf)
            self.grid_digests.append(gf.digest)
            last_raw, last_ns = raw, ns

        if last_raw is not None and self.grids and not self.grids[-1].cells:
            rep = gm.decode(last_raw, self.rel(last_ns))
            rep.digest = self.grids[-1].digest
            self.grids[-1] = rep

    def _load_occs(self) -> None:
        from .msgs.occupancy_grid import decode

        for ns, raw in self.reader.messages(self.topic("occupancy_grid")):
            self.occs.append(decode(raw, self.rel(ns)))

    def _load_odom(self) -> None:
        from .msgs.odometry import decode

        for ns, raw in self.reader.messages(self.topic("odometry")):
            self.odom.append(decode(raw, self.rel(ns)))

    def _traj_from(self, key: str) -> List[TrajFrame]:
        from .msgs.trajectory import decode

        out = []
        for ns, raw in self.reader.messages(self.topic(key)):
            out.append(decode(raw, self.rel(ns)))
        return out

    def _load_traj(self) -> None:
        self.traj = self._traj_from("trajectory")

    def _load_final_traj(self) -> None:
        self.final_traj = self._traj_from("final_trajectory")

    def _load_completed(self) -> None:
        self.completed = list(self._msgs("is_completed"))

    def _load_proc(self) -> None:
        self.proc_ms = list(self._msgs("processing_time"))

    def _load_scenario(self) -> None:
        self.scenario = list(self._msgs("scenario"))

    def _load_objects(self) -> None:
        self.objects = list(self._msgs("objects"))

    def _load_goal(self) -> None:
        msgs = list(self._msgs("goal"))
        self.goal = msgs[0] if msgs else None

    def _load_route(self) -> None:
        msgs = list(self._msgs("route"))
        self.route = msgs[0] if msgs else None

    def _load_rosout(self) -> None:
        self.rosout = list(self._msgs("rosout"))

    def _load_diag(self) -> None:
        self.diagnostics = list(self._msgs("diagnostics"))

    def _load_gear(self) -> None:
        self.gear_status = list(self._msgs("gear_status"))
        self.gear_command = list(self._msgs("gear_command"))

    def _load_control(self) -> None:
        self.control_cmd = list(self._msgs("control_cmd"))
        self.follower_cmd = list(self._msgs("follower_cmd"))

    def _load_actuation(self) -> None:
        self.actuation = list(self._msgs("actuation_cmd"))

    def _load_gate(self) -> None:
        self.is_stopped = list(self._msgs("is_stopped"))
        self.is_start_requested = list(self._msgs("is_start_requested"))
        self.is_auto_available = list(self._msgs("is_autonomous_available"))

    def _load_modes(self) -> None:
        self.operation_mode = list(self._msgs("operation_mode"))
        self.motion = list(self._msgs("motion_state"))
        self.routing = list(self._msgs("routing_state"))

    def _load_aw_state(self) -> None:
        self.aw_state = list(self._msgs("autoware_state"))

    def _load_vehicle(self) -> None:
        self.velocity = list(self._msgs("velocity_status"))
        self.steering = list(self._msgs("steering_status"))

    def _load_shift(self) -> None:
        self.shift_gear = list(self._msgs("shift_gear_cmd"))

    # -- 派生 ------------------------------------------------------------
    def topic_rate(self, key: str) -> Optional[float]:
        meta = self.topic_meta(key)
        if meta is None or meta.count < 2:
            return None
        count, first, last = self.reader.span(self.topic(key))
        if not first or last is None or last <= first:
            return 0.0
        return (count - 1) / ((last - first) * 1e-9)

    def stamp_rate(self, key: str) -> Optional[float]:
        """按 header.stamp 的首末跨度算频率（仿真时钟下与 bag 时间不同）。"""
        meta = self.topic_meta(key)
        if meta is None or meta.count < 2 or meta.type in (
                "std_msgs/msg/Bool", "autoware_internal_debug_msgs/msg/Float64Stamped"):
            return None
        from .cdr import Cdr, payload

        try:
            first_msg = next(self.reader.messages(self.topic(key)))
            last_msg = self.reader.last_message(self.topic(key))
            if last_msg is None:
                return None
            s0 = Cdr(payload(first_msg[1])).time()
            s1 = Cdr(payload(last_msg[1])).time()
        except Exception:
            return None
        if s1 <= s0:
            return 0.0
        return (meta.count - 1) / (s1 - s0)

    def grid_first(self) -> Optional[GridFrame]:
        return self.grids[0] if self.grids else None

    def grid_last(self) -> Optional[GridFrame]:
        for gf in reversed(self.grids):
            if gf.cells:
                return gf
        return self.grids[-1] if self.grids else None

    def grid_change_ratio(self) -> Optional[float]:
        if not self.grid_changes:
            return None
        n = self.grids[0].n_cells * len(self.grids[0].layers)
        diffs = [d / n for _, d, _ in self.grid_changes]
        return max(diffs) if diffs else 0.0

    def occ_last(self) -> Optional[OccFrame]:
        return self.occs[-1] if self.occs else None

    def nearest_odom(self, t: float) -> Optional[OdomFrame]:
        if not self.odom:
            return None
        return min(self.odom, key=lambda o: abs(o.t_bag - t))

    def odom_before(self, t: float) -> Optional[OdomFrame]:
        cand = [o for o in self.odom if o.t_bag <= t]
        if cand:
            return cand[-1]
        return self.odom[0] if self.odom else None

    def nearest_occ(self, stamp: float) -> Optional[OccFrame]:
        if not self.occs:
            return None
        return min(self.occs, key=lambda o: abs(o.stamp - stamp))

    def parking_active(self, s: dict) -> bool:
        return "Parking" in s.get("scenarios", ())

    def parking_windows(self) -> List[Tuple[float, float]]:
        self.load("scenario")
        out = []
        start = None
        for s in self.scenario:
            active = self.parking_active(s)
            if active and start is None:
                start = s["t_bag"]
            elif not active and start is not None:
                out.append((start, s["t_bag"]))
                start = None
        if start is not None and self.scenario:
            out.append((start, self.scenario[-1]["t_bag"]))
        return out

    def first_parking_time(self) -> Optional[float]:
        self.load("scenario")
        for s in self.scenario:
            if self.parking_active(s):
                return s["t_bag"]
        return None

    def edt(self):
        occ = self.occ_last()
        if occ is None:
            return None
        key = (id(occ), occ.stamp)
        if self._edt_key != key:
            self._edt = geometry.edt_from_occupied(occ)
            self._edt_key = key
        return self._edt

    def calibrate(self):
        if self.calibration is not None or self.calibration_error:
            return self.calibration
        self.load("grids")
        self.load("occs")
        gf = self.grid_last()
        if gf is None or not self.occs or not gf.cells:
            self.calibration_error = "缺少 grid_map 或 occupancy_grid"
            return None
        occ = self.nearest_occ(gf.stamp)
        layer = "combined" if "combined" in gf.cells else gf.layers[-1]
        try:
            self.calibration = geometry.calibrate_layout(
                gf.cells[layer], occ, self.threshold("grid_calibration_min")
            )
        except geometry.CalibrationError as e:
            self.calibration_error = str(e)
        return self.calibration

    def calibrated_rows(self, gf: GridFrame):
        """返回标定后的 2D 代价（rows[y][x]）。"""
        layer = "combined" if "combined" in gf.cells else gf.layers[-1]
        cal = self.calibrate()
        if cal is None:
            return geometry.reshape(gf.cells[layer], gf.lx, "row")
        return geometry.apply_transform(geometry.reshape(gf.cells[layer], gf.lx, cal[0]), cal[1])

    # -- 控制链辅助 ------------------------------------------------------
    def speed_at(self, t: float) -> Optional[float]:
        """t 时刻的车速标量（由 odom 最近一帧插值）。"""
        self.load("odom")
        o = self.nearest_odom(t)
        if o is None:
            return None
        return math.hypot(o.vx, o.vy)

    def signed_speed_at(self, t: float) -> Optional[float]:
        """t 时刻带方向的纵向速度（base_link 系，倒车为负）。"""
        self.load("odom")
        o = self.nearest_odom(t)
        return None if o is None else o.vx

    def cmd_at(self, t: float) -> Optional[dict]:
        self.load("control")
        return _nearest(self.control_cmd, t)

    def target_speed_at(self, t: float) -> Optional[float]:
        """t 时刻规划给出的纵向速度上限（最近一帧轨迹的最大 |v|）。"""
        self.load("final_traj")
        f = _nearest(self.final_traj or self.traj, t)
        if f is None or not f.points:
            return None
        return max(abs(p.v_long) for p in f.points)

    def states(self, what: str) -> List[dict]:
        """状态变化表（给 timeline / D-* 规则复用）。"""
        from .msgs.control import AW_STATE, MOTION_STATE, OPERATION_MODE, ROUTING_STATE

        table = {
            # what -> (loader 名, 属性名, 状态字段, 枚举表)
            "aw": ("aw_state", "aw_state", "state", AW_STATE),
            "operation": ("modes", "operation_mode", "mode", OPERATION_MODE),
            "motion": ("modes", "motion", "state", MOTION_STATE),
            "routing": ("modes", "routing", "state", ROUTING_STATE),
            "gear": ("gear", "gear_status", "gear_name", None),
        }
        load, attr, key, names = table[what]
        self.load(load)
        return transitions(getattr(self, attr), key, names)


def _nearest(seq, t: float):
    if not seq:
        return None
    return min(seq, key=lambda x: abs(_t_of(x) - t))
