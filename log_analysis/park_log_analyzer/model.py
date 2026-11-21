"""中间数据模型（开发文档 §4.2）。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class GridDigest:
    t_bag: float
    stamp: float
    nonzero: Dict[str, int] = field(default_factory=dict)
    nan: Dict[str, int] = field(default_factory=dict)
    dirty: Dict[str, int] = field(default_factory=dict)
    vmax: Dict[str, float] = field(default_factory=dict)
    signature: str = ""


@dataclass
class GridFrame:
    t_bag: float
    stamp: float
    frame_id: str
    resolution: float
    lx: int
    ly: int
    center: Tuple[float, float, float]
    layers: Tuple[str, ...]
    cells: Dict[str, object]           # {layer: array('f')} 原始顺序（未标定）
    digest: Optional[GridDigest] = None

    @property
    def n_cells(self) -> int:
        return self.lx * self.ly


@dataclass
class OccFrame:
    t_bag: float
    stamp: float
    frame_id: str
    resolution: float
    w: int
    h: int
    origin: Tuple[float, float, float]   # (x, y, yaw) 左下角
    data: bytes                          # int8


@dataclass(frozen=True)
class TrajPoint:
    x: float
    y: float
    z: float
    yaw: float
    v_long: float
    v_lat: float
    w_heading: float
    accel: float
    front_steer: float
    rear_steer: float
    t_from_start: float


@dataclass
class TrajFrame:
    t_bag: float
    stamp: float
    frame_id: str
    points: Tuple[TrajPoint, ...]


@dataclass
class OdomFrame:
    t_bag: float
    stamp: float
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float


@dataclass
class RouteInfo:
    t_bag: float
    stamp: float
    start: Tuple[float, float, float, float]
    goal: Tuple[float, float, float, float]
    segments: Tuple[Tuple[int, str, Tuple[Tuple[int, str], ...]], ...]
    uuid: str
    allow_modification: bool
    n_primitives: int = 0


@dataclass
class ScalarPoint:
    t_bag: float
    stamp: float
    value: float
    raw: bytes = b""


@dataclass(frozen=True)
class VehicleSpec:
    length: float = 4.89
    width: float = 1.896
    wheel_base: float = 2.79
    rear_overhang: float = 1.1
    front_overhang: float = 1.0
    max_steer_rad: float = 0.70
    margin: float = 0.40

    @property
    def base2back(self) -> float:
        return self.rear_overhang

    @property
    def kappa_limit(self) -> float:
        import math

        return math.tan(self.max_steer_rad) / self.wheel_base

    def footprint(self):
        """规划器口吻的 footprint（margin 每边摊一半，见 §7.3）。"""
        x_min = -(self.rear_overhang + self.margin / 2.0)
        x_max = self.length - self.rear_overhang + self.margin / 2.0
        y = self.width / 2.0 + self.margin / 2.0
        return x_min, x_max, y
