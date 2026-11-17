from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence, Tuple
import math


Vec3 = Tuple[float, float, float]


def distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def project_to_segment(point: Sequence[float], start: Sequence[float], end: Sequence[float]):
    v = [float(e) - float(s) for s, e in zip(start, end)]
    w = [float(p) - float(s) for p, s in zip(point, start)]
    vv = sum(x * x for x in v)
    if vv <= 1e-15:
        raise ValueError("zero-length segment")
    t = sum(x * y for x, y in zip(w, v)) / vv
    tc = max(0.0, min(1.0, t))
    proj = tuple(float(s) + tc * x for s, x in zip(start, v))
    return t, tc, proj, distance(point, proj)


@dataclass(frozen=True)
class NearestEdge:
    from_name: str
    to_name: str
    edge_index: int
    raw_t: float
    clamped_t: float
    cross_track_m: float


class TeachPath:
    def __init__(self, points: Dict[str, Sequence[float]], order: Iterable[str]):
        self.order = list(order)
        self.points: Dict[str, Vec3] = {
            name: tuple(float(v) for v in points[name])  # type: ignore[assignment]
            for name in self.order
        }
        if len(self.order) < 2:
            raise ValueError("teach path needs >= 2 points")

    def route(self, start: str, end: str) -> List[str]:
        i0 = self.order.index(start)
        i1 = self.order.index(end)
        step = 1 if i1 >= i0 else -1
        return [self.order[i] for i in range(i0, i1 + step, step)]

    def xyz_route(self, start: str, end: str) -> List[Vec3]:
        return [self.points[n] for n in self.route(start, end)]

    def nearest_edge(self, xyz: Sequence[float]) -> NearestEdge:
        best = None
        for i in range(len(self.order) - 1):
            a_name = self.order[i]
            b_name = self.order[i + 1]
            raw_t, tc, _, cross = project_to_segment(xyz, self.points[a_name], self.points[b_name])
            item = NearestEdge(a_name, b_name, i, raw_t, tc, cross)
            if best is None or item.cross_track_m < best.cross_track_m:
                best = item
        assert best is not None
        return best

    def remaining_waypoints_from_current(self, xyz: Sequence[float], target: str) -> List[str]:
        target_i = self.order.index(target)
        # Prefer an exact nearby taught point if current is already close to one.
        dists = [(distance(xyz, self.points[name]), i, name) for i, name in enumerate(self.order)]
        dmin, imin, nmin = min(dists)
        if dmin <= 0.005:
            if imin == target_i:
                return []
            step = 1 if target_i > imin else -1
            return self.order[imin + step:target_i + step:step]

        edge = self.nearest_edge(xyz)
        left_i = edge.edge_index
        right_i = edge.edge_index + 1
        if target_i <= left_i:
            return [self.order[i] for i in range(left_i, target_i - 1, -1)]
        if target_i >= right_i:
            return [self.order[i] for i in range(right_i, target_i + 1)]
        return []
