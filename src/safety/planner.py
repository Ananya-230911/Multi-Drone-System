"""Geofence-aware path planning (PLAN.md section 9.3).

Horizontal path: visibility graph over the vertices of the zones and the
boundary, offset by `planner_clearance` beyond the margin so graph nodes lie
just outside the checked shapes (a node exactly on a buffered edge would be
"outside" by the edge rule). Edges are kept only if the segment validates.
A* by length, with nodes in a fixed sorted order so results are deterministic.

The static node-to-node graph is built once per geofence; each query only adds
edges from the start and the goal.

Vertical profile (section 6.5): interior waypoints fly at `cruise_alt`; the
route starts at the drone's altitude and ends at the goal altitude. A start or
goal below `min_alt` (a home position) gets a vertical climb / descent there.

Every route is re-checked with `validate_route` before it is returned.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import nearest_points

from src.config import Point2
from src.safety.geofence import Geofence
from src.safety.route_validator import Waypoint, route_length, validate_point, validate_route

_EPS = 1e-6


@dataclass(frozen=True)
class PlanResult:
    route: list[Waypoint] | None
    reason: str | None = None  # why planning failed

    @property
    def ok(self) -> bool:
        return self.route is not None


class Planner:
    def __init__(self, fence: Geofence, cruise_alt: float, clearance: float):
        self.fence = fence
        self.cruise_alt = cruise_alt
        self.clearance = clearance
        self.nodes: list[Point2] = self._build_nodes()
        self._adj: list[list[tuple[int, float]]] = [[] for _ in self.nodes]
        for i in range(len(self.nodes)):
            for j in range(i + 1, len(self.nodes)):
                if fence.horizontal_segment_violation(self.nodes[i], self.nodes[j]) is None:
                    d = math.dist(self.nodes[i], self.nodes[j])
                    self._adj[i].append((j, d))
                    self._adj[j].append((i, d))
        self._goal_edges: dict[Point2, list[tuple[int, float]]] = {}

    def _build_nodes(self) -> list[Point2]:
        f, off = self.fence, self.fence.margin + self.clearance
        shapes: list[Polygon] = [f.raw_boundary.buffer(-off, join_style="mitre")]
        shapes += [z.raw.buffer(off, join_style="mitre") for z in f.zones]
        pts: set[Point2] = set()
        for shp in shapes:
            for poly in getattr(shp, "geoms", [shp]):
                for ring in [poly.exterior, *poly.interiors]:
                    for x, y in list(ring.coords)[:-1]:
                        if f.horizontal_violation(x, y) is None:
                            pts.add((round(x, 6), round(y, 6)))
        return sorted(pts)

    # ------------------------------------------------------------------ horizontal

    def _edges_from(self, p: Point2) -> list[tuple[int, float]]:
        return [
            (i, math.dist(p, n))
            for i, n in enumerate(self.nodes)
            if self.fence.horizontal_segment_violation(p, n) is None
        ]

    def horizontal_path(self, a: Point2, b: Point2) -> list[Point2] | None:
        f = self.fence
        if f.horizontal_violation(*a) or f.horizontal_violation(*b):
            return None
        if f.horizontal_segment_violation(a, b) is None:
            return [a, b]
        start_edges = self._edges_from(a)
        if b not in self._goal_edges:
            if len(self._goal_edges) > 256:  # bound the cache; goals are mostly homes and patrol points
                self._goal_edges.clear()
            self._goal_edges[b] = self._edges_from(b)
        goal_edges = dict(self._goal_edges[b])
        n = len(self.nodes)
        START, GOAL = n, n + 1
        h = lambda i: math.dist(self.nodes[i], b) if i < n else (math.dist(a, b) if i == START else 0.0)  # noqa: E731
        best = {START: 0.0}
        prev: dict[int, int] = {}
        heap = [(h(START), 0.0, START)]
        while heap:
            _, g, u = heapq.heappop(heap)
            if u == GOAL:
                break
            if g > best.get(u, math.inf) + _EPS:
                continue
            nbrs = start_edges if u == START else list(self._adj[u])
            if u < n and u in goal_edges:
                nbrs = nbrs + [(GOAL, goal_edges[u])]
            for v, w in nbrs:
                ng = g + w
                if ng + _EPS < best.get(v, math.inf):
                    best[v] = ng
                    prev[v] = u
                    heapq.heappush(heap, (ng + h(v), ng, v))
        if GOAL not in prev:
            return None
        path, cur = [b], GOAL
        while prev[cur] != START:
            cur = prev[cur]
            path.append(self.nodes[cur])
        path.append(a)
        return path[::-1]

    # ------------------------------------------------------------------ 3D

    def plan(self, start: Waypoint, goal: Waypoint, revalidate: bool = True) -> PlanResult:
        """Validated 3D route from `start` to `goal`.

        `revalidate=False` skips the final `validate_route` pass. Only for routes used as
        energy estimates and never flown; every segment was already checked while building
        the graph path, so the result is the same, just cheaper.
        """
        f = self.fence
        if validate_point(start, f):
            return PlanResult(None, "START_INVALID")
        if validate_point(goal, f):
            return PlanResult(None, "GOAL_INVALID")
        path = self.horizontal_path(start[:2], goal[:2])
        if path is None:
            return PlanResult(None, "NO_PATH")
        cruise = min(max(self.cruise_alt, f.min_alt), f.max_alt)
        route: list[Waypoint] = [start]
        if start[2] < f.min_alt:  # taking off from home: climb vertically first
            route.append((start[0], start[1], cruise))
        for x, y in path[1:-1]:
            route.append((x, y, cruise))
        if goal[2] < f.min_alt:  # landing at home: arrive at cruise, then descend vertically
            route.append((goal[0], goal[1], cruise))
        route.append(goal)
        route = _dedupe(route)
        if not revalidate:
            return PlanResult(route)
        check = validate_route(route, f)
        if not check.ok:
            return PlanResult(None, f"VALIDATION:{check.reason}@{check.index}")
        return PlanResult(route)


def inspection_point(
    target_pos: Point2,
    r_eff: float,
    planner: Planner,
    standoff_alt: float,
    candidate_count: int,
    from_pos: Point2 | None = None,
) -> Waypoint | None:
    """A geofence-valid point at `standoff_alt` inside the sensor radius of the target.

    1. Directly above the target, if allowed.
    2. Otherwise, along the straight approach from `from_pos`, the allowed point
       closest to the target that is still inside `r_eff`.
    3. Otherwise `candidate_count` points on the circle of radius 0.9 * r_eff,
       picking the one with the shortest horizontal path from `from_pos`.
    Returns None when none of these is valid (e.g. the target is deep inside a zone).
    """
    f = planner.fence
    tx, ty = target_pos
    above = (tx, ty, standoff_alt)
    if validate_point(above, f) is None:
        return above
    if from_pos is not None and math.dist(from_pos, target_pos) > _EPS:
        d = math.dist(from_pos, target_pos)
        ux, uy = (from_pos[0] - tx) / d, (from_pos[1] - ty) / d
        approach = LineString([(tx, ty), (tx + ux * r_eff, ty + uy * r_eff)])
        inside = approach.intersection(f.allowed)
        if not inside.is_empty:
            p = nearest_points(inside, Point(tx, ty))[0]
            dist = math.dist((p.x, p.y), target_pos) + planner.clearance
            cand = (tx + ux * dist, ty + uy * dist, standoff_alt)
            if dist < r_eff and validate_point(cand, f) is None:
                return cand
    best: tuple[float, Waypoint] | None = None
    r = 0.9 * r_eff
    for k in range(candidate_count):
        ang = 2 * math.pi * k / candidate_count
        cand = (tx + r * math.cos(ang), ty + r * math.sin(ang), standoff_alt)
        if validate_point(cand, f):
            continue
        if from_pos is None:
            return cand
        path = planner.horizontal_path(from_pos, cand[:2])
        if path is None:
            continue
        length = route_length([(x, y, 0.0) for x, y in path])
        if best is None or length < best[0] - _EPS:
            best = (length, cand)
    return None if best is None else best[1]


def _dedupe(route: list[Waypoint]) -> list[Waypoint]:
    out = [route[0]]
    for p in route[1:]:
        if math.dist(p, out[-1]) > _EPS:
            out.append(p)
    return out
