"""SEARCH: where to look next, from the map the mission already has. Pure Python + numpy, no ROS.

This is deliberately NOT a frontier explorer. The mission runs on a SAVED map (map once, then navigate: the mapped-arena
mode), so the walls and obstacles are known and what is unknown is which of the free places hold a victim. The question
SEARCH answers is therefore coverage: which free cells has a stationary 360-degree look already reached, and which
viewpoint would reach the most cells still unseen. Coverage is also the documented evidence for "nothing left to find":
the mission may only call itself complete once the free space that could hold a victim has been looked at.

Visibility is a ray cast through the occupancy grid (occupied and never-seen cells both block), out to the range at which a
colour blob still classifies. It ignores the camera's field of view on purpose: a viewpoint is sampled with a full sweep.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Set, Tuple

import numpy as np

from .config import PlanningConfig

OCCUPIED_THRESHOLD = 65        # nav_msgs/OccupancyGrid: occupied probability


@dataclass
class GridMap:
    """A copy of an OccupancyGrid: `data[row, col]`, row 0 at the origin's y. -1 unknown, 0..100 occupancy."""
    resolution: float
    origin_x: float
    origin_y: float
    data: np.ndarray

    @property
    def height(self) -> int:
        return self.data.shape[0]

    @property
    def width(self) -> int:
        return self.data.shape[1]

    def cell(self, x: float, y: float) -> Tuple[int, int]:
        return (int(math.floor((y - self.origin_y) / self.resolution)), int(math.floor((x - self.origin_x) / self.resolution)))

    def centre(self, row: int, col: int) -> Tuple[float, float]:
        return (self.origin_x + (col + 0.5) * self.resolution, self.origin_y + (row + 0.5) * self.resolution)

    def free(self) -> np.ndarray:
        return (self.data >= 0) & (self.data < OCCUPIED_THRESHOLD)

    def blocked(self) -> np.ndarray:
        """Occupied OR never seen: a ray does not pass through either."""
        return ~self.free()


def _distance_to_blocked(blocked: np.ndarray, res: float, limit_cells: int) -> np.ndarray:
    """Distance (m) from each cell to the nearest blocked cell, capped at `limit_cells` (a small brute-force EDT: the
    grid is a few hundred cells a side and this runs once per map)."""
    h, w = blocked.shape
    dist = np.full((h, w), float(limit_cells), dtype=np.float32)
    rows, cols = np.nonzero(blocked)
    if rows.size == 0:
        return dist * res
    for dr in range(-limit_cells, limit_cells + 1):
        for dc in range(-limit_cells, limit_cells + 1):
            d = math.hypot(dr, dc)
            if d > limit_cells:
                continue
            r2, c2 = rows + dr, cols + dc
            ok = (r2 >= 0) & (r2 < h) & (c2 >= 0) & (c2 < w)
            cur = dist[r2[ok], c2[ok]]
            dist[r2[ok], c2[ok]] = np.minimum(cur, d)
    return dist * res


class CoveragePlanner:
    """Greedy next-best-view over a saved map. Holds the covered set; everything else is derived."""

    def __init__(self, grid: GridMap, cfg: PlanningConfig):
        self.grid, self.cfg = grid, cfg
        res = grid.resolution
        self._clear = _distance_to_blocked(grid.blocked(), res, int(math.ceil(max(cfg.viewpoint_clearance_m,
                                                                                    cfg.coverage_cell_clearance_m) / res)) + 1)
        free = grid.free()
        # The cells a victim's centre can occupy: free, and clear of walls/obstacles by the scenario's own rule.
        self.target_cells: np.ndarray = free & (self._clear >= cfg.coverage_cell_clearance_m)
        self.covered = np.zeros_like(self.target_cells)
        self._viewpoints: Optional[List[Tuple[float, float]]] = None
        self._vis: List[np.ndarray] = []
        self.failed: Set[int] = set()
        self.visited: List[Tuple[float, float]] = []

    # ------------------------------------------------------------------------------------------ visibility
    def visible_from(self, x: float, y: float) -> np.ndarray:
        """Boolean grid of the cells a full look from (x, y) reaches: rays until the first blocked cell or the range."""
        g, res = self.grid, self.grid.resolution
        blocked = g.blocked()
        n_rays = int(math.ceil(2 * math.pi * self.cfg.search_range_m / res))
        steps = int(self.cfg.search_range_m / (0.5 * res))
        ang = np.linspace(0.0, 2 * math.pi, n_rays, endpoint=False)
        r = np.arange(1, steps + 1) * 0.5 * res
        px = x + np.outer(np.cos(ang), r)
        py = y + np.outer(np.sin(ang), r)
        col = np.floor((px - g.origin_x) / res).astype(int)
        row = np.floor((py - g.origin_y) / res).astype(int)
        inside = (row >= 0) & (row < g.height) & (col >= 0) & (col < g.width)
        row_c, col_c = np.clip(row, 0, g.height - 1), np.clip(col, 0, g.width - 1)
        hit = blocked[row_c, col_c] | ~inside
        alive = np.cumsum(hit, axis=1) == 0                # True until (and excluding) the first blocked sample
        vis = np.zeros((g.height, g.width), dtype=bool)
        vis[row_c[alive], col_c[alive]] = True
        # A blob nearer than search_min_range_m touches the image border and is never given a position (Phase 5), so a look
        # from there would see the victim without being able to place it. Those cells are not covered by this look.
        rr, cc = np.mgrid[0:g.height, 0:g.width]
        cx_, cy_ = g.origin_x + (cc + 0.5) * res, g.origin_y + (rr + 0.5) * res
        vis &= np.hypot(cx_ - x, cy_ - y) >= self.cfg.search_min_range_m
        return vis

    # ------------------------------------------------------------------------------------------ viewpoints
    def viewpoints(self) -> List[Tuple[float, float]]:
        if self._viewpoints is None:
            g, cfg = self.grid, self.cfg
            stride = max(1, int(round(cfg.viewpoint_spacing_m / g.resolution)))
            ok = g.free() & (self._clear >= cfg.viewpoint_clearance_m)
            pts = []
            for row in range(0, g.height, stride):
                for col in range(0, g.width, stride):
                    if ok[row, col]:
                        pts.append(g.centre(row, col))
            self._viewpoints = pts
            self._vis = [self.visible_from(x, y) & self.target_cells for x, y in pts]
        return self._viewpoints

    def mark_looked(self, x: float, y: float) -> int:
        """A full look from (x, y) happened: credit the cells it reaches. Returns how many were new."""
        seen = self.visible_from(x, y) & self.target_cells
        new = int((seen & ~self.covered).sum())
        self.covered |= seen
        self.visited.append((x, y))
        return new

    def coverage(self) -> float:
        total = int(self.target_cells.sum())
        return 1.0 if total == 0 else float((self.covered & self.target_cells).sum()) / total

    def gain(self, index: int) -> int:
        self.viewpoints()
        return int((self._vis[index] & ~self.covered).sum())

    def mark_failed(self, index: int) -> None:
        self.failed.add(index)

    def next_viewpoint(self, robot: Tuple[float, float]) -> Optional[Tuple[int, Tuple[float, float], int]]:
        """(index, (x, y), gain): the viewpoint with the best new-cells-per-trip, or None when SEARCH is exhausted
        (coverage goal met, or nothing left is worth the trip)."""
        if self.coverage() >= self.cfg.coverage_goal:
            return None
        pts = self.viewpoints()
        best = None
        for i, (x, y) in enumerate(pts):
            if i in self.failed:
                continue
            gain = self.gain(i)
            if gain < self.cfg.min_gain_cells:
                continue
            score = gain / (1.0 + math.hypot(x - robot[0], y - robot[1]) / 2.0)
            if best is None or score > best[0]:
                best = (score, i, (x, y), gain)
        return None if best is None else (best[1], best[2], best[3])
