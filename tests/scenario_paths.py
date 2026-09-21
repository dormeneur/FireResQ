"""EVALUATION-ONLY path oracle over the scenario's TRUE geometry.

Ground truth, for tests and evaluation: it reads the scenario, so no robot code may ever use it (cognition asks Nav2). It
exists so the prioritisation evaluation can be deterministic and needs no simulator: an occupancy grid rasterised from the
scenario's solid surfaces (walls, obstacles, fire, victims), the robot's circumscribed radius as clearance, and 8-connected
A*. Its lengths approximate what Nav2's grid planner returns on a perfect map; the live simulation test uses the real one.
"""
import heapq
import math

from fire_resq_cognition.types import PathInfo
from fire_resq_simulation.scenario import distance_to_surface

ROBOT_CLEARANCE_M = 0.17            # the circumscribed radius Nav2's costmaps use (derived from parameters.xacro at launch)


class ScenarioPathOracle:
    def __init__(self, sc, resolution=0.05, clearance=ROBOT_CLEARANCE_M):
        self.sc, self.res = sc, resolution
        self.hx, self.hy = sc.arena.size_x / 2.0, sc.arena.size_y / 2.0
        self.nx, self.ny = int(round(sc.arena.size_x / resolution)), int(round(sc.arena.size_y / resolution))
        self.blocked = [[distance_to_surface(sc, *self._centre(i, j)) < clearance for j in range(self.ny)] for i in range(self.nx)]
        self.queries = 0

    def _centre(self, i, j):
        return -self.hx + (i + 0.5) * self.res, -self.hy + (j + 0.5) * self.res

    def _cell(self, p):
        return int((p[0] + self.hx) / self.res), int((p[1] + self.hy) / self.res)

    def _inside(self, c):
        return 0 <= c[0] < self.nx and 0 <= c[1] < self.ny

    def __call__(self, start, goal) -> PathInfo:
        self.queries += 1
        s, g = self._cell(start), self._cell(goal)
        if not self._inside(g) or self.blocked[g[0]][g[1]]:
            return PathInfo.unreachable('goal occupied or outside the map')
        if not self._inside(s) or self.blocked[s[0]][s[1]]:
            return PathInfo.unknown('start occupied or outside the map')
        h = lambda c: self.res * (max(abs(c[0] - g[0]), abs(c[1] - g[1])) + (math.sqrt(2) - 1) * min(abs(c[0] - g[0]), abs(c[1] - g[1])))  # noqa: E731
        best, heap = {s: 0.0}, [(h(s), 0.0, s)]
        while heap:
            _, d, c = heapq.heappop(heap)
            if c == g:
                return PathInfo.reachable(d)
            if d > best.get(c, math.inf):
                continue
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == dj == 0:
                        continue
                    n = (c[0] + di, c[1] + dj)
                    if not self._inside(n) or self.blocked[n[0]][n[1]]:
                        continue
                    if di and dj and (self.blocked[c[0] + di][c[1]] or self.blocked[c[0]][c[1] + dj]):
                        continue                                          # no cutting corners
                    nd = d + self.res * (math.sqrt(2) if di and dj else 1.0)
                    if nd < best.get(n, math.inf):
                        best[n] = nd
                        heapq.heappush(heap, (nd + h(n), nd, n))
        return PathInfo.unreachable('no path')
