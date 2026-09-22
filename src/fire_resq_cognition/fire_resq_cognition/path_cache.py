"""A cache in front of a path query, so the same question is not put to the planner twice.

Cognition asks two questions per candidate (out to the approach point, and back to the safe zone), and asks them again on every
re-decision, on SelectTarget, on retry and on refresh. Most of those repeat a question asked a moment ago about a world that has
not changed. This wraps any PathQuery (the Nav2 adapter in production, a fake in tests) and answers repeats from memory.

What is cached, and why only that:
  * a `reachable` answer, for `ttl_s`, keyed by start and goal quantised to `quantum_m` (the costmap resolution): a path length
    does not change when the question is asked again a second later, and the TTL bounds how stale a costmap change can leave it.
  * NEVER `unreachable` or `unknown`. Those are the answers that were found to be transient (Nav2 answers "goal outside the
    map" while its costmap is still starting) and the ones a retry exists for; they are rare, so asking again is cheap.

Pure Python: no ROS, so it is unit-tested with a fake planner and an injected clock.
"""
from __future__ import annotations

import time
from typing import Callable, Dict, Tuple

from .types import XY, PathInfo, PathQuery

Key = Tuple[int, int, int, int]


class CachedPathQuery:
    def __init__(self, inner: PathQuery, ttl_s: float = 15.0, quantum_m: float = 0.05, clock: Callable[[], float] = time.monotonic):
        if quantum_m <= 0:
            raise ValueError('quantum_m must be positive')
        self.inner, self.ttl_s, self.quantum_m, self._clock = inner, float(ttl_s), float(quantum_m), clock
        self._store: Dict[Key, Tuple[float, PathInfo]] = {}
        self.asked = 0          # questions that reached the planner
        self.hits = 0           # questions answered from memory

    def _key(self, start: XY, goal: XY) -> Key:
        q = self.quantum_m
        return (round(start[0] / q), round(start[1] / q), round(goal[0] / q), round(goal[1] / q))

    def __call__(self, start: XY, goal: XY) -> PathInfo:
        if self.ttl_s > 0:
            key = self._key(start, goal)
            hit = self._store.get(key)
            if hit is not None and self._clock() - hit[0] <= self.ttl_s:
                self.hits += 1
                return hit[1]
        self.asked += 1
        info = self.inner(start, goal)
        if self.ttl_s > 0 and isinstance(info, PathInfo) and info.status == 'reachable':
            self._store[key] = (self._clock(), info)
        return info

    def invalidate(self) -> None:
        """Forget everything: used when the answer must be fresh (a retry, the periodic refresh)."""
        self._store.clear()

    def destroy(self) -> None:
        destroy = getattr(self.inner, 'destroy', None)
        if destroy:
            destroy()

    @property
    def queries(self) -> int:
        return self.asked
