"""The world model: entity registry, association, smoothing, confidence, rescue status.

Pure Python with explicit time, so every behaviour is unit-testable without ROS. The node feeds it placed detections
and reads snapshots back; nothing else writes world state.

Association (Implementation_Plan.md section 9): nearest track within a gating radius, else a new track. Three victims in
5 x 5 m does not need a tracker; the EMA is an honest placeholder for a filter (TODO in __init__).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from .config import SafeZone, WorldModelConfig
from .entities import (ACTIVE, DETECTED, FROZEN, RESCUED, STATUS_NAMES, TRANSITIONS, UNKNOWN, FireTrack, Observation,
                       VictimTrack)

CLASS_VICTIM = 'victim'
CLASS_FIRE = 'fire'


@dataclass
class ObserveReport:
    """What one batch of observations did. For logs and tests; nothing above the world model needs it."""
    created: List[str] = field(default_factory=list)
    updated: List[str] = field(default_factory=list)
    confirmed: List[str] = field(default_factory=list)
    expired: List[str] = field(default_factory=list)
    ignored: Dict[str, int] = field(default_factory=dict)

    def ignore(self, reason: str) -> None:
        self.ignored[reason] = self.ignored.get(reason, 0) + 1


@dataclass(frozen=True)
class VictimSnapshot:
    id: str
    x: float
    y: float
    z: float
    confidence: float            # decayed to `now`
    status: int
    last_seen: float
    hits: int


@dataclass(frozen=True)
class FireSnapshot:
    known: bool
    x: float
    y: float
    z: float
    confidence: float


@dataclass(frozen=True)
class WorldSnapshot:
    stamp: float
    victims: Tuple[VictimSnapshot, ...]
    fire: FireSnapshot
    safe_zone: SafeZone
    current_target_id: str


class WorldModel:
    def __init__(self, config: Optional[WorldModelConfig] = None):
        self.cfg = config or WorldModelConfig()
        self._victims: Dict[str, VictimTrack] = {}
        self._fire: Optional[FireTrack] = None
        self._next_id = 1                 # ids are creation order and never reused; unrelated to any scenario id

    # ------------------------------------------------------------------------------ observations
    def observe(self, observations: Sequence[Observation], stamp: float) -> ObserveReport:
        """Fold one message's placed detections into the registry. `stamp` is the message time (seconds)."""
        rep = ObserveReport()
        touched = set()                                            # a track counts one hit per message, not per blob
        # Strongest first: a weak fragment then merges into the track its strong neighbour created or found.
        for o in sorted(observations, key=lambda o: -o.confidence):
            if not all(math.isfinite(v) for v in (o.x, o.y, o.z, o.confidence)):
                rep.ignore('non-finite')
            elif o.confidence < self.cfg.min_confidence:
                rep.ignore('low confidence')
            elif o.class_id == CLASS_VICTIM:
                self._victim(o, stamp, rep, touched)
            elif o.class_id == CLASS_FIRE:
                self._fire_obs(o, stamp, rep, touched)
            else:
                rep.ignore(f'unhandled class {o.class_id!r}')      # TODO(future): obstacles
        rep.expired = self._expire(stamp)
        return rep

    def _victim(self, o: Observation, stamp: float, rep: ObserveReport, touched: set) -> None:
        track, dist = self._nearest_victim(o.x, o.y)
        if track is None or dist > self.cfg.victim_gate_m:
            # A victim seen inside the safe zone is one that was already carried there. It must never become a new
            # candidate, or a released victim would show up as a phantom extra one.
            if self.cfg.safe_zone.contains(o.x, o.y):
                rep.ignore('victim inside the safe zone')
                return
            t = VictimTrack(f'V{self._next_id}', o.x, o.y, o.z, o.confidence, stamp, stamp)
            self._next_id += 1
            if self.cfg.confirm_hits <= 1:
                t.status = DETECTED
            self._victims[t.id] = t
            touched.add(t.id)
            rep.created.append(t.id)
            return
        if track.status in FROZEN:
            rep.ignore('absorbed by a carried/rescued track')
            return
        a = self.cfg.position_alpha
        track.x += a * (o.x - track.x)
        track.y += a * (o.y - track.y)
        track.z += a * (o.z - track.z)
        track.confidence = o.confidence                            # latest, per the plan; decay is applied on read
        track.last_seen = max(track.last_seen, stamp)
        if track.id not in touched:
            touched.add(track.id)
            track.hits += 1
            if track.status == UNKNOWN and track.hits >= self.cfg.confirm_hits:
                track.status = DETECTED
                rep.confirmed.append(track.id)
        rep.updated.append(track.id)

    def _fire_obs(self, o: Observation, stamp: float, rep: ObserveReport, touched: set) -> None:
        f = self._fire
        if f is None:
            self._fire = FireTrack(o.x, o.y, o.z, o.confidence, stamp, stamp, confirmed=self.cfg.confirm_hits <= 1)
            touched.add('fire')
            rep.created.append('fire')
            return
        if math.hypot(o.x - f.x, o.y - f.y) > self.cfg.fire_gate_m:
            rep.ignore('second fire (one fire is tracked)')        # TODO(future): multiple fires
            return
        a = self.cfg.position_alpha
        f.x += a * (o.x - f.x)
        f.y += a * (o.y - f.y)
        f.z += a * (o.z - f.z)
        f.confidence = o.confidence
        f.last_seen = max(f.last_seen, stamp)
        if 'fire' not in touched:
            touched.add('fire')
            f.hits += 1
            if not f.confirmed and f.hits >= self.cfg.confirm_hits:
                f.confirmed = True
                rep.confirmed.append('fire')
        rep.updated.append('fire')

    def _nearest_victim(self, x: float, y: float) -> Tuple[Optional[VictimTrack], float]:
        best, best_d = None, float('inf')
        for t in self._victims.values():
            d = math.hypot(x - t.x, y - t.y)
            if d < best_d:
                best, best_d = t, d
        return best, best_d

    def _expire(self, now: float) -> List[str]:
        """Drop tentative tracks that never confirmed. Confirmed ones are never deleted: a victim out of view is still there."""
        gone = [i for i, t in self._victims.items()
                if t.status == UNKNOWN and now - t.first_seen > self.cfg.tentative_timeout_s]
        for i in gone:
            del self._victims[i]
        if self._fire is not None and not self._fire.confirmed and now - self._fire.first_seen > self.cfg.tentative_timeout_s:
            self._fire = None
            gone.append('fire')
        return gone

    # ------------------------------------------------------------------------------ rescue status
    def set_status(self, victim_id: str, new_status: int) -> Tuple[bool, str]:
        """The one way rescue code changes a victim's status. Returns (success, message)."""
        t = self._victims.get(victim_id)
        if t is None:
            return False, f'unknown victim {victim_id!r}'
        if new_status not in STATUS_NAMES:
            return False, f'invalid status {new_status}'
        if new_status == t.status:
            return True, f'{victim_id} is already {STATUS_NAMES[t.status]}'
        if new_status not in TRANSITIONS[t.status]:
            why = ' (RESCUED is terminal)' if t.status == RESCUED else ''
            return False, f'{victim_id}: {STATUS_NAMES[t.status]} -> {STATUS_NAMES[new_status]} is not allowed{why}'
        if new_status in ACTIVE:
            other = next((v.id for v in self._victims.values() if v.status in ACTIVE and v.id != victim_id), None)
            if other is not None:
                return False, f'{other} is already {STATUS_NAMES[self._victims[other].status]}: one target at a time'
        t.status = new_status
        return True, f'{victim_id} -> {STATUS_NAMES[new_status]}'

    # ------------------------------------------------------------------------------ reading
    def victim(self, victim_id: str) -> Optional[VictimTrack]:
        return self._victims.get(victim_id)

    def current_target_id(self) -> str:
        return next((v.id for v in self._victims.values() if v.status in ACTIVE), '')

    def snapshot(self, now: float) -> WorldSnapshot:
        """The current belief, with confidence decayed to `now` (stale entities lose standing without being deleted)."""
        self._expire(now)
        tau = self.cfg.confidence_decay_tau_s

        def decayed(c: float, last_seen: float) -> float:
            return c * math.exp(-max(0.0, now - last_seen) / tau)

        victims = tuple(VictimSnapshot(t.id, t.x, t.y, t.z, decayed(t.confidence, t.last_seen), t.status, t.last_seen, t.hits)
                        for t in sorted(self._victims.values(), key=lambda t: int(t.id[1:])))
        f = self._fire
        fire = (FireSnapshot(f.confirmed, f.x, f.y, f.z, decayed(f.confidence, f.last_seen)) if f is not None
                else FireSnapshot(False, 0.0, 0.0, 0.0, 0.0))
        return WorldSnapshot(now, victims, fire, self.cfg.safe_zone, self.current_target_id())
