"""Shared helpers for the perception simulation tests (test_perception.py, test_perception_rgb.py)."""
import math

import numpy as np

# Measured by tests/sim/experiments/perception_accuracy.py (docs/Implementation_Plan.md, Phase 5), then given
# a margin of at least 2x. They are loose enough not to flake on physics jitter and tight enough that a broken
# estimator (a wrong prior, a frame mix-up, a units slip) fails.
VICTIM_TOL_M = 0.08        # depth backend, 1-5 m
FIRE_TOL_M = 0.08
RGB_ONLY_TOL_FRAC = 0.02   # known-height: error grows with range, so a fraction of the range, plus a floor
RGB_ONLY_FLOOR_M = 0.05
MATCH_M = 0.5              # a detection further than this from every real object is a FALSE POSITIVE


def nearest(truth, det):
    """(entity id, distance) of the real object of the detection's class closest to its position."""
    cands = {e: p for e, p in truth.items() if (e == 'fire') == (det.class_id == 'fire')}
    return min(((e, math.hypot(det.position.x - x, det.position.y - y)) for e, (x, y) in cands.items()),
               key=lambda t: t[1])


def face_arena(bot):
    """Look back along the start heading (odom yaw 0), where the fire and two victims are in view. Tests in this
    module share one simulation, so each one puts the robot where it needs it rather than trusting the last test."""
    bot.turn_to(0.0, rate=1.0, settle=1.0)


def collect(bot, sim_s, vx=0.0, wz=0.0):
    bot.detections.clear()
    bot.spin_sim(sim_s, vx=vx, wz=wz)
    return [m for _, m in bot.detections]


def valid(msgs):
    return [d for m in msgs for d in m.detections if d.position_valid]


def seen(truth, dets, tol_victim=VICTIM_TOL_M, tol_fire=FIRE_TOL_M):
    """{entity: median error} over the valid detections that match it; asserts nothing is a false positive."""
    err = {}
    for d in dets:
        e, dist = nearest(truth, d)
        assert dist <= MATCH_M, f'false positive: {d.class_id} at ({d.position.x:.2f}, {d.position.y:.2f}), {dist:.2f} m from anything real'
        err.setdefault(e, []).append(dist)
    return {e: float(np.median(v)) for e, v in err.items()}
