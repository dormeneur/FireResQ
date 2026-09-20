"""Phase 5 perception on an RGB-ONLY robot (no depth stream at all), judged against the scenario's ground truth.

Its own module because it needs its own simulation (`use_depth:=false`); the depth-equipped runs are in test_perception.py.
"""
import math

import numpy as np
import pytest

from perceptionlib import RGB_ONLY_FLOOR_M, RGB_ONLY_TOL_FRAC, collect, nearest, valid


@pytest.mark.slow
def test_an_rgb_only_robot_gets_the_same_detections_from_the_known_height_estimator(arena_factory, truth, scenario):
    """No depth stream at all: the SAME node, the SAME message, a different `source_backend`."""
    env = arena_factory(['use_depth:=false', 'perception:=true'], depth=False, perception=True)
    bot = env.bot
    bot.stop(1.0)
    dets = valid(collect(bot, 3.0))
    assert dets, 'no positions without depth'
    assert {d.source_backend for d in dets} == {'known_height'}
    cam = np.array([scenario.world_to_odom(scenario.robot_start.x, scenario.robot_start.y)]).ravel()
    for d in dets:
        e, dist = nearest(truth, d)
        tx, ty = truth[e]
        rng = math.hypot(tx - cam[0], ty - cam[1])
        assert dist <= RGB_ONLY_FLOOR_M + RGB_ONLY_TOL_FRAC * rng, f'{e}: {100 * dist:.1f} cm at {rng:.1f} m'
    assert {'fire', 'victim_1', 'victim_2'} <= {nearest(truth, d)[0] for d in dets}
