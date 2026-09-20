"""Alternative configurations and determinism. Each test launches its own simulation(s), so
these are marked slow: run the fast tier with  -m "not slow"."""
import hashlib
import math
import subprocess
import tempfile
from pathlib import Path

import pytest

from simlib import entity_poses, wrap

pytestmark = pytest.mark.slow


def topics():
    return subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True).stdout.split()


def nodes():
    return subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True).stdout.split()


def test_rgb_only_mode_has_no_depth_but_everything_else_works(arena_factory):
    """use_depth:=false must look exactly like an RGB-only robot: no depth topics, no depth
    bridge, and the RGB / drive / odometry / TF interfaces unchanged."""
    env = arena_factory(['use_depth:=false'], depth=False)
    try:
        t = topics()
        assert not [x for x in t if x.startswith('/camera/depth')], 'depth topics exist in RGB-only mode'
        assert '/bridge_depth' not in nodes()
        assert {'/camera/image_raw', '/camera/camera_info', '/odom', '/cmd_vel', '/joint_states'} <= set(t)
        bot = env.bot
        assert bot.counts['rgb'] > 20
        g0 = env.gt.get()
        bot.drive_distance(0.3)
        g1 = env.gt.get()
        assert math.hypot(g1[0] - g0[0], g1[1] - g0[1]) == pytest.approx(0.3, abs=0.03)
    finally:
        arena_factory.release(env)      # only one simulation may run at a time


def test_the_same_launch_gives_the_same_arena_and_the_same_robot_behaviour(arena_factory, scenario):
    """Determinism: two independent launches -> identical world file, identical entity poses,
    and the same scripted drive ends in (nearly) the same place."""
    runs = []
    for _ in range(2):
        env = arena_factory()
        world = Path(tempfile.gettempdir()) / 'fire_resq_worlds' / f'{scenario.name}.sdf'
        names = set(scenario.entities()) | {'fire_resq'}
        before = entity_poses(names)
        env.bot.drive_distance(1.0)
        env.bot.turn_by(math.pi / 2)
        env.bot.drive_distance(0.5)
        after = env.gt.get()
        runs.append(dict(world=hashlib.sha256(world.read_bytes()).hexdigest(), before=before, after=after))
        arena_factory.release(env)

    a, b = runs
    assert a['world'] == b['world'], 'the generated world differs between launches'
    for n in a['before']:
        assert a['before'][n] == pytest.approx(b['before'][n], abs=1e-3), f'{n} starts differently'
    assert math.hypot(a['after'][0] - b['after'][0], a['after'][1] - b['after'][1]) < 0.01, 'robot ended in a different place'
    assert abs(wrap(a['after'][5] - b['after'][5])) < math.radians(0.5)
