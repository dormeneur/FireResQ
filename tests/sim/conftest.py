"""Fixtures for the simulation regression tier. Each test module gets a FRESH simulation
(module scope), so tests cannot depend on what another module left the robot doing."""
import shutil
import subprocess
import tempfile
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy', reason='ROS 2 not sourced: source /opt/ros/jazzy/setup.bash')

import rclpy  # noqa: E402

from fire_resq_simulation import load_scenario, resolve_scenario  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from routes import LAP_WORLD  # noqa: E402
from simlib import Bot, GroundTruth, lifecycle_state, run_sim  # noqa: E402


def pytest_collection_modifyitems(config, items):
    for it in items:
        it.add_marker(pytest.mark.sim)


@pytest.fixture(scope='session', autouse=True)
def _ros():
    for tool in ('gz', 'ros2'):
        if not shutil.which(tool):
            pytest.skip(f'{tool} not found')
    r = subprocess.run(['ros2', 'pkg', 'prefix', 'fire_resq_simulation'], capture_output=True, text=True)
    if r.returncode != 0:
        pytest.skip('fire_resq_simulation not built/sourced: colcon build && source install/setup.bash')
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope='session')
def scenario():
    return load_scenario(resolve_scenario('default'))


def _bring_up(args, depth, launch_file='arena.launch.py', scan=False, nav=False, wait_map=None, perception=False,
              world=False):
    """Launch, wait until the robot is publishing and has settled, return a handle bundle."""
    ctx = run_sim(args, launch_file=launch_file)
    sim = ctx.__enter__()
    bot = gt = None
    try:
        bot = Bot(depth=depth, scan=scan, nav=nav, perception=perception, world=world)
        gt = GroundTruth()
        needed = (['odom', 'rgb', 'joint_states'] + (['depth'] if depth else []) + (['scan'] if scan else [])
                  + (['detections'] if perception else []) + (['world_state'] if world else []))
        bot.wait_for(lambda: all(bot.counts[k] > 5 for k in needed) and gt.get() is not None
                     and 'rgb_info' in bot.msgs and (not depth or 'depth_info' in bot.msgs)
                     and (not (nav if wait_map is None else wait_map) or (len(bot.grids) > 0 and bot.map_pose() is not None)),
                     timeout=150, what='simulation to publish ' + ', '.join(needed))
        t0 = bot.simt
        bot.wait_for(lambda: bot.simt - t0 > 3.0, timeout=30, what='robot to settle')
        return SimpleNamespace(sim=sim, bot=bot, gt=gt, ctx=ctx)
    except BaseException:
        _teardown(SimpleNamespace(bot=bot, gt=gt, ctx=ctx))
        raise


def _teardown(env):
    if env.gt:
        env.gt.stop()
    if env.bot:
        env.bot.destroy_node()
    env.ctx.__exit__(None, None, None)


@pytest.fixture(scope='module')
def arena():
    env = _bring_up([], depth=True)
    yield env
    _teardown(env)


@pytest.fixture(scope='session')
def truth(scenario):
    t = {'fire': scenario.world_to_odom(scenario.fire.x, scenario.fire.y)}
    t.update({v.id: scenario.world_to_odom(v.x, v.y) for v in scenario.victims})
    return t


@pytest.fixture(scope='module')
def percsim():
    """The arena (60 deg camera) with the perception node running; detections are published in `odom`."""
    env = _bring_up(['perception:=true'], depth=True, perception=True)
    yield env
    _teardown(env)


@pytest.fixture(scope='module')
def percsim_nav():
    """arena_nav.launch.py (87 deg camera) with perception on - proves the navigation launch wires it too.
    No localization, so positions are in `odom`."""
    env = _bring_up(['localization:=none', 'navigation:=false', 'perception:=true', 'perception_frame:=odom'],
                    depth=True, launch_file='arena_nav.launch.py', scan=True, perception=True)
    yield env
    _teardown(env)


@pytest.fixture(scope='module')
def worldsim():
    """The Phase 6 stack: arena_nav (87 deg) + scan-matching SLAM (so a real `map` frame) + perception + world model.
    No Nav2: the tests drive the robot themselves."""
    env = _bring_up(['navigation:=false', 'perception:=true', 'world_model:=true'], depth=True, launch_file='arena_nav.launch.py',
                    scan=True, nav=True, perception=True, world=True)
    yield env
    _teardown(env)


@pytest.fixture(scope='module')
def navsim():
    """The arena with depth->scan only (arena_nav.launch.py, 87 deg camera, no SLAM/Nav2)."""
    env = _bring_up(['localization:=none', 'navigation:=false'], depth=True, launch_file='arena_nav.launch.py', scan=True)
    yield env
    _teardown(env)


@pytest.fixture(scope='session')
def arena_map_yaml(_ros, scenario):
    """Map the arena ONCE per test session: drive a lap under SLAM (scan-matching preset), turn
    back to the start heading, save the map, and tear that simulation down completely. Every
    module that needs a saved map reuses it, and each launches its own fresh simulation after."""
    env = _bring_up(['navigation:=false'], depth=True, launch_file='arena_nav.launch.py', scan=True, nav=True)
    try:
        bot = env.bot
        for wx, wy in LAP_WORLD:
            bot.go_to(*scenario.world_to_odom(wx, wy))
        bot.turn_to(0.0, settle=1.0)          # the lap ends AT the start but facing the way it drove
        bot.spin_wall(3.0)
        out = Path(tempfile.mkdtemp(prefix='fire_resq_map_')) / 'arena'
        r = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(out), '--ros-args',
                            '-p', 'save_map_timeout:=15000.0', '-p', 'use_sim_time:=true'],
                           capture_output=True, text=True, timeout=60)
        assert Path(str(out) + '.yaml').exists(), r.stdout + r.stderr
    finally:
        _teardown(env)
    return Path(str(out) + '.yaml')


def _amcl_bring_up(map_yaml, navigation):
    """Fresh simulation, AMCL on the saved map (+ Nav2 if asked), start pose given on /initialpose.
    The start pose is the MISSION start pose - the map origin, because the map was built from the
    start - not ground truth."""
    env = _bring_up(['localization:=amcl', f'map:={map_yaml}', f'navigation:={"true" if navigation else "false"}'],
                    depth=True, launch_file='arena_nav.launch.py', scan=True, nav=True, wait_map=False)
    try:
        bot = env.bot
        bot.wait_for(lambda: lifecycle_state('/map_server') == 'active' and lifecycle_state('/amcl') == 'active',
                     90, 'map_server and amcl to become active')
        pub = bot.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.pose.pose.orientation.w = 1.0
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05
        m.pose.covariance[35] = 0.05
        for _ in range(8):
            m.header.stamp = bot.get_clock().now().to_msg()
            pub.publish(m)
            bot.spin_wall(0.4)
        bot.wait_for(lambda: bot.map_pose() is not None, 30, 'map->base_link from AMCL')
        if navigation:
            bot.wait_for(lambda: all(lifecycle_state(f'/{n}') == 'active' for n in
                                     ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator')),
                         90, 'Nav2 servers to become active')
        bot.spin_wall(3.0)
        return env
    except BaseException:
        _teardown(env)
        raise


@pytest.fixture(scope='module')
def amclsim(arena_map_yaml):
    """Fresh arena + AMCL on the saved map (no Nav2)."""
    env = _amcl_bring_up(arena_map_yaml, navigation=False)
    yield env
    _teardown(env)


@pytest.fixture(scope='module')
def navamcl(arena_map_yaml):
    """Fresh arena + AMCL on the saved map + Nav2: the mapped-arena navigation mode."""
    env = _amcl_bring_up(arena_map_yaml, navigation=True)
    yield env
    _teardown(env)


@pytest.fixture(scope='module')
def mapsim():
    """The arena with depth->scan and SLAM (scan-matching preset), before any motion."""
    env = _bring_up([], depth=True, launch_file='arena_nav.launch.py', scan=True, nav=True)
    yield env
    _teardown(env)


@pytest.fixture(scope='module')
def navnav():
    """The full stack: arena + depth->scan + SLAM (scan-matching) + Nav2."""
    env = _bring_up([], depth=True, launch_file='arena_nav.launch.py', scan=True, nav=True)
    yield env
    _teardown(env)


@pytest.fixture(scope='module')
def bare_navsim():
    """Arena + depth->scan, with NO localization and NO Nav2: tests start the pieces themselves."""
    env = _bring_up(['localization:=none', 'navigation:=false'], depth=True, launch_file='arena_nav.launch.py',
                    scan=True, nav=True, wait_map=False)
    yield env
    _teardown(env)


@pytest.fixture(scope='module')
def arena_factory():
    """For tests that need their own launches (RGB-only, repeatability)."""
    made = []

    def make(args=(), depth=True, perception=False):
        env = _bring_up(list(args), depth, perception=perception)
        made.append(env)
        return env

    def release(env):
        made.remove(env)
        _teardown(env)

    make.release = release
    yield make
    for env in list(made):
        _teardown(env)
