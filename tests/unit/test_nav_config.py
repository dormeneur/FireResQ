"""Navigation launch/config rules that must not silently regress (no ROS graph, no simulator)."""
import importlib.util
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
NAV = REPO / 'src' / 'fire_resq_navigation'


def load(path):
    spec = importlib.util.spec_from_file_location(path.stem.replace('.', '_'), path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def stack():
    launch = pytest.importorskip('launch')
    return load(NAV / 'launch' / 'nav_stack.launch.py'), launch


def ctx_for(launch, **cfg):
    c = launch.LaunchContext()
    c.launch_configurations.update({'localization': 'slam', 'map': '', 'slam_preset': 'scan_matching',
                                    'navigation': 'true', 'use_rviz': 'false', 'use_sim_time': 'true', **cfg})
    return c


@pytest.mark.parametrize('mode,expect_slam,expect_amcl', [('slam', True, False), ('amcl', False, True), ('none', False, False)])
def test_at_most_one_node_can_publish_map_to_odom(stack, mode, expect_slam, expect_amcl):
    mod, launch = stack
    from launch.actions import IncludeLaunchDescription
    ctx = ctx_for(launch, localization=mode, map='/tmp/m.yaml')
    live = {}
    for e in mod.generate_launch_description().entities:
        if isinstance(e, IncludeLaunchDescription) and e.condition is not None:
            src = repr(e.launch_description_source.__dict__)
            key = 'slam' if 'mapping.launch.py' in src else 'amcl' if 'localization.launch.py' in src else 'other'
            live[key] = bool(e.condition.evaluate(ctx))
    assert live.get('slam') is expect_slam and live.get('amcl') is expect_amcl
    assert not (live.get('slam') and live.get('amcl'))


def test_invalid_localization_and_amcl_without_a_map_are_rejected(stack):
    mod, launch = stack
    with pytest.raises(RuntimeError, match='invalid'):
        mod._check_localization(ctx_for(launch, localization='both'))
    with pytest.raises(RuntimeError, match='needs map'):
        mod._check_localization(ctx_for(launch, localization='amcl', map=''))
    for ok in (dict(localization='slam'), dict(localization='none'), dict(localization='amcl', map='/tmp/m.yaml')):
        assert mod._check_localization(ctx_for(launch, **ok)) == []


def slam(preset):
    return yaml.safe_load((NAV / 'config' / f'slam_toolbox_{preset}.yaml').read_text())['slam_toolbox']['ros__parameters']


def test_every_slam_preset_uses_this_robots_frames():
    for preset in ('scan_matching', 'odom_only'):
        p = slam(preset)
        assert (p['base_frame'], p['odom_frame'], p['map_frame'], p['scan_topic']) == ('base_link', 'odom', 'map', '/scan')


def test_scan_matching_preset_is_real_slam_that_skips_rotation_only_scans():
    p = slam('scan_matching')
    assert p['use_scan_matching'] is True
    assert p['minimum_travel_heading'] >= 3.0, 'rotation-only scans must be skipped (they diverge)'
    assert p['minimum_travel_distance'] > 0
    assert p['do_loop_closing'] is False and p['max_laser_range'] == 8.0


def test_odom_only_preset_is_labelled_not_slam_and_has_matching_off():
    """Mapping with trusted simulator odometry must never be mistaken for scan-matching SLAM."""
    assert slam('odom_only')['use_scan_matching'] is False
    header = (NAV / 'config' / 'slam_toolbox_odom_only.yaml').read_text().split('slam_toolbox:')[0]
    assert 'NOT SLAM' in header and 'baseline' in header.lower()
    assert 'NOT SLAM' not in (NAV / 'config' / 'slam_toolbox_scan_matching.yaml').read_text().split('slam_toolbox:')[0]


def test_amcl_and_depth_to_scan_configs_match_the_robot():
    a = yaml.safe_load((NAV / 'config' / 'amcl.yaml').read_text())['amcl']['ros__parameters']
    assert a['base_frame_id'] == 'base_link' and a['scan_topic'] == 'scan' and a['tf_broadcast'] is True
    d = yaml.safe_load((NAV / 'config' / 'depth_to_scan.yaml').read_text())['depthimage_to_laserscan']['ros__parameters']
    assert d['output_frame'] == 'camera_link' and d['range_max'] == 8.0 and d['scan_height'] <= 6
    assert a['laser_max_range'] == d['range_max']


def test_navigation_package_never_touches_simulation_or_ground_truth():
    """Navigation is shared with hardware: it may only use stable ROS interfaces."""
    import re
    for f in list(NAV.rglob('*.py')) + list(NAV.rglob('*.yaml')):
        code = re.sub(r'(?m)#.*$', '', f.read_text())
        assert not re.search(r'fire_resq_simulation|dynamic_pose|/pose/info|gz\.msgs|scenarios/', code), f


def test_arena_include_is_scoped_so_it_cannot_overwrite_the_navigation_arguments():
    """Launch arguments are global across included launch files. The arena include sets
    use_rviz:=false; unscoped, that silently disabled the navigation RViz (a real bug)."""
    pytest.importorskip('launch')
    from launch.actions import GroupAction, IncludeLaunchDescription
    mod = load(REPO / 'simulation' / 'launch' / 'arena_nav.launch.py')
    entities = mod.generate_launch_description().entities
    for e in entities:
        if isinstance(e, IncludeLaunchDescription) and 'arena.launch.py' in repr(e.launch_description_source.__dict__):
            raise AssertionError('arena.launch.py is included outside a scoped GroupAction')
    groups = [e for e in entities if isinstance(e, GroupAction)]
    assert groups and all(g._GroupAction__scoped for g in groups)


def test_perception_is_opt_in_and_started_once_by_the_navigation_launch():
    """`perception:=true` on arena_nav.launch.py is a GLOBAL argument, so the arena include would see it too and start a
    second node. The include must pin perception:=false, and the nav launch's own perception include must be scoped
    and conditional."""
    pytest.importorskip('launch')
    from launch.actions import GroupAction, IncludeLaunchDescription

    def includes(group, launch_file):
        return [a for a in group.get_sub_entities() if isinstance(a, IncludeLaunchDescription)
                and launch_file in ''.join(getattr(p, 'text', '') for p in a.launch_description_source._LaunchDescriptionSource__location)]
    mod = load(REPO / 'simulation' / 'launch' / 'arena_nav.launch.py')
    entities = mod.generate_launch_description().entities
    args = {e.name: e for e in entities if type(e).__name__ == 'DeclareLaunchArgument'}
    assert args['perception'].default_value[0].text == 'false'
    assert args['perception_frame'].default_value[0].text == 'map'
    arena_groups = [g for g in entities if isinstance(g, GroupAction) and includes(g, 'arena.launch.py')]
    assert len(arena_groups) == 1
    pinned = dict(includes(arena_groups[0], 'arena.launch.py')[0]._IncludeLaunchDescription__launch_arguments)
    assert pinned['perception'] == 'false'
    perception = [g for g in entities if isinstance(g, GroupAction) and includes(g, 'perception.launch.py')]
    assert len(perception) == 1 and perception[0].condition is not None and perception[0]._GroupAction__scoped
