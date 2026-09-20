"""World-model configuration and boundary rules that must not silently regress (no ROS graph, no simulator)."""
import importlib.util
import math
import re
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
PKG = REPO / 'src' / 'fire_resq_world_model'
CFG = PKG / 'config' / 'world_model.yaml'


def params():
    return yaml.safe_load(CFG.read_text())['world_model_node']['ros__parameters']


def load(path):
    spec = importlib.util.spec_from_file_location(path.stem.replace('.', '_'), path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_the_shipped_config_is_valid_and_complete():
    from fire_resq_world_model.config import WorldModelConfig
    p = params()
    keys = {k: v for k, v in p.items() if k in ('victim_gate_m', 'fire_gate_m', 'position_alpha', 'min_confidence', 'confirm_hits',
                                                 'tentative_timeout_s', 'confidence_decay_tau_s') or k.startswith('safe_zone.')}
    cfg = WorldModelConfig.from_dict(keys)
    assert cfg.victim_gate_m < 0.5, 'the scenario keeps victims 0.5 m apart; the gate must stay below that'
    assert len(keys) == 12, f'a parameter is missing from world_model.yaml: {sorted(keys)}'


def test_the_node_declares_every_parameter_the_config_file_sets():
    """A typo in the YAML would otherwise be silently ignored by ROS."""
    src = (PKG / 'fire_resq_world_model' / 'world_model_node.py').read_text()
    declared = set(re.findall(r"'([\w.]+)':", src[src.index('_PARAMS = {'):src.index('_SHAPES')]))
    declared |= set(re.findall(r"\(\('?(\w+)'?, ", src)) | {'detections_topic', 'world_state_topic', 'markers_topic',
                                                           'update_status_service', 'world_frame', 'base_frame', 'publish_rate_hz'}
    assert set(params()) <= declared, sorted(set(params()) - declared)


def test_the_configured_safe_zone_matches_the_scenario_in_the_map_frame():
    """The safe zone is configured infrastructure, not discovered - so a changed scenario must not leave the config behind.
    `map` starts at the robot start pose (as `odom` does), so the scenario's zone expressed in odom is the config."""
    from fire_resq_simulation import load_scenario, resolve_scenario
    sc = load_scenario(resolve_scenario('default'))
    p, z = params(), sc.safe_zone
    x, y = sc.world_to_odom(z.x, z.y)
    assert (p['safe_zone.x'], p['safe_zone.y']) == pytest.approx((x, y), abs=1e-6)
    assert math.cos(p['safe_zone.yaw'] + sc.robot_start.yaw) == pytest.approx(1.0, abs=1e-6), 'zone yaw in map = -robot start yaw'
    assert (p['safe_zone.size_x'], p['safe_zone.size_y']) == pytest.approx((z.size_x, z.size_y))
    from fire_resq_world_model.config import SafeZone
    zone = SafeZone(p['safe_zone.x'], p['safe_zone.y'], p['safe_zone.yaw'], p['safe_zone.size_x'], p['safe_zone.size_y'])
    assert zone.contains(0.0, 0.0), 'the mission start pose (the map origin) must be inside the safe zone'
    for v in sc.victims:                                                                     # and no victim starts inside it
        assert not zone.contains(*sc.world_to_odom(v.x, v.y))


def test_world_model_stays_independent_of_gazebo_hardware_perception_internals_and_the_scenario():
    for f in list((PKG / 'fire_resq_world_model').glob('*.py')) + list((PKG / 'launch').glob('*.py')):
        code = re.sub(r'(?m)#.*$', '', re.sub(r'("""|\'\'\').*?\1', '', f.read_text(), flags=re.S))
        assert not re.search(r'^\s*(import|from)\s+(gz|ignition|ros_gz|RPi|serial|cv2|fire_resq_perception|fire_resq_hardware)', code, re.M), f
        assert not re.search(r'fire_resq_simulation|scenarios/|dynamic_pose|/pose/info|source_backend', code), f


def test_the_pure_library_does_not_import_ros():
    """The rules must stay testable without a ROS graph: only the node module may import rclpy."""
    for f in (PKG / 'fire_resq_world_model').glob('*.py'):
        if f.name == 'world_model_node.py':
            continue
        assert not re.search(r'^\s*(import|from)\s+(rclpy|geometry_msgs|tf2_ros|visualization_msgs|fire_resq_interfaces)',
                             f.read_text(), re.M), f


def test_the_world_model_is_opt_in_and_started_once_by_the_navigation_launch():
    pytest.importorskip('launch')
    from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
    entities = load(REPO / 'simulation' / 'launch' / 'arena_nav.launch.py').generate_launch_description().entities
    args = {e.name: e for e in entities if isinstance(e, DeclareLaunchArgument)}
    assert args['world_model'].default_value[0].text == 'false'

    def includes(group, launch_file):
        return [a for a in group.get_sub_entities() if isinstance(a, IncludeLaunchDescription)
                and launch_file in ''.join(getattr(p, 'text', '') for p in a.launch_description_source._LaunchDescriptionSource__location)]
    groups = [g for g in entities if isinstance(g, GroupAction) and includes(g, 'world_model.launch.py')]
    assert len(groups) == 1 and groups[0].condition is not None and groups[0]._GroupAction__scoped
    passed = dict(includes(groups[0], 'world_model.launch.py')[0]._IncludeLaunchDescription__launch_arguments)
    assert passed['world_frame'] == 'map' and passed['use_sim_time'] == 'true'


def test_the_navigation_rviz_shows_the_world_model_markers():
    view = yaml.safe_load((REPO / 'src' / 'fire_resq_navigation' / 'rviz' / 'navigation.rviz').read_text())
    displays = view['Visualization Manager']['Displays']
    topics = [d.get('Topic', {}).get('Value') for d in displays]
    assert '/fire_resq/world_markers' in topics
