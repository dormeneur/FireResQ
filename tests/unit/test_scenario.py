"""Scenario loading, validation and world generation. Pure Python: no ROS, no Gazebo."""
import dataclasses
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from fire_resq_simulation import ScenarioError, build_world_sdf, load_scenario, resolve_scenario
from fire_resq_simulation import scenario as S

REPO = Path(__file__).resolve().parents[2]
DEFAULT = REPO / 'simulation' / 'config' / 'scenarios' / 'default.yaml'


@pytest.fixture(scope='module')
def sc():
    return load_scenario(DEFAULT)


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


# ---------------------------------------------------------------- default scenario
def test_default_loads_and_matches_spec(sc):
    assert (sc.arena.size_x, sc.arena.size_y) == (5.0, 5.0)          # ~5 m x 5 m
    assert len(sc.victims) == 3
    assert len({v.id for v in sc.victims}) == 3                       # unique identities
    assert len(sc.obstacles) >= 1                                     # navigation complexity


def test_robot_starts_inside_safe_zone(sc):
    z, s = sc.safe_zone, sc.robot_start
    assert abs(s.x - z.x) <= z.size_x / 2 and abs(s.y - z.y) <= z.size_y / 2


def test_no_victim_inside_safe_zone(sc):
    z = sc.safe_zone
    for v in sc.victims:
        assert not (abs(v.x - z.x) <= z.size_x / 2 and abs(v.y - z.y) <= z.size_y / 2)


def test_design_intent_nearest_first_and_risk_first_disagree(sc):
    """The scenario must give prioritisation something to decide. Nearest-first must pick a
    different victim than 'most endangered'; otherwise the Phase 8/11 comparison is vacuous."""
    start = (sc.robot_start.x, sc.robot_start.y)
    fire = (sc.fire.x, sc.fire.y)
    by_robot = sorted(sc.victims, key=lambda v: dist((v.x, v.y), start))
    by_fire = sorted(sc.victims, key=lambda v: dist((v.x, v.y), fire))
    assert by_robot[0].id != by_fire[0].id
    assert by_robot[0].id == 'victim_1' and by_fire[0].id == 'victim_2'
    assert by_robot[-1].id == 'victim_2'          # the endangered one is also the farthest


def test_victims_spread_beyond_association_gate(sc):
    for i, a in enumerate(sc.victims):
        for b in sc.victims[i + 1:]:
            assert dist((a.x, a.y), (b.x, b.y)) >= S.VICTIM_SPACING


def test_visibility_design_from_start(sc):
    """victim_3 sits behind the partition, so it must be hidden from the start pose and need
    exploration; fire, victim_1 and victim_2 are in the open."""
    start = (sc.robot_start.x, sc.robot_start.y)
    los = {v.id: S.has_line_of_sight(sc, start, (v.x, v.y)) for v in sc.victims}
    assert los == {'victim_1': True, 'victim_2': True, 'victim_3': False}
    assert S.has_line_of_sight(sc, start, (sc.fire.x, sc.fire.y))


def test_victim_3_becomes_visible_after_going_around_the_partition(sc):
    v3 = next(v for v in sc.victims if v.id == 'victim_3')
    assert S.has_line_of_sight(sc, (-0.1, 1.0), (v3.x, v3.y))         # the survey vantage point


def test_odom_world_round_trip(sc):
    assert sc.world_to_odom(sc.robot_start.x, sc.robot_start.y) == pytest.approx((0, 0), abs=1e-9)
    for x, y in [(0.3, -1.2), (-2, 2), (1.1, 0.4)]:
        assert sc.odom_to_world(*sc.world_to_odom(x, y)) == pytest.approx((x, y), abs=1e-9)
    # one metre straight ahead of the robot, in odom, is along the start heading in the world
    wx, wy = sc.odom_to_world(1.0, 0.0)
    assert (wx - sc.robot_start.x, wy - sc.robot_start.y) == pytest.approx(
        (math.cos(sc.robot_start.yaw), math.sin(sc.robot_start.yaw)), abs=1e-9)


# ---------------------------------------------------------------- generated world
def test_generation_is_deterministic(sc):
    assert build_world_sdf(sc) == build_world_sdf(sc)


def test_generated_world_contains_every_entity_once(sc):
    root = ET.fromstring(build_world_sdf(sc))
    world = root.find('world')
    assert world.get('name') == S.WORLD_NAME
    names = [m.get('name') for m in world.findall('model')] + [i.findtext('name') for i in world.findall('include')]
    expected = set(sc.entities()) | {'wall_north', 'wall_south', 'wall_east', 'wall_west', 'ground_plane'}
    assert set(names) == expected
    assert len(names) == len(set(names))


def test_generated_world_keeps_base_physics_and_plugins(sc):
    """Physics step and sensor plugins are defined once, in the base world."""
    base = ET.parse(REPO / 'simulation' / 'worlds' / 'empty_world.sdf').getroot().find('world')
    gen = ET.fromstring(build_world_sdf(sc)).find('world')
    assert gen.findtext('physics/max_step_size') == base.findtext('physics/max_step_size') == '0.004'
    assert {p.get('name') for p in gen.findall('plugin')} == {p.get('name') for p in base.findall('plugin')}


def test_victims_and_fire_use_reusable_models(sc):
    world = ET.fromstring(build_world_sdf(sc)).find('world')
    uris = {i.findtext('name'): i.findtext('uri') for i in world.findall('include')}
    assert uris['fire'] == 'model://fire'
    assert all(uris[v.id] == 'model://victim' for v in sc.victims)


def test_overview_camera_is_opt_in(sc):
    assert 'overview_camera' not in build_world_sdf(sc)
    assert 'overview_camera' in build_world_sdf(sc, overview_camera=True)


def test_cli_prints_layout_and_writes_world(tmp_path):
    out = tmp_path / 'w.sdf'
    r = subprocess.run([sys.executable, str(REPO / 'simulation' / 'scripts' / 'generate_world.py'),
                        '--ascii', '--output', str(out)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert 'R' in r.stdout and 'F' in r.stdout and out.exists()
    ET.parse(out)


def test_resolve_scenario_by_name_and_path():
    assert resolve_scenario('default').name == 'default.yaml'
    assert resolve_scenario(str(DEFAULT)) == DEFAULT.resolve()
    with pytest.raises(FileNotFoundError):
        resolve_scenario('does_not_exist')


# ---------------------------------------------------------------- validation
def mutate(sc, **kw):
    return dataclasses.replace(sc, **kw)


def replace_victim(sc, vid, **kw):
    return mutate(sc, victims=tuple(dataclasses.replace(v, **kw) if v.id == vid else v for v in sc.victims))


BAD = {
    'victim too close to a wall': lambda sc: replace_victim(sc, 'victim_2', x=2.3),
    'victim outside the arena': lambda sc: replace_victim(sc, 'victim_2', x=9.0),
    'two victims too close': lambda sc: replace_victim(sc, 'victim_1', x=-1.5, y=1.2),
    'victim inside safe zone': lambda sc: replace_victim(sc, 'victim_1', x=-1.9, y=-1.9),
    'victim on the fire': lambda sc: replace_victim(sc, 'victim_2', x=0.9, y=0.7),
    'victim hugging an obstacle': lambda sc: replace_victim(sc, 'victim_3', y=0.75),
    'duplicate victim id': lambda sc: replace_victim(sc, 'victim_3', id='victim_1'),
    'reserved name': lambda sc: replace_victim(sc, 'victim_3', id='fire'),
    'invalid identifier': lambda sc: replace_victim(sc, 'victim_3', id='vic tim'),
    'obstacle outside arena': lambda sc: mutate(sc, obstacles=(dataclasses.replace(sc.obstacles[0], x=2.4),)),
    'obstacles overlap': lambda sc: mutate(sc, obstacles=sc.obstacles + (
        dataclasses.replace(sc.obstacles[1], name='obstacle_9'),)),
    'obstacle over safe zone': lambda sc: mutate(sc, obstacles=sc.obstacles + (
        dataclasses.replace(sc.obstacles[1], name='obstacle_9', x=-1.9, y=-1.5),)),
    'robot start far from safe zone': lambda sc: mutate(sc, robot_start=S.Pose2(1.0, -2.0, 0.0)),
    'robot start on an obstacle': lambda sc: mutate(sc, robot_start=S.Pose2(-1.3, 0.5, 0.0)),
    'fire in the safe zone': lambda sc: mutate(sc, fire=S.Fire(-1.9, -1.4)),
    'no victims': lambda sc: mutate(sc, victims=()),
}


@pytest.mark.parametrize('label', sorted(BAD))
def test_validation_rejects(sc, label):
    with pytest.raises(ScenarioError):
        S.validate(BAD[label](sc))


def test_validation_reports_every_problem_not_just_the_first(sc):
    bad = replace_victim(replace_victim(sc, 'victim_2', x=9.0), 'victim_1', x=-1.9, y=-1.9)
    with pytest.raises(ScenarioError) as e:
        S.validate(bad)
    assert len(e.value.problems) >= 2


def test_yaml_typo_is_rejected(tmp_path):
    raw = yaml.safe_load(DEFAULT.read_text())
    raw['victims'][0]['posn'] = 1
    f = tmp_path / 'typo.yaml'
    f.write_text(yaml.safe_dump(raw))
    with pytest.raises(ScenarioError, match='unknown key'):
        load_scenario(f)


def test_yaml_missing_section_is_rejected(tmp_path):
    raw = yaml.safe_load(DEFAULT.read_text())
    del raw['fire']
    f = tmp_path / 'nofire.yaml'
    f.write_text(yaml.safe_dump(raw))
    with pytest.raises(ScenarioError, match='missing required key'):
        load_scenario(f)


# ---------------------------------------------------------------- geometry helpers
def test_point_to_rectangle_distance():
    box = S._obb(0, 0, 2, 1, 0)
    assert S._point_obb_distance(0, 0, box) == 0
    assert S._point_obb_distance(2, 0, box) == pytest.approx(1.0)
    assert S._point_obb_distance(2, 1.5, box) == pytest.approx(math.hypot(1, 1))
    rot = S._obb(0, 0, 2, 1, math.pi / 2)                   # same box turned 90 degrees
    assert S._point_obb_distance(0, 1.5, rot) == pytest.approx(0.5)


def test_rectangle_overlap_handles_rotation():
    a = S._obb(0, 0, 2, 0.2, 0)
    assert S._obb_overlap(a, S._obb(0, 0.5, 2, 0.2, 0)) is False
    assert S._obb_overlap(a, S._obb(0, 0.5, 2, 0.2, math.pi / 2)) is True   # crosses it
    assert S._obb_overlap(a, S._obb(0, 0.5, 2, 0.2, 0), margin=0.5) is True  # inflated


# ---------------------------------------------------------------- ground-truth queries
def test_raycast_hits_walls_at_the_exact_distance(sc):
    assert S.raycast(sc, 0.0, -2.0, 1.0, 0.0) == pytest.approx(2.5)          # east wall from (0,-2)
    assert S.raycast(sc, 0.0, -2.0, -1.0, 0.0) == pytest.approx(2.5)         # west
    assert S.raycast(sc, 0.0, -2.0, 0.0, -1.0) == pytest.approx(0.5)         # south
    d = 1 / math.sqrt(2)
    assert S.raycast(sc, 2.0, -2.0, d, -d) == pytest.approx(0.5 * math.sqrt(2))   # corner diagonal


def test_raycast_hits_obstacles_fire_and_victims(sc):
    o2 = next(o for o in sc.obstacles if o.name == 'obstacle_2')             # 0.5 x 0.5 at (0.1,-1.1)
    assert S.raycast(sc, -1.0, o2.y, 1.0, 0.0) == pytest.approx((o2.x - o2.size_x / 2) - (-1.0))
    assert S.raycast(sc, 0.9, 0.0, 0.0, 1.0) == pytest.approx(0.6 - S.FIRE_RADIUS)          # fire at (0.9, 0.6)
    v1 = next(v for v in sc.victims if v.id == 'victim_1')
    assert S.raycast(sc, v1.x, v1.y - 0.5, 0.0, 1.0) == pytest.approx(0.5 - S.VICTIM_SCAN_RADIUS)


def test_raycast_handles_rotated_obstacles_and_misses():
    from dataclasses import replace
    base = load_scenario(DEFAULT)
    rot = replace(base, obstacles=(replace(base.obstacles[1], x=0.0, y=0.0, size_x=1.0, size_y=0.2, yaw=math.pi / 2),))
    assert S.raycast(rot, 0.0, -1.5, 0.0, 1.0) == pytest.approx(1.5 - 0.5)   # the long side now faces south
    assert S.raycast(base, 0.0, 0.0, 1.0, 0.0, max_range=0.1) == math.inf    # nothing within 0.1 m


def test_distance_to_surface(sc):
    assert S.distance_to_surface(sc, 2.5, 0.0) == pytest.approx(0.0)          # on the east wall
    assert S.distance_to_surface(sc, 2.4, 0.0) == pytest.approx(0.1)
    assert S.distance_to_surface(sc, sc.fire.x, sc.fire.y) == 0.0             # inside the fire column
    assert S.distance_to_surface(sc, sc.fire.x + 0.25, sc.fire.y) == pytest.approx(0.10)
