"""The bottle demo's libraries (src/fire_resq_demo): the detector, the behaviour and the dead-reckoned pose.

The behaviour is run against a pretend robot that, like the real one, does not do what it is told: its true turn
rate is unknown to the code under test, and may be several times off, reversed, or zero at low power. The
detector is run against rendered pictures and one REAL frame from the robot's camera (tests/data).
No ROS, no hardware.
"""
import math
import re
from pathlib import Path

import cv2
import numpy as np
import pytest

from fire_resq_demo.colours import colour_name, hue_distance
from fire_resq_demo.fake_world import FakeRobot, FakeWorld, Thing, demo_room
from fire_resq_demo.mapping import FloorMap, PoseTracker, draw_map
from fire_resq_demo.seeker import APPROACH, DONE, IDLE, WAIT, Seeker, SeekerParams, Sighting
from fire_resq_demo.vision import Camera, FloorModel, VisionParams, find_bottles

ROOT = Path(__file__).resolve().parents[2]
DEMO = ROOT / 'src' / 'fire_resq_demo'
RED, BLUE, YELLOW = dict(bgr=(40, 40, 215), hue=0.0), dict(bgr=(200, 110, 30), hue=106.0), dict(bgr=(40, 200, 235), hue=27.0)


# ------------------------------------------------------------------------------------------------ detector
def test_the_real_cameras_black_bottle_is_found_and_the_wall_is_not():
    """One frame from the robot's C270: a black bottle about 0.4 m ahead on a white tiled floor, beige wall."""
    img = cv2.imread(str(ROOT / 'tests' / 'data' / 'c270_black_bottle.jpg'))
    bottles = [b for b in find_bottles(img, Camera(), VisionParams()) if not b.why]
    assert [b.name for b in bottles] == ['black']
    b = bottles[0]
    assert 280 <= b.x <= 292 and 136 <= b.w <= 152, 'the box must fit the bottle, not the dark wall beside it'
    assert abs(math.degrees(b.bearing)) < 5.0 and 0.25 < b.range_m < 0.6


def test_the_real_floor_scan_follows_the_wall_and_the_bottle_but_not_the_tile_joint():
    img = cv2.imread(str(ROOT / 'tests' / 'data' / 'c270_black_bottle.jpg'))
    scan, _ = FloorModel().scan(img, Camera(), VisionParams())
    assert np.isfinite(scan.row).all()
    assert scan.row.max() < 320, 'the tile joint at row ~430 is a line on the floor, not an obstacle'
    middle = scan.row[15:21]                                     # the strips the bottle stands in
    assert (np.abs(middle - 298) < 12).all(), 'the floor stops at the bottle base there'
    assert (scan.row[:10] < 260).all() and (scan.row[-8:] < 260).all(), 'and at the wall on either side'


def _render(things, **robot):
    return FakeWorld(things, robot=FakeRobot(**robot)).render()


def test_coloured_and_black_bottles_are_found_with_their_colour_and_range():
    things = [Thing(1.0, 0.2, **RED), Thing(1.6, -0.3, **BLUE), Thing(0.8, 0.0, bgr=(25, 25, 25), hue=None)]
    found = {b.name: b for b in find_bottles(_render(things), Camera(), VisionParams()) if not b.why}
    assert set(found) == {'red', 'blue', 'black'}
    for thing, name in zip(things, ('red', 'blue', 'black')):
        true_range = math.hypot(thing.x, thing.y) - thing.radius
        assert found[name].range_m == pytest.approx(true_range, rel=0.12)
        assert found[name].bearing == pytest.approx(math.atan2(thing.y, thing.x), abs=math.radians(2.0))


def test_nearest_bottle_comes_first_and_two_touching_bottles_are_two():
    things = [Thing(1.2, 0.040, **RED), Thing(1.2, -0.035, **BLUE), Thing(0.6, 0.15, **YELLOW)]
    names = [b.name for b in find_bottles(_render(things), Camera(), VisionParams()) if not b.why]
    assert names[0] == 'yellow' and set(names) == {'yellow', 'red', 'blue'}


def test_what_is_not_a_bottle_is_rejected_with_a_reason():
    cam = Camera()
    squat = Thing(1.0, 0.0, radius=0.15, height=0.10, **RED)             # a wide red box
    huge = Thing(2.0, 0.0, radius=0.20, height=1.50, **BLUE)             # a blue door
    edge = Thing(0.5, 0.21, **YELLOW)                                    # half out of the picture
    # `huge` runs off the top of the picture, so its size cannot be judged: since 2026-10-13 it is ACCEPTED. The size
    # rule had called a real 26 cm bottle "too big" on the floor because the horizon row is only a guess.
    for thing, why in ((squat, 'shape'), (huge, ''), (edge, 'half out of view')):
        blobs = find_bottles(_render([thing]), cam, VisionParams())
        assert [b.why for b in blobs] == [why], thing
        assert [b.partial for b in blobs] == [why == 'half out of view'], 'only a would-be bottle counts as half-seen'
    floating = np.full((cam.height, cam.width, 3), 128, np.uint8)
    cv2.rectangle(floating, (300, 40), (340, 150), (40, 40, 215), -1)    # a red shirt above the horizon
    assert [b.why for b in find_bottles(floating, cam, VisionParams())] == ['not on the floor']


def test_a_dim_wall_is_neither_coloured_nor_black():
    """The measured room: wall saturation 60-100 and brightness 57-107. Neither threshold may catch it."""
    p = VisionParams()
    assert p.sat_min > 100 and p.dark_max < 57
    wall = np.zeros((480, 640, 3), np.uint8)
    wall[:] = cv2.cvtColor(np.uint8([[[29, 95, 70]]]), cv2.COLOR_HSV2BGR)[0, 0]
    assert find_bottles(wall, Camera(), p) == []


def test_floor_scan_sees_an_obstacle_at_its_distance_and_clear_floor_elsewhere():
    box = Thing(0.6, 0.0, radius=0.09, height=0.16, bgr=(96, 100, 104), hue=None, bottle=False)
    scan, _ = FloorModel().scan(_render([box]), Camera(), VisionParams())
    hit = np.isfinite(scan.x)
    assert 3 <= hit.sum() <= 12
    assert scan.x[hit] == pytest.approx(0.51, abs=0.06)
    assert np.abs(scan.y[hit]).max() < 0.13
    assert not np.isfinite(FloorModel().scan(_render([]), Camera(), VisionParams())[0].x).any()


def test_colour_identity():
    assert [colour_name(h) for h in (0, 175, 12, 27, 60, 106, 140, 160, None)] == \
        ['red', 'red', 'orange', 'yellow', 'green', 'blue', 'purple', 'pink', 'black']
    assert hue_distance(2, 178) == 4 and hue_distance(None, None) == 0 and math.isinf(hue_distance(None, 5))


# ----------------------------------------------------------------------------------------------- behaviour
def picture(yaw, blank=False):
    """A stand-in for the tiny grey camera picture: vertical stripes fixed in the room, so it shifts as the robot
    turns. `blank`: a featureless wall, the same from every angle."""
    columns = np.linspace(0.39, -0.39, 32)                       # each column's bearing (a 45 degree view)
    row = np.full(32, 128.0) if blank else 128.0 + 80.0 * np.sin(5.0 * (yaw + columns))
    return np.tile(row, (24, 1)).astype(np.uint8)


def run(robot=None, things=None, seconds=300.0, start=True, frame_gap=0.15, frame_delay=0.15, hook=None, textured=True,
        **params):
    """Drive the Seeker against the pretend world: 40 Hz control, ~7 fps pictures that arrive late."""
    params.setdefault('left_boost', 1.0)                         # the pretend robot turns as told: no extra push needed
    params.setdefault('right_boost', 1.0)
    world = FakeWorld(things if things is not None else demo_room(), robot=robot or FakeRobot())
    tracker = PoseTracker()
    seeker = Seeker(SeekerParams(**params), tracker)
    log = {'closest': {}, 'waits': [], 'cmds': [], 'targets': [], 'world': world, 'tracker': tracker}
    t, dt, pending, next_frame, wait_from = 0.0, 0.025, [], 0.0, None       # 40 Hz, as demo_node's control timer
    if start:
        seeker.start(t)
    while t < seconds and (seeker.state != DONE or seeker.moving):
        if hook:
            hook(t, world, seeker)
        if t >= next_frame:
            seen = picture(world.robot.yaw + 3.0 * world.robot.x + 2.0 * world.robot.y) if textured else None
            pending.append((t + frame_delay, (*world.observe(), seen)))        # the picture changes as the robot moves
            next_frame = t + frame_gap
        while pending and pending[0][0] <= t:
            seeker.on_frame(t, *pending.pop(0)[1])
        tracker.on_odom(*world.robot.odom)
        v, w = seeker.command(t)
        log['cmds'].append((t, seeker.state, v, w))
        world.robot.step(t, v, w, dt)
        for i, thing in enumerate(world.things):
            gap = world.distance_to(thing) - thing.radius
            log['closest'][i] = min(log['closest'].get(i, 9.0), gap)
        if seeker.state == WAIT and wait_from is None:
            wait_from = t
        if seeker.state != WAIT and wait_from is not None:
            log['waits'].append(t - wait_from)
            wait_from = None
        if seeker.target is not None and (not log['targets'] or log['targets'][-1] != seeker.target.name):
            log['targets'].append(seeker.target.name)
        t += dt
    log['t'] = t
    return seeker, log


def visited(seeker):
    return sorted(b.name for b in seeker.bottles if b.visited)


# The tests marked OLD_SEARCH describe the first behaviour: stand still, turn on the spot to search, go to remembered
# bottles, and finish when a full turn finds nothing. On 2026-10-13 the maintainer replaced that with WANDERING (this
# robot turns badly and drives forward well), in an open world with no walls these runs never end. They are kept, skipped,
# as the record of what the search did; the wandering behaviour is tested at the end of this file.
OLD_SEARCH = pytest.mark.skip(reason='searching by turning on the spot was replaced by wandering (2026-10-13)')

ALL = ['black', 'blue', 'red', 'yellow']
BAD_ROBOTS = {
    'as believed': FakeRobot(),
    'turns 3x slower than odometry says': FakeRobot(turn_gain=0.35),
    'turns 2.5x faster': FakeRobot(turn_gain=2.5),
    'left and right swapped': FakeRobot(swapped=True),
    'does not turn at all below 1.6 rad/s': FakeRobot(min_turn=1.6),
    'does not turn at all below 2.4 rad/s': FakeRobot(min_turn=2.4),
    'drives at 0.4x speed': FakeRobot(speed_gain=0.4),
    'drives at 2x speed': FakeRobot(speed_gain=2.0),
    'slow link (0.4 s)': FakeRobot(delay_s=0.4),
}


@OLD_SEARCH
@pytest.mark.parametrize('name', BAD_ROBOTS)
def test_every_bottle_is_visited_whatever_the_robot_really_does(name):
    """Four bottles (one behind the start, one hidden behind a box) on robots whose real motion the code is not told."""
    import copy
    seeker, log = run(copy.deepcopy(BAD_ROBOTS[name]))
    assert visited(seeker) == ALL, (name, seeker.state, [e[1] for e in seeker.events][-6:])
    assert seeker.state == DONE
    # "Go very close" (2026-10-13): at the believed speed it ends about 7 cm from the bottle; a robot twice as fast
    # as believed just touches it. That is the price of the blind final push, and it is a slider.
    assert all(log['closest'][i] > -0.02 for i in range(4)), f"drove into a bottle: {log['closest']}"
    assert all(log['closest'][i] < 0.13 for i in range(4)), f"stopped too far away (lens to bottle): {log['closest']}"
    assert len(log['waits']) == 4 and all(5.0 <= w < 5.3 for w in log['waits']), log['waits']


@OLD_SEARCH
def test_a_robot_too_weak_to_turn_is_noticed_with_no_bottle_in_view_and_given_more_power():
    """Nothing in view to measure by: only the unchanged picture says the search pulses are doing nothing."""
    seeker, log = run(FakeRobot(min_turn=2.0), things=[Thing(-1.0, 0.3, **RED)])      # the only bottle is behind it
    assert visited(seeker) == ['red'] and seeker.turn_scale > 1.3
    seeker, log = run(FakeRobot(min_turn=99.0), things=[], seconds=600.0)             # it cannot turn at all
    assert seeker.state == DONE and seeker.turn_scale == seeker.p.still_max_scale, 'bounded power, bounded pulses'


@OLD_SEARCH
def test_a_blank_wall_is_not_mistaken_for_a_robot_that_is_not_turning():
    """Plain wall, plain floor: the picture is the same before and after every turn. That must NOT raise the turn
    power (seen in the rehearsal: the power climbed to 4x while the robot was turning perfectly well)."""
    seeker = Seeker(SeekerParams(), PoseTracker())
    seeker.start(0.0)
    t = 0.0
    while seeker.state != DONE and t < 120.0:
        seeker.on_frame(t, [], None, picture(0.0, blank=True))
        seeker.command(t)
        t += 0.05
    assert seeker.state == DONE and seeker.turn_scale == 1.0 and seeker._stretch == 1.0
    seeker, log = run(textured=False)                             # and with no picture at all, as before
    assert visited(seeker) == ALL and seeker.turn_scale == 1.0


@OLD_SEARCH
def test_swapped_wiring_is_noticed_and_said():
    seeker, log = run(FakeRobot(swapped=True))
    assert seeker.turn_sign == -1 and log['tracker'].yaw_gain < 0
    assert any('swapped' in text for _, text in seeker.events)


@OLD_SEARCH
def test_the_real_turn_rate_is_learned_from_the_bottles():
    for gain in (0.35, 1.0, 2.5):
        seeker, log = run(FakeRobot(turn_gain=gain))          # learned from search pulses with a bottle in view
        assert log['tracker'].yaw_gain == pytest.approx(gain, rel=0.15)
        true_rate = gain * seeker.p.turn_speed
        assert true_rate * 0.8 < seeker.turn_rate < true_rate * 1.8, 'per commanded second: pulses run a tick or two long'


def test_nearest_first_and_a_nearer_bottle_that_appears_takes_over():
    far, near = Thing(2.2, 0.0, **RED), Thing(1.2, 0.2, **BLUE)

    def hook(t, world, seeker):                                  # the near one is put down once the robot is on its way
        if t >= 3.0 and near not in world.things:
            world.things.append(near)

    seeker, log = run(things=[far], hook=hook)
    assert log['targets'][:2] == ['red', 'blue'], log['targets']
    assert visited(seeker) == ['blue', 'red'], 'it left the red one for later'
    assert [e[1] for e in seeker.events if 'Reached' in e[1]] == ['Reached the blue bottle', 'Reached the red bottle']
    assert any('closer' in text.lower() for _, text in seeker.events)

    seeker, log = run(things=[Thing(2.0, 0.3, **RED), Thing(0.9, -0.2, **BLUE)])
    assert log['targets'][0] == 'blue'


def test_a_bottle_only_slightly_nearer_does_not_steal_the_target():
    seeker, log = run(things=[Thing(1.50, 0.25, **RED), Thing(1.45, -0.25, **BLUE)])
    assert log['targets'] == ['blue', 'red'], 'no flip-flopping between two bottles at about the same distance'
    assert not any('closer' in text.lower() for _, text in seeker.events)


@OLD_SEARCH
def test_an_obstacle_in_the_way_is_gone_round_not_through():
    box = Thing(0.9, 0.10, radius=0.09, height=0.16, hue=None, bottle=False)     # beside the line, inside the robot's width
    seeker, log = run(things=[Thing(2.0, 0.0, **RED), box])
    assert visited(seeker) == ['red']
    assert any('Obstacle' in text for _, text in seeker.events)
    width = seeker.p.half_width_m
    assert log['closest'][1] > width, f"too close to the box: {log['closest'][1]:.2f} m"
    seeker, log = run(things=[Thing(2.0, 0.0, **RED), box], avoid=False)
    assert log['closest'][1] < width - 0.02, 'the box really is in the way (so the test above means something)'


@OLD_SEARCH
def test_it_finishes_when_a_full_turn_shows_nothing_and_when_told_how_many():
    seeker, log = run(things=[])
    assert seeker.state == DONE and not seeker.bottles and 10.0 < log['t'] < 40.0
    turned = abs(log['world'].robot.yaw)
    assert turned > math.radians(350), 'it must look all the way round before giving up'

    seeker, log = run(expected_bottles=2)
    assert len(visited(seeker)) == 2 and seeker.state == DONE


@OLD_SEARCH
def test_a_bottle_that_is_taken_away_is_given_up_on_not_chased_forever():
    def hook(t, world, seeker):
        if t >= 3.0 and len(world.things) == 2:
            del world.things[0]                                  # someone picks the far bottle up

    seeker, log = run(things=[Thing(2.0, 0.6, **RED), Thing(1.0, -0.2, **BLUE)], hook=hook)
    assert seeker.state == DONE and visited(seeker) == ['blue']
    assert [b.name for b in seeker.bottles if b.lost] == ['red']


def test_it_does_not_move_until_started_or_after_stop_or_without_pictures():
    seeker, log = run(start=False, seconds=5.0)
    assert seeker.state == IDLE and all(v == 0 and w == 0 for _, _, v, w in log['cmds'])

    def stop_at_8(t, world, seeker):
        if 8.0 <= t < 8.02:
            seeker.stop()

    seeker, log = run(hook=stop_at_8, seconds=15.0)
    assert any(v or w for t, _, v, w in log['cmds'] if t < 8.0)
    assert all(v == 0 and w == 0 for t, _, v, w in log['cmds'] if t >= 8.0)

    seeker = Seeker(SeekerParams(), PoseTracker())               # started, then the camera goes quiet while driving
    seeker.start(0.0)
    sight = [Sighting(0.0, 0.0, 1.5, 0.06, 0.02, -0.02)]
    t = 0.0
    while seeker.state != APPROACH and t < 5.0:
        seeker.on_frame(t, sight, None)
        seeker.command(t)
        t += 0.05
    assert seeker.state == APPROACH and seeker.command(t)[0] > 0
    assert seeker.command(t + seeker.p.stale_s + 0.1) == (0.0, 0.0) and 'camera' in seeker.note


@OLD_SEARCH
def test_it_stops_close_and_the_dial_and_the_blind_push_bring_it_closer():
    """On the floor a stop about 20 cm short was "very far": the default must end within 10 cm of the bottle."""
    gaps = {}
    for name, params in (('default', {}), ('far', {'arrive_cover_frac': 0.40}), ('push', {'final_push_s': 0.3})):
        seeker, log = run(things=[Thing(1.5, 0.1, **RED)], **params)
        assert visited(seeker) == ['red']
        gaps[name] = log['closest'][0]
    assert 0.0 < gaps['default'] < 0.13, gaps
    assert gaps['far'] > gaps['default'] + 0.04 and 0.0 < gaps['push'] < gaps['default'] - 0.02, gaps


def test_next_to_the_target_a_half_seen_bottle_is_still_steered_at_but_never_a_new_bottle():
    seeker = Seeker(SeekerParams(), PoseTracker())
    seeker.start(0.0)
    half = Sighting(0.0, 0.25, 0.2, 0.5, partial=True)
    for i in range(6):                                           # only ever half in view: not a bottle to go and find
        seeker.on_frame(i * 0.1, [half], None)
    assert seeker.bottles == [] and not seeker.accepts_partial(0.0)
    t = 1.0
    while seeker.state != APPROACH and t < 6.0:                  # a full view of it, getting close
        seeker.on_frame(t, [Sighting(0.0, 0.0, 0.3, 0.4, 0.05, -0.05, cover_frac=0.2)], None)
        seeker.command(t)
        t += 0.05
    assert seeker.state == APPROACH and seeker.accepts_partial(0.0) and not seeker.accepts_partial(106.0)
    seeker.on_frame(t, [half], None)                             # now it runs off the left edge
    v, w = seeker.command(t)
    assert v > 0 and w > 0, 'keeps driving, steering left towards the part in view'
    assert seeker.command(t + 0.05)[0] > 0


def test_left_turns_are_pushed_harder_and_only_left_turns():
    seeker = Seeker(SeekerParams(left_boost=1.5, right_boost=2.0), PoseTracker())
    assert seeker._turn(1.0) == 1.5 and seeker._turn(-1.0) == -2.0
    seeker.turn_sign = -1                                        # wiring swapped: "left" is sent as a negative command
    assert seeker._turn(1.0) == -1.5 and seeker._turn(-1.0) == 2.0
    sent = {}
    for boost in (1.0, 1.5):                                     # an empty room is searched by turning left
        _, log = run(things=[], left_boost=boost, celebrate=False)
        sent[boost] = {round(w, 3) for _, _, v, w in log['cmds'] if w}
    assert sent == {1.0: {1.2}, 1.5: {1.8}}


def test_two_blobs_of_one_colour_are_one_bottle_and_a_flicker_is_none():
    seeker = Seeker(SeekerParams(), PoseTracker())
    for i in range(5):
        seeker.on_frame(i * 0.1, [Sighting(0.0, 0.1, 1.0, 0.06), Sighting(1.5, -0.2, 2.0, 0.03)], None)
    assert [b.name for b in seeker.bottles if b.confirmed] == ['red']
    seeker.on_frame(1.0, [Sighting(60.0, 0.0, 1.0, 0.05)], None)             # green, for a single frame
    seeker.on_frame(4.0, [], None)
    assert [b.name for b in seeker.bottles] == ['red']


# ------------------------------------------------------------------------------------------- pose and map
def test_pose_tracker_rescales_turning_and_keeps_distance():
    tracker = PoseTracker()
    tracker.yaw_gain = 0.5
    for x, y, yaw in ((0, 0, 0), (0.4, 0, 0), (0.4, 0, math.pi / 2), (0.4, 0.4, math.pi / 2)):
        tracker.on_odom(x, y, yaw)                               # 0.4 m forward, quarter turn left, 0.4 m forward
    assert tracker.raw_yaw == pytest.approx(math.pi / 2)
    assert tracker.yaw == pytest.approx(math.pi / 4), 'the robot really turned half of what /odom said'
    end = (0.4 + 0.4 * math.cos(math.pi / 4), 0.4 * math.sin(math.pi / 4))
    assert (tracker.x, tracker.y) == pytest.approx(end)
    tracker.on_odom(0.0, 0.0, 0.0)                               # the driver restarted and /odom jumped: not motion
    assert (tracker.x, tracker.y) == pytest.approx(end)


def test_floor_map_marks_the_obstacle_where_it_is_and_draws():
    world = FakeWorld([Thing(0.8, 0.0, radius=0.09, bottle=False)])
    floor = FloorMap()
    for _ in range(3):
        floor.add_scan((0.0, 0.0, 0.0), world.observe()[1])
    iy, ix = np.nonzero(floor.occupied())
    assert len(ix) and np.allclose((ix - floor.n // 2 + 0.5) * floor.cell, 0.72, atol=0.06)
    assert floor.free[floor.n // 2, floor.n // 2 + 6] > 0, 'the floor between robot and obstacle is clear'
    img = draw_map(400, PoseTracker(), floor, [{'x': 1.0, 'y': 0.2, 'hue': 0.0, 'status': 'target', 'name': 'red'}], 0.8)
    assert img.shape == (400, 400, 3)


# -------------------------------------------------------------------------------------------------- guards
def _code(path):
    text = re.sub(r'(\"\"\"|\'\'\').*?\1', '', path.read_text(), flags=re.S)
    return re.sub(r'(?m)#.*$', '', text)


def test_the_demo_reaches_the_robot_only_through_ros_topics():
    for p in (DEMO / 'fire_resq_demo').glob('*.py'):
        text = _code(p)
        assert 'fire_resq_hardware' not in text, p
        assert not re.search(r'^\s*(import|from)\s+(serial|RPi|gz|ros_gz)', text, re.M), p


def test_the_behaviour_is_pure():
    """seeker.py decides; it must stay testable without ROS, a clock or OpenCV."""
    text = _code(DEMO / 'fire_resq_demo' / 'seeker.py')
    assert not re.search(r'^\s*(import|from)\s+(rclpy|cv2|time)\b', text, re.M)


def test_the_map_is_never_called_slam():
    """Dead reckoning plus a camera sketch is not SLAM (CLAUDE.md, "Real robot"). The dashboard's map panel is
    titled plainly (the maintainer removed its "dead reckoning, not SLAM" caption); it must never CLAIM SLAM."""
    page = (DEMO / 'fire_resq_demo' / 'index.html').read_text()
    assert 'SLAM' not in page
    assert 'NOT SLAM' in (DEMO / 'fire_resq_demo' / 'mapping.py').read_text()


def test_the_pretend_robot_can_only_be_launched_as_a_rehearsal_and_the_dashboard_is_local():
    launch = (DEMO / 'launch' / 'demo.launch.py').read_text()
    assert re.search(r"executable='fake_robot_node'.*?condition=IfCondition\(LaunchConfiguration\('rehearsal'\)\)", launch, re.S)
    assert "DeclareLaunchArgument('rehearsal', default_value='false')" in launch
    script = (ROOT / 'demo.sh').read_text()
    assert re.search(r'launch 1 rehearsal:=true', script) and not re.search(r'launch 0 .*rehearsal', script)
    assert 'host: 127.0.0.1' in (DEMO / 'config' / 'demo.yaml').read_text()


@OLD_SEARCH
def test_a_remembered_bottle_that_gets_hidden_is_gone_back_for():
    """Seen from the start, then a box is put in front of it: a full turn does not find it, so the robot drives to
    where it remembers it, meets the box, goes round, and finds the bottle behind it."""
    box = Thing(1.2, 0.0, radius=0.10, height=0.30, hue=None, bottle=False)

    def hook(t, world, seeker):
        if t >= 2.0 and box not in world.things:
            world.things.append(box)

    seeker, log = run(things=[Thing(1.9, 0.0, **RED)], hook=hook)
    assert visited(seeker) == ['red'], [e[1] for e in seeker.events]
    assert any('where it was' in text for _, text in seeker.events)
    assert log['closest'][1] > 0.10, 'and it did not drive through the box to get there'


# ------------------------------------------------------------------------------------------------ wandering
def walled_room(things, half=1.6, step=0.25):
    """`things` inside a square room whose walls are rows of posts (the pretend world only has cylinders)."""
    posts, n = [], int(round(2 * half / step))
    for i in range(n + 1):
        a = -half + i * step
        for x, y in ((a, -half), (a, half), (-half, a), (half, a)):
            posts.append(Thing(x, y, radius=0.14, height=0.5, hue=None, bottle=False))
    return list(things) + posts


def test_it_wanders_a_room_visits_what_it_finds_waits_and_moves_on():
    """The behaviour asked for on 2026-10-13: wander, find objects, go to them, stop 5 s, move again. No turning on
    the spot to search, and it never declares itself finished."""
    bottles = [Thing(1.0, 0.3, **RED), Thing(-0.9, 0.9, **BLUE), Thing(0.2, -1.1, **YELLOW)]
    seeker, log = run(things=walled_room(bottles), seconds=420.0)
    assert len(visited(seeker)) >= 2, (visited(seeker), [e[1] for e in seeker.events][-8:])
    assert seeker.state != DONE, 'it keeps wandering until it is stopped'
    assert all(5.0 <= w < 5.3 for w in log['waits']) and len(log['waits']) == len(visited(seeker))
    walls = [log['closest'][i] for i in range(3, len(log['closest']))]
    assert min(walls) > 0.0, f'drove through a wall: {min(walls):.2f} m'
    turns = [w for _, state, v, w in log['cmds'] if state == 'WANDER' and w and not v]
    assert turns and all(w > 0 for w in turns), 'when it has to turn while wandering, it turns left (its good side)'
