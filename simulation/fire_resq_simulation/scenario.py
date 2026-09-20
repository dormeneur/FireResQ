"""Scenario schema, validation, and Gazebo world generation.

A scenario is a YAML file describing one rescue arena: bounds, obstacles, the safe zone, the
fire, the victims and the robot start. `build_world_sdf` turns it into a Gazebo world by
injecting the scenario into the shared base world (worlds/empty_world.sdf), so physics and
sensor plugins are defined exactly once.

Coordinate convention: WORLD frame, metres, arena centred on the origin, +x right, +y up
(north), yaw counter-clockwise from +x. The robot's `odom` frame starts at its spawn pose, so
odom = the world frame translated/rotated by `robot_start` (see Scenario.world_to_odom).

Generation is deterministic: the same YAML always yields byte-identical SDF.
"""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

WORLD_NAME = 'rescue_arena'

# ---- layout rules (metres). They exist so a generated scenario cannot be unwinnable or
# ---- unperceivable; the default scenario is validated against exactly these.
WALL_CLEARANCE = 0.35        # victim/fire centre to inner wall face
OBSTACLE_CLEARANCE = 0.30    # victim/fire/robot-start to an obstacle surface (robot must fit)
VICTIM_SPACING = 0.50        # victim to victim; must exceed the world model's association gate
VICTIM_FIRE_MIN = 0.45       # fire column radius 0.15 + room to approach
ROBOT_START_CLEARANCE = 0.40
START_ZONE_TOLERANCE = 0.30  # robot start must be inside the safe zone, or within this of it

RESERVED_NAMES = {'fire', 'safe_zone', 'ground_plane', 'fire_resq', 'overview_camera'}
_NAME_RE = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')


class ScenarioError(ValueError):
    """Raised with EVERY problem found, not just the first."""

    def __init__(self, problems: List[str]):
        self.problems = list(problems)
        super().__init__('invalid scenario:\n  - ' + '\n  - '.join(self.problems))


# ------------------------------------------------------------------ data model
@dataclass(frozen=True)
class Arena:
    size_x: float = 5.0
    size_y: float = 5.0
    wall_height: float = 0.5
    wall_thickness: float = 0.10


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class SafeZone:
    x: float
    y: float
    size_x: float
    size_y: float


@dataclass(frozen=True)
class Fire:
    x: float
    y: float
    model: str = 'fire'


@dataclass(frozen=True)
class Victim:
    id: str
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class Obstacle:
    name: str
    x: float
    y: float
    size_x: float
    size_y: float
    size_z: float = 0.40
    yaw: float = 0.0


@dataclass(frozen=True)
class Scenario:
    name: str
    arena: Arena
    robot_start: Pose2
    safe_zone: SafeZone
    fire: Fire
    victims: Tuple[Victim, ...]
    obstacles: Tuple[Obstacle, ...] = field(default_factory=tuple)
    description: str = ''

    def entities(self) -> Dict[str, Tuple[str, float, float]]:
        """Ground-truth table name -> (kind, x, y) in the WORLD frame. For tests/evaluation
        only - no robot code may consume this."""
        out = {'fire': ('fire', self.fire.x, self.fire.y),
               'safe_zone': ('safe_zone', self.safe_zone.x, self.safe_zone.y)}
        for v in self.victims:
            out[v.id] = ('victim', v.x, v.y)
        for o in self.obstacles:
            out[o.name] = ('obstacle', o.x, o.y)
        return out

    def world_to_odom(self, x: float, y: float) -> Tuple[float, float]:
        """Express a world-frame point in the robot's odom frame (origin = spawn pose)."""
        s = self.robot_start
        dx, dy = x - s.x, y - s.y
        c, sn = math.cos(-s.yaw), math.sin(-s.yaw)
        return dx * c - dy * sn, dx * sn + dy * c

    def odom_to_world(self, x: float, y: float) -> Tuple[float, float]:
        s = self.robot_start
        c, sn = math.cos(s.yaw), math.sin(s.yaw)
        return s.x + x * c - y * sn, s.y + x * sn + y * c


# ------------------------------------------------------------------ loading
def _take(d, ctx, required=(), optional=()):
    if not isinstance(d, dict):
        raise ScenarioError([f'{ctx}: expected a mapping, got {type(d).__name__}'])
    problems = [f'{ctx}: missing required key "{k}"' for k in required if k not in d]
    problems += [f'{ctx}: unknown key "{k}" (allowed: {sorted(set(required) | set(optional))})'
                 for k in d if k not in set(required) | set(optional)]
    if problems:
        raise ScenarioError(problems)
    return d


def _num(d, key, ctx, default=None):
    v = d.get(key, default)
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ScenarioError([f'{ctx}.{key}: expected a number, got {v!r}'])
    return float(v)


def load_scenario(path) -> Scenario:
    path = Path(path)
    with open(path, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f)
    root = _take(raw, path.name, required=('name', 'arena', 'robot_start', 'safe_zone', 'fire',
                                           'victims'),
                 optional=('description', 'obstacles'))
    a = _take(root['arena'], 'arena', optional=('size_x', 'size_y', 'wall_height',
                                                 'wall_thickness'))
    arena = Arena(*(_num(a, k, 'arena', getattr(Arena, k)) for k in
                    ('size_x', 'size_y', 'wall_height', 'wall_thickness')))
    r = _take(root['robot_start'], 'robot_start', required=('x', 'y'), optional=('yaw',))
    start = Pose2(_num(r, 'x', 'robot_start'), _num(r, 'y', 'robot_start'),
                  _num(r, 'yaw', 'robot_start', 0.0))
    z = _take(root['safe_zone'], 'safe_zone', required=('x', 'y', 'size_x', 'size_y'))
    zone = SafeZone(*(_num(z, k, 'safe_zone') for k in ('x', 'y', 'size_x', 'size_y')))
    f = _take(root['fire'], 'fire', required=('x', 'y'), optional=('model',))
    fire = Fire(_num(f, 'x', 'fire'), _num(f, 'y', 'fire'), str(f.get('model', 'fire')))
    victims = []
    for i, v in enumerate(root['victims'] or []):
        v = _take(v, f'victims[{i}]', required=('id', 'x', 'y'), optional=('yaw',))
        victims.append(Victim(str(v['id']), _num(v, 'x', f'victims[{i}]'),
                              _num(v, 'y', f'victims[{i}]'), _num(v, 'yaw', f'victims[{i}]', 0.0)))
    obstacles = []
    for i, o in enumerate(root.get('obstacles') or []):
        o = _take(o, f'obstacles[{i}]', required=('name', 'x', 'y', 'size_x', 'size_y'),
                  optional=('size_z', 'yaw'))
        c = f'obstacles[{i}]'
        obstacles.append(Obstacle(str(o['name']), _num(o, 'x', c), _num(o, 'y', c),
                                  _num(o, 'size_x', c), _num(o, 'size_y', c),
                                  _num(o, 'size_z', c, 0.40), _num(o, 'yaw', c, 0.0)))
    sc = Scenario(str(root['name']), arena, start, zone, fire, tuple(victims), tuple(obstacles),
                  str(root.get('description', '')).strip())
    validate(sc)
    return sc


# ------------------------------------------------------------------ geometry helpers
def _obb(cx, cy, sx, sy, yaw):
    return (cx, cy, sx / 2.0, sy / 2.0, yaw)


def _corners(b):
    cx, cy, hx, hy, yaw = b
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy)
            for dx, dy in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]


def _point_obb_distance(px, py, b):
    cx, cy, hx, hy, yaw = b
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = (px - cx) * c - (py - cy) * s, (px - cx) * s + (py - cy) * c
    return math.hypot(max(abs(lx) - hx, 0.0), max(abs(ly) - hy, 0.0))


def _obb_overlap(a, b, margin=0.0):
    """Separating-axis test between two oriented rectangles, `a` inflated by `margin`."""
    a = (a[0], a[1], a[2] + margin, a[3] + margin, a[4])
    for box in (a, b):
        yaw = box[4]
        for ax in ((math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))):
            pa = [x * ax[0] + y * ax[1] for x, y in _corners(a)]
            pb = [x * ax[0] + y * ax[1] for x, y in _corners(b)]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


# ------------------------------------------------------------------ validation
def validate(sc: Scenario) -> None:
    """Raise ScenarioError listing every violated rule."""
    p: List[str] = []
    ar = sc.arena
    hx, hy = ar.size_x / 2.0, ar.size_y / 2.0
    if ar.size_x <= 0 or ar.size_y <= 0 or ar.wall_height <= 0 or ar.wall_thickness <= 0:
        p.append('arena: sizes, wall_height and wall_thickness must be positive')

    names = ['fire', 'safe_zone'] + [v.id for v in sc.victims] + [o.name for o in sc.obstacles]
    for n in [v.id for v in sc.victims] + [o.name for o in sc.obstacles]:
        if not _NAME_RE.match(n):
            p.append(f'name "{n}" is not a valid identifier (letters, digits, underscore)')
        if n in RESERVED_NAMES or n.startswith('wall_'):
            p.append(f'name "{n}" is reserved')
    dup = sorted({n for n in names if names.count(n) > 1})
    if dup:
        p.append(f'duplicate entity names: {dup}')
    if len(sc.victims) < 1:
        p.append('at least one victim is required')

    def inside(x, y, margin):
        return abs(x) <= hx - margin and abs(y) <= hy - margin

    zone_box = _obb(sc.safe_zone.x, sc.safe_zone.y, sc.safe_zone.size_x, sc.safe_zone.size_y, 0.0)
    obst = [(o, _obb(o.x, o.y, o.size_x, o.size_y, o.yaw)) for o in sc.obstacles]

    # bounds
    for v in sc.victims:
        if not inside(v.x, v.y, WALL_CLEARANCE):
            p.append(f'{v.id}: ({v.x}, {v.y}) is closer than {WALL_CLEARANCE} m to a wall or outside')
    if not inside(sc.fire.x, sc.fire.y, WALL_CLEARANCE):
        p.append(f'fire: ({sc.fire.x}, {sc.fire.y}) is closer than {WALL_CLEARANCE} m to a wall or outside')
    for o, b in obst:
        if not all(abs(cx) <= hx and abs(cy) <= hy for cx, cy in _corners(b)):
            p.append(f'{o.name}: extends outside the arena')
    if not all(abs(cx) <= hx and abs(cy) <= hy for cx, cy in _corners(zone_box)):
        p.append('safe_zone: extends outside the arena')

    # victims
    for i, a in enumerate(sc.victims):
        for b in sc.victims[i + 1:]:
            d = math.hypot(a.x - b.x, a.y - b.y)
            if d < VICTIM_SPACING:
                p.append(f'{a.id} and {b.id} are {d:.2f} m apart (< {VICTIM_SPACING} m)')
        d = math.hypot(a.x - sc.fire.x, a.y - sc.fire.y)
        if d < VICTIM_FIRE_MIN:
            p.append(f'{a.id} is {d:.2f} m from the fire (< {VICTIM_FIRE_MIN} m)')
        for o, b in obst:
            d = _point_obb_distance(a.x, a.y, b)
            if d < OBSTACLE_CLEARANCE:
                p.append(f'{a.id} is {d:.2f} m from {o.name} (< {OBSTACLE_CLEARANCE} m)')
        if _point_obb_distance(a.x, a.y, zone_box) == 0.0:
            p.append(f'{a.id} starts inside the safe zone (it would already be "rescued")')

    # fire
    for o, b in obst:
        d = _point_obb_distance(sc.fire.x, sc.fire.y, b)
        if d < OBSTACLE_CLEARANCE:
            p.append(f'fire is {d:.2f} m from {o.name} (< {OBSTACLE_CLEARANCE} m)')
    if _point_obb_distance(sc.fire.x, sc.fire.y, zone_box) < VICTIM_FIRE_MIN:
        p.append('fire is inside or too close to the safe zone')

    # obstacles
    for i, (oa, ba) in enumerate(obst):
        if _obb_overlap(ba, zone_box):
            p.append(f'{oa.name} overlaps the safe zone')
        for ob, bb in obst[i + 1:]:
            if _obb_overlap(ba, bb):
                p.append(f'{oa.name} overlaps {ob.name}')

    # robot start
    s = sc.robot_start
    if not inside(s.x, s.y, 0.15):
        p.append('robot_start: outside the arena')
    if _point_obb_distance(s.x, s.y, zone_box) > START_ZONE_TOLERANCE:
        p.append(f'robot_start must be inside the safe zone or within {START_ZONE_TOLERANCE} m of it')
    for o, b in obst:
        if _point_obb_distance(s.x, s.y, b) < ROBOT_START_CLEARANCE:
            p.append(f'robot_start is too close to {o.name}')
    if math.hypot(s.x - sc.fire.x, s.y - sc.fire.y) < ROBOT_START_CLEARANCE + 0.15:
        p.append('robot_start is too close to the fire')
    for v in sc.victims:
        if math.hypot(s.x - v.x, s.y - v.y) < ROBOT_START_CLEARANCE:
            p.append(f'robot_start is too close to {v.id}')
    if p:
        raise ScenarioError(p)


def has_line_of_sight(sc: Scenario, a: Tuple[float, float], b: Tuple[float, float],
                      step: float = 0.02) -> bool:
    """True if the straight segment a->b (world frame, 2D) crosses no obstacle.

    Ground truth for TESTING scenario design ("victim_3 is hidden from the start"); a robot
    must never call this - it learns visibility from its camera."""
    boxes = [_obb(o.x, o.y, o.size_x, o.size_y, o.yaw) for o in sc.obstacles]
    n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
    for i in range(n + 1):
        t = i / n
        x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
        if any(_point_obb_distance(x, y, bx) == 0.0 for bx in boxes):
            return False
    return True


# What a scan at camera height sees of the round objects: the fire column and a victim's torso
# (the steel ring below is out of the scan plane).
FIRE_RADIUS = 0.15
VICTIM_SCAN_RADIUS = 0.05


def raycast(sc: Scenario, ox: float, oy: float, dx: float, dy: float, max_range: float = math.inf) -> float:
    """Distance along the ray (ox,oy)+t*(dx,dy) (unit direction, world frame) to the nearest solid
    - arena walls, obstacles, fire, victims. `inf` if nothing is hit within `max_range`.

    GROUND TRUTH for tests and evaluation (does the depth->scan pipeline see the real geometry?).
    A robot must never call this."""
    best = math.inf
    hx, hy = sc.arena.size_x / 2.0, sc.arena.size_y / 2.0
    for nx, ny, off in ((1, 0, hx), (-1, 0, hx), (0, 1, hy), (0, -1, hy)):        # inner wall faces
        den = dx * nx + dy * ny
        if den > 1e-9:
            best = min(best, (off - (ox * nx + oy * ny)) / den)
    for o in sc.obstacles:                                                         # oriented boxes, slab method
        c, sn = math.cos(-o.yaw), math.sin(-o.yaw)
        lox, loy = (ox - o.x) * c - (oy - o.y) * sn, (ox - o.x) * sn + (oy - o.y) * c
        ldx, ldy = dx * c - dy * sn, dx * sn + dy * c
        t0, t1, hit = 0.0, math.inf, True
        for lo, d, h in ((lox, ldx, o.size_x / 2.0), (loy, ldy, o.size_y / 2.0)):
            if abs(d) < 1e-12:
                hit = hit and abs(lo) <= h
            else:
                a, b = (-h - lo) / d, (h - lo) / d
                t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
        if hit and t0 <= t1 and t0 > 0:
            best = min(best, t0)
    for cx, cy, r in [(sc.fire.x, sc.fire.y, FIRE_RADIUS)] + [(v.x, v.y, VICTIM_SCAN_RADIUS) for v in sc.victims]:
        fx, fy = ox - cx, oy - cy
        b = fx * dx + fy * dy
        disc = b * b - (fx * fx + fy * fy - r * r)
        if disc >= 0:
            t = -b - math.sqrt(disc)
            if t > 0:
                best = min(best, t)
    return best if best <= max_range else math.inf


def distance_to_surface(sc: Scenario, px: float, py: float) -> float:
    """Distance from a world point to the nearest true solid surface (0 if inside one).
    Used to score occupancy grids: an occupied cell far from every real surface is wrong."""
    hx, hy = sc.arena.size_x / 2.0, sc.arena.size_y / 2.0
    d = [abs(hx - abs(px)), abs(hy - abs(py))]
    d += [_point_obb_distance(px, py, _obb(o.x, o.y, o.size_x, o.size_y, o.yaw)) for o in sc.obstacles]
    d.append(max(math.hypot(px - sc.fire.x, py - sc.fire.y) - FIRE_RADIUS, 0.0))
    d += [max(math.hypot(px - v.x, py - v.y) - VICTIM_SCAN_RADIUS, 0.0) for v in sc.victims]
    return min(d)


# ------------------------------------------------------------------ world generation
def _share_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('fire_resq_simulation'))
    except Exception:  # not built/sourced: fall back to the source tree
        return Path(__file__).resolve().parent.parent


def resolve_scenario(name_or_path: str) -> Path:
    """`default` -> config/scenarios/default.yaml; an existing file path is used as given."""
    p = Path(name_or_path)
    if p.is_file():
        return p.resolve()
    cand = _share_dir() / 'config' / 'scenarios' / (name_or_path if name_or_path.endswith('.yaml')
                                                     else name_or_path + '.yaml')
    if cand.is_file():
        return cand
    raise FileNotFoundError(f'scenario "{name_or_path}" not found (looked at {cand})')


def _f(v: float) -> str:
    return f'{v:.6f}'.rstrip('0').rstrip('.') if v != 0 else '0'


def _static_box(name, x, y, z, sx, sy, sz, yaw, rgba, collision=True):
    m = ET.Element('model', name=name)
    ET.SubElement(m, 'static').text = 'true'
    ET.SubElement(m, 'pose').text = f'{_f(x)} {_f(y)} {_f(z)} 0 0 {_f(yaw)}'
    link = ET.SubElement(m, 'link', name='link')
    geom_size = f'{_f(sx)} {_f(sy)} {_f(sz)}'
    if collision:
        c = ET.SubElement(link, 'collision', name='collision')
        ET.SubElement(ET.SubElement(ET.SubElement(c, 'geometry'), 'box'), 'size').text = geom_size
    v = ET.SubElement(link, 'visual', name='visual')
    ET.SubElement(ET.SubElement(ET.SubElement(v, 'geometry'), 'box'), 'size').text = geom_size
    mat = ET.SubElement(v, 'material')
    ET.SubElement(mat, 'ambient').text = rgba
    ET.SubElement(mat, 'diffuse').text = rgba
    return m


def _include(uri: str, name: str, x: float, y: float, yaw: float) -> ET.Element:
    inc = ET.Element('include')
    ET.SubElement(inc, 'uri').text = uri
    ET.SubElement(inc, 'name').text = name
    ET.SubElement(inc, 'pose').text = f'{_f(x)} {_f(y)} 0 0 0 {_f(yaw)}'
    return inc


# Colours: everything that is NOT a perception target is low-saturation so it cannot match the
# victim (blue), fire (orange) or safe-zone (green) hue classes.
_WALL_RGBA = '0.72 0.70 0.66 1'
_OBSTACLE_RGBA = '0.32 0.32 0.36 1'
_SAFE_RGBA = '0.10 0.70 0.20 1'


def _overview_camera(sc: Scenario) -> ET.Element:
    """Opt-in debug aid: a static top-down camera (image up = +y, right = +x)."""
    m = ET.Element('model', name='overview_camera')
    ET.SubElement(m, 'static').text = 'true'
    ET.SubElement(m, 'pose').text = '0 0 5.5 0 1.5707963 1.5707963'
    link = ET.SubElement(m, 'link', name='link')
    s = ET.SubElement(link, 'sensor', name='overview', type='camera')
    ET.SubElement(s, 'always_on').text = 'true'
    ET.SubElement(s, 'update_rate').text = '5'
    ET.SubElement(s, 'topic').text = 'overview/image_raw'
    cam = ET.SubElement(s, 'camera')
    ET.SubElement(cam, 'horizontal_fov').text = '1.0472'
    img = ET.SubElement(cam, 'image')
    ET.SubElement(img, 'width').text = '800'
    ET.SubElement(img, 'height').text = '800'
    ET.SubElement(img, 'format').text = 'R8G8B8'
    clip = ET.SubElement(cam, 'clip')
    ET.SubElement(clip, 'near').text = '0.1'
    ET.SubElement(clip, 'far').text = '20'
    return m


def build_world_sdf(sc: Scenario, base_world: Optional[Path] = None,
                    overview_camera: bool = False) -> str:
    """Return the SDF text of the world for this scenario. Deterministic."""
    base_world = Path(base_world) if base_world else _share_dir() / 'worlds' / 'empty_world.sdf'
    tree = ET.parse(base_world)
    sdf = tree.getroot()
    world = sdf.find('world')
    world.set('name', WORLD_NAME)

    ar = sc.arena
    t, h = ar.wall_thickness, ar.wall_height
    hx, hy = ar.size_x / 2.0, ar.size_y / 2.0
    new: List[ET.Element] = [
        _static_box('wall_north', 0, hy + t / 2, h / 2, ar.size_x + 2 * t, t, h, 0, _WALL_RGBA),
        _static_box('wall_south', 0, -hy - t / 2, h / 2, ar.size_x + 2 * t, t, h, 0, _WALL_RGBA),
        _static_box('wall_east', hx + t / 2, 0, h / 2, t, ar.size_y, h, 0, _WALL_RGBA),
        _static_box('wall_west', -hx - t / 2, 0, h / 2, t, ar.size_y, h, 0, _WALL_RGBA),
    ]
    for o in sc.obstacles:
        new.append(_static_box(o.name, o.x, o.y, o.size_z / 2, o.size_x, o.size_y, o.size_z,
                               o.yaw, _OBSTACLE_RGBA))
    z = sc.safe_zone
    # A 4 mm slab, visual only: the robot drives over it. Sits just above the ground plane.
    new.append(_static_box('safe_zone', z.x, z.y, 0.002, z.size_x, z.size_y, 0.004, 0.0,
                           _SAFE_RGBA, collision=False))
    new.append(_include(f'model://{sc.fire.model}', 'fire', sc.fire.x, sc.fire.y, 0.0))
    for v in sc.victims:
        new.append(_include('model://victim', v.id, v.x, v.y, v.yaw))
    if overview_camera:
        new.append(_overview_camera(sc))

    # Insert after the base world's own content, before nothing else: order is deterministic.
    for el in new:
        world.append(el)
    ET.indent(tree, space='  ')
    header = (f'<!-- GENERATED by fire_resq_simulation.scenario from scenario "{sc.name}". '
              f'Do not edit; change the scenario YAML. -->\n')
    return '<?xml version="1.0"?>\n' + header + ET.tostring(sdf, encoding='unicode') + '\n'


def write_world(sc: Scenario, out_path, overview_camera: bool = False) -> Path:
    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(build_world_sdf(sc, overview_camera=overview_camera), encoding='utf-8')
    return out


# ------------------------------------------------------------------ human-readable layout
def ascii_map(sc: Scenario, res: float = 0.1) -> str:
    """Top-down text map (north up) for review and documentation."""
    ar = sc.arena
    nx, ny = int(round(ar.size_x / res)), int(round(ar.size_y / res))
    zone = _obb(sc.safe_zone.x, sc.safe_zone.y, sc.safe_zone.size_x, sc.safe_zone.size_y, 0.0)
    obst = [_obb(o.x, o.y, o.size_x, o.size_y, o.yaw) for o in sc.obstacles]
    rows = []
    for j in range(ny - 1, -1, -1):
        row = ''
        for i in range(nx):
            x, y = -ar.size_x / 2 + (i + 0.5) * res, -ar.size_y / 2 + (j + 0.5) * res
            ch = ' '
            if _point_obb_distance(x, y, zone) == 0.0:
                ch = '.'
            if any(_point_obb_distance(x, y, b) == 0.0 for b in obst):
                ch = 'X'
            if math.hypot(x - sc.fire.x, y - sc.fire.y) <= 0.15:
                ch = 'F'
            for v in sc.victims:
                if math.hypot(x - v.x, y - v.y) <= res * 0.75:
                    ch = v.id[-1]
            if math.hypot(x - sc.robot_start.x, y - sc.robot_start.y) <= res * 0.75:
                ch = 'R'
            row += ch
        rows.append('#' + row + '#')
    edge = '#' * (nx + 2)
    return '\n'.join([edge] + rows + [edge])
