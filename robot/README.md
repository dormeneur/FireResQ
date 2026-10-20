# The real robot (Raspberry Pi 3B + Arduino Uno)

The physical robot is **not** the ESP32 / 2WD / encoder / RealSense design in [docs/Hardware.md](../docs/Hardware.md).
What is actually built (October 2026):

| Part | What |
| --- | --- |
| Computer | Raspberry Pi 3B (905 MB RAM, Debian 13, no ROS), `ssh pi@pi.local` |
| Motor controller | Arduino Uno + L293D shield (Adafruit Motor Shield v1), USB serial `/dev/ttyUSB0`, 115200 baud |
| Drive | 4 TT motors, **no encoders**. M1 front-left, M4 front-right, M2 back-left, M3 back-right. One motor is slower |
| Power | Motors: 8×AA as two 4-cell packs **in parallel** (6 V) on EXT_PWR, PWR jumper OFF. Pi: its own power bank |
| Camera | OV5647 ribbon camera (Picamera2), **mounted rotated 90° clockwise**: every frame is rotated 90° CCW first |
| Range | HC-SR04 sonar on a servo (SERVO 1 = D10), TRIG A3, ECHO A4; straight ahead is servo 128° |

**The task:** the fire is the red shoe box; the victims are yellow-hubbed wheels on the floor. Survey from the start,
rank the victims by distance to the fire, then for each in that order drive right up to it and come back to the start.
Every run maps afresh, so the wheels can be moved: nothing about the layout is in the code.

The ROS 2 stack in `src/` cannot run on this robot (Pi 3B, no encoders, no depth camera, no ROS installed), so it runs
plain Python. The Pi keeps the files in `~/rescue` (a copy of this folder).

| File | What |
| --- | --- |
| `rescue_bot.py` | **The mission**, its dashboard server, calibration modes and self-tests |
| `dashboard.html` | The web dashboard (live camera, map, actions, reasoning), served by `rescue_bot.py` |
| `wander_bot.py` | Obstacle-avoiding wanderer, proven on the robot; `rescue_bot.py` reuses its serial and drive code |
| `wander_fw.ino` / `.hex` | Uno firmware: `M a b c d` motor powers (400 ms watchdog), `U`/`G`/`P` servo + sonar |
| `servo.py` | Point the sonar servo at an angle and read the distance there |
| `calibration.json` | Written by `--calibrate-horizon` (not in git: it belongs to one robot's camera mount) |

## Run it

```bash
scp robot/*.py robot/dashboard.html robot/wander_fw.hex pi@pi.local:~/rescue/
ssh pi@pi.local
cd ~/rescue
python3 -u rescue_bot.py --selftest                # the maths, no hardware (~1 s)
python3 -u rescue_bot.py --view                    # camera + detection only, motors untouched
python3 -u rescue_bot.py --calibrate-horizon 1.0   # ONCE: one wheel at a measured 1.00 m straight ahead (no motors)
python3 -u rescue_bot.py --survey                  # survey, map and rank; turns in place, drives nowhere
python3 -u rescue_bot.py                           # THE MISSION, started at the start spot facing the arena
```

Watch it on **http://pi.local:8090**: live camera (top left), the map with the fire's safety ring, the victims
labelled A/B/C left to right with their rescue rank, the planned route, the robot and its trail (top right), what the
robot is doing (bottom left) and why (bottom right). The map is built from the camera; it is not lidar SLAM.

If `targetfire.service` (the drone project) is running it holds the camera: `sudo systemctl stop targetfire` first.

## Depth: how a flat picture gives distances, and why the order comes out right

The camera is 10 cm above a flat floor, looking level, so the horizon is a fixed image row. A point on the floor
*N* pixels below the horizon is at

    range = (camera height − z) × focal length (px) / N

where *z* is the point's height (0 for the box's base, the wheel radius for a hub's centre). With the bearing from the
pixel column, every sighting becomes a point on the floor. Object *size* is not used: a hub is ~15 px tall at 2 m, so
a pixel is a 7% error.

The catch is that the error grows with range squared: a pixel row is ~2 cm at 1 m but ~9 cm at 2 m for a hub. What
matters is only the **order**, "which victim is closest to the fire", and four things protect it:

1. **Same-photo distances.** Each victim's distance to the fire is measured between the two *in the same frame*
   wherever both are in view, so the robot's own position and heading errors cancel exactly.
2. **Many photos.** Three frames per view, each object seen from 2–3 views, medians throughout. Two blobs in one photo
   are always two objects; one far wheel is never split in two (its range tolerance grows with its range error).
3. **Close calls are re-measured from nearer.** If two victims' fire distances differ by less than twice their
   combined uncertainty, the robot drives halfway to the fire, where a pixel costs a quarter of the depth, and looks
   again.
4. **Identity by bearing *and* range.** Victims look alike, so the robot always picks "which wheel is that" by
   both, each weighed by its own expected error. Bearing alone confuses wheels that line up, especially after a few
   degrees of heading drift.

`--robustness` measures it: 2000 random arenas with a simulated camera (1.5 px noise per frame, heading drift, and a
2 px camera-tilt error shared by every picture). The rescue order is right in:

| Smallest gap between the victims' true fire distances | uncalibrated tilt (2 px) | after `--calibrate-horizon` (~0.5 px) |
| --- | --- | --- |
| 30 cm or more | 98.8% | 99.7% |
| 20–30 cm | 95.5% | 97.4% |
| 10–20 cm | 92.9% | 96.4% |
| under 10 cm | 70% | 75% |

Under ~10 cm apart the camera can't reliably tell them apart from where it stands; the dashboard says "too close to
call" and shows the uncertainty. A wheel standing *directly behind* the box or another wheel, as seen from the start,
can't be seen at all (about 1 arena in 10 when the wheels are placed at random).

## Moving without encoders

Measured on the floor, 2026-10-20: skid and pivot turns do not reliably turn this robot on tile (all four motors are
fine in the air). So:

- **Turns are arcs.** All wheels roll forward (or all backward), one side slower; a forward arc then a backward arc
  turns it roughly on the spot. No wheel is dragged sideways.
- **Heading comes from the camera.** The camera thread adds up the sideways image shift between every pair of
  consecutive frames, so a jerky turn can never outrun it. Turns run until that heading is reached.
- **Straight legs hold that heading.** This also cancels the slow motor's pull. If the steering stays biased, the
  log prints `steering bias` and `WEAK_MOTOR` / `WEAK_RATIO` need tuning.
- **The way back is driven in reverse** along the same route, so the robot never turns round and a wrong
  `CRUISE_MPS` cancels out. Then the fire, seen from the start, re-anchors heading and position.
- **Reaching a victim.** The robot homes in on the wheel with the camera until the hub reaches the bottom of the
  frame (~15 cm), then creeps 3 cm blind, leaving a gap of about 5 cm.
- **Safety stops.** The sonar stops any forward leg for an obstacle (it goes on the map and the route is re-planned
  round it), and the camera stops it if the fire is within 25 cm ahead.

## Calibration knobs (top of `rescue_bot.py`)

| Knob | Status |
| --- | --- |
| `COLOURS` | Sampled in this room's dim light. Hubs: H 20–33, S ≥ 150, V ≥ 70 (the yellow cardboard box is duller). Retune with `--view` if the light changes |
| `HORIZON_Y` | 318 by default; `--calibrate-horizon` measures it properly into `calibration.json` |
| `CAM_HEIGHT_M`, `HFOV_DEG` | 0.10 m, 41.4° (OV5647 spec). A wrong value scales every distance alike, so the order is unaffected |
| `SONAR_CENTER_DEG` | 128, found by hand with `servo.py` |
| `WEAK_MOTOR`, `WEAK_RATIO` | M3 at 85%, a guess from the old `robot.yaml`. Tune from the `steering bias` lines |
| `CRUISE_MPS` | 0.25 m/s, a guess. The reverse return and the fire re-anchoring make it matter little |
