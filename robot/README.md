# The real robot (Raspberry Pi 3B + Arduino Uno)

The physical robot is **not** the ESP32 / 2WD / encoder / RealSense design in [docs/Hardware.md](../docs/Hardware.md).
What is actually built (October 2026):

| Part | What |
| --- | --- |
| Computer | Raspberry Pi 3B (905 MB RAM, Debian 13, no ROS), `ssh pi@pi.local` |
| Motor controller | Arduino Uno + L293D shield (Adafruit Motor Shield v1), USB serial `/dev/ttyUSB0`, 115200 baud |
| Drive | 4 TT motors, skid steer, **no encoders**. M1 front-left, M4 front-right, M2 back-left, M3 back-right |
| Camera | OV5647 ribbon camera (Picamera2), **mounted rotated 90° clockwise**: every frame is rotated 90° CCW first |
| Range | HC-SR04 sonar on a servo (SERVO 1 = D10), TRIG A3, ECHO A4 |
| Pick-up | **permanent** magnet on the front (not used yet: the current task is only to *reach* each victim) |

**Measured on the floor, 2026-10-20 (4×AA on the shield's EXT_PWR, PWR jumper off):** straight driving works; a skid
spin (one side forward, the other backward) does **not** turn the robot at all; a pivot (one side driving, the other
stopped) turns ~12° per 0.6 s pulse at full power, fading to nothing after 2–3 pulses as the pack sags, and back after a
few seconds' rest. So `rescue_bot.py` measures every turn with the camera. Fresher or higher-voltage cells (e.g. 2×18650
Li-ion, 7.4 V, which the L293D handles) would make turning far more reliable. The Pi's undervoltage seen once came from
running the motors with no battery on EXT_PWR.

**Current task:** the fire is the red shoe box, the victims are three yellow-hubbed wheels on the floor. Visit them in
order of closeness to the fire, return to the start after each, finish at the start.

The ROS 2 stack in `src/` cannot run on this robot as it stands (Pi 3B, no encoders, no depth camera, no ROS
installed), so the robot runs plain Python scripts. The Pi keeps them in `~/rescue` (a copy of this folder).

| File | What |
| --- | --- |
| `wander_fw.ino` / `.hex` | Uno firmware: `M a b c d` motor powers (400 ms watchdog), `U`/`G`/`P` servo + sonar |
| `wander_bot.py` | Obstacle-avoiding wanderer. Proven on the robot; `rescue_bot.py` imports its serial and drive code |
| `rescue_bot.py` | **The mission**: survey from the start, map fire + victims, visit them closest-to-fire first on routes round the fire, return to the start after each |

## Deploy and run

```bash
scp robot/rescue_bot.py robot/wander_bot.py robot/wander_fw.hex pi@pi.local:~/rescue/
ssh pi@pi.local
cd ~/rescue
python3 -u rescue_bot.py --selftest      # maths only
python3 -u rescue_bot.py --view          # camera + detection, motors untouched -> http://pi.local:8090
python3 -u rescue_bot.py --sonar-cal     # book 30 cm straight ahead; servo only, no motor power needed
python3 -u rescue_bot.py --survey        # pivots right through 4 views and prints the map + rescue order, no driving
python3 -u rescue_bot.py                 # the mission, started facing the arena
```

**Dashboard:** every mode serves http://pi.local:8090 (`dashboard.html`, read fresh on each load): live camera,
the map (fire with its safety ring, victims labelled A/B/C left to right with their rescue rank, obstacles the
sonar found, the planned route, the robot and its trail), an action log, and the reasoning behind each decision.
The map is built from the camera survey and corrected against the fire; it is not lidar SLAM.

## Calibration knobs (top of `rescue_bot.py`)

| Knob | Status |
| --- | --- |
| `COLOURS` | Sampled in this room's dim light. Hubs: H 20–33, S >= 150, V >= 70; the cardboard box nearby is duller and stays out. Retune with `--view` if the light changes |
| `HORIZON_Y`, `CAM_HEIGHT_M` | 318 px / 0.10 m: the box's floor contact and its size agree with these. Ranges are from the floor contact |
| `HFOV_DEG` | 41.4°, the OV5647 spec for this mode. Not measured |
| `SONAR_CENTER_DEG` | 128, found by hand: servo 90 looked well to the right. `python3 servo.py <deg>` moves it and reads a distance |
| `CRUISE_MPS` | 0.25 m/s, a guess. Re-localising on the fire after each leg hides most of its error |

## How the mission decides and moves

1. **Survey** from the start: turns right to 0°, −35°, −70°, −105°, measuring each turn with the camera, and places
   every red/yellow blob on the floor (bearing from the pixel column, range from where it meets the floor). Sightings
   within 0.3 m merge. This is the map; the start pose is its origin.
2. **Order:** victims sorted by distance to the fire, closest first.
3. **Visit:** a route that keeps 0.4 m from the fire (straight, or via one detour point beside it), driven as timed
   legs, then **visual homing** onto the hub nearest the map's prediction until it reaches the bottom of the image
   (~0.22 m away). The sonar stops it for anything closer than 22 cm that is nearer than the target.
4. **Re-localise:** at the victim the position is known (it is where the map says, minus 0.22 m) and the fire's
   bearing fixes the heading. Driving home, the heading is trusted and the fire's bearing + range fix the position.
5. **Return** to the start (corrected up to 3 times), face the arena, next victim. Ends at the start.
