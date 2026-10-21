# The real robot (Raspberry Pi 3B + Arduino Uno)

The physical robot is **not** the ESP32 / 2WD / encoder / RealSense design in [docs/Hardware.md](../docs/Hardware.md).
What is actually built (October 2026):

| Part | What |
| --- | --- |
| Computer | Raspberry Pi 3B (905 MB RAM, Debian 13, no ROS), `ssh pi@pi.local` |
| Motor controller | Arduino Uno + L293D shield (Adafruit Motor Shield v1), USB serial `/dev/ttyUSB0`, 115200 baud |
| Drive | 4 TT motors, **no encoders**. M1 front-left, M4 front-right, M2 back-left, M3 back-right. One motor is slower |
| Power | Motors: 8×AA as two 4-cell packs **in parallel** (6 V) through the motor switch to EXT_PWR, PWR jumper OFF. Pi: its own power bank |
| Camera | OV5647 ribbon camera (Picamera2), **mounted rotated 90° clockwise**: every frame is rotated 90° CCW first |
| Range | HC-SR04 sonar on a servo (SERVO 1 = D10), TRIG A3, ECHO A4; straight ahead is servo **128°** |

## The task

The **fire** is the tan cardboard box; the **victims** are wheels lying flat on the floor. Everything is placed in
front of the robot, in view from its start spot, so it never scans by turning. Each round the robot:

1. looks (it stays put; the sonar servo, already centred, sweeps left and right),
2. finds the fire and every victim, and picks the victim **closest to the fire**,
3. drives to it and touches it, on a route that keeps clear of the box,
4. reverses back to the start spot,
5. waits for you to lift that victim away, then looks again and takes the next closest one.

It stops when no victim is left in view. Nothing about the layout is in the code: every round decides afresh, so the
wheels can be anywhere it can see.

## Run it

```bash
scp robot/*.py robot/dashboard.html robot/wander_fw.hex pi@pi.local:~/rescue/
ssh pi@pi.local
cd ~/rescue
sudo systemctl stop targetfire                   # the drone project starts at boot and holds the camera
python3 -u rescue_bot.py --selftest              # the maths and the detector, no hardware (~1 s)
python3 -u rescue_bot.py --plan                  # look and decide, never moves: check it picked the right one
python3 -u rescue_bot.py                         # THE MISSION - start it with the MOTOR SWITCH OFF
```

**When to switch the motors on:** start the mission with the motor switch **off**. The robot centres the servo,
looks around, finds the fire and victims and decides, while the wheels stay still. Then the dashboard (and the
terminal) say **"READY: switch the MOTOR battery ON now - moving in 15 s"**. Switch it on then. If the motors stay
off, the first turn finds the robot not moving and the mission stops itself.

Watch it on **http://pi.local:8090**: the live camera (top left), the map with the start point, the fire's base edge,
the victims with their rank, the route and the robot's trail (top right), what it is doing (bottom left) and why
(bottom right).

## How it sees

In this room's light, colour alone cannot separate things: the box, the wheels' yellow, a chair and the curtain all
read as dull tan/olive. What stands out is the bright floor (V 160–180), so:

- **Victim** = a wheel lying flat: a thin pale-yellow rim directly on top of a near-black tyre band, which ends on
  bright floor. Its real width must be wheel-sized. Fallback: a small dark puck sitting on the floor.
- **Fire** = the largest tan blob whose base stands on the floor. Only its base counts (a chair behind the box merges
  into the blob higher up).

**Depth.** The camera is 10 cm above a flat floor, looking level, so the horizon is a fixed image row. Where an object
meets the floor *N* pixels below the horizon it is `0.10 × focal length / N` metres away; the pixel column gives its
bearing. A pixel row is ~2 cm at 1 m, ~9 cm at 2 m.

**Closest to the fire** means the distance to the box's **base edge**, a 40 cm segment on the floor. A wheel by a
corner of the box is close to the fire even though it is far from the box's centre. Each victim's distance is
measured inside each photo, against the fire in the same photo, so the robot's own position errors cancel; the
median over 6 photos is used. An object must appear in most photos to count.

`--calibrate-horizon 1.0` (one wheel at a measured 1.00 m, lens to its near edge) measures the camera's tilt, the
biggest depth error, into `calibration.json` (kept out of git: it belongs to one camera mount).

## Moving without encoders

Measured on the tile floor, 1 s each (2026-10-21): a skid spin and an arc (one side full, the other half) barely turn
the robot (an arc: 1.7°); a **pivot**, where one side drives and the other stops, turns it 13–19°. So:

- **Turns are pivots.** A forward pivot then a backward pivot of the other side: both rotate the same way and
  their creep cancels, so the robot turns roughly on the spot.
- **Heading comes from the camera.** It adds up the image shift between consecutive frames; measured, it tracks
  the pivots while they happen.
- **Straight legs hold that heading.** This also cancels the slow motor's pull. If the log keeps printing
  `steering bias`, tune `WEAK_MOTOR` / `WEAK_RATIO`.
- **Homing is stop-and-go.** Look at a still picture, turn to face the victim if needed, drive a hop of at most
  20 cm, repeat. In this light a moving camera blurs the thin yellow rim away. When the victim is right in front
  (~22 cm), the robot creeps `TOUCH_CREEP_M` (17 cm) blind to touch it.
- **The way home is driven in reverse along the same route,** so a wrong `CRUISE_MPS` cancels out. Then the fire,
  seen from the start, re-anchors position and heading.
- **Safety stops.** Forward legs stop for a sonar obstacle (it goes on the map and the route is re-planned) or for
  the fire close ahead. The Uno's firmware releases every motor if the Pi goes quiet for 0.4 s.

## Firmware

`wander_fw.ino` (compiled to `wander_fw.hex`): `M a b c d` motor powers with a 400 ms watchdog, plus `U`/`G`/`P`
servo and sonar. At power-on it releases the motors and points the servo straight ahead (128°). It never moves on
its own.

If the car drives or sweeps by itself when the motor switch goes on, the Uno has other firmware on it (a car kit's
obstacle-avoider does exactly that). Put ours back:

```bash
avrdude -p atmega328p -c arduino -P /dev/ttyUSB0 -b 115200 -U flash:w:/home/pi/rescue/wander_fw.hex:i
```

Rebuild it after editing (`arduino-cli`, AVR core, "Adafruit Motor Shield library" and Servo):
`arduino-cli compile -b arduino:avr:uno --output-dir out wander_fw`.

## Calibration knobs (top of `rescue_bot.py`)

| Knob | Status |
| --- | --- |
| `RIM_HSV`, `TAN_HSV`, `FLOOR_V`, `DARK_V` | Sampled in this room, 2026-10-21. Re-sample with `--view` if the light changes a lot |
| `HORIZON_Y` | 318 by default; `--calibrate-horizon` measures it |
| `TOUCH_CREEP_M` | 0.17 m: raise it if the robot stops short of the wheel, lower it if it pushes the wheel |
| `SONAR_CENTER_DEG` | 128 (also `SERVO_CENTER_DEG` in the firmware) |
| `WEAK_MOTOR`, `WEAK_RATIO` | M3 at 85%, a guess. Tune from the `steering bias` lines |
| `CRUISE_MPS` | 0.25 m/s, a guess. The reverse return and the fire re-anchoring make it matter little |
