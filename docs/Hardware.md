# FireResQ — Hardware & Integration Plan

> **Revised 2026-10-13 for the robot as built.** The original plan (2WD + caster, ESP32, wheel encoders, a depth camera, laptop-only compute) was written before any parts existed. The physical robot is different, and this document now describes that robot. Where the build leaves a planned capability without hardware, it is listed under [§13 Open decisions](#13-decision-status) rather than papered over. The simulated robot ([§12](#12-simulation-equivalents)) is unchanged.

## 1. Hardware Philosophy
The physical robot is a **low-cost 4WD skid-steer base** carrying its own small computer. Every component is chosen for cost, availability, and simplicity.

The split is deliberate and has three tiers: the **PC** performs the heavy ROS 2 work (perception, world model, cognition, planning, navigation, visualisation, simulation); the **Raspberry Pi** on the robot runs the device drivers and whatever else it can afford; the **Arduino** performs motor control only. Hardware stays modular so simulation drivers can be replaced by physical devices without rewriting the cognitive layer.

Keep complexity out until the MVP works. Adding hardware is a decision to be justified, not a default.

## 2. Drive System
As built (verified from the firmware that was on the board and from the maintainer):

- **4WD skid steer**: four DC gear motors, one per wheel, no caster, no steering mechanism. Heading comes from the left/right speed difference, so every turn is wheel slip.
- **Adafruit-style L293D motor shield (v1, `AFMotor`)** on an **Arduino Uno** (CH340 clone). Shield terminals **M1, M2 = front motors; M3, M4 = back motors**. Which of each pair is left, and each motor's polarity, are found with `motor_check` and recorded in [`config/robot.yaml`](../src/fire_resq_hardware/config/robot.yaml) — never in firmware.
- **No wheel encoders.** Nothing on the robot measures wheel motion.
- Motors are powered from **4× AA** through the shield. The L293D drops roughly 1.5–2.5 V, so the motors see well under the pack voltage.

The robot still consumes `geometry_msgs/Twist` on `cmd_vel` (linear `x`, angular `z`) and publishes `nav_msgs/Odometry` plus the matching TF, so it remains interface-compatible with Nav2. **But that odometry is dead reckoning from commanded motor power** ([`drive.py`](../src/fire_resq_hardware/fire_resq_hardware/drive.py)): it cannot see slip, stall, battery sag or a blocked robot. It exists so the stack has an `odom` frame and knows when the robot is being told to move. It is not a position source.

## 3. Computing
```text
PC ("station")                Raspberry Pi 3B ("robot")       Arduino Uno + L293D shield
──────────────────────────    ─────────────────────────────   ──────────────────────────
ROS 2 Jazzy (Docker)          ROS 2 Jazzy (Docker)            base_controller firmware
image decoding                camera_node  (USB camera)       per-motor PWM + direction
perception                    base_node    (serial bridge,    400 ms command watchdog
world model, cognition          open-loop odometry)           identifies itself on request
planning (rescue FSM)
Nav2, localisation/mapping
RViz, Gazebo, tests
        └────── ROS 2 over Wi-Fi (domain 42) ──────┘└──── USB serial, 115200 ────┘
```

- **Both computers run the same ROS 2 Jazzy binaries in Docker.** Neither host OS has Jazzy packages (the PC is Ubuntu 26.04, the Pi is Raspberry Pi OS / Debian 13). Images and scripts: [`docker/`](../docker).
- **The Pi 3B has 905 MB of RAM and four slow cores.** It runs the drivers today. Moving more onto it (colour detection is the obvious candidate) is a measured decision, not a default: an `rclpy` node costs tens of MB and a meaningful share of a core there.
- **The real robot is on ROS domain 42; simulation stays on domain 0, localhost-only.** A simulated `/cmd_vel` can therefore never turn a real wheel.
- The Arduino is an implementation detail behind ROS interfaces. Replacing it must not require changes above `fire_resq_hardware`.

## 4. Vision
A **Logitech C270** USB webcam, RGB only, plugged into the Pi. Its native MJPEG frames are published untouched (`/camera/image_raw/compressed`, ~22 kB per 640×480 frame, 0.2 % of one Pi core) and decoded on the PC into the `/camera/image_raw` that perception reads in simulation too.

| Property | Value | Status |
| --- | --- | --- |
| Horizontal field of view | ≈ 45° | **Nominal**, from the datasheet's 55° diagonal. Not calibrated |
| Frame rate | 15 fps requested | **Measured 7.6 fps in a dim room**: the camera halves its rate to lengthen exposure |
| Depth | none | Spatial estimates come from the known-height estimator (`spatial_backend:=known_height`) |

45° is about half the 87° the simulated navigation stack was tuned with, and there is no depth image at all. The perception interface still hides the backend; cognition cannot tell.

## 5. Localization & Mapping
- **No LiDAR, no depth camera, no encoders.** The simulated pipeline (depth → `/scan` → SLAM Toolbox / AMCL → Nav2) has **no sensor input on this robot**: there is nothing to make a scan from and no measured odometry to seed a matcher.
- SLAM remains a core project capability, so a localisation and mapping source must be chosen for this hardware. It is the main [open decision](#13-decision-status). The plan anticipated it: "RGB-only realistically forces RTAB-Map or markers" ([Implementation_Plan.md](Implementation_Plan.md) §16).
- Whatever is chosen publishes `map → odom` and leaves the layers above untouched.

TF2 frame chain:
```text
map → odom → base_link → camera frame(s)
```
`odom → base_link` is published by `base_node` (dead reckoning), `base_link → camera frame(s)` from the robot description. **Nothing publishes `map → odom` yet.** No cognitive code hardcodes coordinates.

## 6. Rescue Mechanism
**Not fitted.** The design is unchanged — a low-voltage electromagnet at the front, switched through a MOSFET or relay module with a flyback diode, never a bare GPIO pin — and the software side is ready: `magnet_node` speaks only `/fire_resq/magnet/energize` and `/fire_resq/magnet/state`, so the hardware driver is one new node in `fire_resq_hardware`.

Free pins for it on the Arduino (the shield uses D3–D8, D11, D12): **D2, D13, A0–A5**. The shield's two servo headers are D9 and D10.

Actuator-level sequence:
```text
APPROACH → ALIGN → ELECTROMAGNET ON → ATTACH → TRANSPORT → ELECTROMAGNET OFF → RELEASE
```

## 7. Power
| Load | Supply | Notes |
| --- | --- | --- |
| Raspberry Pi 3B, USB camera, Arduino logic | USB power bank | **The Pi reported under-voltage and throttling** (`vcgencmd get_throttled` = `0x50000`) within 15 minutes of boot, with nothing but the desktop running. Suspect the cable or the bank's current limit before anything else |
| 4 motors | 4× AA via the motor shield | Separate from the logic supply, as it should be; grounds are common through the shield |
| Electromagnet | — | Not fitted. Budget its holding current before choosing its supply |

An under-volted Pi corrupts SD cards and resets under load. Fix this before long autonomous runs.

## 8. Hardware–Software Boundary
```text
HIGH LEVEL  (PC)
Perception · World Model · Cognition · Planning · Navigation
        ↓
ROS 2 interfaces (Wi-Fi)
  cmd_vel
  odometry (dead reckoning) + TF
  camera image stream (compressed) + camera info
  electromagnet ATTACH / RELEASE / state      (driver not written: no actuator yet)
        ↓
DRIVERS     (Raspberry Pi: fire_resq_hardware)
camera_node · base_node
        ↓  serial
LOW LEVEL   (Arduino Uno)
Motor shield · 4 motors
```

Device access lives in `fire_resq_hardware` and nowhere else (unit-guarded). Cognitive and planning layers must not touch a serial port, a camera or GPIO.

### Safety: three independent stops
Required before autonomous physical testing, and in place:

1. `base_node` zeroes its command when `/cmd_vel` is silent for 0.5 s (a crashed planner, a dropped Wi-Fi link).
2. The firmware releases all motors when no valid command arrives for 400 ms (a crashed driver, a hung or unplugged Pi). Measured on the board: motors released 0.35–0.45 s after the last command.
3. `base_node` sends nothing at all unless the board identifies itself as the FireResQ base firmware. The same Uno has carried an arm sketch in which the byte `S` starts a servo sequence.

## 9. Data Flow
```text
USB camera ──► Pi: camera_node ──Wi-Fi──► PC: image_decoder_node ──► perception ──► world model ──► cognition ──► planning
                                                                                                                    │
4 motors ◄── Arduino ◄──serial── Pi: base_node ◄──────────────Wi-Fi────────────── /cmd_vel ◄── Nav2 / teleop ◄──────┘
```

## 10. Component List

### On the robot
| # | Component | Purpose |
| --- | --- | --- |
| 1 | 4WD chassis, 4 DC gear motors | Robot base |
| 2 | Arduino Uno (CH340) + L293D motor shield | Motor control |
| 3 | Raspberry Pi 3 Model B | On-board drivers |
| 4 | Logitech C270 USB camera | Visual perception |
| 5 | USB power bank | Pi, camera, Arduino logic |
| 6 | 4× AA battery pack | Motors |
| 7 | PC (RTX 4060) | ROS 2, perception, cognition, navigation |

### Planned but not on the robot
| Component | Needed for | Status |
| --- | --- | --- |
| Wheel encoders | Measured odometry | **Absent.** Changes the localisation approach (§5) |
| Depth camera | The simulated SLAM/Nav2 sensor pipeline | **Absent.** Same |
| Electromagnet + MOSFET module + flyback diode | Picking up victims | **Absent** (§6) |
| 3 victim models with steel attachment points; an orange "fire" | Rescue targets | To make |
| ArUco / AprilTag markers | Localisation aid; printable | Candidate for §5 |

Explicitly **not** in scope: LiDAR, robotic arms, industrial sensors.

## 11. Build Order
| Phase | What | Status |
| --- | --- | --- |
| 1 — Base | Chassis, motors, shield, Arduino, power; firmware with watchdog; `cmd_vel` → motors | Firmware and driver **done and verified on the board with the motors unpowered**. Wheel mapping and powered motion not yet checked |
| 2 — Odometry | `/odom` and `odom → base_link` | **Open-loop only** (no encoders); three calibration numbers still placeholders |
| 3 — Camera | Driver, `base_link → camera` transform, calibration | Driver **done**; intrinsics nominal; mounting position not measured |
| 4 — Rescue mechanism | Electromagnet, switching, driver node | Blocked on hardware |
| 5 — Navigation | Localisation/mapping + Nav2 on the real robot | Blocked on the §5 decision |
| 6 — Full integration | The complete rescue loop | — |

## 12. Simulation Equivalents
Every physical component has a simulated counterpart, and **simulation never depends on the hardware being present**. The simulated robot is still the originally planned one (2WD + caster, encoders, optional depth camera); it is the regression rig for everything above the drivers.

| Physical | Simulated |
| --- | --- |
| 4WD skid-steer chassis | Differential-drive robot model with a caster link (**differs**: see §13) |
| *(no encoders: dead reckoning)* | Joint-state-driven odometry publisher (noise-free) |
| RGB camera, 45° | Gazebo camera sensor, 60° / 87° |
| *(no depth camera)* | Gazebo depth sensor, enabled by config |
| Electromagnet attach/release *(not fitted)* | Detachable-joint style attach/release on command |
| 3× metal victims | Victim models with an attachment link |
| Fire source | Fire model / visual marker |
| Obstacles, walls | Static arena geometry |
| 5 m × 5 m arena | Gazebo rescue world |
| Ground-truth pose (sim only) | Replaced on hardware by a localisation source (§5) |

## 13. Decision Status

### Settled by the build
- 4WD skid steer on an Arduino Uno + L293D shield; no encoders.
- RGB-only USB camera (Logitech C270).
- Three-tier compute: PC, Raspberry Pi 3B, Arduino; ROS 2 Jazzy in Docker on both computers.
- Camera-agnostic perception, no LiDAR, electromagnet pickup with no arm: unchanged from the original plan.

### Open decisions
- **Localisation and mapping source** for a robot with one narrow RGB camera and no measured odometry (§5). Until it is made, nothing above the drivers can run in `map`.
- **Pickup actuator**: which electromagnet, what switches it, where it mounts (§6).
- **Whether the robot description changes to 4WD.** `parameters.xacro` still describes the 2WD simulated robot and is shared by simulation and hardware. `base_node` reads only `wheel_separation` from it; `base_link` for a skid-steer base is the centre of the four wheels, not an axle midpoint.

### Hardware-dependent (measure on the robot)
- Which shield terminal is which wheel, and motor polarity (`motor_check`).
- `full_speed_mps`, `min_power`, `track_slip_factor` in `robot.yaml`.
- Camera calibration; camera position, height and tilt on the chassis.
- Electromagnet voltage and holding force, from victim mass and attachment area.

## 14. Hardware TODOs
- TODO: fix the Pi's power supply (under-voltage, §7).
- TODO: run `motor_check`; confirm the wheel mapping and polarity with the motors powered.
- TODO: measure the three drive-calibration numbers; check whether the motors can turn the robot on the spot at all on the arena floor (skid steer on 4× AA through an L293D is marginal).
- TODO: camera calibration (checkerboard), mounting measurements into `parameters.xacro`.
- TODO: colour ranges and object-height priors for the real victims and fire ([`perception.yaml`](../src/fire_resq_perception/config/perception.yaml)).
- TODO: localisation source (§5), then Nav2 tuning on the physical robot.
- TODO: electromagnet hardware and its driver node (switching circuit, flyback, ATTACH/RELEASE + state topic).
- TODO: a stall check. Dead-reckoned odometry reports motion from a robot that is stuck against a wall.
- TODO: decide what else, if anything, moves from the PC to the Pi, from measured headroom.
