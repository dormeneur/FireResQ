# FireResQ — Hardware & Integration Plan

## 1. Hardware Philosophy
The physical MVP is a **low-cost 2WD differential-drive robot** — not a 4WD platform and not an arm-equipped manipulator. Every component is chosen for cost, availability, and simplicity.

The split is deliberate: the laptop performs ROS 2, perception, cognition, planning, and simulation. The ESP32/Arduino-class controller performs low-level motor and actuator control only. Hardware stays modular so simulation drivers can be replaced by physical devices without rewriting the cognitive layer.

Keep complexity out until the MVP works. Adding hardware is a decision to be justified, not a default.

## 2. Drive System
Confirmed configuration:

- 2WD differential drive.
- Two independently driven DC geared motors, one per wheel.
- One passive caster wheel for stability (third contact point, no actuation).
- No 4WD. No steering mechanism — heading comes from the wheel-speed difference.
- Wheel encoders on both driven wheels. **Odometry is required**, so prefer motors that ship with encoders over adding them later.
- A dual-channel motor driver (TB6612FNG or equivalent) drives both motors.
- An ESP32 or equivalent microcontroller generates PWM/direction and reads the encoders.

Differential drive must remain compatible with ROS 2 Nav2: the robot consumes `geometry_msgs/Twist` on `cmd_vel` (linear `x`, angular `z`; all other components ignored) and publishes `nav_msgs/Odometry` plus the matching TF. Wheel separation and wheel radius are the only kinematic parameters the conversion needs, and they belong in a config file, not in code.

## 3. Computing
```text
Laptop / PC                      ESP32
─────────────────────────        ─────────────────────────
ROS 2 Jazzy                      Motor PWM + direction
Gazebo, RViz                     Encoder acquisition
Nav2                             Electromagnet switching
SLAM / localization              Low-level hardware interface
OpenCV + YOLO-family detector
World model
Cognitive decision layer
Planning
```

The ESP32 is an implementation detail behind ROS interfaces. Replacing it with another microcontroller — or with a different odometry source entirely — must not require changes above the control layer.

## 4. Vision
The software supports **either** backend, and the choice is not baked into cognition:

| Backend | Provides | Spatial estimation |
| --- | --- | --- |
| RGB USB camera | Colour image | Requires an alternative method — calibration plus scene/ground-plane geometry, known victim size, or marker-assisted estimation |
| RGB-D / Intel RealSense | Colour + depth image | More direct and more reliable per-detection range |

The camera detects victims, fire, and obstacles/environmental features. Depth, **when available**, improves spatial estimation; the RGB-only path must not assume depth exists, and must degrade to an explicit alternative rather than silently producing bad estimates.

The perception interface hides which backend is active. Cognition must not be able to tell.

## 5. Localization & Mapping
- **No LiDAR is a required component.** It is not in the MVP list and nothing may depend on it.
- SLAM remains a core project capability.
- The available sensor configuration determines the practical SLAM/localization approach — this is a hardware-dependent choice, not a fixed one. A depth camera can feed a laser-scan-based SLAM package via a depth-to-scan conversion; RGB-only sensing points toward visual SLAM or marker-assisted localization instead. Pick after the camera is confirmed.
- Wheel encoder odometry is required in every configuration.

TF2 frame chain:
```text
map → odom → base_link → camera frame(s)
```
`map → odom` is published by the SLAM/localization node, `odom → base_link` by the odometry node fed from encoders, and `base_link → camera frame(s)` from the robot description. No cognitive code hardcodes coordinates.

## 6. Rescue Mechanism
A low-voltage electromagnet mounted at the **front** of the robot. Victim models carry a ferromagnetic/metal attachment point. Switching is electronic, through a MOSFET or suitable relay/driver module — never a bare GPIO pin, and with a flyback path across the coil because the load is inductive.

Actuator-level sequence:
```text
APPROACH → ALIGN → ELECTROMAGNET ON → ATTACH → TRANSPORT → ELECTROMAGNET OFF → RELEASE
```
This is the hardware expansion of the planner's PICK UP / RETURN / RELEASE steps, not a replacement for the task sequence in [Architecture.md](Architecture.md) §3.

Simple and lightweight by design: **no robotic arm is required for the MVP**. Mounting height and front overhang must suit the victim attachment point — see the TODO list.

## 7. Power
Four loads with different characteristics. Document and budget them separately.

| Load | Character | Notes |
| --- | --- | --- |
| Motors | Highest draw; spikes at stall/direction change | Dominates battery sizing; a noisy rail — keep it off the logic supply |
| ESP32 / control electronics | Small, continuous | Needs a clean regulated rail to avoid brownout resets during motor transients |
| Camera | Typically none from the robot | RGB USB and RealSense are normally bus-powered by the laptop, not the robot battery |
| Electromagnet | Moderate, intermittent, inductive | Only drawn while holding a victim; switching transients need suppression |

Rules rather than part numbers:
- Motors and logic get separate supply rails (or a regulator sized for the worst-case motor transient), with a **common ground**.
- **No specific battery model is fixed.** Select the battery and regulator from measured motor stall/running current, electromagnet holding current, electronics draw, and the target runtime — after the chassis and electromagnet are chosen, not before.

## 8. Hardware–Software Boundary
```text
HIGH LEVEL  (laptop)
Perception · World Model · Cognition · Planning
        ↓
ROS 2 interfaces
  cmd_vel
  odometry
  TF
  camera image stream
  camera depth stream (when available)
  electromagnet ATTACH / RELEASE / state
  hardware health / state where practical
        ↓
LOW LEVEL   (ESP32)
Motor driver · Motors · Encoders · Electromagnet
```

Hardware-specific GPIO logic belongs in the hardware/control layer. Cognitive and planning layers must not directly control GPIO.

## 9. Data Flow
```text
Camera / RealSense
       ↓
     Laptop
      ROS 2
       ↓
     ESP32
   ┌───┴────┐
Motors    Encoders

ESP32 → MOSFET/Relay → Electromagnet
```

## 10. Component List

### Required for MVP
| # | Component | Purpose |
| --- | --- | --- |
| 1 | 2WD robot chassis + 1 passive caster | Robot base |
| 2 | 2× DC geared motors with encoders | Drive and wheel odometry |
| 3 | Dual-channel motor driver (TB6612FNG or equivalent) | Motor control |
| 4 | ESP32 | Low-level controller |
| 5 | RGB USB camera | Visual perception |
| 6 | Low-voltage electromagnet | Victim pickup |
| 7 | MOSFET / relay driver module | Electromagnet switching |
| 8 | Battery pack | Robot power (sized per §7) |
| 9 | Voltage regulator | Stable logic rail |
| 10 | 3× lightweight victim models with metal attachment points | Rescue targets |
| 11 | Wiring, connectors, mounting hardware | Electrical and mechanical integration |
| 12 | Laptop | ROS 2 + AI processing |

### Optional / upgrade
| Component | Purpose | Status |
| --- | --- | --- |
| Intel RealSense (or other RGB-D) | Depth sensing, more direct spatial estimation | Borrowed/optional — software must work without it |
| ArUco / AprilTag markers | Localization aid; printable, near-zero cost | Optional |
| Separate encoder modules | Only if the chosen motors lack integrated encoders | Contingency |

Explicitly **not** in scope: LiDAR, robotic arms, industrial sensors, and any actuator beyond the two drive motors and the electromagnet.

## 11. Build Order
### Phase 1 — Base
Assemble chassis, two motors and wheels, caster, motor driver, ESP32, battery/regulator. Verify manual movement in both directions and on-the-spot rotation.

### Phase 2 — Odometry
Wire encoders. Verify counts, direction sign, computed odometry, `/odom`, and the `odom → base_link` TF.

### Phase 3 — Camera
Mount a fixed RGB or RGB-D camera; calibrate it and publish the `base_link → camera` transform.

### Phase 4 — Rescue Mechanism
Mount the electromagnet at the front. Connect ESP32 → MOSFET/relay → electromagnet. Test attach and release against a victim model.

### Phase 5 — Navigation
Integrate odometry and camera/depth data with the chosen SLAM approach and Nav2. Validate mapping and autonomous navigation.

### Phase 6 — Full Integration
```text
Perceive
→ Map/Localize
→ Update World Model
→ Prioritize
→ Navigate
→ Pickup
→ Return
→ Release
→ Reassess
```

## 12. Simulation Equivalents
Every physical component has a simulated counterpart, and **simulation never depends on the hardware being present**.

| Physical | Simulated |
| --- | --- |
| 2WD differential-drive chassis + caster | Differential-drive robot model with a caster link |
| Wheel encoders | Joint-state-driven odometry publisher |
| RGB camera | Gazebo camera sensor |
| RGB-D camera (optional) | Gazebo depth/RGBD camera sensor, enabled by config |
| Electromagnet attach/release | Detachable-joint style attach/release on command |
| 3× metal victims | Victim models with an attachment link |
| Fire source | Fire model / visual marker |
| Obstacles, walls | Static arena geometry |
| 5 m × 5 m arena | Gazebo rescue world |
| Ground-truth pose (sim only) | Replaced on hardware by SLAM + encoder odometry/TF |

## 13. Decision Status

### Confirmed project decisions
- 2WD differential drive, two driven wheels plus one passive caster.
- Encoder-based wheel odometry is required.
- ESP32-class microcontroller for motors, encoders, and electromagnet.
- Electromagnet pickup at the front; no arm in the MVP.
- No required LiDAR; SLAM stays a core capability.
- Laptop holds all perception, cognition, planning; ESP32 holds no autonomy.
- Camera-agnostic perception: RGB and RGB-D are both valid.

### Hardware-dependent (decide when parts are in hand)
- Specific SLAM/localization approach, which follows from the final camera choice.
- Battery and regulator ratings, from measured currents.
- Electromagnet voltage/holding force, from victim mass and attachment area.
- Encoder resolution and the resulting odometry tuning.
- Camera mounting height, tilt, and FOV.

### Future upgrades (not MVP)
- RGB-D upgrade if a RealSense becomes available.
- Marker-assisted localization if visual odometry proves unreliable.
- Battery/resource awareness in the decision layer.

## 14. Hardware TODOs
- TODO: real camera integration (driver, calibration, `camera_info`).
- TODO: RealSense integration and validation of the RGB-D backend.
- TODO: ESP32 ↔ ROS 2 communication (transport, message framing, reconnection).
- TODO: encoder calibration (ticks/rev, wheel radius, wheel separation).
- TODO: electromagnet hardware interface (switching circuit, flyback, ATTACH/RELEASE + state topic).
- TODO: physical robot calibration (odometry drift, straight-line and rotation error).
- TODO: hardware safety/watchdog and failsafe — **required before autonomous physical testing**.
- TODO: exact chassis dimensions and mass budget.
- TODO: motor voltage/current and stall current measurement.
- TODO: Nav2 tuning on the physical robot.
