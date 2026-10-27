# FireResQ — Hardware Integration

## Goal
Replace simulation drivers with physical hardware while keeping perception, cognition, planning, and high-level ROS 2 behavior unchanged.

## Interface Boundary
```text
HIGH LEVEL
Perception
World Model
Cognition
Planning
      ↓
ROS 2 interfaces
      ↓
DRIVERS (Raspberry Pi 3B on the robot: fire_resq_hardware)
camera_node · base_node
      ↓ serial
LOW LEVEL
Arduino Uno + L293D motor shield
4 motors
Electromagnet (not fitted yet)
```

The robot as built has **no encoders and no depth camera** ([Hardware.md](Hardware.md)). The interfaces below are unchanged; what feeds them is not what this page originally assumed.

## Camera Options
RGB camera:
- RGB image.
- Visual detections.
- Spatial estimates through calibration/scene geometry when needed.

RGB-D / RealSense:
- RGB image.
- Depth image.
- More direct spatial estimation.

The perception interface hides this difference.

## Motor Interface
High-level software sends velocity commands. `base_node` on the Pi converts them to a power per motor; the Arduino applies it. There are no encoders: `/odom` is dead reckoning from the power the firmware reports applying, and is not a position source.

## Electromagnet
High-level planner sends:
```text
ATTACH
RELEASE
```
The Arduino will switch the MOSFET/relay (free pins: D2, D13, A0–A5). No actuator is fitted and the driver node is not written.

## Integration Rule
Hardware-specific GPIO logic belongs in the hardware/control layer, not cognition.

## Physical Validation
1. Motors.
2. Encoders.
3. Odometry.
4. Camera.
5. Electromagnet.
6. SLAM.
7. Nav2.
8. Pickup/release.
9. Full rescue sequence.

Watchdog/failsafe: in place and measured ([Hardware.md](Hardware.md) §8): `/cmd_vel` timeout in the driver, a 400 ms command watchdog in the firmware, and a firmware identity check before any drive byte is sent.

TODO: a stall check (dead-reckoned odometry cannot see a blocked robot).
