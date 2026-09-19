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
LOW LEVEL
ESP32
Motor Driver
Motors
Encoders
Electromagnet
```

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
High-level software sends velocity commands. ESP32 converts them to motor-driver commands. Encoders return wheel motion for odometry.

## Electromagnet
High-level planner sends:
```text
ATTACH
RELEASE
```
ESP32 controls the MOSFET/relay.

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

TODO: add watchdog/failsafe behavior before autonomous physical testing.
