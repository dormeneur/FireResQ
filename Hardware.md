# FireResQ — Hardware & Integration Plan

## 1. Hardware Philosophy
The robot is inexpensive and modular. The laptop performs ROS 2, perception, cognition, simulation, and higher-level planning. The ESP32/Arduino-class controller performs low-level motor and actuator control.

## 2. Components
1. 2WD/4WD robot chassis — Robot base
2. 2× DC geared motors — Drive movement
3. TB6612FNG or equivalent — Motor control
4. ESP32 — Low-level controller
5. 2× wheel encoders — Wheel odometry
6. RGB USB camera — Visual perception
7. Intel RealSense — Depth sensing (borrowed/optional)
8. Electromagnet — Victim pickup
9. MOSFET/relay module — Electromagnet switching
10. Battery pack — Robot power
11. Voltage regulator — Stable power
12. 3× lightweight metal victim models — Rescue targets
13. ArUco/AprilTag markers — Localization aid
14. Wiring/connectors — Electrical integration
15. Laptop — ROS 2 + AI processing

LiDAR is not required for the baseline.

## 3. Data Flow
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

## 4. Hardware Interfaces
Required ROS capabilities:
- Velocity commands.
- Wheel odometry.
- TF transforms.
- Camera image/depth streams.
- Electromagnet command/state.
- Hardware health/state where practical.

Cognitive/planning layers must not directly control GPIO.

## 5. Build Order
### Phase 1 — Base
Assemble chassis, motors, wheels, motor driver, ESP32, battery/regulator. Verify manual movement.

### Phase 2 — Odometry
Install encoders. Verify counts, direction, odometry, `/odom`, and TF.

### Phase 3 — Camera
Mount a fixed RGB or RealSense camera and configure/calibrate it.

### Phase 4 — Rescue Mechanism
Mount the electromagnet. Connect ESP32 → MOSFET/Relay → electromagnet. Test pickup and release.

### Phase 5 — Navigation
Integrate odometry and camera/depth data with SLAM/Nav2. Validate mapping and autonomous navigation.

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

## 6. Simulation-to-Hardware Mapping
- Gazebo camera → RGB/RealSense driver.
- Simulated depth → RealSense depth if available.
- Simulated drive → ESP32 + motor driver.
- Simulated odometry → wheel encoders.
- Simulated electromagnet → physical electromagnet.
- Gazebo pose → SLAM + odometry/TF.
- Simulated victims → physical metal models.

## 7. Hardware TODOs
- TODO: exact chassis dimensions.
- TODO: motor voltage/current.
- TODO: encoder interface.
- TODO: battery/regulator ratings.
- TODO: electromagnet holding force.
- TODO: RealSense model/driver if borrowed.
- TODO: camera mounting/FOV.
- TODO: wheel-odometry tuning.
- TODO: Nav2 tuning.
- TODO: physical watchdog/failsafe.
