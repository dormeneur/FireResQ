# FireResQ — Verified Environment

Phase 0 record. Everything below was **measured on this machine**, not assumed. Re-run the checks in §7 if the machine, GPU driver, or WSL version changes.

Verified: 2026-09-19.

## 1. Platform

| Item | Value |
| --- | --- |
| OS | Ubuntu 24.04.5 LTS |
| Kernel | 6.18.33.1-microsoft-standard-WSL2 (WSL2) |
| CPU cores | 16 |
| GPU | NVIDIA GeForce RTX 3050 6GB Laptop GPU |
| NVIDIA driver | 610.62 |
| WSL GPU paravirt | `/dev/dxg` present; `/usr/lib/wsl/lib` provides `libd3d12.so`, `libcuda.so` |
| Display | WSLg — `DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0` |

## 2. Software versions

| Component | Version |
| --- | --- |
| ROS 2 | Jazzy (`/opt/ros/jazzy`) |
| Gazebo | Harmonic — `gz sim` 8.15.0 |
| Mesa | 25.2.8 |
| Python | 3.12.3 |
| pytest | 7.4.4 |
| OpenCV (`cv2`) | 4.6.0 |
| colcon | colcon-core 0.21.3 |

### Relevant ROS 2 packages — present
`slam_toolbox` · `depthimage_to_laserscan` · `nav2_bringup` · `nav2_simple_commander` · `ros_gz_sim` · `ros_gz_bridge` · `ros_gz_image` · `robot_state_publisher` · `joint_state_publisher` · `xacro` · `cv_bridge` · `vision_msgs` · `image_transport` · `robot_localization`

### Gazebo system plugins — present
`gz-sim8-diff-drive-system` · `gz-sim8-detachable-joint-system` (supports both `attach_topic` and `detach_topic`) · `gz-sim8-sensors-system` · `gz-sim8-joint-state-publisher-system` · `gz-sim8-pose-publisher-system`

### Absent, and not required for Phases 1–11
`ros2_control` / `gz_ros2_control` (the Gazebo `diff-drive` plugin covers the MVP) · `rtabmap_slam` (visual-SLAM fallback) · `ultralytics` (MVP detector is OpenCV-based) · `micro_ros_agent` (Phase 12) · `twist_mux` · `camera_info_manager`

## 3. Rendering status — important

**The default OpenGL path is software rasterization.** Despite the RTX 3050 and working `/dev/dxg`, stock `glxinfo` reports:

```text
Device: llvmpipe (LLVM 20.1.2, 256 bits)      Accelerated: no
```

Hardware acceleration is available through Mesa's D3D12 Gallium driver, but **only when explicitly requested**. Without the adapter hint it selects the Intel iGPU, not the discrete GPU:

```bash
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
# → Device: D3D12 (NVIDIA ...), Accelerated: yes, 6001 MB dedicated
```

Both paths work for this project. The accelerated path is preferred mainly for **CPU headroom**, not frame rate — see §4.

## 4. Measured sensor performance

Test rig: throwaway world, one `camera` and one `rgbd_camera`, both 640×480 at a requested 30 Hz, headless server (`gz sim -s -r`), rates read with `ros2 topic hz` through `ros_gz_bridge`.

| Bridged topic | Software (llvmpipe) | Accelerated (D3D12/NVIDIA) |
| --- | --- | --- |
| `/probe/rgb` | 26.4 Hz | 22.7 Hz |
| `/probe/rgbd/image` | 28.3 Hz | 28.0 Hz |
| `/probe/rgbd/depth_image` | **15.6 Hz** | **22.1 Hz** |
| `/probe/rgbd/points` | 24.7 Hz | 22.7 Hz |
| Real-time factor | 1.00 | 1.00 |
| CPU (user) | 13.6 % | **3.6 %** |

Single ~18 s samples, so treat individual figures as ±2–3 Hz. Two results are robust:

- **Real-time factor holds at 1.00 on both paths.** Simulation is not the bottleneck.
- **The accelerated path cuts user CPU from 13.6 % to 3.6 %.** That reclaimed CPU is what perception, SLAM, and Nav2 will compete for from Phase 4 onward, which is the real reason to use it.

Depth is the slowest stream on both paths and is the one to watch as scene complexity grows.

### Content validation
Rendered frames contain correct geometry, not blank buffers:

- RGB: 640×480 `rgb8`, correct scene (ground plane, wall, shadows).
- Depth: 640×480 `32FC1`, 1.16–9.73 m, 75 % finite — consistent with the test geometry.
- HSV colour-blob segmentation on a rendered frame returned **one clean contour per coloured target** (89×99 px and 98×96 px, single blob each). This directly supports the Phase 5 decision to start with OpenCV colour detection rather than YOLO.

### Depth → laser scan (Phase 4 keystone, tested early)
`depthimage_to_laserscan` consumes the bridged depth stream without modification:

| Property | Value |
| --- | --- |
| Rate | 17.0 Hz |
| Beams | 640 |
| Field of view | **59.9°** |
| Valid beams | 86 % |
| Range observed | 2.83 – 6.62 m (matches test geometry) |

The pipeline works. The 59.9° FOV against a LiDAR's 360° is the quantified form of the risk recorded in [Implementation_Plan.md](Implementation_Plan.md) §8 — mapping will require deliberate rotation to fill the occupancy grid.

## 5. GUI status

Both GUI tools launch and stay alive under WSLg with no errors logged:

- `gz sim -g` — alive, clean log.
- `rviz2` — alive, clean log.

GUI use is fine for development and demos. Headless (`-s`) is preferred for automated runs and Phase 11 trials.

## 6. Conclusion

**Gazebo rendering on this machine is acceptable for the project.** No blocking WSL2 limitation was found. Phase 1 can begin.

Recommended default for launch files and development shells:

```bash
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
```

Software rendering remains a working fallback if the D3D12 path breaks after a driver or WSL update — the project does not depend on GPU acceleration, it merely benefits from the CPU headroom.

### Known limitations
- No native GPU OpenGL; acceleration is D3D12-translated, so unusual Ogre2 features may behave differently than on native Linux.
- `vulkaninfo` is unavailable; the Vulkan render path was not tested.
- Measurements come from a two-sensor scene. Re-measure once the full arena, robot, SLAM, and Nav2 are running — Phase 4 is the natural checkpoint.

## 7. How to re-verify

```bash
# 1. Is OpenGL accelerated, and on which adapter?
glxinfo -B | grep -E "Device:|Accelerated:"
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA glxinfo -B | grep -E "Device:|Accelerated:"

# 2. Sensor rates — with a world containing camera/rgbd_camera sensors
gz sim -s -r -v 2 <world>.sdf &
gz topic -l                       # confirm sensor topics exist
ros2 run ros_gz_bridge parameter_bridge <topic>@sensor_msgs/msg/Image[gz.msgs.Image &
ros2 topic hz <topic>

# 3. Simulation keeping up?
gz topic -e -t /stats -n 3 | grep real_time_factor    # want ~1.0
```

The Phase 0 probe world was throwaway and was not committed; Phase 2 creates the real arena under `simulation/worlds/`.
