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

## 6a. Phase 2 measurements — full robot + both sensors

Re-measured with the real robot, differential drive, joint-state publisher, RGB + depth cameras (640×480, 30 Hz requested) and the ROS bridge, on the D3D12/NVIDIA path. Supersedes the two-sensor probe in §4 for planning purposes. Single samples, ±2–3 Hz.

| Topic | Headless | GUI + RViz also running |
| --- | --- | --- |
| `/camera/image_raw` | 23–26 Hz | 26 Hz |
| `/camera/depth/image_raw` | 22–25 Hz | 17 Hz |
| `/odom` | 50 Hz | 50 Hz |
| `/joint_states` | 250 Hz | 250 Hz |
| Real-time factor | 1.00 | 1.00 |

- **Depth is the stream that suffers under load.** With the Gazebo GUI and RViz rendering alongside, depth dropped to ~17 Hz in the empty world; in the Phase 3 arena (GUI + RViz) it measured ~22.6 Hz with RGB ~22 Hz — run-to-run variation is real, so treat 17–25 Hz as the range. Watch it in Phase 4 when SLAM consumes it via `depthimage_to_laserscan`. Use `gui:=false` for automated runs.
- **RGB and depth are unsynchronised:** ~40 ms constant content offset, ~10 px at 0.4 rad/s even with matching timestamps (Phase 3). Depth is exactly time-aligned to the true pose (Phase 4), so it is the RGB image that is late. See CLAUDE.md / Implementation_Plan.md.
- **Physics step is 4 ms, not 1 ms.** The stock Gazebo joint-state publisher has no rate limit; at 1 ms it emitted 1 kHz and the ROS bridge burned ~40% of a core forwarding it. At 4 ms: 250 Hz, bridge ~20%.
- Headless CPU with the robot idle (one `top` sample): gz server ~73–100%, bridges ~18% + ~9%, `robot_state_publisher` ~10%.
- **No new dependencies** were needed for Phase 2. `teleop_twist_keyboard` is installed.

## 6b. Phase 4 measurements — full navigation stack

Arena + 87° RGB and depth cameras + `depthimage_to_laserscan` + SLAM Toolbox + Nav2 (controller, planner, behaviours, BT navigator), headless, D3D12/NVIDIA. Measured with `tests/sim/experiments/nav_performance.py` (single ~15–20 s windows; SLAM run in the odometry-only preset, whose cost is the same order as scan-matching's ~0.05 core when gated).

| | Idle (robot stationary) | Nav2 driving a detour goal |
| --- | --- | --- |
| `/camera/depth/image_raw` | 15.2 Hz | 18.2 Hz |
| `/scan` | 16.0 Hz | 18.5 Hz |
| `/camera/image_raw` | 27.2 Hz | 25.7 Hz |
| `/odom`, `/joint_states` | 50 / 250 Hz | 50 / 249 Hz |
| `/map` update | every 1.0 s | every 1.0 s |
| Real-time factor | 1.00 | 1.00 |
| CPU, whole stack | 1.70 cores | 1.67 cores |

CPU by process (cores): Gazebo server 0.90, ros_gz bridges 0.30, depth→scan 0.05, SLAM Toolbox 0.05–0.14, controller/planner/BT/behaviour servers 0.05–0.10 each, robot_state_publisher 0.05–0.07.

- **`/scan` is depth-limited.** The scan is one message per depth frame, so it runs at the depth rate (15–18 Hz here versus 22–25 Hz for the 60° camera with less else running). Nothing in Phase 4 needs more; `test_scan_rate` requires ≥ 15 Hz.
- **No new dependencies.** SLAM Toolbox 2.8.5, Nav2 1.3.13 and `depthimage_to_laserscan` 2.5.1 were already installed; `rtabmap_slam` remains absent and unneeded.
- **RViz** (`fire_resq_navigation/rviz/navigation.rviz`) starts cleanly (one instance; standalone under D3D12, llvmpipe and forced-software GL: 0 errors), every displayed topic carries data in the right frame (`/scan` in `camera_link`; `/map` and the global costmap in `map`; the local costmap and footprint in `odom`), `/plan` carries the path, and a goal published on `/goal_pose` — what the "2D Goal Pose" tool sends — drives the robot. **Not verified visually:** once the Map display is created RViz logs `[ERROR] rviz/glsl120/indexed_8bit_image ... GLSL link result :` (empty). Screen capture is unavailable under WSLg (`X get_image` fails), so whether the map layer renders could not be confirmed by eye. If the map layer is blank, try `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch ... use_rviz:=true`. (An earlier double-RViz launch-argument leak was a separate bug and is fixed and guarded by a unit test.)
- **Nav2 and SLAM need the sim clock.** Everything in the stack runs with `use_sim_time:=true`; `arena_nav.launch.py` sets it.

## 6c. Phase 5 measurements — perception

Arena + RGB and depth cameras + `perception_node`, headless, D3D12/NVIDIA; one simulator and one node running (measured with a per-thread CPU script, after an earlier reading was inflated by a leftover second node).

| | Measured |
| --- | --- |
| `/fire_resq/detections` rate | 22–25 Hz (one message per RGB frame, so it follows the camera) |
| Detection latency (message stamp vs receipt, sim time) | < 0.25 s median (asserted); the stamp is the image's own |
| Processing per RGB frame | 5.6–5.9 ms average inside the node (detector 3–6 ms, depth estimator < 1 ms, conversions < 1 ms) |
| CPU, `perception_node` | **0.49 cores** (main thread 0.24–0.27, TF listener thread ~0.03, the rest DDS) |

- **Why it is single-threaded.** The first version ran under a `MultiThreadedExecutor` with the TF listener on the node: **1.24 cores** and 24.7 ms per frame, almost all of it rclpy executor wait-set bookkeeping (`profile`: `Waitable.__add__` 1.5 M calls in 50 s), not the vision work. A single-threaded node with the listener on its own node and thread costs 0.49 cores.
- **OpenCV uses 16 threads by default**, which flatters per-call time (3.4 ms detector vs 5.9 ms on one thread). Nothing sets `cv2.setNumThreads`; if perception ever competes with SLAM/Nav2 for cores, that is a knob.
- **No new dependencies:** OpenCV 4.6 and numpy 1.26 were already installed; no `cv_bridge` (image conversion is done directly, so no ABI coupling to NumPy 2).
- The accuracy numbers are in [Implementation_Plan.md](Implementation_Plan.md) Phase 5; `tests/sim/experiments/perception_accuracy.py` reproduces them.

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
