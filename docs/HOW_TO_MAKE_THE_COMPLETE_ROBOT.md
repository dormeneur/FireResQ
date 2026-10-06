# How to make the complete robot

Step by step, from the parts on your desk to the full autonomous rescue. Short on purpose: wherever more detail exists, this page links to it.

---

## Where things stand (read this first)

| Part of the system | Status |
| --- | --- |
| All the "brain" software (perception, world model, decision, mission, Nav2 setup) | ✅ Done. Runs on the laptop, the same as in simulation |
| ESP32 **bench test** (drive the motors + switch the magnet/LED by typing commands) | ✅ Done: [`src/fire_resq_hardware/firmware/bench_test/bench_test.ino`](src/fire_resq_hardware/firmware/bench_test/bench_test.ino) |
| ESP32 **robot firmware** + laptop **bridge** (ROS `cmd_vel` → motors, encoders → odometry, magnet commands) | ❌ **Not written yet.** This is Phase 12; see [`src/fire_resq_hardware/fire_resq_hardware/__init__.py`](src/fire_resq_hardware/fire_resq_hardware/__init__.py) |

Until Phase 12 exists, the real robot can do **two separate demos**: (1) drive and switch the "magnet" from the keyboard; (2) the laptop camera finds victims and fire. It **cannot run the autonomous mission yet.**

---

## What you can do TODAY with what you have

You have: ESP32, 2 geared DC motors, motor driver, battery pack, 2WD chassis, RGB camera.

1. **Wire it and run the bench test** (steps 2–4 below). The robot drives forward, back and turns on your commands, and an LED stands in for the electromagnet (ON/OFF).
2. **Camera demo** (step 5). The laptop sees a blue "victim" and an orange "fire" through your camera and publishes the detections, using the real project perception code.

That is a real partial demo of the low-level control and the perception layer.

---

## 1. Parts: have vs. still to buy

| Part | Have? | Why it's needed |
| --- | --- | --- |
| ESP32 dev board, 2WD chassis + caster, 2 geared DC motors, motor driver, battery pack | ✅ | the base |
| RGB USB camera | ✅ | perception |
| LED + 220 Ω resistor | buy (cheap) | stands in for the electromagnet now |
| **Motors with encoders** (or encoder discs + sensors) | ❌ **needed** | odometry. Without encoders the robot cannot know where it is |
| **Depth camera** (Intel RealSense D435 or similar) | ❌ **needed for autonomy** | mapping (SLAM) and navigation are built on a depth scan; an RGB camera alone does perception only. Why: [CLAUDE.md](CLAUDE.md) → "Navigation" |
| Low-voltage electromagnet + **logic-level MOSFET module** + **flyback diode** (1N4007) | ❌ later | picking up victims |
| 5 V buck regulator | ❌ if your battery is > 5 V | clean power for the ESP32 |
| 3 small victims: **blue**, with a **steel ring/washer** at magnet height | make | targets |
| An **orange** object | make | the "fire" |

Full list and power rules: [Hardware.md](Hardware.md) §7 and §10.

---

## 2. Build the base

1. Mount both motors and wheels, and the caster at the **back**.
2. Mount the ESP32 and motor driver on top. Keep wires short.
3. Mount the camera at the **front**, low, looking straight ahead (sim: 5 cm in front of the wheel axle, 5.5 cm above it).
4. Leave the **front-bottom** free for the electromagnet later. It must not touch the floor.
5. Measure **wheel radius**, **distance between the two wheel centres** and the camera position. You'll need them for odometry in Phase 12; they go in [`src/fire_resq_description/urdf/parameters.xacro`](src/fire_resq_description/urdf/parameters.xacro) (`wheel_radius`, `wheel_separation`, `camera_x`, `camera_z`). Don't change them yet, because the simulation uses the same file.

---

## 3. Wire it

### Motors and driver

| ESP32 pin | L298N pin | TB6612FNG pin | Goes to |
| --- | --- | --- | --- |
| GPIO 25 | ENA | PWMA | left motor speed |
| GPIO 26 | IN1 | AIN1 | left motor direction |
| GPIO 27 | IN2 | AIN2 | left motor direction |
| GPIO 14 | ENB | PWMB | right motor speed |
| GPIO 32 | IN3 | BIN1 | right motor direction |
| GPIO 33 | IN4 | BIN2 | right motor direction |
| 3.3 V | — | STBY **and** VCC | TB6612 only: enable + logic power |
| GND | GND | GND | **common ground (must!)** |

- Left motor → driver OUT1/OUT2 (L298N) or AO1/AO2 (TB6612). Right motor → OUT3/OUT4 or BO1/BO2.
- Battery **+** → driver motor supply (L298N `12V`, TB6612 `VM`). Battery **−** → driver GND.
- **L298N:** remove the ENA/ENB jumpers (the ESP32 drives those pins).
- **L298N:** leave its `5V` pin unconnected while the ESP32 is powered from USB.

### "Magnet" (LED now)

```
GPIO 4 ──[220 Ω]──▶|── GND        (LED: long leg toward the resistor)
```

The on-board LED (GPIO 2) mirrors it.

### Later: the real electromagnet (same pin, no code change)

```
GPIO 4 ─▶ MOSFET module SIG          MOSFET module GND ─ common GND
Battery + ─▶ magnet (+)   magnet (−) ─▶ MOSFET module OUT/DRAIN
1N4007 across the magnet: stripe (cathode) on the magnet (+) side
```

Never drive the magnet straight from a GPIO pin. Why: [Hardware.md](Hardware.md) §6.

### Later: encoders (pins reserved, Phase 12)

| Encoder | ESP32 pin |
| --- | --- |
| Left A / B | GPIO 34 / 35 |
| Right A / B | GPIO 36 (VP) / 39 (VN) |
| Encoder VCC / GND | 3.3 V / GND |

GPIO 34–39 have no internal pull-ups: add 10 kΩ from each to 3.3 V, unless your encoder board already has them.

### Power

- **Bench testing:** power the ESP32 from the laptop USB and the motors from the battery. Grounds connected.
- **Untethered later:** battery → 5 V buck regulator → ESP32 `5V`/`VIN` pin. Never feed USB and VIN at the same time.

---

## 4. Flash the bench test and drive

1. Install **Arduino IDE 2** (on Windows, simplest).
2. Add the ESP32 boards: *File → Preferences → Additional boards manager URLs* →
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
   then *Tools → Board → Boards Manager* → install **esp32 by Espressif**.
3. Open [`src/fire_resq_hardware/firmware/bench_test/bench_test.ino`](src/fire_resq_hardware/firmware/bench_test/bench_test.ino).
4. *Tools → Board* → **ESP32 Dev Module**; *Tools → Port* → your ESP32 → **Upload**.
   (No port? Install the CP210x or CH340 USB driver, whichever matches the chip on your board. Still failing? Hold **BOOT** while it says "Connecting…".)
5. **Lift the wheels off the table**, open *Serial Monitor* (115200 baud, "Newline"), and type:

| Type | Does |
| --- | --- |
| `f` / `b` | forward / backward for 1 s |
| `l` / `r` | spin left / right for 1 s |
| `s` | stop |
| `1`…`9` | speed |
| `m1` / `m0` | magnet (LED) ON / OFF |

**Checks:**
- `f` turns both wheels forward. If one turns backwards, set `L_DIR` or `R_DIR` to `-1` at the top of the sketch and upload again.
- `l` spins the robot left.
- Every move stops by itself after 1 s (safety).

Then put it on the floor and drive it around. **Demo 1 done.**

---

## 5. Camera demo: the laptop finds victims and fire

Uses the real perception node. The robot can stay still on the floor; the camera plugs into the laptop.

**5a. WSL2 only: pass the camera into Linux** (Windows PowerShell as admin):

```powershell
winget install usbipd
usbipd list                              # find your camera's BUSID, e.g. 2-3
usbipd bind --busid 2-3
usbipd attach --wsl --busid 2-3
```

In Ubuntu: `sudo modprobe uvcvideo; ls /dev/video*` → you should see `/dev/video0`.

**5b. Install the camera driver and calibration tool (once):**

```bash
sudo apt install -y ros-jazzy-v4l2-camera ros-jazzy-camera-calibration
```

**5c. Start the camera** (terminal 1):

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -r __ns:=/camera -p video_device:=/dev/video0 \
  -p image_size:="[640,480]" -p camera_frame_id:=camera_optical_frame
# -> /camera/image_raw and /camera/camera_info, exactly what perception reads
```

**5d. Calibrate once** (print a checkerboard with 9×7 squares, so 8×6 inner corners; measure one square). Terminal 2:

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
  --ros-args -r image:=/camera/image_raw -p camera:=/camera
```

Move the board around until CALIBRATE lights up → **CALIBRATE** → **COMMIT** (saved to `~/.ros/camera_info/`). Restart terminal 1 afterwards so the calibration is loaded.

**5e. Run perception** (terminals 2–4):

```bash
# robot shape + camera position (TF), no RViz
ros2 launch fire_resq_description display.launch.py use_rviz:=false
```

```bash
# the robot is standing still: odom = base_link
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link
```

```bash
ros2 launch fire_resq_perception perception.launch.py target_frame:=odom spatial_backend:=known_height
```

**5f. Show it a blue object and an orange object** (terminal 5):

```bash
ros2 topic echo /fire_resq/detections
```

You'll see `victim` / `fire` detections with their positions in front of the robot (`x` forward, `y` left, in metres). An occasional `position_valid: false` is normal: positions are held back for a moment whenever the camera might be moving. **Demo 2 done.**

(Checked on the laptop with a synthetic camera image: the node placed a victim 1.1 m ahead and 0.4 m left, as drawn.)

- **Nothing detected?** The colour ranges were tuned for the simulation. Adjust `hsv_lo`/`hsv_hi` in [`src/fire_resq_perception/config/perception.yaml`](src/fire_resq_perception/config/perception.yaml) (`classes.victim`, `classes.fire`).
- **Distances off?** Set your real objects' heights (`top_height_m`) in the same file. How the estimate works: [CLAUDE.md](CLAUDE.md) → "Perception".

---

## 6. Later: the full autonomous robot

Do these in order. Each one is checkable on its own (the list follows [Hardware.md](Hardware.md) §11).

| # | Step | You'll know it works when |
| --- | --- | --- |
| 1 | Fit encoder motors, wired as in step 3 | wheel ticks count up/down with direction |
| 2 | **Phase 12 software (to be written):** ESP32 firmware + laptop bridge: `cmd_vel` → motors, encoders → `/odom` + TF, magnet ↔ `/fire_resq/magnet/energize` + `/fire_resq/magnet/state`, a stop-if-no-command watchdog | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` drives the robot and `/odom` follows it |
| 3 | Put your measured values in `parameters.xacro` (step 2.5) | a 1 m drive reads ≈ 1 m on `/odom` |
| 4 | Mount the depth camera | `/camera/depth/image_raw` publishes |
| 5 | Fit the electromagnet + MOSFET (step 3); run `ros2 launch fire_resq_control magnet.launch.py` | `ros2 service call /fire_resq/set_magnet fire_resq_interfaces/srv/SetMagnet "{attach: true}"` lifts a victim |
| 6 | Map the real arena, then run the mission: the **same commands** as the simulation guide, steps 4–5, without Gazebo | the robot rescues all victims by itself |

The interfaces the bridge must speak are fixed already: [docs/Hardware_Integration.md](docs/Hardware_Integration.md) and [CLAUDE.md](CLAUDE.md) → "Magnet". The bridge goes in `src/fire_resq_hardware/` (the only package allowed to touch hardware).

---

## Safety rules

- Wheels off the table for every first test.
- Keep a hand near the power switch. The bench sketch auto-stops after 1 s; the real firmware **must** have a watchdog before any autonomous run ([Hardware_Integration.md](docs/Hardware_Integration.md)).
- Common ground between battery, driver and ESP32. Separate the motor power from the ESP32 power.
- Flyback diode on the electromagnet, always.
