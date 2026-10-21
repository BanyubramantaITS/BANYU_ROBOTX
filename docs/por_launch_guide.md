# Proof of Readiness (POR) Unified Launch & Deployment Guide

This guide covers the **unified POR mission stack** on branch [`feat/por-unified`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/), combining all capabilities from the `feat/por-*` branches into a single, cohesive architecture:
- **Dead-Thruster Compensation** (`feat/por-dead-thruster`): 45° diagonal surge/sway mapping for damaged thrusters 2 & 3, GPS-denied `ALT_HOLD` flight via RC override, no-RC headless mode via ESP32 trigger, and `DISARM` fallback.
- **Fast Timed Strategies** (`feat/por-fast-strategies`): Ultra-fast strafe-box (~55s), diamond reverse (~33.3s), diamond 180° turn (~39.1s), and land-test mock auto mode.
- **Timed Strategies & Heading Hold** (`feat/por-timed-strategy`): Straight-leg IMU heading hold, speed capping, obstacle collision proximity guard, sensor-driven depth reached completion, and slow tuning profile (0.12 m/s).
- **Vision-Assisted Servoing** (`feat/por-vision-assisted`): Per-object HSV filtering for yellow gate and orange marker with live slider tuning (`por_hsv_tuner.py`), lateral-only alignment, and camera-guided return legs.

The launcher script [`tools/scripts/launch_por.sh`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/tools/scripts/launch_por.sh) easily starts the entire stack both in **Gazebo Harmonic SITL simulation** and on **physical AUV hardware**.

---

## 1. Quick Start Guide

### A. Gazebo Harmonic Simulation (SITL)

1. **Launch with 3D GUI (default fastest diamond profile):**
   ```bash
   ./tools/scripts/launch_por.sh --sim --profile fastest
   ```

2. **Launch headless (Gazebo server only, optimal for CI or low-spec machines):**
   ```bash
   ./tools/scripts/launch_por.sh --sim --headless --profile safe
   ```

3. **Check link and trigger mission:**
   ```bash
   # Verify MAVROS connection to ArduSub SITL:
   ros2 topic echo /mavros/state --once

   # Trigger autonomous POR mission start:
   ros2 service call /banyu/mission/start std_srvs/srv/Trigger "{}"

   # If needed, command software abort (switches mode or disarms immediately):
   ros2 service call /banyu/mission/abort std_srvs/srv/Trigger "{}"
   ```

---

### B. Real AUV Physical Deployment

1. **Autonomous Run with Radio Transmitter (TX16S):**
   ```bash
   ./tools/scripts/launch_por.sh --real
   ```
   *(Or simply `./tools/scripts/launch_por.sh`)*

2. **Headless / No-RC Mode (when transmitter battery is flat or running without RC):**
   ```bash
   ./tools/scripts/launch_por.sh --real --no-rc --profile dead-thruster-norc
   ```

3. **Dry Run (verify ports, MAVProxy, parameters, and generated command without launching):**
   ```bash
   ./tools/scripts/launch_por.sh --real --dry-run
   ```

---

## 2. Launch Script Reference (`tools/scripts/launch_por.sh`)

[`tools/scripts/launch_por.sh`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/tools/scripts/launch_por.sh) unifies execution flags:

| Flag | Category | Default | Description |
| :--- | :--- | :--- | :--- |
| **`--sim`**, **`--gazebo`** | Target | | Launches Gazebo Harmonic simulation via `main_launch.launch.py`. Automatically clears stale `ardusub`/`mavproxy` processes. |
| **`--real`**, **`--hardware`** | Target | Real | Launches physical vehicle hardware bringup via `sub_bringup.launch.py`. Validates serial ports. |
| **`--headless`** | Simulation | False | Disables Gazebo Harmonic GUI and MAVProxy X11 console. |
| **`--gui`** | Simulation | True | Runs full Gazebo Harmonic 3D GUI. |
| **`--no-clean`** | Simulation | False | Skips killing existing SITL processes before launching. |
| **`--rc`** | Hardware | True | Enables `banyu_teleop` to listen to TX16S RC and drive ESP32 kill relay. |
| **`--no-rc`** | Hardware | False | Disables teleop; enables `bypass_rc` and `esp32_trigger_node` for headless autonomous runs. |
| **`--profile <name>`** | Strategy | Auto | Selects the parameter profile YAML (see catalog below). |
| **`--no-cam`** | Hardware | False | Disables front camera V4L2 driver to reduce CPU load on pure timed runs. |
| **`--mock-auto`** | Testing | False | Enables land-test mock auto mode in `banyu_teleop` (spins motors at 1540 PWM on land). |
| **`--dry-run`** | General | False | Prints environment, device checks, and the exact `ros2 launch` command without executing. |

---

## 3. Strategy Profiles Catalog

All profile files live in [`banyu_bringup/config/`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/):

| Preset Alias | YAML File | Strategy Description | Estimated Time | Best For |
| :--- | :--- | :--- | :--- | :--- |
| `fastest` / `fastest-diamond` | [`por_params_timed_fastest.yaml`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_timed_fastest.yaml) | Reverse diamond trajectory (no yaw changes). Outbound 0.8 m/s, strafes 0.6 m/s, returns backward through gate. | **~33.3 s** | Fastest competition scoring. |
| `turn` / `fastest-turn` | [`por_params_timed_fastest_turn.yaml`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_timed_fastest_turn.yaml) | Forward diamond with 180° turn around marker and forward return pass. | **~39.1 s** | Pool runs where forward cameras monitor the return leg. |
| `fast` / `fast-strafe` | [`por_params_timed_fast.yaml`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_timed_fast.yaml) | Strafe box polygon (forward -> strafe right -> pass pole -> strafe left -> reverse return). | **~55.0 s** | Reliable open-loop timed run with maximum obstacle clearance. |
| `safe` | [`por_params_timed_safe.yaml`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_timed_safe.yaml) | Conservative timed strafe box with wider safety margins and gentle accelerations. | **~87.0 s** | Initial verification in unfamiliar pools. |
| `slow` | [`por_params_timed_slow.yaml`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_timed_slow.yaml) | Slow 0.12 m/s timed legs with straight-line heading hold. | **~120.0 s** | Gentle initial debugging, hydrophone tests, or water leaks checks. |
| `dead-thruster` | [`por_params_dead_thruster_2_3_no_pos.yaml`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_dead_thruster_2_3_no_pos.yaml) | 45° forward-left thrust compensation for dead thrusters 2 & 3 in `ALT_HOLD` mode with TX16S RC active. | **~48.0 s** | Real vehicle with thrusters 2 and 3 unpowered or disabled. |
| `dead-thruster-norc` | [`por_params_dead_thruster_2_3_no_rc_no_pos.yaml`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_dead_thruster_2_3_no_rc_no_pos.yaml) | Dead-thruster profile configured for headless operation (`bypass_rc: true`, `esp32_trigger`). | **~48.0 s** | Dead-thruster testing when transmitter is unavailable. |
| `vision` | [`por_params_vision_assisted.yaml`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_vision_assisted.yaml) | Closed-loop vision centering on gate at start, lateral alignment on marker, and camera turn-and-pass return. | Variable | Water with good visibility (front camera tracking). |
| `hybrid` | [`por_params_hybrid.yaml`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_hybrid.yaml) | Timed outbound legs + camera-guided gate return with timed fallback. | Variable | Environments where outbound marker detection is difficult. |
| `default` | [`por_params.yaml`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params.yaml) | Baseline POR profile with closed-loop gate/marker vision tracking. | Variable | Baseline testing in simulation and standard operations. |

---

## 4. Key Launch Arguments & Architecture Details

### Bringup Launch Files

1. **Simulation:** [`banyu_bringup/launch/main_launch.launch.py`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/launch/main_launch.launch.py)
   - `mode`: `"por"` (default) or `"task2"`.
   - `por_params_file`: Absolute path to selected YAML profile.
   - `headless`: `"False"` (default, GUI) or `"True"` (server-only).
   - `use_sim`: `"True"`.
   - `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw`: Vehicle spawn location.

2. **Real AUV:** [`banyu_bringup/launch/sub_bringup.launch.py`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/launch/sub_bringup.launch.py)
   - `mode`: `"por"`.
   - `por_params_file`: Absolute path to selected YAML profile.
   - `enable_teleop`: `"True"` (when using TX16S) or `"False"`.
   - `enable_esp32_trigger`: `"True"` (when running `--no-rc`) or `"False"`.
   - `enable_cameras`: `"True"` or `"False"`.
   - `enable_mock_auto`: `"True"` (land motor spin) or `"False"`.
   - `fcu_url`: `"udp://127.0.0.1:14551@"`.

### Flight & Control Parameters

- **`autonomy_mode`**:
  - `"ALT_HOLD"`: Essential for GPS-denied pools without DVL. ArduSub arms and flies without any local position estimate (`position_ok() == false`), using Bar30 for depth and IMU compass for heading.
  - `"GUIDED"`: Used in simulation or on vehicles with active DVL/position odometry.
- **`control.output_mode`**:
  - `"rc_override"`: Converts velocity intents into RC stick deflections streamed over `/mavros/rc/override`. Required for `ALT_HOLD` autonomy.
  - `"mavros_setpoint"`: Streams velocity setpoints on `/mavros/setpoint_raw/local` (requires `GUIDED`).
- **`fallback_mode`**:
  - `"DISARM"`: Immediately disarms the vehicle (`/mavros/cmd/arming value: false`) upon mission completion or abort, ensuring props do not spin in the air when surfaced.
  - `"SURFACE"`: Classic ArduSub surface flight mode.
- **`timed_move.heading_hold.enabled`**:
  - `true`: Timed straight segments capture the initial heading and command proportional yaw corrections to eliminate drift from thruster asymmetry or current.
- **`timed_move.max_horizontal_speed_mps`**:
  - Sets an upper bound on horizontal surge/sway speed during timed segments (e.g., `0.12` for slow pool testing).
- **`enable_collision_guard`**:
  - `true`: `TimedMoveServer` monitors `/banyu/detection` and automatically halts forward motion if an obstacle is within `collision_distance_m` (default: 0.8m).

---

## 5. Physical Switch Operations & Safety Procedures

### With TX16S Radio Active:
- **`SF` UP (`↑`)**: **Disarmed** (Pixhawk thrusters unpowered). Must be kept UP while handling vehicle on land or deck.
- **`SF` DOWN (`↓`)**: **Armed** (only toggle DOWN once the vehicle is submerged in water).
- **`SB` UP (`↑`)**: **Autonomous Mode** (ESP32 relay energized, LED GREEN, autonomy mission permitted).
- **`SB` MID (`-`)**: **Manual Takeover** (instantly relinquishes software control to physical transmitter sticks).
- **`SB` DOWN (`↓`)**: **Hardware Emergency Kill** (ESP32 relay cuts all thruster power, LED RED, Pixhawk disarmed).

### Without RC Active (`--no-rc`):
- **Power ON ESP32 Relay**:
  ```bash
  ros2 topic pub --once /banyu/trigger/cmd std_msgs/msg/String "{data: '0'}"
  ```
- **Hardware Kill Relay**:
  ```bash
  ros2 topic pub --once /banyu/trigger/cmd std_msgs/msg/String "{data: '2'}"
  ```
- **Software Mission Abort & Immediate Disarm**:
  ```bash
  ros2 service call /banyu/mission/abort std_srvs/srv/Trigger "{}"
  ```

---

## 6. Helper Tools

- **Color Tuning for Gate & Marker**:
  ```bash
  python3 tools/scripts/por_hsv_tuner.py
  ```
  Provides live OpenCV GUI sliders over WiFi or USB to adjust gate and marker HSV boundaries, then saves directly to YAML via ROS 2 trigger service.
- **Dead Thruster Parameter Generator**:
  ```bash
  python3 tools/scripts/dead_thruster_por.py --help
  ```
  Generates custom timed profiles tailored to any thruster degradation or pool geometry.
- **ESP32 Command CLI**:
  ```bash
  python3 tools/scripts/esp32_kill_cmd.py --cmd auto   # (or remote / kill)
  ```
