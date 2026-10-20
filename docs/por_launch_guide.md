# Proof of Readiness (POR) Launch Guide & Parameter Options

This document details all launch arguments for `sub_bringup.launch.py`, key YAML configuration parameters, the strategy profiles across each `feat/por-*` branch, and how to use the unified helper script [`tools/scripts/launch_por.sh`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/tools/scripts/launch_por.sh).

---

## 1. `sub_bringup.launch.py` Launch Arguments

The root hardware bringup entrypoint for the physical AUV is [`banyu_bringup/launch/sub_bringup.launch.py`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/launch/sub_bringup.launch.py).

| Argument | Type / Choices | Default | Description |
| :--- | :--- | :--- | :--- |
| **`mode`** | `por`, `task2` | `por` | Selects the active mission stack and BehaviorTree. |
| **`por_params_file`** | File path | `por_params.yaml` | Absolute path to the YAML parameter file configuring mission segments, speeds, controllers, and thresholds. |
| **`enable_teleop`** | `True`, `False` | `True` | Starts `banyu_teleop` to listen to `/mavros/rc/in` from the TX16S receiver and control the ESP32 kill relay on `/dev/ttyUSB0`. Set `False` when running without a transmitter. |
| **`enable_mock_auto`** | `True`, `False` | `False` | Land-test mode in `banyu_teleop`. If `True`, putting switch `SB` UP sets mode to `MANUAL` and spins thrusters slowly (1540 PWM). Must be `False` for real pool runs. |
| **`enable_esp32_trigger`** | `True`, `False` | `False` | Starts `esp32_trigger_node`, exposing `/banyu/trigger/cmd` (topic) and `/banyu/trigger/command` (service) to command the ESP32 relay via ROS. Use when `enable_teleop:=False`. |
| **`enable_esp32_bridge`** | `True`, `False` | `False` | Starts `esp32_bridge_node` for streaming raw kill switch status to ROS. Keep `False` when teleop or trigger node owns `/dev/ttyUSB0`. |
| **`enable_cameras`** | `True`, `False` | `True` | Launches the USB front camera V4L2 driver (`/dev/video0`). Set `False` to conserve CPU/USB bandwidth if running purely timed strategies. |
| **`use_gps`** | `True`, `False` | `False` | GPS sensor enablement. Keep `False` for GPS-denied pool testing. |
| **`use_dvl`** | `True`, `False` | `False` | DVL sensor enablement. Keep `False` when running without DVL. |
| **`fcu_url`** | String | `udp://127.0.0.1:14551@` | MAVLink connection URL bridged by `tools/scripts/run_mavproxy.sh`. |
| **`debug`** | `True`, `False` | `False` | Enables OpenCV image processing display windows (only when an X11 display is connected). |

---

## 2. Key Architecture & Safety Parameters

These parameters live under their respective node namespaces in the `por_params_*.yaml` files:

### `banyu_mission`
- **`autonomy_mode`**: 
  - `"ALT_HOLD"`: Recommended for GPS-denied pool testing. Holds depth via the Bar30 pressure sensor and heading via the IMU/compass. ArduSub accepts arming and ALT_HOLD without any position estimate.
  - `"GUIDED"`: Requires ArduSub `position_ok() == true` (needs GPS, DVL, or visual odometry). Will be rejected if no position source is available.
- **`fallback_mode`**: 
  - `"DISARM"`: Immediately disarms the vehicle (`/mavros/cmd/arming` with `value: false`) and switches flight mode to `MANUAL` upon mission completion, abort (`/banyu/mission/abort`), or failure. Prevents thrusters from spinning in the air.
  - `"SURFACE"`: Classic ArduSub surface flight mode. In shallow pools, barometer offset can cause vertical thrusters to spin upward indefinitely.

### `banyu_vehicle_status`
- **`bypass_rc`**:
  - `false`: Normal mode with TX16S transmitter. Requires active RC packets (`rc_fresh == true`) and `operational_mode == 2` (switch `SB` UP) to allow autonomy.
  - `true`: Headless / No-RC mode (used when RC battery is dead). Sets `rc_fresh = true` and `operational_mode = bypass_operational_mode` (2) automatically.
- **`rc_takeover_channel`**:
  - `-1`: Disabled. Recommended when using `banyu_teleop` switch `SB` for mode arbitration.
  - *(Note: Channel `4` corresponds to switch `SF` [Arm/Disarm], so setting this to `4` causes arming to trigger an immediate false operator takeover).*

### `vehicle_execution_node`
- **`control.output_mode`**:
  - `"rc_override"`: Converts guidance velocity intent into RC stick deflections streamed over `/mavros/rc/override`. Required for `ALT_HOLD` autonomy.
  - `"mavros_setpoint"`: Streams velocity setpoints on `/mavros/setpoint_raw/local`. Requires `GUIDED` mode.

---

## 3. Profiles Across `feat/por-*` Branches

| Git Branch | Primary Goal / Description | Recommended YAML Profile (With RC) | No-RC Fallback Profile |
| :--- | :--- | :--- | :--- |
| **`feat/por-dead-thruster`** | Compensates for degraded/dead horizontal thrusters 2 & 3 by pushing along the live forward-left diagonal. | [`por_params_dead_thruster_2_3_no_pos.yaml`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_dead_thruster_2_3_no_pos.yaml) | [`por_params_dead_thruster_2_3_no_rc_no_pos.yaml`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_dead_thruster_2_3_no_rc_no_pos.yaml) |
| **`feat/por-fast-strategies`** | Fast open-loop timed strategies: Strafe box (55s, safest), Diamond reverse (33.3s, fastest), and Diamond 180° turn (39.1s). | [`por_params_timed_fast.yaml`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_timed_fast.yaml) | Add `bypass_rc: true` to profile |
| **`feat/por-timed-strategy`** | Baseline timed dead-reckoning POR with heading hold; includes slow variant for gentle pool testing (0.12 m/s). | [`por_params.yaml`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params.yaml) or [`por_params_timed_slow.yaml`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_timed_slow.yaml) | Add `bypass_rc: true` to profile |
| **`feat/por-vision-assisted`** | Closed-loop vision servoing with front camera gate/marker detection and hybrid camera return. | [`por_params_vision_assisted.yaml`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_vision_assisted.yaml) or [`por_params_hybrid.yaml`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/banyu_bringup/config/por_params_hybrid.yaml) | Add `bypass_rc: true` to profile |

---

## 4. Unified Launch Script (`tools/scripts/launch_por.sh`)

[`tools/scripts/launch_por.sh`](file:///home/gryd/Dev/ros2_ws_robotx/src/BANYU_ROBOTX/tools/scripts/launch_por.sh) automatically inspects your active git branch, checks connected hardware ports, and starts `sub_bringup` with the matching arguments.

### Usage Examples

1. **Standard Autonomous Run with TX16S RC:**
   ```bash
   ./tools/scripts/launch_por.sh
   ```
   *(Auto-detects branch; enables `banyu_teleop`, disables mock auto, applies profile).*

2. **Headless / No-RC Run (When RC battery is dead):**
   ```bash
   ./tools/scripts/launch_por.sh --no-rc
   ```
   *(Switches to `no_rc` profile, sets `enable_teleop:=False`, and launches `esp32_trigger_node`).*

3. **Specify a Custom Profile / Preset:**
   ```bash
   # Run the slow timed strategy (0.12 m/s):
   ./tools/scripts/launch_por.sh --profile slow

   # Run the fastest diamond strategy:
   ./tools/scripts/launch_por.sh --profile fastest

   # Run vision-assisted POR:
   ./tools/scripts/launch_por.sh --profile vision
   ```

4. **Pure Timed Run without Camera (Conserve CPU):**
   ```bash
   ./tools/scripts/launch_por.sh --no-cam
   ```

5. **Dry Run (Verify launch command and port checks without executing):**
   ```bash
   ./tools/scripts/launch_por.sh --dry-run
   ```

---

## 5. Summary of Switch Positions & Emergency Actions

### With TX16S RC Active:
- **`SF` UP (`↑`)**: Disarmed (Pixhawk thrusters unpowered).
- **`SF` DOWN (`↓`)**: Armed (only flip down when vehicle is submerged in water).
- **`SB` UP (`↑`)**: Autonomous (ESP32 relay ON, LED green, autonomy allowed).
- **`SB` MID (`-`)**: Manual Takeover (instantly releases software override, live sticks).
- **`SB` DOWN (`↓`)**: Hardware Kill (ESP32 relay cuts PDB power, LED red, Pixhawk disarmed).

### Without RC Active (`--no-rc`):
- **Power ON relay**: `ros2 topic pub --once /banyu/trigger/cmd std_msgs/msg/String "{data: '0'}"`
- **Hardware Kill**: `ros2 topic pub --once /banyu/trigger/cmd std_msgs/msg/String "{data: '2'}"`
- **Software Abort**: `ros2 service call /banyu/mission/abort std_srvs/srv/Trigger "{}"` (disarms thrusters immediately).
