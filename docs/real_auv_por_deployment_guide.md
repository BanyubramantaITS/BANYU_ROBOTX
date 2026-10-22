# BANYU Real AUV — Proof of Readiness (POR) Deployment Guide (GPS-Denied)

This guide covers the end-to-end hardware wiring, ArduSub/Pixhawk configuration, pre-dive checks, launch commands, and safety procedures for running the **Proof of Readiness (POR)** autonomous mission (`bt/por_mission.xml`) on the physical BANYU AUV **without GPS**.

---

## 1. Hardware & Communication Architecture

```
[RadioMaster TX16S] ──(RC Receiver)──► [Pixhawk (ArduSub)]
                                              │
                                    (/dev/pixhawk or /dev/ttyACM0)
                                              │
                                              ▼
                                 [tools/scripts/run_mavproxy.sh]
                                  ├──► udp:192.168.0.155:14550  (QGroundControl Laptop)
                                  ├──► udp:127.0.0.1:14551      (MAVROS on Mini-PC)
                                  └──► udp:127.0.0.1:14552      (nogps_odom_beacon)

[USB Front Camera (/dev/video0)] ──► [v4l2_camera_node] ──► /front_camera/image_raw
                                                                    │
                                                                    ▼
                                                             [por_detection]
                                                                    │
[ESP32 Kill Relay (/dev/ttyUSB0)] ◄── [banyu_teleop] ──► /banyu/hardware/kill_switch
                                                                    │
                                                                    ▼
                                                        [banyu_vehicle_status]
                                                                    │
                                                                    ▼
                                            [banyu_mission] ◄──► [vehicle_execution_node]
```

### Why POR Works Without GPS

> [!IMPORTANT]
> **No DVL on the real AUV.** The team does not own a DVL, so real-world runs never use one. Always keep `use_dvl:=False` on hardware; DVL support exists for simulation only.

- Every POR phase (`AcquireGate` → `AlignGate` → `PassGate` → `SearchMarker` → `AlignMarker` → `OrbitMarker` → `TurnAround` → `SearchGate` → `AlignGate` → `PassGate` → `Surface`) uses **barometer depth**, **IMU/compass heading**, and **camera visual servoing** (`/banyu/detection`).
- POR flies in **`ALT_HOLD` with RC override** (`control.output_mode: rc_override`), never `GUIDED`. `ALT_HOLD` needs no position estimate: depth comes from the Bar30 and heading from the compass/IMU (`/mavros/imu/data` when no pose is available).
- When launched with `use_gps:=False` (the default in `sub_bringup.launch.py`):
  1. `ekf_source_switch.py` sets `EK3_SRC_OPTION` to **Source Set 2 (`SRC2`)** even at the surface (`POSXY=NONE`, `VELXY=EXTNAV`, `POSZ=BARO`, `YAW=COMPASS`).
  2. When `use_dvl:=False`, `nogps_odom_beacon.py` streams 10 Hz zero-delta `VISION_POSITION_DELTA` to `127.0.0.1:14552` so EKF3 immediately enters `AID_RELATIVE` and publishes real barometer depth and compass yaw on `/mavros/local_position/pose`.

---

## 2. RadioMaster TX16S Switch Mapping (`banyu_teleop`)

> [!NOTE]
> Single source of truth for all `banyu_teleop` parameters and channel assignments is [`banyu_teleop/config/teleop_rcin.yaml`](../banyu_teleop/config/teleop_rcin.yaml).

Ensure the TX16S is powered on and bound before starting the software stack:

| TX16S Control | RC Channel (`0`-indexed) | Position | PWM | Action |
| :--- | :--- | :--- | :--- | :--- |
| **`SB`** (3-pos) | `CH6` (`kill_channel: 5`) | Up (`↑`) | `988` | **Autonomous (`2`)**: ESP32 `'2'`, automatically arms Pixhawk (if not armed), enables autonomous mission execution. Locks out manual gimbals & SC flight mode changes. |
| **`SB`** (3-pos) | `CH6` (`kill_channel: 5`) | Mid (`-`) | `1500` | **Remote / Manual (`1`)**: ESP32 `'1'`, automatically arms Pixhawk, enables gimbal manual control and SC flight mode changes. |
| **`SB`** (3-pos) | `CH6` (`kill_channel: 5`) | Down (`↓`) | `2011` | **Hardware Kill / Disarm (`0`)**: ESP32 `'0'` (cuts relay power) + disarms Pixhawk thrusters. |
| **`SC`** (3-pos) | `CH8` (`mode_channel: 7`) | Up / Mid / Down | `988` / `1500` / `2011` | Flight mode: `STABILIZE` (`↑`) / `MANUAL` (`-`) / `ALT_HOLD` (`↓`). Active only in Manual/Disarmed mode; rejected in Autonomous mode. |
| **`SF`** (2-pos) | — | — | — | **Unmapped / Unused**: Pixhawk arming and disarming are unified with the ESP32 kill switch on `SB`. |

---

## 3. One-Time Pixhawk Parameter Check (QGroundControl)

Load or verify the parameters in `banyu_bringup/config/banyu_defaults.parm` on the Pixhawk via QGroundControl:

1. **EKF3 & Visual Odometry (only needed for `/mavros/local_position/pose`; POR itself flies `ALT_HOLD`):**
   - `VISO_TYPE = 1` (MAVLink)
   - `EK3_SRC2_POSXY = 0` (None)
   - `EK3_SRC2_VELXY = 6` (ExternalNav)
   - `EK3_SRC2_POSZ = 1` (Baro)
   - `EK3_SRC2_VELZ = 0` (None)
   - `EK3_SRC2_YAW = 1` (Compass)
2. **Failsafes:**
   - `FS_EKF_ACTION = 0` (Disabled for GPS-less operation; set in `banyu_defaults.parm`)
   - `FS_EKF_THRESH = 0.8` and `ARMING_CHECK = 1`: check these in QGC; `banyu_defaults.parm` does not set them.
3. Reboot the Pixhawk if `VISO_TYPE` was changed from `0` to `1`.

---

## 4. Pre-Launch Hardware Device Verification (On AUV Mini-PC)

SSH into the AUV mini-PC and verify device paths:

```bash
# 1. Check Pixhawk USB serial port
ls -l /dev/pixhawk /dev/ttyACM*

# 2. Check ESP32 USB serial port (used by banyu_teleop)
ls -l /dev/ttyUSB*

# 3. Check USB cameras (front camera defaults to /dev/video0 in hardware_params.yaml)
v4l2-ctl --list-devices
```

If the front camera is not `/dev/video0` or the ESP32 is not `/dev/ttyUSB0`, update:
- `banyu_bringup/config/hardware_params.yaml`: `front_camera.ros__parameters.video_device`
- `banyu_teleop/config/teleop_rcin.yaml`: `banyu_teleop.ros__parameters.serial_port`

---

## 5. Step-by-Step Launch Procedure

### Step 1 — Start MAVProxy on the AUV Mini-PC
`run_mavproxy.sh` bridges `/dev/pixhawk` (or `/dev/ttyACM0`) at 10 Hz (`--streamrate=10`) to QGroundControl (`14550`), MAVROS (`127.0.0.1:14551`), and the EKF beacon/DVL (`127.0.0.1:14552`):

```bash
./src/BANYU_ROBOTX/tools/scripts/run_mavproxy.sh
```
*(Optional: pass custom device and QGC laptop IP: `./src/BANYU_ROBOTX/tools/scripts/run_mavproxy.sh /dev/ttyACM0 192.168.0.155`)*

### Step 2 — Launch the AUV POR Stack (Terminal 1)
On the AUV mini-PC:

```bash
bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
  ros2 launch banyu_bringup sub_bringup.launch.py mode:=por'
```

*(Default arguments for `sub_bringup.launch.py`: `mode:=por`, `use_gps:=False`, `use_dvl:=False`, `enable_teleop:=True`, `enable_esp32_bridge:=False`, `enable_cameras:=True`, `fcu_url:=udp://127.0.0.1:14551@`)*

### Step 3 — Verify Readiness Before Putting the AUV in the Gate Lane (Terminal 2)
Run these checks in a second terminal on the mini-PC:

1. **Verify `ekf_source_switch` and `/mavros/local_position/pose`:**
   - Look for `[ekf_source_switch]: EKF source set 2 accepted` in Terminal 1.
   - Check that `/mavros/local_position/pose` is streaming (~10 Hz):
     ```bash
     bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
       ros2 topic hz /mavros/local_position/pose'
     ```
2. **Verify TX16S switches and `/banyu/vehicle_status`:**
   - Place TX16S **`SB` in Up (`↑`, Autonomous)** (note: keep on bench/stand or disable thrusters before arming).
   - Echo `/banyu/vehicle_status`:
     ```bash
     bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
       ros2 topic echo /banyu/vehicle_status --once'
     ```
   - Confirm:
     - `fcu_connected: true`
     - `fcu_status_fresh: true`
     - `rc_fresh: true`
     - `operator_takeover: false`
     - `operational_mode: 2` (`OPERATIONAL_MODE_AUTONOMOUS`)
     - `hardware_kill_active: false`
3. **Verify Front Camera & Gate Detection:**
   - Point the AUV at the gate and check `/banyu/detection`:
     ```bash
     bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
       ros2 topic echo /banyu/detection --once'
     ```
   - Confirm `detected: true` (`object_id: 0` for gate, `1` for yellow marker buoy).

### Step 4 — Start the Autonomous POR Mission
Place the AUV in the water facing the gate, ensure TX16S **`SB` is Up (`↑`, Autonomous)**, and call `/banyu/mission/start`:

```bash
bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
  ros2 service call /banyu/mission/start std_srvs/srv/Trigger "{}"'
```

What happens automatically:
- **In Timed Mode (`mission.use_vision: false`, the default in every timed profile — loads `bt/por_timed_mission.xml`):**
  1. `ArmAndSetGuided` switches ArduSub to `ALT_HOLD` (the configured `autonomy_mode`) and arms the thrusters.
  2. `dive_to_gate` dives in place to `gate_depth_m` (`2.0 m`); it ends when the depth is reached (`check_depth_reached`) or after `dive_duration_s` (`5.0 s`).
  3. `pass_gate` drives forward (`timed_pass_gate_speed_x_mps: 0.6`) for `timed_pass_gate_duration_s` (`6.0 s`) while holding `gate_depth_m`.
  4. `approach_pole` drives forward (`approach_pole_speed_x_mps: 0.6`) for `approach_pole_duration_s` (`8.0 s`) toward the marker pole.
  5. `circle_pole_polygon` executes 4 compartmentalized turn+forward legs around the pole (`circle_turn_1` → `circle_forward_1` → `circle_turn_2` → `circle_forward_2` → `circle_turn_3` → `circle_forward_3` → `circle_turn_4` → `circle_forward_4`).
  6. `return_turn` turns to face the gate (`return_turn_duration_s` / `return_turn_yaw_rate_rad_s`), and `return_to_gate` drives forward back through the gate (`return_to_gate_duration_s` / `return_to_gate_speed_x_mps`).
  7. `surface` ascends (`timed_surface_duration_s` / `timed_surface_speed_z_mps`) until `surface_depth_m` (`0.0 m`).
- **In Vision Mode (`mission.use_vision: true` — loads `bt/por_mission.xml`):**
  1. `ArmAndSetGuided` → `AcquireGate` → `AlignGate` → `PassGate` → `SearchMarker` → `AlignMarker` → `OrbitMarker` → `TurnAround` → `SearchGate` → `AlignGate` → `PassGate` → `Surface`.

---

## 6. Tuning the Timed POR Strategy (`banyu_bringup/config/por_params.yaml`)

All timed POR parameters live under `banyu_mission.ros__parameters.mission` in `banyu_bringup/config/por_params.yaml`:

| Segment | Duration Parameter (`s`) | Speed / Rate Parameters | Notes |
| :--- | :--- | :--- | :--- |
| **Sensor Master Toggle** | — | `use_sensors: true` | `true` (default) = uses Pixhawk Barometer & IMU. Set `false` for 100% pure timer + speed without any sensors. |
| **Mode toggle** | — | `use_vision: false` | `false` = `por_timed_mission.xml`, `true` = `por_mission.xml` |
| **Pixhawk Baro Depth** | — | `use_baro_depth: true`, `gate_depth_m: 2.0` | Holds `gate_depth_m` via Pixhawk barometer on all segments (set `false` or `use_sensors: false` for open-loop `*_speed_z_mps`) |
| **Pixhawk IMU Yaw** | — | `use_imu_yaw_cap: true` | Uses Pixhawk IMU heading & stops turns when reaching `*_target_angle_rad` (`90°`) or `*_duration_s` |
| **1. Dive to Gate** | `dive_duration_s: 5.0` | `dive_speed_x/y/z_mps` (`z: -0.4`), `dive_yaw_rate_rad_s` | Timed descent to gate center depth |
| **2. Pass Gate** | `timed_pass_gate_duration_s: 6.0` | `timed_pass_gate_speed_x/y/z_mps` (`x: 0.6`), `timed_pass_gate_yaw_rate_rad_s` | Forward through gate |
| **3. Approach Pole** | `approach_pole_duration_s: 8.0` | `approach_pole_speed_x/y/z_mps` (`x: 0.6`), `approach_pole_yaw_rate_rad_s` | Forward from gate to pole |
| **4a. Circle Turn 1** | `circle_turn_1_duration_s: 2.62` | `circle_turn_1_speed_x/y/z_mps`, `circle_turn_1_yaw_rate_rad_s: -0.6` | `-0.6 rad/s` = right turn (`-90°`) |
| **4b. Circle Leg 1** | `circle_forward_1_duration_s: 3.0` | `circle_forward_1_speed_x/y/z_mps` (`x: 0.5`), `circle_forward_1_yaw_rate_rad_s` | Side 1 of box |
| **4c. Circle Turn 2** | `circle_turn_2_duration_s: 2.62` | `circle_turn_2_speed_x/y/z_mps`, `circle_turn_2_yaw_rate_rad_s: 0.6` | `+0.6 rad/s` = left turn (`+90°`) |
| **4d. Circle Leg 2** | `circle_forward_2_duration_s: 5.0` | `circle_forward_2_speed_x/y/z_mps` (`x: 0.5`), `circle_forward_2_yaw_rate_rad_s` | Side 2 of box |
| **4e. Circle Turn 3** | `circle_turn_3_duration_s: 2.62` | `circle_turn_3_speed_x/y/z_mps`, `circle_turn_3_yaw_rate_rad_s: 0.6` | `+0.6 rad/s` = left turn (`+90°`) |
| **4f. Circle Leg 3** | `circle_forward_3_duration_s: 5.0` | `circle_forward_3_speed_x/y/z_mps` (`x: 0.5`), `circle_forward_3_yaw_rate_rad_s` | Side 3 of box |
| **4g. Circle Turn 4** | `circle_turn_4_duration_s: 2.62` | `circle_turn_4_speed_x/y/z_mps`, `circle_turn_4_yaw_rate_rad_s: 0.6` | `+0.6 rad/s` = left turn (`+90°`) |
| **4h. Circle Leg 4** | `circle_forward_4_duration_s: 5.0` | `circle_forward_4_speed_x/y/z_mps` (`x: 0.5`), `circle_forward_4_yaw_rate_rad_s` | Side 4 of box |
| **5a. Return Turn** | `return_turn_duration_s: 2.62` | `return_turn_speed_x/y/z_mps`, `return_turn_yaw_rate_rad_s: 0.6` | Align heading back toward gate |
| **5b. Return to Gate**| `return_to_gate_duration_s: 14.0` | `return_to_gate_speed_x/y/z_mps` (`x: 0.6`), `return_to_gate_yaw_rate_rad_s` | Drive back through gate |
| **6. Surface** | `timed_surface_duration_s: 6.0` | `timed_surface_speed_x/y/z_mps` (`z: 0.4`), `timed_surface_yaw_rate_rad_s` | Timed ascent to surface |

> **Tip:** Setting any segment's `*_duration_s` to `0.0` immediately skips that segment with `SUCCESS` (no code or XML edits needed). Because `banyu_bringup` uses `--symlink-install`, edits to `src/BANYU_ROBOTX/banyu_bringup/config/por_params.yaml` take effect immediately on the next `ros2 launch` without rebuilding.

---

## 7. Emergency Stop & Manual Takeover Procedures

At any point during the run, you can interrupt or stop the AUV immediately:

1. **TX16S `SB` → Middle (`-`, Remote/Manual):**
   - Immediately sets `operational_mode = 1` (`MANUAL`), triggering `vehicle_execution_node`'s safety fault, aborting all active goals, and zeroing velocity setpoints. You can then fly manually using `SC` (`MANUAL` / `ALT_HOLD` / `STABILIZE`).
2. **TX16S `SB` → Down (`↓`, Kill):**
   - Immediately sends `'0'` to the ESP32 over `/dev/ttyUSB0` (cutting thruster relay power), sets `hardware_kill_active = true`, and disarms the Pixhawk.
3. **Software Abort Service:**
   ```bash
   bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
     ros2 service call /banyu/mission/abort std_srvs/srv/Trigger "{}"'
   ```
   - Triggers the BehaviorTree abort branch, cancels the active action, and applies `fallback_mode` (`DISARM`): the Pixhawk is disarmed and set to `MANUAL`, so the thrusters stop immediately and the AUV only rises by its own buoyancy.

---

## 8. Simulation Testing & Verification

The timed POR strategy can be fully tested in Gazebo Harmonic + ArduSub SITL + MAVROS simulation.

### 8.1 Launch Simulation

To run with full Gazebo GUI:
```bash
bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
  ros2 launch banyu_bringup main_launch.launch.py mode:=por'
```

To run headless (server-only, faster execution, ideal for automated test runs):
```bash
bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
  ros2 launch banyu_bringup main_launch.launch.py mode:=por headless:=True'
```

### 8.2 Verify Readiness

Check that MAVROS and ArduSub SITL have initialized and calibrated:
```bash
bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
  ros2 topic echo /banyu/vehicle_status --once'
```
Expect: `fcu_connected: true`, `rc_fresh: true`, `operational_mode: 2` (AUTONOMOUS).

### 8.3 Start the Mission

Call the mission start service:
```bash
bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
  ros2 service call /banyu/mission/start std_srvs/srv/Trigger "{}"'
```

### 8.4 Verification Output

During execution, watch the logs:
1. Vehicle arms and switches flight mode to `ALT_HOLD`.
2. Segments execute sequentially:
   - `[BT] Executing: dive_to_gate (TimedMove)`
   - `[BT] Executing: pass_gate (TimedMove)`
   - `[BT] Executing: approach_pole (TimedMove)`
   - `[BT] Executing: circle_turn_1 ... circle_forward_4` (polygon circling)
   - `[BT] Executing: return_turn (TimedMove)`
   - `[BT] Executing: return_to_gate (TimedMove)`
   - `[BT] Executing: surface (TimedMove)`
3. Completion: `BehaviorTree execution finished: status=SUCCESS, interrupted=false`.

