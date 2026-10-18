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
                                  └──► udp:127.0.0.1:14552      (nogps_odom_beacon / DVL)

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
- Every POR phase (`AcquireGate` → `AlignGate` → `PassGate` → `SearchMarker` → `AlignMarker` → `OrbitMarker` → `TurnAround` → `SearchGate` → `AlignGate` → `PassGate` → `Surface`) uses **barometer depth**, **IMU/compass heading**, and **camera visual servoing** (`/banyu/detection`).
- ArduSub requires its EKF3 estimator to be in `AID_RELATIVE` (`position_ok() == true`) before entering `GUIDED` mode or publishing `/mavros/local_position/pose`.
- When launched with `use_gps:=False` (the default in `sub_bringup.launch.py`):
  1. `ekf_source_switch.py` sets `EK3_SRC_OPTION` to **Source Set 2 (`SRC2`)** even at the surface (`POSXY=NONE`, `VELXY=EXTNAV`, `POSZ=BARO`, `YAW=COMPASS`).
  2. When `use_dvl:=False`, `nogps_odom_beacon.py` streams 10 Hz zero-delta `VISION_POSITION_DELTA` to `127.0.0.1:14552` so EKF3 immediately enters `AID_RELATIVE` and publishes real barometer depth and compass yaw on `/mavros/local_position/pose`.

---

## 2. RadioMaster TX16S Switch Mapping (`banyu_teleop`)

Ensure the TX16S is powered on and bound before starting the software stack:

| TX16S Control | RC Channel (`0`-indexed) | Position | PWM | Action |
| :--- | :--- | :--- | :--- | :--- |
| **`SF`** (2-pos) | `CH5` (`arm_channel: 4`) | Up (`↑`) | `988` | Disarm Pixhawk thrusters |
| **`SF`** (2-pos) | `CH5` (`arm_channel: 4`) | Down (`↓`) | `2011` | Arm Pixhawk thrusters |
| **`SB`** (3-pos) | `CH6` (`kill_channel: 5`) | Up (`↑`) | `988` | **Autonomous (`2`)**: ESP32 `'0'`, enables autonomous mission execution |
| **`SB`** (3-pos) | `CH6` (`kill_channel: 5`) | Mid (`-`) | `1500` | **Remote / Manual (`1`)**: ESP32 `'1'`, immediately aborts autonomous motion (`operator_takeover`) |
| **`SB`** (3-pos) | `CH6` (`kill_channel: 5`) | Down (`↓`) | `2011` | **Hardware Kill (`0`)**: ESP32 `'2'` (cuts relay power) + disarms Pixhawk |
| **`SE`** (3-pos) | `CH7` (`mode_channel: 6`) | Up / Mid / Down | `988` / `1500` / `2011` | Flight mode: `MANUAL` / `ALT_HOLD` / `STABILIZE` |

---

## 3. One-Time Pixhawk Parameter Check (QGroundControl)

Load or verify the parameters in `banyu_bringup/config/banyu_defaults.parm` on the Pixhawk via QGroundControl:

1. **EKF3 & Visual Odometry (Required for GPS-denied `GUIDED` mode):**
   - `VISO_TYPE = 1` (MAVLink)
   - `EK3_SRC2_POSXY = 0` (None)
   - `EK3_SRC2_VELXY = 6` (ExternalNav)
   - `EK3_SRC2_POSZ = 1` (Baro)
   - `EK3_SRC2_VELZ = 0` (None)
   - `EK3_SRC2_YAW = 1` (Compass)
2. **Failsafes:**
   - `FS_EKF_ACTION = 0` (Disabled for GPS-less operation; `FS_EKF_THRESH = 0.8`)
   - `ARMING_CHECK = 1`
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

If the front camera is not `/dev/video0` or the ESP32 is not `/dev/ttyUSB0`, update `banyu_bringup/config/hardware_params.yaml`:
- `front_camera.ros__parameters.video_device`
- `banyu_teleop.ros__parameters.rc_in.serial_port`

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
   - Place TX16S **`SB` in Up (`↑`, Autonomous)** and **`SF` in Up (`↑`, Disarmed)**.
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
- **In Timed Mode (`mission.use_vision: false`, default on `feat/por-timed-strategy` — loads `bt/por_timed_mission.xml`):**
  1. `ArmAndSetGuided` switches ArduSub to `GUIDED` mode and arms the thrusters.
  2. `dive_to_gate` dives in place to `gate_depth_m` (`2.0 m`) for `dive_duration_s` (`5.0 s`).
  3. `pass_gate` drives forward (`pass_gate_forward_speed_mps: 0.6 m/s`) for `pass_gate_duration_s` (`8.0 s`) while holding `gate_depth_m`.
  4. `approach_pole` drives forward (`approach_pole_forward_speed_mps: 0.6 m/s`) for `approach_pole_duration_s` (`8.0 s`) toward the marker pole.
  5. `circle_pole_polygon` executes 4 compartmentalized turn+forward legs around the pole (`circle_turn_1` → `circle_forward_1` → `circle_turn_2` → `circle_forward_2` → `circle_turn_3` → `circle_forward_3` → `circle_turn_4` → `circle_forward_4`).
  6. `return_turn` turns to face the gate (`return_turn_duration_s` / `return_turn_yaw_rate_rad_s`), and `return_to_gate` drives forward back through the gate (`return_to_gate_duration_s` / `return_to_gate_forward_speed_mps`).
  7. `Surface` ascends to `surface_depth_m` (`0.0 m`).
- **In Vision Mode (`mission.use_vision: true` — loads `bt/por_mission.xml`):**
  1. `ArmAndSetGuided` → `AcquireGate` → `AlignGate` → `PassGate` → `SearchMarker` → `AlignMarker` → `OrbitMarker` → `TurnAround` → `SearchGate` → `AlignGate` → `PassGate` → `Surface`.

---

## 6. Tuning the Timed POR Strategy (`banyu_bringup/config/por_params.yaml`)

All timed POR parameters live under `banyu_mission.ros__parameters.mission` in `banyu_bringup/config/por_params.yaml`:

| Segment | Duration Parameter (`s`) | Speed / Rate Parameter | Notes |
| :--- | :--- | :--- | :--- |
| **Mode toggle** | — | `use_vision: false` | `false` = `por_timed_mission.xml`, `true` = `por_mission.xml` |
| **Depth mode** | — | `use_baro_depth: true`, `gate_depth_m: 2.0` | Holds `gate_depth_m` via barometer P-loop on all segments (`false` = open-loop `dive_vertical_speed_mps`) |
| **IMU yaw cap** | — | `use_imu_yaw_cap: false` | `false` = pure timer turns; `true` = also stops turn when IMU reaches `*_target_angle_rad` |
| **1. Dive to Gate** | `dive_duration_s: 5.0` | `dive_vertical_speed_mps: -0.4` | Dives to `gate_depth_m` |
| **2. Pass Gate** | `pass_gate_duration_s: 8.0` | `pass_gate_forward_speed_mps: 0.6`, `pass_gate_lateral_speed_mps: 0.0` | Straight through gate |
| **3. Approach Pole** | `approach_pole_duration_s: 8.0` | `approach_pole_forward_speed_mps: 0.6`, `approach_pole_lateral_speed_mps: 0.0` | Straight from gate to pole |
| **4a. Circle Turn 1** | `circle_turn_1_duration_s: 3.14` | `circle_turn_1_yaw_rate_rad_s: -0.5` | `-0.5 rad/s` = right turn ( kicks off clockwise box) |
| **4b. Circle Leg 1** | `circle_forward_1_duration_s: 4.0` | `circle_forward_1_speed_mps: 0.5`, `circle_forward_1_lateral_speed_mps: 0.0` | Side 1 of box |
| **4c. Circle Turn 2** | `circle_turn_2_duration_s: 3.14` | `circle_turn_2_yaw_rate_rad_s: 0.5` | `+0.5 rad/s` = left turn around pole |
| **4d. Circle Leg 2** | `circle_forward_2_duration_s: 5.0` | `circle_forward_2_speed_mps: 0.5`, `circle_forward_2_lateral_speed_mps: 0.0` | Side 2 of box |
| **4e. Circle Turn 3** | `circle_turn_3_duration_s: 3.14` | `circle_turn_3_yaw_rate_rad_s: 0.5` | `+0.5 rad/s` = left turn around pole |
| **4f. Circle Leg 3** | `circle_forward_3_duration_s: 5.0` | `circle_forward_3_speed_mps: 0.5`, `circle_forward_3_lateral_speed_mps: 0.0` | Side 3 of box |
| **4g. Circle Turn 4** | `circle_turn_4_duration_s: 3.14` | `circle_turn_4_yaw_rate_rad_s: 0.5` | `+0.5 rad/s` = left turn around pole |
| **4h. Circle Leg 4** | `circle_forward_4_duration_s: 4.0` | `circle_forward_4_speed_mps: 0.5`, `circle_forward_4_lateral_speed_mps: 0.0` | Side 4 of box |
| **5a. Return Turn** | `return_turn_duration_s: 3.14` | `return_turn_yaw_rate_rad_s: 0.5` | Align heading back toward gate |
| **5b. Return to Gate**| `return_to_gate_duration_s: 16.0` | `return_to_gate_forward_speed_mps: 0.6`, `return_to_gate_lateral_speed_mps: 0.0` | Drive back through gate |

> **Tip:** Setting any segment's `*_duration_s` to `0.0` immediately skips that segment with `SUCCESS` (no code or XML edits needed). Because `banyu_bringup` uses `--symlink-install`, edits to `src/BANYU_ROBOTX/banyu_bringup/config/por_params.yaml` take effect immediately on the next `ros2 launch` without rebuilding.

---

## 7. Emergency Stop & Manual Takeover Procedures

At any point during the run, you can interrupt or stop the AUV immediately:

1. **TX16S `SB` → Middle (`-`, Remote/Manual):**
   - Immediately sets `operational_mode = 1` (`MANUAL`), triggering `vehicle_execution_node`'s safety fault, aborting all active goals, and zeroing velocity setpoints. You can then fly manually using `SE` (`MANUAL` / `ALT_HOLD` / `STABILIZE`).
2. **TX16S `SB` → Down (`↓`, Kill):**
   - Immediately sends `'2'` to the ESP32 over `/dev/ttyUSB0` (cutting thruster relay power), sets `hardware_kill_active = true`, and disarms the Pixhawk.
3. **Software Abort Service:**
   ```bash
   bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
     ros2 service call /banyu/mission/abort std_srvs/srv/Trigger "{}"'
   ```
   - Triggers the BehaviorTree abort branch, cancels the active action, and transitions the vehicle to `fallback_mode` (`SURFACE`).

