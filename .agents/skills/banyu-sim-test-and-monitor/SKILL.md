---
name: banyu-sim-test-and-monitor
description: Launch, test, and monitor underwater ROV simulation in Gazebo Harmonic + ArduSub SITL + MAVROS on ROS 2 Jazzy, including topic health checks and mission verification
---

# BANYU Simulation Testing & Performance Monitor

This skill guides running full or headless simulations, triggering autonomous missions, and monitoring topics and performance for the BANYU ROV stack on ROS 2 Jazzy.

## Pre-flight Checklist (Mandatory)

Before launching any simulation or mission node, stale processes must be eliminated to prevent port 5501 / MAVLink bind errors:

```bash
# Clean up any orphaned ArduSub or MAVProxy processes
killall -9 ardusub mavproxy mavros_node 2>/dev/null || true
pkill -9 -f "sim_vehicle.py" 2>/dev/null || true
sleep 1
```

Always execute ROS 2 commands within bash sourcing the workspace (ROS 2 Jazzy):
```bash
bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && <command>'
```

---

## 1. Launching the Simulation

### Option A: Full Gazebo Harmonic + SITL + Mission Composition
To launch the full visual environment, vehicle model, SITL, and perception pipeline:
- **MAVROS is never started by a launch file.** Start it separately (second terminal / background job) and restart it whenever the sim restarts (the sim clock resets):
  ```bash
  ./src/BANYU_ROBOTX/tools/scripts/run/launch_mavros.sh --sim
  ```
- **Proof of Readiness (POR):** GPS-denied like the robot (`use_gps:=False` also starts `nogps_odom_beacon`). Preferred: `env -u TMUX ./src/BANYU_ROBOTX/tools/scripts/por/launch_por.sh --sim --profile <name>` (docs/por/README.md).
  ```bash
  bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch banyu_bringup main_launch.launch.py mode:=por use_gps:=False'
  ```
- **Task 2 (Pipeline & Rings):**
  ```bash
  bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch banyu_bringup main_launch.launch.py mode:=task2'
  ```

### Option B: Mission Stack Only (Headless / Deterministic Fixtures)
For rapid automated testing without GUI overhead:
- **Proof of Readiness (POR):**
  ```bash
  bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch banyu_bringup mission.launch.py mode:=por'
  ```
- **Task 2 (Pipeline Inspection & Ring Repair):**
  ```bash
  bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch banyu_bringup mission.launch.py mode:=task2 use_sim:=True'
  ```

---

## 2. Triggering the Mission

Once MAVROS is connected and the behavior tree is loaded:

### For POR Mode:
```bash
# Check if MAVROS is receiving telemetry and connected to ArduSub
ros2 topic echo /mavros/state --once

# Trigger autonomous mission start
ros2 service call /banyu/mission/start std_srvs/srv/Trigger "{}"

# If needed: Trigger mission abort
# ros2 service call /banyu/mission/abort std_srvs/srv/Trigger "{}"
```

### For Task 2 Mode (RobotX SoS Interface):
Task 2 uses the RobotX SoS protocol rather than a simple `/banyu/mission/start` trigger:
1. Operator calls prepare autonomy:
   ```bash
   ros2 service call /banyu/mission/prepare_autonomy std_srvs/srv/Trigger "{}"
   ```
2. The run starts on GCS `CMD_RUN_START` published on `/system/mission/command` (or via `tools/sos_integration/` fixtures).

---

## 3. Topic Health & Performance Monitoring

Monitor key publishers to verify publishing rates, latency, and data integrity:

### Rate Verification (`ros2 topic hz`)
Run these to ensure critical control and perception loops meet frequency requirements:
- **Camera Bridge**: `ros2 topic hz /front_camera/image_raw` (target: ~30 Hz)
- **MAVROS Local Position**: `ros2 topic hz /mavros/local_position/pose` (target: 20–50 Hz)
- **Velocity Setpoint Stream (Task 2, GUIDED)**: `ros2 topic hz /mavros/setpoint_raw/local` (target: 10–50 Hz continuous; must NEVER drop to 0 in GUIDED)
- **RC Override Stream (POR, ALT_HOLD)**: `ros2 topic hz /mavros/rc/override` (target: ~20 Hz while armed in ALT_HOLD; channels 2/1/3/4 = forward/lateral/throttle/yaw)
- **POR Perception**: `ros2 topic hz /banyu/detection` (target: ~10–30 Hz when target is in frame)
- **Task 2 Pipeline Perception**: `ros2 topic hz /banyu/pipeline/observation`
- **Vehicle Status**: `ros2 topic hz /banyu/vehicle_status` (target: ~10 Hz)
- **RobotX Comm Heartbeat**: `ros2 topic hz /system/vehicle/uuv/heartbeat` (target: ~2 Hz)

### Control Loop & State Inspections
- **Setpoint Quality & Explicit Zero Yaw Rate**:
  ```bash
  # Check if velocity setpoints have explicit zero yaw rate (type_mask 0x5C7, ENU frame)
  ros2 topic echo /mavros/setpoint_raw/local --once
  ```
- **Vehicle Status & Active Action**:
  ```bash
  ros2 topic echo /banyu/vehicle_status --once
  ```
- **Preview Target & Guidance Intent**:
  ```bash
  ros2 topic echo /banyu/execution/preview_target --once
  ros2 topic echo /banyu/guidance/velocity_intent --once
  ```

---

## 4. Automated Smoke / Unit Tests

Run isolated unit tests without crosstalk from live simulation DDS traffic:

```bash
ROS_DOMAIN_ID=42 ./build/banyu_vehicle_execution/test_vehicle_execution
```

---

## 5. Teardown & Post-Run Analysis

When testing finishes, tear down all launched nodes and check logs:
```bash
killall -9 ardusub mavproxy mavros_node 2>/dev/null || true
pkill -9 -f "ros2 launch" 2>/dev/null || true
```
Check if ArduSub logged errors such as port collisions (`ArduPilot controller has reset` or `bind failed`).
