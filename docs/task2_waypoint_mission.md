# Task 2 Waypoint Navigation & Communication Test Mission

A specialized test mission designed specifically to verify **surface GPS waypoint navigation** and **inter-vehicle communication** (USV → UUV → UAV / GCS) without diving into the water.

## Mission Overview

| Step | What the AUV does | BT Node / Action |
|---|---|---|
| 1 | Starts automatically when **SB is flicked to AUTO (Up)**, or on GCS `CMD_RUN_START` / `prepare_autonomy` | `RequireMissionStarted`, `RequireVehicleReady` |
| 2 | Stands by on the surface waiting for the USV's `rx_msgs/PipelineBuoys` | `WaitForBuoyTarget` |
| 3 | Transits by GPS on the surface midway between the buoys (midpoint strategy), keeping clear of both buoys | `NavigateSurfaceGps` |
| 4 | Aligns heading toward the end buoy, holding GPS position | `AlignHeading` |
| 5 | Marks survey complete and surfaced (without diving) | `GiveUpSurvey`, `ConfirmSurfaceAction` |
| 6 | Publishes the survey report and resource delivery (`rx_msgs/ResourceDelivery` on `/system/vehicle/uuv/task2/delivery`, relayed to UAV) | `PublishPipelineSurvey best_effort="true"` |
| 7 | Mission succeeds and disarms thrusters, remaining safely floating on the surface | Success / Completion |

---

## How to Run

### 1. On the Real AUV (Physical Hardware)

```bash
# Terminal 1: MAVProxy
./run_mavproxy.sh

# Terminal 2: GPS Forwarder
./src/BANYU_ROBOTX/tools/scripts/run/run_gps_bridge.sh

# Terminal 3: MAVROS
./src/BANYU_ROBOTX/tools/scripts/run/launch_mavros.sh

# Terminal 4: Launch Waypoint Mission
ROS_DOMAIN_ID=40 ./src/BANYU_ROBOTX/tools/scripts/run/launch_task2.sh --real --waypoint-mission
```

#### Starting the run:
- **Option A (RC switch only - automatic start):**
  Check RTK GPS fix (`./tools/scripts/check/rtk_status.sh`), then **flick SB Up (AUTO)**. The vehicle automatically prepares, waits for the USV's `PipelineBuoys`, navigates to the midpoint, and publishes the reports for the UAV/GCS.
- **Option B (With helper script / GCS stand-in):**
  If testing without a live USV, send the buoys and monitor the report with:
  ```bash
  ROS_DOMAIN_ID=40 ./src/BANYU_ROBOTX/tools/scripts/run/task2_test_waypoint.py <start LAT> <start LON> --end <end LAT> <end LON>
  ```
- **Option C (started by RoboCommand, reports to RoboCommand):**
  Options A and B start run 1 on the AUV itself, and RoboCommand's Run Start is then rejected (`NOT_READY` or
  `RUN_CONFLICT`), so the reports belong to a run RoboCommand never declared. Launch with `--no-auto-start`
  (`auto_start:=False`, overlay `task2_no_auto_start.yaml`): SB Up only arms the switch. Then call
  `ros2 service call /banyu/mission/prepare_autonomy std_srvs/srv/Trigger "{}"`, wait for READY and press Run Start in
  RoboCommand; the OCS forwards it as `CMD_RUN_START`. Without the USV, send only the buoys:
  ```bash
  ROS_DOMAIN_ID=40 ./src/BANYU_ROBOTX/tools/scripts/run/launch_task2.sh --real --waypoint-mission --no-auto-start
  ROS_DOMAIN_ID=40 ./src/BANYU_ROBOTX/tools/scripts/run/task2_test_waypoint.py <start LAT> <start LON> --end <end LAT> <end LON> --no-start --no-course
  ```

---

### 2. In Simulation (Gazebo Harmonic + ArduSub SITL)

```bash
# Terminal 1: Launch Stack
./src/BANYU_ROBOTX/tools/scripts/run/launch_task2.sh --sim --waypoint-mission

# Terminal 2: Launch MAVROS
./src/BANYU_ROBOTX/tools/scripts/run/launch_mavros.sh --sim

# Terminal 3: Run waypoint test sender
ROS_DOMAIN_ID=40 ./src/BANYU_ROBOTX/tools/scripts/run/task2_test_waypoint.py --course-center 33.810313 -118.393866
```

---

## Verifying Topics

During and after the mission:
- **USV Buoys received:** `/system/vehicle/usv/task2/pipeline_buoys` and `/banyu/task2/buoy_target`
- **Surface transit progress:** `/banyu/actions/navigate_surface_gps/_action/feedback`
- **Heading alignment:** `/banyu/actions/align_heading/_action/feedback`
- **Report published to GCS & UAV:**
  - Pipeline survey: `/system/vehicle/uuv/task2/pipeline_survey`
  - UAV Resource delivery: `/system/vehicle/uuv/task2/delivery` (relayed by GCS to UAV as `CMD_DELIVERY`)
