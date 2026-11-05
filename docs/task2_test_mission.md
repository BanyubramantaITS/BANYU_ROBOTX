# Task 2 test mission

A short end-to-end check of the real Task 2 run on the AUV, without a pipe. It uses the real run start and the real
waypoint source, and sends the real report, but the survey is replaced by a dive and two timed legs. The main Task 2
mission (`banyu_mission.xml`) is unchanged; the test is selected only with `--test-mission`.

| Step | What the AUV does | BT node / action |
|---|---|---|
| 1 | Waits on the surface for the run start (SB Up, `prepare_autonomy`, GCS `CMD_RUN_START`) | `RequireMissionStarted`, `RequireVehicleReady` |
| 2 | Waits for the waypoint: the USV `PipelineBuoys` active buoy, as in the real run | `WaitForBuoyTarget` |
| 3 | Goes there on the surface by GPS (GUIDED), arrival radius `gps_tolerance_m` 1 m, no standoff | `NavigateSurfaceGps` (retried, ends after `test.transit_timeout_s`) |
| 4 | ALT_HOLD, dives to `test.depth_m` (1 m); done once within 0.2 m for 0.5 s, or after `test.dive_timeout_s` | `TimedMove dive` |
| 5 | Holds still for `test.settle_s`, forward `test.leg_speed_mps` for `test.leg_duration_s`, holds still, then the same leg backward. Depth is held at 1 m and the heading is the one at the start of each leg | `TimedMove settle/forward/pause/backward` |
| 6 | Surfaces, waits until the surface is confirmed, sends the Core `PipelineSurvey` (active buoy = waypoint, all segments GREEN because no ring is seen) | `GiveUpSurvey`, `Surface`, `PublishPipelineSurvey best_effort` |

Steps 4 and 5 never stop the run: if one fails, the AUV still surfaces and reports. SB Mid or SB Down takes over at
any time, as in every run.

Tuning: `banyu_bringup/config/task2_test_mission.yaml` (loaded last when `test_mission:=True`).

## Real AUV

```bash
# Pi: the usual order
./run_mavproxy.sh
./tools/scripts/run/launch_mavros.sh
./tools/scripts/run/launch_task2.sh --real --test-mission       # records to ~/banyu_bags/<date>_task2_test_full/

# Check the GPS fix (./tools/scripts/check/rtk_status.sh), then flick SB Up (AUTO).
# Laptop or Pi, on the UUV domain:
ROS_DOMAIN_ID=40 ./tools/scripts/run/task2_test_waypoint.py <LAT> <LON>
```

`task2_test_waypoint.py` does the GCS and USV parts so that a single command starts the run:

1. waits until `/banyu/vehicle_status` shows SB Up,
2. publishes a course (`BANYU-TEST`, a 300 m square around the waypoint) about once per second,
3. calls `prepare_autonomy`, waits for `READY` and sends `CMD_RUN_START` (run 1, seq 1; `prepare_autonomy` opens a
   new run identity window, so run 1 / seq 1 works for every test),
4. publishes the waypoint as the USV `PipelineBuoys` (buoy A, active A),
5. prints the lifecycle, the UUV heartbeat (position, depth) and the `PipelineSurvey` when the AUV sends it.

When the real GCS or USV is on the link, use `--no-course`, `--no-start`, or leave out `LAT LON`.

## Simulation

```bash
./tools/scripts/run/launch_task2.sh --sim --test-mission --record
./tools/scripts/run/launch_mavros.sh --sim
ROS_DOMAIN_ID=40 ./tools/scripts/run/task2_test_waypoint.py --course-center 33.810313 -118.393866
```

In the sim leave out `LAT LON`: `task2_sim_provider.py` acts as the USV and sends the world's start buoy as the waypoint.

## After the run

Every real run is recorded (`--no-record` / `--no-record-images` to change that). The bag has all topics, including
action feedback and `/rosout`. `params/` holds `task2_params.yaml` and every overlay the run loaded, among them
`task2_test_mission.yaml`.

| What to check | Topic |
|---|---|
| BT steps (`[BT] Executing/Completed`), `[TimedMove] Starting 'forward' ...` | `/rosout` |
| Run phases and reasons | `/banyu/mission/run_lifecycle` |
| Depth during the dive and the legs (feedback `depth_m`, `elapsed_s`), results | `/banyu/actions/timed_move/_action/feedback`, `.../status` |
| Surface transit (distance to the goal) | `/banyu/actions/navigate_surface_gps/_action/feedback` |
| Calibrated depth | `/banyu/vehicle/water_depth` |
| Commanded sticks / modes | `/mavros/rc/override`, `/mavros/state`, `/banyu/vehicle_status` |
| Waypoint received and locked | `/system/vehicle/usv/task2/pipeline_buoys`, `/banyu/task2/buoy_target` |
| Report sent | `/system/vehicle/uuv/task2/pipeline_survey`, `/system/vehicle/uuv/heartbeat` |
