# Task 2 test mission

A short end-to-end check of the real Task 2 run on the AUV, without a pipe. It uses the real run start, the real buoy
source (the USV's start and end buoys) and the real midpoint dive point, and sends the real report, but the survey is
replaced by a dive and two timed legs. The main Task 2
mission (`banyu_mission.xml`) is unchanged; the test is selected only with `--test-mission`.

| Step | What the AUV does | BT node / action |
|---|---|---|
| 1 | Waits on the surface for the run start (SB Up, `prepare_autonomy`, GCS `CMD_RUN_START`) | `RequireMissionStarted`, `RequireVehicleReady` |
| 2 | Waits for the USV `PipelineBuoys` (start = active buoy, and the end buoy), as in the real run | `WaitForBuoyTarget` |
| 3 | Goes midway between the buoys on the surface by GPS (GUIDED), keeping `buoy_keep_out_m` (1.2 m) clear of both, arrival radius `gps_tolerance_m` (0.4 m): the real `dive_point:=midpoint`. With only the start buoy it goes `buoy_standoff_m` inside it, like the real run | `NavigateSurfaceGps` (retried, ends after `test.transit_timeout_s`) |
| 4 | Turns to face the end buoy (start→end bearing), holding its GPS position | `AlignHeading` (60 s limit) |
| 5 | ALT_HOLD, dives to `test.depth_m` (1 m); done once within 0.2 m for 0.5 s, or after `test.dive_timeout_s` | `TimedMove dive` |
| 6 | Holds still for `test.settle_s`, forward (toward the end buoy) `test.leg_speed_mps` 0.3 m/s for `test.leg_duration_s` 6 s (~1.8 m), holds still, then the same leg backward. Depth is held at 1 m and the heading is the one at the start of each leg | `TimedMove settle/forward/pause/backward` |
| 7 | Surfaces, waits until the surface is confirmed, sends the Core `PipelineSurvey` (the active buoy's position, all segments GREEN because no ring is seen) | `GiveUpSurvey`, `Surface`, `PublishPipelineSurvey best_effort` |

Steps 4 to 6 never stop the run: if one fails, the AUV still surfaces and reports. SB Mid or SB Down takes over at
any time, as in every run.

Tuning: `banyu_bringup/config/task2_test_mission.yaml` (loaded last when `test_mission:=True`). Keep the forward leg well
short of half the buoy spacing: the end buoy's mooring line is at the dive depth.

## Real AUV

```bash
# Pi: the usual order
./run_mavproxy.sh
./tools/scripts/run/launch_mavros.sh
./tools/scripts/run/launch_task2.sh --real --test-mission       # records to ~/banyu_bags/<date>_task2_test_full/

# Check the GPS fix (./tools/scripts/check/rtk_status.sh), then flick SB Up (AUTO).
# Laptop or Pi, on the UUV domain:
ROS_DOMAIN_ID=40 ./tools/scripts/run/task2_test_waypoint.py <start LAT> <start LON> --end <end LAT> <end LON>
```

`task2_test_waypoint.py` does the GCS and USV parts so that a single command starts the run:

1. waits until `/banyu/vehicle_status` shows SB Up,
2. publishes a course (`BANYU-TEST`, a 300 m square around the buoys) about once per second,
3. calls `prepare_autonomy`, waits for `READY` and sends `CMD_RUN_START` (run 1, seq 1; `prepare_autonomy` opens a
   new run identity window, so run 1 / seq 1 works for every test),
4. publishes the buoys as the USV `PipelineBuoys` (start = buoy A, active; `--end` = buoy B),
5. prints the lifecycle, the UUV heartbeat (position, depth) and the `PipelineSurvey` when the AUV sends it.

When the real GCS or USV is on the link, use `--no-course`, `--no-start`, or leave out the buoys.

## Simulation

```bash
./tools/scripts/run/launch_task2.sh --sim --test-mission --record
./tools/scripts/run/launch_mavros.sh --sim
ROS_DOMAIN_ID=40 ./tools/scripts/run/task2_test_waypoint.py --course-center 33.810313 -118.393866
```

In the sim leave out the buoys: `task2_sim_provider.py` acts as the USV and sends the world's two buoys.

## After the run

Every real run is recorded (`--no-record` / `--no-record-images` to change that). The bag has all topics, including
action feedback and `/rosout`. `params/` holds `task2_params.yaml` and every overlay the run loaded, among them
`task2_test_mission.yaml`.

| What to check | Topic |
|---|---|
| BT steps (`[BT] Executing/Completed`), `[TimedMove] Starting 'forward' ...` | `/rosout` |
| Run phases and reasons | `/banyu/mission/run_lifecycle` |
| Depth during the dive and the legs (feedback `depth_m`, `elapsed_s`), results | `/banyu/actions/timed_move/_action/feedback`, `.../status` |
| Surface transit (distance to the goal), turn to the end buoy | `/banyu/actions/navigate_surface_gps/_action/feedback`, `/banyu/actions/align_heading/_action/feedback` |
| Calibrated depth | `/banyu/vehicle/water_depth` |
| Commanded sticks / modes | `/mavros/rc/override`, `/mavros/state`, `/banyu/vehicle_status` |
| Buoys received, midpoint goal locked | `/system/vehicle/usv/task2/pipeline_buoys`, `/banyu/task2/buoy_target` |
| Report sent | `/system/vehicle/uuv/task2/pipeline_survey`, `/system/vehicle/uuv/heartbeat` |
