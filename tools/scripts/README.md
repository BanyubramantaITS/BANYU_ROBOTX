# tools/scripts

Operator and developer scripts, grouped by what they do. Run them from the workspace root
(`~/ros_ws` on the Pi), e.g. `./src/BANYU_ROBOTX/tools/scripts/check/check_devices.sh`.
Hardware context: [`docs/hardware_setup.md`](../../docs/hardware_setup.md).

| Folder | Purpose | Scripts |
|---|---|---|
| `run/` | Start and keep running during a session | `run_mavproxy.sh`, `launch_mavros.sh`, `tmux_panes.sh` (2x4 pane tmux window), `run_gps_bridge.sh` (starts `gps_forwarder.py`: u-blox USB NAV-PVT → Pixhawk `GPS_INPUT`, `GPS1_TYPE 14`; run on its own after MAVProxy), `launch_task2.sh` (Task 2 launcher, `--sim` / `--real`, `--test-mission`), `task2_test_waypoint.py` (test mission: course, run start and the waypoint as USV `PipelineBuoys`, prints the report; `docs/task2_test_mission.md`), `task4_command.py` (Task 4: sends RoboCommand's keep-out zone, all-clear and moving-object commands, or a moving object updated periodically (`moving-sim`); prints the acks and `/banyu/task4/zones`; `docs/task4.md`) |
| `check/` | Health checks and bench tests (read-only, PASS/FAIL) | `check_devices.sh`, `test_hardware.sh` (+ `.py`), `rtk_status.sh`, `gps_receiver.py`, `yolo_test.sh` (+ `.py`), `check_pixhawk_params.py`, `check_thrust_limit.py`, `task2_score_run.py` (scores a Task 2 sim run from Gazebo ground truth) |
| `debug/` | Drive one device by hand | `front_camera.sh`, `bottom_camera.sh`, `camera.sh`, `kill_lamp.sh`, `esp32_kill_cmd.py`, `mock_esp32.py`, `reset_pixhawk_usb.sh` |
| `move/` | Movement tests on the real AUV: one manoeuvre, then disarm ([`docs/movement_tests.md`](../../docs/movement_tests.md)) | `dive.sh`, `forward.sh`, `backward.sh`, `sway.sh`, `yaw.sh`, `waypoint.sh`, `where.sh` (all wrap `auv_move.py`) |
| `sim/` | Simulation only: Task 2 sim runs and experiments | `task2_sim_e2e.sh` (one run, mock GCS), `task2_sim_batch.sh` (one run per light seed, scored), `task2_sim_cleanup.sh`, `gps_loss_mode_experiment.py`, `surface_field_replay.py` (UAV buoy field replay through the surface planner) |
| `setup/` | One-time installation on a computer | `rtk_setup.sh`, `fix_rtk_rover.sh` (teammate script, vendored), `yolo_setup.sh` (Hailo-8) |
| `por/` | Proof of Readiness (finished; kept for reference) | `launch_por.sh`, `dead_thruster_por.py`, `por_hsv_tuner.py`, `por_teach_extract.py`, `por_track_stretch.py`, `por_path_compare.py` |

Bring-up order on the robot: `run/run_mavproxy.sh` →
`run/launch_mavros.sh` → `check/check_devices.sh` → mission launch script.

Scripts in `setup/` change the system as soon as they run (packages, services, venvs); read them
first. Every script answers `--help` without doing anything, except the vendored teammate script
`setup/fix_rtk_rover.sh` (no help; run it only through `setup/rtk_setup.sh`).

New scripts go in the folder that matches their function.

The workspace-root symlinks (`run_mavproxy.sh`,
`launch_mavros.sh`, `launch_por.sh`) point into these folders.
