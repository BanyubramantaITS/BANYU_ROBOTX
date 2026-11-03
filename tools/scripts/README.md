# tools/scripts

Operator and developer scripts, grouped by what they do. Run them from the workspace root
(`~/ros_ws` on the Pi), e.g. `./src/BANYU_ROBOTX/tools/scripts/check/check_devices.sh`.
Hardware context: [`docs/hardware_setup.md`](../../docs/hardware_setup.md).

| Folder | Purpose | Scripts |
|---|---|---|
| `run/` | Start and keep running during a session | `run_mavproxy.sh`, `launch_mavros.sh`, `run_gps_bridge.sh` |
| `check/` | Health checks and bench tests (read-only, PASS/FAIL) | `check_devices.sh`, `test_hardware.sh` (+ `.py`), `rtk_status.sh`, `yolo_test.sh` (+ `.py`), `check_pixhawk_params.py`, `check_thrust_limit.py` |
| `debug/` | Drive one device by hand | `front_camera.sh`, `bottom_camera.sh`, `camera.sh`, `kill_lamp.sh`, `esp32_kill_cmd.py`, `mock_esp32.py`, `reset_pixhawk_usb.sh` |
| `setup/` | One-time installation on a computer | `rtk_setup.sh`, `fix_rtk_rover.sh` (teammate script, vendored), `yolo_setup.sh` (Hailo-8) |
| `por/` | Proof of Readiness (finished; kept for reference) | `launch_por.sh`, `dead_thruster_por.py`, `por_hsv_tuner.py`, `por_teach_extract.py`, `por_track_stretch.py`, `por_path_compare.py` |

Bring-up order on the robot: `run/run_mavproxy.sh` → `run/run_gps_bridge.sh` →
`run/launch_mavros.sh` → `check/check_devices.sh` → mission launch script.

Scripts in `setup/` change the system as soon as they run (packages, services, venvs); read them
first. Every script answers `--help` without doing anything, except the vendored teammate script
`setup/fix_rtk_rover.sh` (no help; run it only through `setup/rtk_setup.sh`).

New scripts go in the folder that matches their function.

**Merging the Task 2 branch** (`feat/task2-core-disruptive-sim` still has the old flat layout):
after the merge, move its scripts into the folders and apply the same path fix (`SCRIPT_DIR/../..`
→ `SCRIPT_DIR/../../..` for anything that finds the repo or the config):

| Script on the Task 2 branch | Goes to |
|---|---|
| `launch_task2.sh` | `run/` (also: check `/dev/esp32_kill`, `/dev/gps_rtk`, `/dev/bottom_camera`, and that `run/run_gps_bridge.sh` is running) |
| `task2_sim_e2e.sh`, `task2_sim_batch.sh`, `task2_sim_cleanup.sh`, `gps_loss_mode_experiment.py` | new `sim/` (simulation runs and experiments) |
| `task2_score_run.py` | `check/` |

Files that exist on both branches (`run_mavproxy.sh`, `check_*.py`, `esp32_kill_cmd.py`, the POR
tools, ...) follow the rename automatically when git detects it; check `git status` after the merge. The workspace-root symlinks (`run_mavproxy.sh`,
`launch_mavros.sh`, `launch_por.sh`) point into these folders.
