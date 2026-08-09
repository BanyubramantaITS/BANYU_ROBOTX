# AGENTS.md — BANYU_ROBOTX

ROS 2 Humble underwater ROV autonomy stack: ArduSub SITL + Gazebo Harmonic + MAVROS + BehaviorTree.CPP mission manager. Workspace root: `/home/gryd/Dev/ros2_ws_robotx`. ArduPilot source (non-ROS, read-only reference) lives at `/home/gryd/Dev/ardupilot_ws/ardupilot`.

## Architecture (non-obvious wiring)

- Packages: `banyu_bringup` (launch + config), `banyu_simulation` (worlds/models), `banyu_teleop` (gamepad → `mavros/manual_control/send`), `banyu_msgs` (`Detection.msg`), `banyu_mission` (BT mission manager), `banyu_detection` (**fresh scaffold, WIP — empty impl**).
- `proof_of_readiness.launch.py` is the main sim entrypoint: spawns `bluerov2_heavy` (from `model.xacro`, NOT `.urdf` — the install tree has a stale dangling `model.urdf` symlink, ignore it), `por_pool`, `Gate`, `yellow_buoy`, runs SITL + MAVROS + `banyu_mission`.
- Camera: gz topic `front_camera/image_raw` → `ros_gz_image image_bridge` → ROS `/front_camera/image_raw` (30 Hz). `camera/camera_info` is NOT bridged. Never modify the `front_camera` sensor block in `bluerov2_gz/models/bluerov2_heavy/model.xacro` — user-configured.
- Frames: Gazebo world is **ENU**; MAVROS `PositionTarget` is **NED** (z down). `NavToGate` captures origin from `/mavros/local_position/pose` and streams absolute `FRAME_LOCAL_NED` position setpoints (`type_mask 0xDF8`); velocity-only = `0xDC7`. ArduSub accepts `setpoint_raw/local` **only in GUIDED**, and setpoints must stream continuously (~10–50 Hz), not once.
- Ports: MAVROS on `udp://127.0.0.1:14551`, SITL exports `14550` for QGC (works fine as a second GCS). In GUIDED, ArduSub ignores MANUAL_CONTROL RC overrides, so teleop's abort role is mode-switch only.
- Setpieces: Gate at x≈−4 (posts Y=±1), `yellow_buoy` at x≈6 z≈−2.1, sub spawns (−7, 0, −2). `mav.parm` at workspace root is an auto-dumped param reference.

## Build / run (shell is zsh!)

- ROS `setup.bash` uses `BASH_SOURCE`, which is unset in zsh → sourcing it resolves the prefix to CWD and errors. Always run ROS commands via: `bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && <cmd>'` (from workspace root; install is symlink-install).
- Focused build: `colcon build --packages-select banyu_msgs banyu_mission banyu_detection` (from workspace root). Build `banyu_msgs` before dependents.
- No unit tests; only `ament_lint_auto` (copyright/cpplint disabled in every CMakeLists — keep that pattern).
- Launch-description sanity check without booting sim: `ros2 launch banyu_bringup proof_of_readiness.launch.py --show-args`.

## Mission manager gotchas (`banyu_mission`)

- Uses **core `behaviortree_cpp` 4.9.0 only** (apt `ros-humble-behaviortree-cpp`; headers at `/opt/ros/humble/include/behaviortree_cpp`). `behaviortree_ros2` is NOT packaged for Humble and unnecessary here — do not add it.
- BT4 API differs from v3: `NodeStatus` is `enum class`; `BehaviorTreeFactory` has **no `setup()`**; register with `factory.registerNodeType<T>("ID", extraArgs...)`; run via `tree.tickWhileRunning()` in a `std::thread`; Groot2 monitor via `Groot2Publisher` (default port param 1666).
- `shared_from_this()` in the ctor throws `bad_weak_ptr` — `BanyuMission` uses two-phase init: `main()` calls `node->initialize()` after `std::make_shared`.
- Tree `bt/mission_tree.xml`: `ReactiveFallback[AbortedOrTeleop | ReactiveSequence[MissionActive→NavToGate→GateAlign→GatePass→SearchMarker→MarkerAlign→MarkerOrbit→ReturnToStart]]`. `NavToGate`/`ReturnToStart` are real; the middle five are `StubHold` (zero-velocity hold). `AbortedOrTeleop` only reacts to the `/banyu/mission/abort` service — auto-abort on mode-change/takeback is a known gap.
- BT nodes don't use blackboard for data flow; they share a mutex-protected `MissionContext` (pose/state/detection). `/banyu/detection` contract: one best Detection per frame; `object_id` 0=gate, 1=yellow buoy; `offset_x/y` normalized ±1, +y = image-down.

## Repo-specific build/launch traps

- `banyu_msgs/package.xml`: `<member_of_group>rosidl_interface_packages</member_of_group>` must be a **top-level** element, NOT inside `<export>` — Humble's `package_xml_2_cmake.py` only fills `MEMBER_OF_GROUPS` from top level, otherwise `rosidl_generate_interfaces` fails.
- `mavros_node` must be launched via `ExecuteProcess` (`ros2 run mavros mavros_node`) — a launch `Node` action injects `-r __node:=mavros` which triggers mavlink/mavros#1977 (crashes on `CompanionProcessStatus`). Do not "fix" this. `system_id: 255` in `mavros_config.yaml` is required for ArduSub to accept MANUAL_CONTROL.
- ArduPilot param names in this checkout are modern: `WP_SPD` (NOT `WPNAV_SPEED`), `WP_SPD_UP/DN`, `PILOT_SPEED_UP/DN`, `FS_EKF_ACTION`. Verify against `ArduSub/Parameters.cpp` / `libraries/AC_WPNav`. Curated file: `banyu_bringup/config/banyu_defaults.parm`, applied via `--defaults` (bare ardusub) and `--add-param-file=<path>` (sim_vehicle).
- ArduSub has no `MAV_CMD_DO_ORBIT`/autonomous CIRCLE — orbit must be hand-rolled tangent-velocity setpoints. Type-mask dispatch lives in `ArduSub/GCS_MAVLink_Sub.cpp` (`0xDF8` position-only → `guided_set_destination`, `0xDC7` velocity-only → `guided_set_velocity`).
- GZ mesh resource paths depend on hooks: `banyu_simulation/hooks/gazebo_env.dsv` and `bluerov2_gz/hooks/setup.dsv` (`GZ_SIM_RESOURCE_PATH` incl. `share`) — don't drop them.

## Git

- `src/BANYU_ROBOTX` is its own git repo; `ardupilot_gazebo` and `bluerov2_gz` are separate repos (pinned via `import.repos`). Commit changes only inside BANYU_ROBOTX.
