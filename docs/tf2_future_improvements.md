# Future `tf2` Improvements for BANYU_ROBOTX

Status: **Proposal / Design Note** (not yet implemented).

## 1. Current State

Today `tf2` is barely used outside Gazebo hydrophone geometry:

- **Robot model TF only runs in simulation:** `robot_state_publisher` is launched in `banyu_bringup/launch/gazebo_main.launch.py` and `banyu_bringup/launch/gazebo_por.launch.py`, but **not** in `banyu_bringup/launch/sub_bringup.launch.py`.
- **Single TF consumer:** `banyu_detection/src/pinger_localization_node.cpp` is the only node using `tf2_ros::Buffer` (`lookupTransform("base_link", "hydrophone_...", tf2::TimePointZero)`).
- **Camera frames exist in headers only on hardware:** `banyu_bringup/config/hardware_params.yaml` configures `usb_cam` to stamp `frame_id: "front_camera_optical_frame"` and `frame_id: "bottom_camera_optical_frame"`, but no `/tf_static` publisher links them to `base_link` on the real ROV.
- **Duplicated mounting geometry:**
  - `bluerov2_gz/models/bluerov2_heavy/model.xacro` defines `bottom_camera_joint` (`xyz="0.20 0 0.14"`, `rpy="0 1.343903524 0"`, i.e. 77° forward pitch = `0.227 rad` from vertical).
  - `banyu_bringup/config/task2_params.yaml` and `banyu_bringup/config/hardware_params.yaml` repeat those numbers as `ring_tracker` parameters (`camera_forward_m: 0.2`, `camera_tilt_from_vertical_rad: 0.227`).
  - `banyu_detection/src/ring_tracker.cpp` computes `ahead = config_.camera_forward_m + height * std::tan(angle)`, assuming zero vehicle roll/pitch.
- **Odometry without `/tf`:** `banyu_vehicle_execution/src/underwater_odometry_node.cpp` publishes `nav_msgs/msg/Odometry` (`frame_id: "dive_origin"`, `child_frame_id: "base_link"`), without broadcasting `dive_origin -> base_link` on `/tf`.

---

## 2. Proposed `tf2` Improvements

### 2.1 Publish Robot Static TFs on the Physical AUV (`sub_bringup.launch.py`)
Run `robot_state_publisher` (with a hardware URDF/xacro) or `tf2_ros/static_transform_publisher` in `banyu_bringup/launch/sub_bringup.launch.py` so the real vehicle exposes the same `base_link` tree as simulation:

- `base_link -> front_camera_link -> front_camera_optical_frame`
- `base_link -> bottom_camera_link -> bottom_camera_optical_frame`
- `base_link -> probe_link -> probe_tip` (defined in `banyu_simulation/models/bluerov2_task2/model.xacro`)

**Frame convention reminder:**
- Body frame (`base_link`): `+X` forward, `+Y` left, `+Z` up (FLU).
- Optical frame (`*_optical_frame`): `+Z` forward out of lens, `+X` right, `+Y` down (RDF).
- Standard body-to-optical rotation is `rpy="-1.5708 0 -1.5708"` (plus any physical roll if the hardware camera is mounted rotated).

### 2.2 Use TF for `ring_tracker` Mounting & Pitch/Roll Compensation
Instead of maintaining separate `camera_forward_m` and `camera_tilt_from_vertical_rad` copies in `task2_params.yaml` and `hardware_params.yaml`:

1. Have `ring_tracker_node` look up `base_link -> bottom_camera_optical_frame` via `tf2_ros::Buffer` (falling back to the YAML parameters if TF is unavailable, so unit tests and existing configs keep working).
2. Optionally compose that with live vehicle attitude (roll and pitch from `/mavros/local_position/pose`) when projecting a ring observation's image ray onto the horizontal pipe plane.
   - **Benefit:** Eliminates drift in forward-distance estimates caused by ROV pitching during acceleration/braking, and keeps mounting geometry in one place.
   - **Note:** Camera intrinsics / FOV (`camera_half_vfov_rad`, `focal_ratio`, or `sensor_msgs/msg/CameraInfo`) still belong in `CameraInfo` / YAML—TF only models pose, not lens FOV.

### 2.3 Camera-to-Probe Offset Lookup for Task 2 Ring Repair
`banyu_simulation/models/bluerov2_task2/model.xacro` already models `probe_joint` (`0.27 0 0.14` from `base_link`) and `probe_tip_joint` (`0 0 -0.40` from `probe_link`).

- Looking up `bottom_camera_optical_frame -> probe_tip` in TF gives the exact 3D offset between the camera's optical center and the magnetic probe tip.
- If the camera or probe bracket is relocated on the physical frame, updating the URDF/TF updates alignment geometry automatically without re-deriving `CenterOnRingServer` / `HoldProbeOnRingServer` pixel offsets by trial and error.

### 2.4 Broadcast Dynamic Pose TF for RViz & Bag Replay
Add an optional parameter (e.g. `publish_tf: true/false`) to `underwater_odometry_node` (and/or a lightweight broadcaster for `/mavros/local_position/pose`) using `tf2_ros::TransformBroadcaster`:

- Surface / global view: `map -> base_link`
- Dive view: `dive_origin -> base_link`

**Benefit:** Real pool rosbags and live runs can be inspected in RViz with the BlueROV2 model, camera frames, `probe_tip`, `/banyu/dive/path`, and `Task4Zones` aligned in 3D.

---

## 3. Boundaries: What NOT to Replace with TF

1. **Do not replace `banyu_msgs/msg/Detection` or `banyu_msgs/msg/PipelineObservation` with TF-based target tracking.**
   - `blue_auv`-style `lookupTransform(tag_frame, camera_frame, tf2::TimePointZero)` lacks detection `status` (`TRACKED` / `NO_PIPE`), `confidence`, and explicit lost-target signaling, and depends on noisy underwater monocular 3D PnP.
   - Keep normalized image-space servoing (`offset_x`, `offset_y`, `apparent_size` / `width`, `confidence`, `header.stamp`) for PoR and Task 2 visual alignment.
2. **TF does not fix underwater position drift.**
   - Without a DVL, `dive_origin -> base_link` remains dead-reckoned from velocity commands, depth, and IMU yaw.

---

## 4. Suggested Rollout

1. **Phase 1 (Zero behavior change):** Add `robot_state_publisher` (or static TFs) for `front_camera_optical_frame`, `bottom_camera_optical_frame`, and `probe_tip` to `sub_bringup.launch.py`, plus optional `publish_tf` on `underwater_odometry_node`. Verify the TF tree in RViz (`ros2 run tf2_tools view_frames`) against a real bag.
2. **Phase 2 (`ring_tracker`):** Add optional TF lookup in `ring_tracker_node` with fallback to existing `camera_forward_m` / `camera_tilt_from_vertical_rad` parameters; validate against existing `test_ring_tracker` cases and sim runs.
3. **Phase 3 (Repair alignment):** Derive camera-to-probe offset from `bottom_camera_optical_frame -> probe_tip` when `probe_tip` TF is present.
