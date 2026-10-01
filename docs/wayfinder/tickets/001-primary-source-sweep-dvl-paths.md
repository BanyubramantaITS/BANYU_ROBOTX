# 001 — Primary-source sweep: what DVL paths actually exist

`wayfinder:research` · parent: [Map: DVL in sim, fused by ArduSub EKF3](../map-dvl-sim-ekf3.md)
· status: **CLOSED** · assignee: research subagent
· blocks: 002, 003, 006, 007 — all now unblocked

## Question

Against primary sources only (upstream source, official docs, protocol specs),
establish for this stack:

1. **ArduSub side** — which MAVLink messages ArduSub actually consumes for
   body-frame velocity (`VISION_POSITION_DELTA`, `VISION_SPEED_ESTIMATE`,
   `ODOMETRY`, …), where they are handled in ArduPilot source, and the exact
   parameter set that routes them into EKF3 (`VISO_*`, `EK3_SRC1_*`,
   `AHRS_EKF_TYPE`). Real param names verified against this checkout's source —
   AGENTS.md warns the names here are modern.
2. **Gazebo Harmonic side** — confirm the `DopplerVelocityLog` sensor in
   gz-sensors/gz-sim 8: exact SDF element name, required parameters, published
   message type and topic naming, from upstream source.
3. **The bridge** — whether `ros_gz_bridge` supports the DVL message type at
   all. If it does not, say so plainly; that turns 005 into real work.
4. **ardupilot_gazebo** — whether the pinned plugin has any visual-odometry path
   back into SITL, or whether data must go ROS → MAVROS → ArduSub.
5. **MAVROS** — which plugin emits the message from (1), its ROS topic and type,
   and exactly what frame conversion MAVROS does and does not perform.

Also cover real-hardware DVL interfaces at a survey level — **not** because
hardware is in scope (it is not), but so the sim sensor model isn't shaped in a
way that would be absurd against a real A50.

## Notes

Findings land in `src/docs/dvl-integration-research.md`. Everything unverifiable
must be marked UNVERIFIED rather than smoothed over — several tickets downstream
are blocked on facts, not guesses.

## Resolution

Full findings: [`src/docs/dvl-integration-research.md`](../../dvl-integration-research.md) (869 lines).

Five results that changed the map:

1. **The sim can't prove anything by default.** SITL synthesises GPS from Gazebo
   ground truth (`SIM_GPS1_ENABLE`, `libraries/SITL/SITL.cpp` L730-737), so
   ArduSub already has a perfect position estimate and EKF3 has no reason to
   want a DVL. No DVL test here is meaningful until synthetic GPS is off. → new
   ticket 010, and it gates the destination.
2. **Message choice is forced, and MAVROS can't make it.**
   `VISION_SPEED_ESTIMATE` and `ODOMETRY`-twist feed `writeExtNavVelData`, which
   is *not* among the conditions moving EKF3 out of `AID_NONE`
   (`AP_NavEKF3_Control.cpp` L283-299). Only `VISION_POSITION_DELTA` bootstraps
   aiding from a DVL alone — which is why BlueRobotics' driver defaults to it.
   **Verified locally:** MAVROS 2.14.0 exposes 40 plugins
   (`/opt/ros/humble/share/mavros_extras/mavros_plugins.xml`) and none emits
   `VISION_POSITION_DELTA`. → rewrites 002.
3. **`ros_gz_bridge` on Humble cannot bridge the gz DVL sensor.** The installed
   `libros_gz_bridge_lib.so` lists 49 gz types, none DVL; upstream `humble`
   `mappings.py` has no "dvl". The mapping exists only from `jazzy`
   (`Mapping('Dvl','DVLVelocityTracking')`). Either backport (~1 file) or copy
   DAVE's standalone `DVLBridge.cc` (~200 lines). → sizes 005.
4. **`ardupilot_gazebo` has no back-door.** Its FDM JSON carries only
   imu/gyro/accel_body/position/quaternion/velocity/rng_1..6/windvane. Route (b)
   is dead; DVL goes ROS → MAVLink → ArduSub exactly as on hardware.
5. **Two live landmines in this repo, independent of DVL work.** The
   `mavros_config.yaml` denylist entry `vision_speed_estimate` does not match the
   actual plugin name `vision_speed`, so the deny is a **no-op** and the plugin
   loads (**verified locally** against the installed plugin XML). And the
   `odometry` plugin needs `map_ned`/`base_link_frd` TF frames MAVROS does not
   publish, where `lookup_static_transform` swallows the exception and leaves an
   uninitialised `Affine3d` for `odom_cb` — missing frames yield garbage, not a
   clean failure. → new ticket 011.

**On the AGENTS.md nav collision:** a DVL changes the feedback term, not the
actuation. It makes the existing velocity-loop design viable on hardware and
argues *harder* for it — `AID_RELATIVE` has no EKF origin, so absolute position
targets are meaningless. The out-of-scope ruling on nav rework stands, and the
research explicitly flags the temptation to retry `0xDF8` as a trap.

**Reframing worth stating plainly:** the DVL's real job is hardware enablement.
This map's sim-only destination is legitimate as a derisking exercise, but 010
is what keeps it honest rather than theatre.

### Marked UNVERIFIED by the agent

- Whether `waterlinked/waterlinked_dvl` (first-party ROS 2 driver, publishes
  `nav_msgs/Odometry` on `~/odom`) builds on Humble — its CI tests `rolling`
  only. Load-bearing for any future hardware effort, not for this map.
- `EK3_GPS_TYPE`, which the BlueOS driver sets — not located in current master.
- Teledyne/Nortek per-field protocol details (first-party PDFs cited, not read).
- DAVE's exact ROS 2 + gz version targets.

### Trusted but not independently re-verified

The `AP_NavEKF3_Control.cpp` L283-299 aiding-gate claim — the linchpin of finding
2 — is the agent's reading with a file+line citation. Worth one confirming look
before 006 hardens params around it.
