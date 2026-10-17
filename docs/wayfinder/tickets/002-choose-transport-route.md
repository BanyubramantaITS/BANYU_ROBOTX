# 002 — Choose the transport route from gz sensor to ArduSub

`wayfinder:grilling` · parent: [Map: DVL in sim, fused by ArduSub EKF3](../map-dvl-sim-ekf3.md)
· status: **CLOSED** · resolved with the map owner
· blocked by: ~~001~~ (closed) · blocks: 005, 008 — both now unblocked

## Question

**Rewritten after 001.** The original three candidates are down to a real
dilemma, and it is the most consequential decision on this map.

What 001 settled:

- Route **(b)** — into `ardupilot_gazebo`'s SITL link — is **dead**. Its FDM JSON
  has no channel for it.
- The only MAVLink message that bootstraps EKF3 aiding from a DVL alone is
  `VISION_POSITION_DELTA`. `VISION_SPEED_ESTIMATE` and `ODOMETRY`-twist reach
  `writeExtNavVelData`, which does not move EKF3 out of `AID_NONE`.
- **MAVROS 2.14.0 cannot emit `VISION_POSITION_DELTA`** — 40 plugins, none does.

So route (a), the conventional ROS → MAVROS path, cannot carry the message that
works. The decision is now:

- **(a′) Use MAVROS anyway, via the `odometry` plugin**, and accept that the DVL
  will only be *fused* once aiding is established by something else. In sim with
  synthetic GPS on, that "something else" exists — which is precisely why this
  would look like it works while proving nothing (see 010). Cheapest to build,
  highest risk of a false positive, and it does not transfer to hardware.
- **(c) Speak MAVLink directly to SITL** from a small ROS node, emitting
  `VISION_POSITION_DELTA`. Bypasses MAVROS entirely for this path. Matches what
  BlueRobotics' own driver does, so it transfers to hardware unchanged. Costs a
  second MAVLink endpoint alongside MAVROS on `udp://127.0.0.1:14551`.
- **(d) Add a `VISION_POSITION_DELTA` plugin to MAVROS.** Keeps one MAVLink
  endpoint and one config surface; costs maintaining a MAVROS fork or carrying a
  patch, for a plugin upstream has declined to have.

Judge on: whether the sim result will still mean something on hardware, how much
of the work is ours versus a fork, and what it does to `mavros_config.yaml`.

**Do not pick (a′) for being easy without saying out loud that it cannot prove
the destination.** If it is chosen as a deliberate stepping stone, record that.

Resolution names the route, the message, and why the others were dropped.

## Resolution

**Chosen: (c) a ROS node speaking `VISION_POSITION_DELTA` directly to SITL over
its own MAVLink endpoint, bypassing MAVROS for the DVL path.**

Decisive reason: it is the only option whose sim result transfers to hardware
unchanged, and it matches what BlueRobotics' own DVL driver does. No fork, no
upstream patch.

Accepted costs: a second MAVLink endpoint alongside MAVROS on
`udp://127.0.0.1:14551`, and DVL data not appearing as a ROS topic unless the
node also publishes one for observability.

### The source chain this commits to — verified in `/home/donjoe/ardupilot`

`VISION_POSITION_DELTA` → `AP_VisualOdom_Backend.cpp:62` `writeBodyFrameOdom()`
→ `readyToUseBodyOdm()` (`AP_NavEKF3_Control.cpp:559-573`) → `AID_RELATIVE`
(`AP_NavEKF3_Control.cpp:284-299`).

Two gates fall out of this and are binding on 006:

- `readyToUseBodyOdm()` returns false unless `useVelXYSource()` is `EXTNAV` or
  `WHEEL_ENCODER` — i.e. `EK3_SRC*_VELXY` must be set accordingly.
- It also requires `bodyOdmDataNew.velErr < 1.0f` and data fresher than 200 ms.
  **The velocity-error field is not decorative** — the node must populate it
  honestly from the DVL's figure-of-merit, or the EKF silently refuses to aid.

### Why the others were dropped

- **(d) patch a MAVROS plugin** — same wire result, but buys a fork/patch burden
  on every rebuild for a plugin upstream has declined. Rejected on maintenance.
- **(e) integrate to pose in ROS, send via `vision_pose`** — reaches
  `readyToUseExtNav()` and `AID_ABSOLUTE`, which is *stronger* aiding, but does
  the EKF's integration outside the EKF with no corrector, so it drifts
  unboundedly. Rejected as architecturally wrong, not merely lossy.
- **(a′) MAVROS `odometry` twist** — reaches `writeExtNavVelData`, which is in
  neither `AID_NONE` exit branch. Cannot bootstrap aiding. In sim it would read
  green purely because synthetic GPS is aiding. Rejected as a false positive by
  construction.

### Consequence for the rest of the map

Nothing on the DVL route now passes through MAVROS. That takes
**Fix the two MAVROS config landmines** off the route entirely — see the map's
Out of scope.
