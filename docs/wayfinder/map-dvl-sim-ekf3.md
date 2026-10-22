# Map: DVL in sim, fused by ArduSub EKF3

> **Note:** simulation only. The team has no physical DVL, so none of this is used in the real world.

`wayfinder:map` · tracker: local markdown · tickets in `./tickets/`

## Destination

A Doppler Velocity Log sensor exists in the Gazebo Harmonic sim, its body-frame
velocity reaches ArduSub, and **EKF3 demonstrably fuses it** — confirmed from the
autopilot side (velocity source active and healthy, no fallback), inside the
existing `proof_of_readiness` SITL run.

## Notes

- **Domain:** ROS 2 Humble + ArduSub SITL + Gazebo Harmonic + MAVROS. Read
  `src/AGENTS.md` before touching anything — it holds the non-obvious wiring and
  several hard-won traps.
- **This map carries execution.** Overrides wayfinder's plan-don't-do default:
  the destination is working software in sim, not a spec.
- **Sim only.** Real DVL hardware is out of scope (see below).
- **Fidelity: real sensor model.** Use gz-sim's DVL sensor with beams, range
  limits and noise — not a ground-truth fake. Decided at charting.
- **Orient on this before choosing a ticket (from 001):** a DVL's real job is
  hardware enablement — in sim, ArduSub already has ground-truth GPS. The
  sim-only destination is legitimate derisking, but only with SITL's synthetic
  GPS disabled (010). Any DVL result obtained with it still on is a false
  positive.
- Skills to consult: `/research` for anything upstream, `/grilling` +
  `/domain-modeling` for decision tickets, `/prototype` where a rough artifact
  beats an argument.
- Standing traps from AGENTS.md that this effort will collide with: never modify
  the `front_camera` sensor block in `bluerov2_heavy/model.xacro` (a DVL is a
  *new* block); MAVROS must stay an `ExecuteProcess`, not a launch `Node`;
  Gazebo/MAVROS ROS fields are ENU while MAVLink is NED; run ROS commands via
  `bash -c 'source ... && <cmd>'` because the shell is zsh.

## Decisions so far

<!-- one line per closed ticket -->

- [001 — Primary-source sweep: what DVL paths actually exist](./tickets/001-primary-source-sweep-dvl-paths.md)
  — SITL fakes GPS from ground truth so the sim proves nothing until it's off;
  only `VISION_POSITION_DELTA` bootstraps EKF3 aiding and MAVROS cannot emit it;
  `ros_gz_bridge` on Humble can't bridge the gz DVL message; `ardupilot_gazebo`
  has no back-door; two pre-existing MAVROS config defects found.
  Findings: [`src/docs/dvl-integration-research.md`](../dvl-integration-research.md).
- [002 — Choose the transport route from gz sensor to ArduSub](./tickets/002-choose-transport-route.md)
  — a ROS node speaks `VISION_POSITION_DELTA` straight to SITL on its own MAVLink
  endpoint, bypassing MAVROS; only route that transfers to hardware unchanged.
  Binds 006 to `EK3_SRC*_VELXY = EXTNAV`, and makes the `velErr` field
  load-bearing.
- [009 — Reconcile the bluerov2_heavy model tree with AGENTS.md](./tickets/009-reconcile-bluerov2-heavy-model-tree.md)
  — nothing was lost; the checkout sat on `main` while `import.repos` pins
  `banyu`. Checked out `banyu`, `model.xacro` and `front_camera` restored.
  `gz-sim-sensors-system` was a false alarm — it loads from `model.xacro:67-70`.
- [012 — How does the waterlinked_dvl model attach to bluerov2_heavy?](./tickets/012-attach-dvl-model-to-vehicle.md)
  — a literal `<include>` of the standalone model plus a fixed `dvl_joint`, so
  the sensor config stays single-source. `gz sdf -p` cannot resolve `model://`
  but `gz sim` can; on this box `gz-sim-sensors-system` segfaults headless and
  the DVL needs no rendering.
- [003 — Mount the DVL sensor on bluerov2_heavy and see it publish](./tickets/003-mount-dvl-on-bluerov2-heavy.md)
  — DVL is on the vehicle and locking: `DVL_TARGET_BOTTOM`, 2.273 m slant from a
  1.98 m altitude. Mount pose `0, 0, -0.02` on `base_link` feeds `VISO_POS_*`
  directly. `gz-sim-dvl-system` is mandatory or the sensor is silently ignored.
- [004 — Does por_pool give the DVL bottom-lock at mission depths?](./tickets/004-bottom-lock-in-por-pool.md)
  — yes, 35/35 reports locked. Floor at world z −2.5, sub at −2 → ~0.5 m
  altitude, ~0.4 m during `MarkerOrbit`. `gz-sim-dvl-system` must be
  **world**-level and `gz-sim-sensors-system` declared **exactly once**; both
  fixed. Real A50 would be in its tightest range mode here.
- [006 — ArduSub parameters that make EKF3 fuse the velocity](./tickets/006-ardusub-params-for-ekf3-fusion.md)
  — `VISO_TYPE 1`, `VISO_POS_Z` (FRD flips the xacro's mount z), `VISO_ORIENT`,
  `EK3_SRC1_VELXY 6` in `banyu_defaults.parm`. The VELXY line is what actually
  lets the DVL aid the filter. `SIM_GPS1_ENABLE` confirmed as 010's parameter.
  Two values here were later corrected: `VISO_POS_Z` is **0.10** after 003's
  mount moved, and `VISO_ORIENT` is **0**, not 25 — see 005.
- [007 — How do we read EKF3 velocity-source health from this stack?](./tickets/007-how-to-read-ekf3-health.md)
  — `/mavros/estimator_status` flags (`const_pos_mode` false + `pos_horiz_rel`
  true = aiding via `AID_RELATIVE`) **plus** the `VISO` dataflash message, which
  proves arrival independently of fusion. Neither alone separates "never
  arrived" from "arrived and rejected".
- [005 — Carry DVL velocity from gz to MAVLink](./tickets/005-carry-dvl-velocity-to-mavlink.md)
  — one Python process, `banyu_bringup/scripts/dvl_mavlink_bridge.py`. gz-msgs
  and gz-transport have Python bindings, so **no bridge backport was needed**.
  FRD mapping measured on all three axes. Needs `MAVLINK_DIALECT=ardupilotmega`
  **and** `MAVLINK20=1` set before importing pymavlink. Forced `VISO_ORIENT` to
  0.
- [010 — Turn off SITL's synthetic GPS so the DVL has a job](./tickets/010-disable-sitl-synthetic-gps.md)
  — `SIM_GPS1_ENABLE 0`, `EK3_SRC1_POSXY 0`, `EK3_SRC1_POSZ 1`. Negative control
  (GPS off, DVL off) reads `CONST_POS_MODE` set and `POS_HORIZ_REL` clear, so the
  aiding result is honest. `VELOCITY_HORIZ` sets in *both* arms and proves
  nothing alone. Exposed the free-fall trap: an unarmed vehicle sinks out of
  bottom lock in ~20 s and fakes a fusion failure.
- [008 — End-to-end: EKF3 fusing the simulated DVL in a real run](./tickets/008-end-to-end-fusion-verified.md)
  — **destination reached.** `flags=0x12f` (`POS_HORIZ_REL` set, `CONST_POS_MODE`
  and `POS_HORIZ_ABS` clear) held 165 s with zero `stopped aiding`, 1321 deltas
  at `skipped=0`, and the whole mission — `MarkerOrbit` included — completed with
  GPS off and no nav changes.

## Not yet specified

<!-- Empty. The way to the destination is walked; see "Where this stops" below. -->

## Where this stops

The destination — a real gz DVL whose velocity EKF3 demonstrably fuses inside
the `proof_of_readiness` SITL run — is reached, and the map is closed. Three
things the effort touched are deliberately *not* on the far side of it:

- **Noise, dropout and latency realism.** The sensor runs with the model's
  default noise and a placeholder `VISO_DELAY_MS 10`. Nothing here was tuned
  against a measurement, and 008 proved acceptance, not accuracy — no drift was
  measured. Sharpening this is a fresh effort with its own destination
  (probably "the sim's DVL misbehaves like an A50"), not a leftover ticket.
- **Whether the committed defaults should relax `ARMING_CHECK`.** With GPS off
  the prearm checks veto arming; every run here set it over MAVLink at runtime
  and changed nothing in the repo. Real question, deliberately left open — see
  [010](./tickets/010-disable-sitl-synthetic-gps.md).
- **What `banyu_mission` should do with DVL velocity.** This was fog while
  `AID_RELATIVE` was expected to starve the BT's P-controllers. It doesn't —
  008 completed the mission on relative aiding alone — so the question loses its
  premise. If it returns it belongs with the nav-architecture work already ruled
  out of scope below.

## Out of scope

- **Real DVL hardware integration** (Water Linked A50/A125, Nortek, Teledyne) —
  the vehicle-side wiring, drivers, and BlueOS path. Scoped out at charting: the
  destination stops at sim. Returns only as a fresh effort.
- [011 — Fix the two MAVROS config landmines](./tickets/011-fix-mavros-config-landmines.md)
  — the dead `vision_speed_estimate` denylist entry and the `odometry` plugin's
  dirty failure on missing TF frames. Both real, both worth fixing; 002 routed
  the DVL around MAVROS entirely, so neither is on this route. Separate work.
- **Revisiting the velocity-loop navigation architecture.** AGENTS.md's finding
  that ArduSub GUIDED position targets (`0xDF8` / AC_WPNav) crawl and let depth
  drift stands untouched. Whether a fused DVL makes position setpoints viable
  again is a separate later effort, not a step on this route.
