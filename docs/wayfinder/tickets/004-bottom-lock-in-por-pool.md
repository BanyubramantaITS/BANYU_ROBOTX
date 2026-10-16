# 004 — Does por_pool give the DVL bottom-lock at mission depths?

`wayfinder:task` · parent: [Map: DVL in sim, fused by ArduSub EKF3](../map-dvl-sim-ekf3.md)
· status: **CLOSED** · resolved by experiment
· blocked by: ~~003~~ (closed) · blocks: 008

## Question

A DVL only produces velocity when its beams reach a surface within range —
bottom-lock. Confirm the `por_pool` world actually provides that for the whole
mission.

Check across the real mission envelope: the sub spawns at (−7, 0, −2),
`NavToGate` dives to ~2 m down, the gate sits at x≈−4, `yellow_buoy` at x≈6
z≈−2.1, and `MarkerOrbit` strafes a 3 m standoff circle around the buoy.

Specifically:

- Does `por_pool` have a floor with collision geometry the beams can return
  from, at every point along that path — or is it an open-bottomed box?
- What is the actual sub-to-floor distance across the mission, versus the
  sensor's configured min/max beam range from 003?
- Does the buoy or gate geometry occlude beams during the orbit?

If bottom-lock is absent or intermittent, that is a finding that reshapes the
map — say so rather than quietly widening the sensor's range to paper over it.
Resolution records the measured altitude range and any dropout regions.

## Resolution

**`por_pool` gives bottom lock. The sleeper did not bite.**

Measured in the real `proof_of_readiness.world` with the real pool and vehicle:
**35 DVL reports, 35 with `DVL_TARGET_BOTTOM` — 100% lock**, slant range
1.010 m at vehicle z = −1.653 (≈0.83 m altitude), consistent with the 30° beam
tilt. No dropouts.

### The geometry

- `por_pool` is 19 × 10 × 3 m with a **real collision floor** — a 0.2 m box whose
  top surface is at z = 0 in pool frame.
- The launch spawns it at **z = −2.5** (`proof_of_readiness.launch.py:200`), so
  the floor sits at world z = −2.5.
- The sub spawns at z = −2 → **≈0.5 m altitude** at mission depth, ≈0.48 m at the
  sensor. `yellow_buoy` sits at z = −2.1, so `MarkerOrbit` runs at ≈0.4 m.

That clears the A50's 5 cm minimum comfortably.

### The fidelity gap worth knowing about

0.4–0.5 m altitude puts a **real** A50 in range mode 1 (0.3–3.0 m), whose lower
bound is 0.3 m — so the mission flies with only ~0.1–0.2 m of margin before the
real sensor would change mode. Our model declares `minimum_range` 0.05 and has
no mode concept, so **the sim is more forgiving here than the hardware would
be**. Anything that makes the sub fly lower — a deeper dive, a trim change —
degrades on real hardware well before it degrades in sim.

Lock is genuinely lost when the sub rests on the floor: an unpowered spawn sank
to z = −2.517 and reported beams with no target, exactly as the A50 docs
describe for sitting on the bottom.

### Required config changes — made

Two, both discovered here, both necessary for any DVL to appear at all:

1. **`gz-sim-dvl-system` must be declared at WORLD level.** Declaring it inside
   `bluerov2_heavy/model.xacro` is **silently ignored** — no error, no topic.
   Added to `banyu_simulation/worlds/proof_of_readiness.world`; the dead
   model-level declaration was removed and replaced with a comment.
2. **`gz-sim-sensors-system` must be declared exactly once.** It is what creates
   the sensor entity, so it cannot be dropped — but declaring it in both the
   world and the model crashes the render thread with
   `Scene already exists with name: scene` → segfault in
   `SensorsPrivate::RenderThread()`. It currently lives in
   `bluerov2_heavy/model.xacro` and must stay in exactly one place.

The full matrix, established by elimination:

| world-level sensors | model-level sensors | world-level dvl | result |
|---|---|---|---|
| yes | no | yes | works |
| yes | **yes** | yes | **segfault** |
| no | yes | yes | works |
| no | yes | **no** | silent, no topic |
| no | **no** | yes | silent, no topic |

This also **corrects** the earlier claim in
[How does the waterlinked_dvl model attach to bluerov2_heavy?](./012-attach-dvl-model-to-vehicle.md)
that the segfault came from broken headless EGL. It did not; the EGL warnings
are unrelated noise.

### Caveat on this measurement

The rig had no autopilot, so the sub was drifting downward under buoyancy rather
than holding depth. The lock measurement is therefore taken while descending
through the mission depth band, not station-keeping at it. Sustained lock
through the actual mission — especially `MarkerOrbit`'s strafe — is
[008](./008-end-to-end-fusion-verified.md)'s job, not this ticket's.

## Amendment (after 010/008)

Altitudes above were measured with the DVL at the old `dvl_z=-0.02` mount; it is
now −0.10 (see [003](./003-mount-dvl-on-bluerov2-heavy.md)), so the sensor sits
≈8 cm lower and every figure shifts by that much.

More importantly, the **static** altitude budget was never the real constraint.
[010](./010-disable-sitl-synthetic-gps.md) found the vehicle free-falls at
≈0.13 m/s whenever it is not actively holding depth, so an idle sub eats the
whole budget in seconds regardless of where it spawned. Under active control the
budget is comfortable: [008](./008-end-to-end-fusion-verified.md) measured a
steady 0.65 m slant range through the entire mission, `MarkerOrbit` included,
with `skipped=0` — the strafe this ticket flagged as the likely dropout never
dropped a single report.

This ticket's "real A50 would be in its tightest range mode here" conclusion
stands unchanged, and matters more for hardware than for the sim.
