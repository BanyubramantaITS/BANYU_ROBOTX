# 003 — Mount the DVL sensor on bluerov2_heavy and see it publish

`wayfinder:task` · parent: [Map: DVL in sim, fused by ArduSub EKF3](../map-dvl-sim-ekf3.md)
· status: **CLOSED** · resolved AFK
· blocked by: ~~001~~, ~~009~~, ~~012~~ (all closed) · blocks: ~~004~~, ~~008~~

> **Approach corrected by the map owner.** The DVL is **its own model**, like
> `bluerov2_ping` — not a sensor block added to `bluerov2_heavy/model.xacro`.
> An earlier attempt to edit the heavy model directly was reverted.
>
> **Done so far:** `bluerov2_gz/models/waterlinked_dvl/` created
> (`model.config`, `model.sdf`, `WaterLinkedDVL.md`) and **verified publishing
> bottom lock** — `DVL_TARGET_BOTTOM`, slant range 2.296 m from a 2.0 m mount,
> consistent with the 30° beam tilt. Sitting on the bottom drops lock, matching
> the A50's documented 5 cm minimum altitude.
>
> **Still open:** attaching it to the vehicle — blocked on 012, because URDF has
> no `<include>` for an SDF model.

## Question

Add a downward-facing DVL sensor block to
`bluerov2_gz/models/bluerov2_heavy/model.xacro` using the SDF element 001
confirms, and verify it publishes on a gz topic during a normal sim run.

Includes deciding the **mount pose** — position and orientation relative to the
vehicle body origin. That pose is not cosmetic: it becomes the `VISO_POS_*` /
lever-arm offsets in 006, so record the exact numbers in the resolution.

Hard constraints:

- **Do not touch the `front_camera` sensor block** — user-configured, AGENTS.md
  is explicit. The DVL is a new, separate block.
- The model is spawned from `model.xacro`, **not** `model.urdf` — the install
  tree has a stale dangling `model.urdf` symlink. Ignore it.
- Mesh/resource paths depend on the `gazebo_env.dsv` / `setup.dsv` hooks; don't
  disturb them.

Done when the gz topic carries plausible velocity while the sub moves under
teleop or an existing mission phase. ROS-side visibility is 005's problem, not
this ticket's.

## Resolution

The DVL is on `bluerov2_heavy` and publishing bottom lock.

**What exists now**, all on the `banyu` branch of the `bluerov2_gz` fork:

- `models/waterlinked_dvl/` — the sensor as its own Gazebo model
  (`model.config`, `model.sdf`, `WaterLinkedDVL.md`), per the owner's steer that
  it be its own model like `bluerov2_ping`.
- `models/bluerov2_heavy/model.xacro` — `<include>` of that model plus a fixed
  `dvl_joint`, and the `gz-sim-dvl-system` plugin. Mount pose lives in the
  `dvl_x/y/z` xacro properties. See
  [How does the waterlinked_dvl model attach to bluerov2_heavy?](./012-attach-dvl-model-to-vehicle.md)
  for why include beat the alternatives.

**Verified in a running sim**, heavy held static 2.0 m above a ground plane:

```
type: DVL_TYPE_PHASED_ARRAY
target { type: DVL_TARGET_BOTTOM  range { mean: 2.2726953 } }
velocity { reference: DVL_REFERENCE_SHIP  mean { x: -0.0013  y: 0.0006  z: 0.0003 } }
```

2.273 m slant range against a 1.98 m altitude is consistent with the 30° beam
tilt (1.98/cos30° = 2.29). Velocity ≈ 0 for a static body, scattered at the
configured noise stddev. Free-falling onto the plane drops lock, matching the
A50's documented 5 cm minimum altitude.

**Two toolchain facts worth carrying forward:**

- `gz-sim-sensors-system` is **not** what services a DVL. `gz-sim-dvl-system`
  (`gz::sim::systems::DopplerVelocityLogSystem`) is required, and without it the
  sensor is silently ignored — no error, no topic.
- `gz:type="dvl"` survives xacro → URDF → SDF intact, but xacro needs
  `xmlns:gz` declared on the `<robot>` element or the document will not parse.

### Facts 006 inherits

Mount pose is `dvl_x=0.0, dvl_y=0.0, dvl_z=-0.02` relative to `base_link`, and
the DVL model's link origin is deliberately the beam-axis intersection point —
so those numbers go into `VISO_POS_*` directly, with no arithmetic. This mirrors
how `params/ping.params` carries
`RNGFND1_POS_X -0.18  # From models/bluerov2_ping/model.sdf`.

### Not covered here

Whether the sub keeps bottom lock through the *actual mission* — particularly
`MarkerOrbit`'s strafe — is [004](./004-bottom-lock-in-por-pool.md). All
verification here was against a flat ground plane, not `por_pool`.

## Amendment (after 010/008)

**The mount pose recorded above is stale.** `dvl_z` is now **−0.10**, not −0.02.

−0.02 was derived from the collision box, but the DVL beams raycast against
**visual** geometry, and the hull's visual bottom is ≈−0.085. At −0.02 the
sensor sat inside the hull mesh, level with the battery bracket. The current
value was found by sweeping the mount z until the beams cleared the visual hull.
`VISO_POS_Z` in `banyu_defaults.parm` is **0.10** to match (FRD flips the sign).

The collision/visual mismatch on `base_link` is upstream's simplification and
was left alone — see the note in [004](./004-bottom-lock-in-por-pool.md).
