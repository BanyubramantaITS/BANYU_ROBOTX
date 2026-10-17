# 012 — How does the waterlinked_dvl model attach to bluerov2_heavy?

`wayfinder:grilling` · parent: [Map: DVL in sim, fused by ArduSub EKF3](../map-dvl-sim-ekf3.md)
· status: **CLOSED** · resolved by experiment
· blocks: 003 — now unblocked

## Question

The DVL now exists as its own Gazebo model
(`bluerov2_gz/models/waterlinked_dvl/`, verified publishing bottom-lock). It is
not yet attached to a vehicle, and there is a toolchain constraint in the way.

**The constraint:** `bluerov2_heavy` is URDF/xacro (`model.xacro`, on the
`banyu` branch), processed by `xacro.process()` in
`proof_of_readiness.launch.py:60-64` and handed to `ros_gz` as a URDF string.
**URDF has no `<include>` for an SDF model.** The `model://waterlinked_dvl`
include that works in a world file has no URDF equivalent, so the standalone
model cannot simply be pulled into the heavy model the way it is written today.

Candidate resolutions:

- **(i) Mirror it as a xacro macro.** Keep `waterlinked_dvl/model.sdf` as the
  canonical standalone model and add a xacro macro carrying the same `<gz:dvl>`
  block, which heavy's `model.xacro` includes. Works with the existing
  toolchain; costs a second copy of the sensor config that can drift.
- **(ii) Include at world level, join to the vehicle.** Spawn
  `waterlinked_dvl` as its own entity and fix it to the ROV with a joint in the
  world rather than inside the model. One definition, no duplication; costs
  wiring in the launch and a joint whose pose must track the vehicle spawn.
- **(iii) Move heavy to SDF.** Convert `bluerov2_heavy` to `model.sdf` like
  `bluerov2` and `bluerov2_ping` already are, so `<include>` works natively.
  Cleanest end state, biggest blast radius — the launch, the camera block, and
  `robot_description` all depend on the xacro path today.

Judge on: whether the sensor config stays single-source, how much of
`proof_of_readiness.launch.py` has to change, and whether the result still
matches the `bluerov2_ping` precedent the owner asked for.

Whatever wins, the attachment owns the **frame rotation**: the DVL model's axes
are FRD (x fwd, y starboard, z down) while the BlueROV2 body frame is z-up. The
existing `imu_sensor` block solves the same problem with
`<pose>0 0 0 3.142 0 0</pose>` and is the precedent to follow.

## Resolution

**Chosen: (ii) a literal `<include>` of the standalone model plus a fixed
joint.** Single source of truth, no duplicated sensor config.

In `bluerov2_heavy/model.xacro`, inside the model-level `<gazebo>` block:

```xml
<include>
  <uri>model://waterlinked_dvl</uri>
  <name>dvl</name>
  <pose>${dvl_x} ${dvl_y} ${dvl_z} 0 0 0</pose>
</include>
<joint name="dvl_joint" type="fixed">
  <parent>base_link</parent>
  <child>dvl::dvl_link</child>
</joint>
```

Verified in a running sim: `/dvl/velocity` publishes, and `gz model -m rov`
shows `dvl_joint` with parent link `base_link`, child link `dvl::dvl_link` — a
real attachment, not a free-floating second model. Option (i)'s xacro macro was
built, then **deleted**; option (iii) was never needed.

**No rotation is required.** The beams point −Z at identity, so a downward mount
is the identity pose. The FRD note above concerns the *reported velocity axes*,
which is 005's problem, not the mount's.

### Correction — the reasoning that nearly lost this

An earlier pass concluded the include "silently dropped" and fell back to the
macro. That was **wrong, on confounded evidence**. Two failures were being read
as one:

1. `gz sdf -p` genuinely cannot resolve `model://` URIs — it runs with
   `sdf::findFile()`'s callback unset. That is a limitation of the **CLI tool
   only**; `gz sim` resolves the URI fine via `GZ_SIM_RESOURCE_PATH`.
2. The spawn that produced no DVL topic was killed by something unrelated — see
   the environment note below — which would have killed *any* variant equally.

Re-running the include route once that was understood showed it working. The
lesson for this map: **`gz sdf -p` failing is not evidence that `gz sim` will
fail.**

### Environment finding — not a repo defect, but it will bite

`gz-sim-sensors-system` **segfaults on this machine** whenever it builds a
render scene for `bluerov2_heavy`:

```
libEGL warning: egl: failed to create dri2 screen
[Err] [BaseRenderEngine.cc:227] Scene already exists with name: scene
Segmentation fault  in SensorsPrivate::RenderThread()
```

With the camera stripped it still dies, in the same path, on an Ogre material
datablock collision. It takes the whole sensors pipeline down with it, which is
why the DVL appeared absent.

**The DVL does not need rendering.** It is serviced by
`DopplerVelocityLogSystem`, and every verification here was obtained by dropping
`gz-sim-sensors-system` from the test world.

> **CORRECTION, from [004](./004-bottom-lock-in-por-pool.md).** The diagnosis
> above is wrong about the *cause*. The segfault is not broken EGL — it is
> `gz-sim-sensors-system` being declared **twice**, once in the test world and
> once in `bluerov2_heavy/model.xacro`. Two instances race to create the render
> scene, hence "Scene already exists with name: scene". Dropping it from the
> test world happened to leave exactly one instance, which is why that worked.
> The EGL warnings are unrelated noise. See 004 for the full plugin matrix.

### Untested here — later settled by 004

All successful runs had `gz-sim-dvl-system` declared at **world** level as well
as in the model, so the model-level declaration was never isolated. 004 isolated
it: **model-level is silently ignored**, and the plugin must be world-level.
