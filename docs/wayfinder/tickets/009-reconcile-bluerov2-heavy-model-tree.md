# 009 — Reconcile the bluerov2_heavy model tree with AGENTS.md

`wayfinder:task` (HITL) · parent: [Map: DVL in sim, fused by ArduSub EKF3](../map-dvl-sim-ekf3.md)
· status: **CLOSED** · resolved AFK
· blocks: 003 — now unblocked

## Question

The working tree contradicts AGENTS.md, and 003 cannot start until this is
settled. Independently verified this session:

- `bluerov2_gz/models/bluerov2_heavy/` contains `model.config`, `model.sdf`,
  `model.sdf.in`, `configs.yaml`, `meshes/` — and **no `model.xacro`**.
- `proof_of_readiness.launch.py` L60-64 loads
  `models/bluerov2_heavy/model.xacro` and calls `xacro.process()` on it.
- `grep -rl front_camera bluerov2_gz/models/` returns **nothing** — the camera
  block AGENTS.md says never to touch is not in this tree.
- AGENTS.md L8-9 asserts both the `model.xacro` spawn and the `front_camera`
  block as established fact.

As it stands the launch file references a file that does not exist. Likely
story: `bluerov2_gz` is a separate repo pinned via `import.repos`, its directory
timestamps are newer than the rest of the workspace, and local customisations
(the xacro conversion and the camera block) were never committed upstream — so a
re-clone wiped them. **Not confirmed; that is this ticket's job.**

Settle:

- Do the `model.xacro` and its `front_camera` block exist somewhere recoverable
  (another checkout, stash, reflog, backup)?
- If not, are they to be reconstructed from `model.sdf.in`, and by whom?
- How does this stop recurring, given `bluerov2_gz` is a pinned third-party repo
  that a re-import will overwrite again? A fork, a patch applied at build, or
  moving the customised model into `banyu_simulation` where it is actually
  version-controlled by this team.

Also relevant to the DVL: 001 reports `proof_of_readiness.world` does not load
`gz-sim-sensors-system` at all — which a DVL sensor will need. Confirm and fold
into the answer.

Whatever is decided, **AGENTS.md needs correcting** — it currently documents a
tree that is not there, and that misinformation will burn the next session.

## Resolution

**Nothing was lost. The checkout was on the wrong branch.** AGENTS.md was
correct; the working tree was not.

- `import.repos` already pins `bluerov2_gz` to `version: banyu` — correctly.
- The working tree was checked out on **`main`**, whose `bluerov2_heavy/`
  carries `model.sdf`/`model.sdf.in` and no camera.
- `origin/banyu` tip is `fde2ca2 "feat: added front camera"`, which renames
  `model.urdf → model.xacro` and adds the `front_camera` block. That rename is
  also the origin of the "stale dangling `model.urdf` symlink" AGENTS.md warns
  about in the install tree.
- `bluerov2_gz` is a **team fork** (`git@github.com:BanyubramantaITS/bluerov2_gz.git`),
  not upstream, so the customisations were always safely committed.

**Action taken:** `git -C bluerov2_gz checkout banyu`. `model.xacro` is present,
the `front_camera` block is intact (5 references). No reconstruction needed, no
re-fork needed.

### The `gz-sim-sensors-system` question — answered, and it was a false alarm

The plugin **is** loaded, from `models/bluerov2_heavy/model.xacro:67-70`
(`filename="gz-sim-sensors-system"`, `name="gz::sim::systems::Sensors"`) — not
from a world file. 001 looked in `proof_of_readiness.world` and concluded it was
absent. It is in the model. Nothing needs adding for a DVL sensor to be serviced.

### Facts 003 inherits

- **Where a sensor block goes:** a `<gazebo reference="base_link">` wrapper
  containing a `<sensor>` element, alongside the existing `imu_sensor`
  (`model.xacro:43-52`) and `front_camera` (`:54-65`).
- **The frame trick is already in the file.** The IMU block uses
  `<pose>0 0 0 3.142 0 0</pose>` with the comment "Transform from the Gazebo body
  frame (x-forward, y-left, z-up) to the ArduPilot body frame (x-forward,
  y-right, z-down)". A DVL block can use the same 180°-roll convention — and 005
  must know whether it did, since that path owns its own ENU↔NED conversion.

### Recurrence

`import.repos` is already right, so no config change is warranted. The failure
mode was a checkout landing on `main` after import — worth a branch check after
`vcs import` rather than a structural fix. Deliberately not over-engineering
this.

### Left alone deliberately

**AGENTS.md was not edited.** Its paths (`/home/gryd/Dev/ros2_ws_robotx`,
`/home/gryd/Dev/ardupilot_ws/ardupilot`) describe a different machine — on this
one the workspace is `/home/donjoe/Documents/Banyubramanta/robotx` and ArduPilot
is at `/home/donjoe/ardupilot`. That is machine drift, not an error to correct,
and rewriting it is outside this ticket's remit. Flagged for the owner.
