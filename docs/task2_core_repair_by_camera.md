# Task 2 Core: repair by camera (no fixed depth)

Status: **confirmed in the sim** (Gazebo ground truth) on all four damaged nodes for Core, Advanced and Disruptive; see `task2_feasibility_verdict.md`
and the hypotheses H1-H23 in `task2_run_log.md`. The notes below are the first design; later changes: creep during the hold, rise 0.6 m
after the repair for the flash code, no centring while the code is read, lit-ring only on the node.

## Why
The handbook (3.3.3 and hidden task elements) says the pipeline need not be coplanar and may span variable depths, and a
light may sit on a vertical support. A fixed `repair_depth_m` cannot work, so the approach is by camera.

## What changed
- `PipelineRingObservation.size`: longest side of the lit lid over the image width (grows as the camera closes in).
  The old top-hat dot is the same size at any range, so the lid is measured (`lightExtent` in `pipeline_vision.cpp`).
- `HoldProbeOnRing` goal has an approach mode (`descent_rate_mps > 0`): sink while the ring stays centred, stop at
  `contact_ring_size`, hold until the ring stays GREEN for 1.5 s, climb back to the starting depth. Result has
  `ring_repaired`. With `descent_rate_mps = 0` it behaves as before.
- Core branch of `RepairRedRing` uses it (`mission.approach_*` in `task2_params.yaml`). Advanced and Disruptive still use the
  old fixed-depth steps.
- `task2_sim_e2e.sh --gui` runs the same end-to-end run with the Gazebo GUI.

## Measurements in the sim (bottom camera, probe tip at the switch when base is about 0.26 m above it)
| Base height above switch | ring size | ring offset_y |
|---|---|---|
| about 1.2 m | 0.044 | 0.18-0.4 |
| about 0.8 m | 0.07 | 0.35 |
| about 0.5 m | 0.12 | 0.15 |
| contact | 0.15-0.16 | 0 to -0.04 |

`contact_ring_size` is 0.14. Teleporting the vehicle so the tip is on the switch turns the node green after the 2 s hold, so
the sim switch itself works.

## Run 1 finding
The action returned success after 12 s because the detector flickers red/green on the lid at close range; one green frame
was taken as the repair. Fix: require 1.5 s of steady GREEN with no red between (and a 14 s hold budget).

## Harness note
`mock_gcs.py` sends `RUN-START` once. If it arrives before the UUV has the course it is dropped and the mission never starts;
use `task2_sim_e2e.sh`, which has worked.

## Open
- Confirm repair end to end (node 2 goes GREEN in Gazebo, `repair_verified`, correct survey).
- Advanced/Disruptive still on the fixed-depth path; vertical-support node not simulated.
