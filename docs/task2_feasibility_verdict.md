# Task 2 Core, Advanced, Disruptive: feasibility verdict (simulation, minimal sensors)

Branch `feat/task2-core-disruptive-sim`. Evidence: `task2_results/`, `task2_run_log.md`, `task2_mode_experiments.md`.
The sim is Gazebo + ArduSub SITL with the GPS cut under water (worst case: lost at 0.1 m, back 10 s after surfacing),
no DVL, Bar30 depth, IMU, bottom camera, and a mock GCS and UAV. Simulation only; nothing here is a hardware result.

## Verdict per tier

| Tier | Verdict | What the sim shows |
|---|---|---|
| Core | **PARTIAL, mostly working** | The whole mission runs with the GPS lost under water: surface GPS transit in GUIDED, ALT_HOLD + RC override for the dive, pipeline acquire and follow, surface, survey report delivered to the mock GCS. After the ring tracker fix the light **order** was right in 5 of 6 matrix runs (node 3 wrong); before the fix it was right in about 4 of 11. |
| Advanced | **NO (not shown)** | One run: the mission completed, but the repair was not confirmed and the flash was not read, so it fell back to the Core survey (order wrong). No resource request was sent. |
| Disruptive | **NO (not shown)** | Five runs. The tier plumbing, the repair descent, the decoder window and the Core fallback work, but the repair and the two-colour code were never read end to end in the sim. The decoder itself is unit-tested. |

## Success rates

| Case | Result |
|---|---|
| Core, damaged node 2, 7 runs under the final fixture | order correct 3, wrong 3, no report 1 (`core_batch1_summary.md`, `core_repro1_*`, run 9) |
| Core, red node matrix 1/2/3/4, one run each | 1 correct (node 1), node 2 no report, node 3 wrong (red one slot late), node 4 wrong (red never recorded) (`core_red_node_matrix_summary.md`) |
| Core overall, before the tracker fix | 4 correct of 11 runs that sent or should have sent a report; 2 had no report |
| Core, red node matrix after the tracker fix, one run per node plus node 2 twice more | 5 correct of 6; node 3 wrong; every run sent a report (`core_matrix2_summary.md`) |
| Advanced, 1 run | repair not confirmed, code not read, Core fallback survey sent, order wrong |
| Disruptive, final run (dis7) | same as Advanced |
| Flash-code decoder, unit tests | 10 of 10: 9 tin x circle pairs, 10% dropped frames, 10 fps, green tin, mid-code start, Advanced, repair needs red then steady green |

## Where it fails and why

1. **Ring order (Core, 150 points).** Fixed for nodes 1, 2 and 4 in the matrix (a node first seen green then red is now one red record). Node 3 is still wrong (red one slot late). Before the fix the ring tracker recorded the damaged node twice (green, then red, at the same spot) in
   some runs and misses the last node, or misses the red one. In the correct runs each node is accepted once. The camera sees a
   ring about 1.3 m ahead of the vehicle (measured with Gazebo ground truth), which the tracker is not compensating for
   when it matches the course. Unresolved.
2. **No report in 2 of 11 Core runs before the tracker fix.** The BT never reached the surface step after `WaitSurveyComplete`. It did not recur in the 6 runs after the fix, but the cause was not found.
3. **Repair (Advanced and Disruptive).** In the sim the probe tip must be within 0.15 m of the node switch, which puts the camera
   about 0.3 m above the ring. At that range the detector no longer reports the ring and `CenterOnRing` fails or hangs, so the
   repair succeeded in only one run (the ring went green and the decoder confirmed it) and was not repeatable.
4. **Flash code.** The code can only be read from the inspection depth (about 1.6 m above the ring). After the climb the camera
   often looks at a neighbouring green node, so the red/blue flash is not seen. The decode window stayed open long enough, so
   this is a pointing problem, not a window problem.
5. **Not done:** the underwater search prior from odometry and surface-to-fix (plan W3), the pulse detector parameters and the
   hydrophone cases (S2, S3, W7), the other-end-active cases (R5-R8), R9-R16, S4-S12, the 9 colour pairs in the sim, and
   the 30 s GPS reacquire delay.

## Sensor limits that decided the result (plan section 13)

- **No position under water.** GUIDED is unsafe once the GPS is gone (a GUIDED entered after the loss is accepted, then
  stops following commands and rises); ALT_HOLD + RC override held depth and heading within 0.04 m over 30 s. The IMU drift
  that a real robot would have could not be injected in this sim, so the GUIDED drift risk is unmeasured.
- **Bottom camera** is the only sensor for the pipeline, rings and the flash code. It sees rings only in a band of distance:
  too close (touch distance for the probe) and the ring is lost; and it looks ahead of the vehicle, so ring position
  and ordering are off unless compensated.
- **Depth sensor** reads about 5% above the true depth in the sim, and the datum jumps 0.15-0.4 m at arming; the repair
  depth had to be set from the world geometry, which a real run does not know (no altimeter).
- **Hydrophone and front camera** were not used.

## Open items

1. Fix the ring tracker (duplicate green/red node, last node missed, look-ahead offset), then rerun the Core matrix.
2. Make the close-range repair work: approach with the ring still detectable, then a short blind descent, or give the
   centring a target offset for the probe tip.
3. Keep the camera on the repaired node while reading the code.
4. Implement W3 (search prior, surface-to-fix) and the hydrophone cases; run R5-R16 and the 9 colour pairs.
5. Find why some runs never reach the surface step.

Delegations: 1 job (`launch_task2.sh`, Sonnet), accepted after verification, 0 rejected.
