# Pipeline search after reaching the buoy (Issue #9)

RobotX 2026 · Task 2 · UUV search strategy proposal · October 2026

This document reviews the current acquisition strategy, lists what the handbook and the RoboNation
Discord tell us about the course, and proposes a replacement: a **crab-ladder search across the
buoy corridor, cued by the pipeline's LED rings**. It covers starting conditions, the search
pattern, detection confirmation, retry/timeout/abort rules, the BehaviorTree handoff, parameters to
calibrate, and a test plan.

## 1. What we know about the course

| Fact | Source | Consequence for the search |
|---|---|---|
| Pipeline is 5 straight PVC segments, 2"–3" pipe (OD 60–89 mm), joined by 45° elbows, not necessarily coplanar, "variability in all three axes" | Handbook 3.5.5, build guide, Discord 2026-07-01 | Pipe direction can change by 45°–90° between segments; a tight heading-prior gate rejects real pipe. |
| Horizontal run is about 6 m (2 × 1000 mm + 6 × 450 mm + 3 × 200 mm + fittings) | Build guide parts list | The whole pipe fits inside a ~6–10 m buoy-to-buoy corridor. |
| Suspended 1–2 m above the seafloor on 3 vertical 2" risers (3" tee → reducer → 1.5 m pipe) standing on rectangular PVC (maybe aluminium) base frames | Handbook 3.3.3, build guide, Discord 2026-08-03 | False targets: risers (vertical, seen end-on from above), base frames (closed rectangles, 1–2 m deeper). |
| Light nodes face **upward**, 16-LED RGB ring, one RED, rest GREEN; constant node count, random spacing | Discord 2026-07-06, 2026-08-03 | The rings are self-luminous beacons visible from above and obliquely from the front camera. |
| Buoys are placed "near the ends of the pipeline"; pinger hangs below each buoy | Handbook 3.3.3 | Pipe lies roughly along the buoy-to-buoy line. Diving directly on the buoy risks fouling the pinger line (run termination, rule 11). |
| Survey report is ordered from the end nearest the active buoy | Handbook 3.3.3 | We must know which pipe end is the start, even if we first touch the pipe in the middle. |
| **Fully submerged:** any part breaking the surface during a scoring run is a breach; organizers confirmed this reading | Handbook 5.3.2, Discord 2026-08-17 | Our surface GPS transit may count as a breach (see §2.2). Task 2 only says the UUV "may surface to complete this reporting". |
| Venue: Promontory at Marina Bay (Marina Reservoir, fresh water, typically low visibility) | Handbook 1.1 | Camera range, not FOV, will limit search altitude. `BARO_SPEC_GRAV` default 1.0 is already correct for fresh water. |

## 2. Problems with the current strategy

Current flow (`bt/banyu_mission.xml`): surface GPS transit to a configured buoy point → align to a
configured `heading` → dive in place → `AcquirePipeline` (surge forward at 0.3 m/s along
`heading_prior`, accept only a pipe within `heading_tolerance` of the prior) → on timeout
`SearchPipeline` (retrace 1.5 m, then yaw in place) → retry ×3.

### 2.1 Search geometry

1. **Driving along the expected axis is the worst direction for finding a line.** If the buoy is
   offset sideways from the pipe by more than half the camera swath (about 1 m at 2 m altitude,
   60° FOV), a surge along the prior never crosses the pipe. Crossing perpendicular guarantees an
   intersection whatever the lateral offset.
2. **Yaw sweeps do almost nothing with a downward camera.** Rotating in place spins the footprint
   around the vehicle: with the sim camera (60° HFOV, pitched 77°, 0.2 m forward) at 1.5 m
   altitude the covered disc has a radius of about 1.4 m. It only finds a pipe that was already
   nearly in view.
3. **The heading-prior gate rejects real pipe.** `acquire_pipeline_server.cpp` refuses a pipe whose
   direction differs from the prior by more than the tolerance. Two 45° elbows can put a segment
   90° off the buoy axis, so we may reject the pipe we are looking at.
4. **The dive point is the buoy itself.** The pinger and its line hang directly below it.

### 2.2 Hardware and rule blockers (not search patterns, but they decide which pattern is possible)

1. **No DVL on the real vehicle.** The sim has one (`EK3_SRC2_VELXY 6`), but the listed hardware
   (NUC, Pi 5, Pixhawk 2.4.8, M8N, two cameras, lights) does not. ArduSub GUIDED refuses to run
   without a position estimate (`ArduSub/mode_guided.cpp:36`, `sub.position_ok()`), and once the
   GPS fix is lost underwater the EKF only dead-reckons briefly. Our velocity-setpoint execution
   layer therefore will not hold underwater on hardware. The search below is designed to work
   **without XY position**: legs are timed at constant compass heading and depth, closed only by
   vision. Execution needs an ALT_HOLD + `MANUAL_CONTROL` backend (x/y/r with depth hold) for the
   dived phase, or a DVL/visual-odometry velocity source.
2. **Surface GPS transit vs. the fully-submerged rule.** The M8N needs its antenna above water,
   which during a scoring run is a breach under 5.3.2. Ask the organizers whether a surface transit
   to the start point is allowed. If not: take the GPS fix and heading before `RunStart`, transit
   submerged at shallow depth on compass + timed thrust, and widen the search corridor to cover the
   dead-reckoning error (see `corridor_half_width_m` below).

## 3. Alternatives considered

| # | Strategy | Expected time to contact | Needs position? | Verdict |
|---|---|---|---|---|
| A | **Crab ladder across the buoy corridor** (heading parallel to buoy axis, sway across it, step along it) | First leg in most cases (~20 s) | No (timed legs + compass) | **Recommended core** |
| B | **LED-ring cueing**: detect ring glow in the front camera (and bottom camera from altitude), yaw to its bearing, drive until the bottom camera has it | Seconds when a ring is in range | No | **Recommended add-on** to A |
| C | **Lights-off ring scan**: switch the four downward lights off during search so emitter rings dominate and backscatter disappears; switch them on to confirm/follow | — | No | **Recommended**, trivial to add |
| D | **Altitude-staged search**: search as high as visibility allows (larger swath), descend once a candidate is found | Depends on visibility | Needs depth + altitude | Use if visibility > 3 m; needs an altimeter (e.g. Ping2 as ArduSub rangefinder) to be reliable |
| E | **Per-course pipe map from practice runs**: survey each course's pipeline during practice slots and store its endpoints per `course_id` | Near-zero search | Yes for dive point | Strong prior if layouts stay fixed; confirm with organizers it is allowed; still keep A as fallback |
| F | Expanding square around the dive point | Several minutes | Benefits from position | Fallback only when no axis prior exists |
| G | Hydrophone homing on the active pinger | — | No | Not possible with current hardware |
| — | Current: surge along prior + yaw sweep | Fails when offset > ½ swath | Yes (GUIDED) | Replace |

## 4. Recommended strategy

### 4.1 Starting conditions

| Item | Value / source | Uncertainty |
|---|---|---|
| Active buoy position `B_a` | `rx_msgs/PipelineBuoys` from USV/UAV (see `pipeline_buoys_proposal.md`), else per-course map (E), else config | M8N 2–3 m CEP + observer error (`uncertainty_m`) |
| Corridor axis `u` | `B_a → B_b` bearing; else configured heading; else none (fallback F) | ±30° (buoys "near" the ends, not on the axis) |
| Dive point `D` | `B_a − d_back · u` laterally offset by `d_side` (start behind the active end, clear of the pinger line) | Adds DR error if transit is submerged |
| Heading at dive | `u` (vehicle points along the expected pipe direction) | Compass error ±5–10° (calibrate near thrusters) |
| Search depth | `water_depth − pipe_height − h_search`, or altimeter hold at `h_search` above seafloor | Pipe 1–2 m above seafloor |

### 4.2 Search pattern: crab ladder with ring cueing

The vehicle keeps heading `u` and **sways** across the corridor, so any pipe roughly along the
axis sweeps across the full image width and, when found, is already aligned with the vehicle for
the handoff to `FollowPipeline`. After each leg it surges forward one leg spacing and sways back.

```text
          active end                                     far end
  B_a  o .............................................  o  B_b      (u →)
         |<- d_back ->|
   +L  ---+   +---+   +---+   +---+   +---+
          |   |   |   |   |   |   |   |   |      sway legs, heading fixed = u
   -L     +---+   +---+   +---+   +---+   +--- ...
          D   s
```

- Swath at altitude `h` above the pipe: `W = 2 · h · tan(HFOV/2)`.

  | `h` | HFOV 60° (sim) | HFOV 90° |
  |---|---|---|
  | 1.0 m | 1.2 m | 2.0 m |
  | 1.5 m | 1.7 m | 3.0 m |
  | 2.0 m | 2.3 m | 4.0 m |
  | 3.0 m | 3.5 m | 6.0 m |

- Leg spacing `s = 0.6 · W` (overlap absorbs timed-leg scale error without DVL).
- Corridor half-width `L = σ_buoy + σ_nav + 1 m` (about 3 m surfaced-GPS case, 5 m submerged-DR
  case).
- Ladder length `L_along = pipe span + 2 m` (about 8–10 m), so the ladder covers the case where the
  buoy sits past the end of the pipe.
- Typical cost: leg 6 m at 0.3 m/s = 20 s; first contact is expected on the first leg because the
  pipe runs roughly along `u`. A full 8 m ladder at `s = 1.4 m` is 7 legs ≈ 3 min.

**Ring cueing (B) runs in parallel with every leg.** The front camera looks for saturated
red/green ring glow below the horizon; a ring 1.5 m below and 4 m ahead appears about 20° below
the image centre, inside a typical vertical FOV. A front-camera ring hit (`N` frames, stable
bearing) pre-empts the ladder: yaw to its bearing, surge until the bottom camera reports the ring
or the pipe, then return to heading `u` (or to the detected pipe heading).

**Lights-off scan (C):** during the ladder the downward lights stay off (or dimmed); they turn on as
soon as a candidate appears, for confirmation and following.

### 4.3 Detection confirmation (reject base, risers and clutter)

A detection becomes `CONFIRMED` only when the evidence below agrees. These rules can live in a
small acquisition-evidence stage in `banyu_detection` that publishes the acquisition state.

| Check | Accept | Rejects |
|---|---|---|
| Elongation | Straight segment exiting two image borders, or ≥ 60% of the image diagonal | Riser tops (seen end-on as a blob), ring enclosures alone, debris |
| Apparent width | `w_px ≈ f · OD / h` within ±40% for OD 60–89 mm; prefer the widest of parallel lines | Base frame (1–2 m deeper, thinner), ropes/pinger lines (< 2 cm) |
| Topology | Open line; one 45° kink allowed | Base frame (closed rectangle with 90° corners, parallel lines ~0.45 m apart) |
| Motion consistency | Line translates opposite to the sway direction at the expected rate for ≥ 0.3 m of motion | Reflections, lens/backscatter artefacts, our own tether |
| Persistence | ≥ `confirm_frames` consecutive frames | Single-frame noise |
| Ring evidence (strong) | A red/green LED blob lies on the line (within 2 · OD) | Unlit PVC elsewhere, non-pipeline structures |
| Heading prior (soft) | Weight only; never reject within ±100° of `u` | — (no longer a hard gate) |
| Location (soft) | Down-weight candidates within 1 m of the buoy position | Pinger line and buoy anchor |

`CONFIRMED` needs elongation + width + topology + persistence, and either ring evidence or motion
consistency over ≥ 0.5 m. Anything that fails is published as `REJECTED` with a reason (useful for
the logs and for the test plan below).

### 4.4 Handoff to pipeline following

1. **Brake and centre:** stop the sway, servo `offset_x → 0` with sway, keep depth.
2. **Pick the direction:** of the two directions along the detected line, choose the one with
   positive projection on `u` (away from the active buoy).
3. **Start end check:** if the first contact is not near the active end (end confirmer does not
   fire when looking back, or contact happened after the first ladder leg), run `SeekStartEnd`:
   follow toward the active buoy until `PipelineEnded`, U-turn, then start the survey. This keeps
   the report order correct and gives a second look at the rings near the start. Rings seen on the
   seek leg are not reported twice; the ring tracker counts the survey pass.
4. **Follow:** hand off to the existing `TraversePipeline` subtree.

### 4.5 Success, retry, timeout and abort

| Stage | Success | Retry | Timeout | Abort |
|---|---|---|---|---|
| Dive at `D` | Depth within tolerance, heading within tolerance | — | `dive_timeout_s` | Leak/FCU/authority loss (existing interrupt branch) |
| Ring-cued approach | Bottom camera candidate | Return to the ladder leg it pre-empted | `cue_timeout_s` (≈15 s) | — |
| Ladder (`L`, `s`, `h`) | `CONFIRMED` | 1) same ladder mirrored (start sway on the other side); 2) `L × 1.5`, `h` raised one step if visibility allows | `ladder_timeout_s` per attempt (≈ legs × leg time × 1.3) | — |
| Expanding square at `D` (no axis prior or ladders exhausted) | `CONFIRMED` | — | `square_timeout_s` | — |
| Whole acquisition | Handoff to follow | — | `acquire_total_timeout_s` (≈ 6 min) | Surface, publish a survey with status "pipeline not found" so Core-tier reporting still happens, then hold |

`REJECTED` candidates never end a ladder; the ladder continues and the rejection reason is logged.

## 5. BehaviorTree and interfaces

```xml
<BehaviorTree ID="DiveAndAcquirePipeline">
  <Sequence>
    <DiveToDepth target_depth="{search_depth}" target_heading="{corridor_heading}"
      depth_tolerance="{depth_tolerance}" heading_tolerance="{heading_tolerance}" />
    <Timeout msec="{acquire_total_timeout_ms}">
      <Fallback name="acquire">
        <SearchPipelineCorridor attempt="0" />  <!-- crab ladder + ring cueing -->
        <SearchPipelineCorridor attempt="1" />  <!-- mirrored -->
        <SearchPipelineCorridor attempt="2" />  <!-- wider, higher -->
        <SearchPipelineExpandingSquare />
      </Fallback>
    </Timeout>
    <AlignToPipeline direction_hint="{corridor_heading}" />
    <SeekStartEndIfNeeded />
  </Sequence>
</BehaviorTree>
```

The in-traverse `ReacquirePipeline` keeps its retrace step, but its yaw sweep is replaced by a
short sway box (±`W` either side, then ±2`W`), for the same reason as §2.1.

| Interface | Direction | Content |
|---|---|---|
| `SearchPipelineCorridor` action (new, `banyu_msgs`) | mission → execution | Goal: `corridor_heading_rad`, `half_width_m`, `leg_spacing_m`, `along_length_m`, `sway_speed`, `mirror`, `timeout_s`. Feedback: leg index, candidate state. Result: `found`, `pipeline_heading_rad`, `contact_leg`, `ring_seen`, `reason`. |
| `/banyu/pipeline/acquisition` (new) | detection → mission, execution, logs | `NONE / CANDIDATE / CONFIRMED / REJECTED`, reason, heading, width, ring evidence. |
| `/banyu/pipeline/front_ring_cue` (new) | front-camera ring detector → execution | Bearing, elevation, colour, confidence. |
| `/banyu/mission/pipeline_prior` (new, from `PipelineBuoys` or course map) | communication → mission | `B_a`, `B_b`, axis heading, uncertainty, source. |
| Existing | — | `/banyu/pipeline/observation`, `/banyu/pipeline/ring_observation`, `/banyu/pipeline/end`, `/banyu/vehicle_status`. |
| Execution backend (new) | execution → FCU | ALT_HOLD + `MANUAL_CONTROL` x/y/r for dived phases when no velocity source is available; GUIDED velocity remains for sim/DVL. |

## 6. Parameters to calibrate

| Parameter | Initial value | How to calibrate |
|---|---|---|
| `h_search`, `search_depth` | 1.5–2.0 m above pipe | Max range at which the pipe *and* a lit ring are detected in the venue water; use 70% of it |
| Bottom camera HFOV, `f` | From calibration | Checkerboard in water (refraction changes FOV) |
| `corridor_half_width_m` | 3 m (GPS) / 5 m (DR) | Measured buoy position error + transit error |
| `leg_spacing_m` | 0.6 · W | Reduce if pool tests miss crossings |
| `sway_speed`, `surge_step_speed` | 0.25 / 0.3 m/s | Fastest speed that still gives ≥ `confirm_frames` with the pipe in view |
| Thrust → speed model | Measured | Timed legs in the pool, measured with tape/overhead camera (needed without DVL) |
| `d_back`, `d_side` | 1.5 m, 1.0 m | Pinger-line clearance |
| `confirm_frames`, motion-consistency distance | 5 frames, 0.3–0.5 m | False-positive tests below |
| Width tolerance, ring-on-line distance | ±40%, 2 · OD | Pool tests with both 2" and 3" pipe |
| Front-ring cue thresholds (HSV, min area, frames) | From pool footage | Lights-off and lights-on footage at 2–6 m |
| `acquire_total_timeout_ms` | 360000 | Run time budget for Task 2 |

## 7. Test plan

Run each case in simulation (Gazebo world variants with `task2_sim_provider`) and, where marked,
in the pool with a 6 m PVC mock pipe, two lit nodes and one base frame.

| # | Case | Setup | Pass criteria |
|---|---|---|---|
| 1 | Visible at dive | Pipe under `D` | `CONFIRMED` within 10 s; follows away from the active buoy (sim, pool) |
| 2 | Lateral offset | Pipe parallel to `u`, offset 1, 2, 3 m both sides | Found on the first leg (sim, pool) |
| 3 | Rotated | Pipe at 45° and 90° to `u` | Found; aligned heading = detected pipe, not `u` (sim, pool) |
| 4 | Buoy past the end | `B_a` 2 m beyond the near end | Found by ladder step; `SeekStartEnd` not needed (sim) |
| 5 | Mid-pipe contact | Dive point beside the middle segment | `SeekStartEnd` runs; report order matches the active end (sim) |
| 6 | Wrong prior | Axis prior off by 90° and 180° | Found by attempt 1–2 or expanding square within the total timeout (sim) |
| 7 | Missing pipeline | No pipe in the world | All attempts time out; surfaces and publishes "not found"; no crash (sim) |
| 8 | Base frame only | Base without pipe | `REJECTED: topology/width`; search continues (sim, pool) |
| 9 | Riser / rope / pinger line | Vertical pipe and 1 cm rope | `REJECTED: elongation/width` (pool) |
| 10 | Unlit PVC elsewhere | Second pipe without nodes, outside the corridor | Not confirmed unless motion-consistent and in corridor; logged (sim) |
| 11 | Lone LED | Green LED blob with no pipe | Cue followed, not `CONFIRMED` without a line (pool) |
| 12 | No DVL | Sim with DVL disabled, ALT_HOLD backend | Ladder completes on timed legs; case 2 still passes (sim) |
| 13 | Turbidity / lights | Sim fog or pool at night, lights on vs off | Lights-off ring detection range ≥ lights-on range (pool) |
| 14 | Abort during search | `/banyu/mission/abort` mid-leg | Interrupt branch wins within one tick; zero command (sim) |

## 8. Questions for the organizers (Discord)

1. Does a surface transit with a GPS antenna above water count as a breach under 5.3.2 during a
   Task 2 run, or only after the UUV has dived?
2. Maximum horizontal distance between a buoy and the nearest pipeline end?
3. Water depth and expected visibility at the Task 2 location?
4. May teams survey the pipeline position during practice slots, and do layouts stay fixed for the
   event?

## 9. Suggested order of work

1. Raise §8 questions; decide the no-DVL execution backend (§2.2), since it changes every dived
   action.
2. Implement `SearchPipelineCorridor` in `banyu_vehicle_execution` and the acquisition-evidence
   stage in `banyu_detection`; remove the hard heading-prior gate from `AcquirePipeline`.
3. Add sim world variants for tests 1–8 and 12.
4. Front-camera ring cue and lights-off scan.
5. Pool tests 1–3, 8–13 to set the §6 parameters.
