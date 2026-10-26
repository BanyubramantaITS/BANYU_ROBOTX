# Prompt: autonomous run of the Task 2 simulation plan

Paste the block below as the first message of a fresh Claude Code session started in the colcon workspace
(`ros2_ws_robotx/`). Plan: `docs/task2_core_disruptive_plan.md`. Start the session with a mode that does not
ask for permission (for example `claude --dangerously-skip-permissions`), on this machine only.

```
Execute the plan in docs/task2_core_disruptive_plan.md end to end, autonomously, in
simulation only. I am away from the PC for a long time. Do NOT ask me questions, do NOT use
AskUserQuestion, do NOT wait for approval, and do NOT stop to summarize midway. When a decision
is needed, pick the default stated in the plan (section 9 assumptions, section 5 decisions),
write the decision and reason to docs/task2_run_log.md, and continue.

You are the ORCHESTRATOR AGENT: the smart, critical lead who is accountable for the result. You may
delegate (see "Delegation"), but you verify everything every subagent produces, every time.

## Ground rules
- Repo: BANYU_ROBOTX, branch feat/dive-trajectory-path. Create a work branch
  feat/task2-core-disruptive-sim from it and work there. Commit locally in small steps with clear
  messages. Do NOT push. Do NOT add Co-Authored-By or any Claude/AI attribution to commits
  (repo rule). Only you commit.
- Run ROS commands from the colcon workspace root (ros2_ws_robotx) via
  bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && <cmd>' (shell is zsh).
  No absolute paths in committed files.
- Simulation only. Never touch real hardware, serial ports, udev rules, ESP32, or Pixhawk
  parameters on a real device. Do not edit the front_camera sensor block in the bluerov2 model.
- Isolate DDS: use ROS_DOMAIN_ID=40 for the UUV side and 10 for the GCS side (as in
  tools/sos_integration/README.md). When running unit tests alongside sim nodes, use a separate
  ROS_DOMAIN_ID (e.g. 42).
- Before each sim run kill stale ardusub, mavproxy, gz and ros2 processes that this session
  started (see AGENTS.md "stale processes"). Do not kill anything else. About one Gazebo launch
  in three fails to hand over the world: relaunch (max 3 tries) instead of asking.
- Follow AGENTS.md rules: every new or renamed node parameter goes in
  banyu_bringup/config/task2_params.yaml, is passed in mission.launch.py, and the operator
  script tools/scripts/launch_task2.sh (create it like launch_por.sh) and docs are updated in
  the same change. Verify the script with --dry-run for --sim and --real.
- Do not delete files you did not create. Do not rewrite git history.

## Machine resources (you may use them)
This PC: 20 CPU threads, 31 GB RAM (about 15 GB free at start), NVIDIA RTX 5050 Laptop GPU (8 GB), only
about 25 GB free disk (87% full). You may use the CPU, GPU and memory freely, with these limits:
- Gazebo needs the NVIDIA GPU: run it with __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia.
- Builds and unit tests: colcon build --parallel-workers 8 (or more); run tests and agent jobs in parallel.
- Simulations: run ONE ArduSub SITL + Gazebo at a time by default (fixed ports such as 5501 clash, and the
  GPU has 8 GB). Run a second one in parallel only if you give it separate ports, instance id, and
  ROS_DOMAIN_ID, and only after the first one runs cleanly. Use headless/no-GUI where the launch files allow.
- Memory: if free RAM drops below about 3 GB, stop extra jobs and agents before continuing.
- Disk is the tight resource: keep at least 5 GB free (check with df -h before every sim run). Record bags
  with compression and without raw images, keep only the bags you need for failed cases and the verdict, and
  delete bags and build artifacts YOU created once their results are logged. Never delete anything you did not
  create. If free disk falls below 5 GB, clean your own artifacts first, then continue.

## Delegation (orchestrator rules)
Delegate cheap, well-bounded work to cheaper agents. Anything that cannot be delegated safely, or
that you cannot check afterwards, you do yourself. Never delegate a decision or a design choice.

Who does what:
- Claude subagents (Agent tool with model: "haiku", or "sonnet" for slightly harder work) for small
  self-contained jobs: boilerplate and parameter plumbing (YAML/launch/params wiring), unit-test
  scaffolds, log and bag summaries, formatting docs, renaming, repetitive scenario scripts
  (the R1-R16 run loops), parsing result files into tables.
- Antigravity CLI (agy, ~/.local/bin/agy) for the same class of jobs, in parallel or as a second
  opinion. Tested on this machine. Model: gemini-3.8-flash-high (use gemini-3.8-flash-medium or
  -low for trivial jobs; claude-sonnet-4-6 or claude-opus-4-6-thinking only if a job really needs
  them; list with `agy models`). The prompt must be attached to --print with an equals sign:
    agy --dangerously-skip-permissions --print-timeout 900s --model gemini-3.8-flash-high \
        --add-dir <only the directory it needs> --print='<self-contained task + acceptance check>'
  --dangerously-skip-permissions is the default for agy so it never blocks waiting for approval.
  Because it skips every permission check, ALWAYS run agy inside an isolated git worktree or a copy
  of the files it needs (never on the main work branch, never with the workspace root as
  --add-dir), and tell it which files it may touch. You merge only what you accept.
  If agy fails, hangs, or is not logged in, do not wait: do the job yourself or use a Claude subagent.
- You yourself (no delegation): architecture and control decisions, the Pixhawk mode experiments
  and their interpretation, BT changes, the vehicle_execution mode switch, the flash decoder logic
  and ring tracker/ack handling (R11), mock_gcs correctness, safety-relevant parameters,
  stage-gate judgement, and the final feasibility verdict.

How to delegate:
- One clear goal per job, the exact files it may touch, the interface or format it must match,
  and a command that proves it is done (test, build, lint or diff check). Tell it not to touch any
  other file, not to commit, not to push, and not to run the simulation unless the job is exactly that.
- Run independent jobs in parallel; never let two agents edit the same file.
- Do not spawn agents for the sake of it: use one only when the job is mechanical, bounded, and
  cheaper to verify than to write. When in doubt, do it yourself.

Verification (mandatory, every subagent, every time, no exceptions):
- Treat all subagent output as untrusted claims, including from the agents you created yourself.
- Read the actual diff (git diff / git status in the worktree). Confirm no file outside the allowed
  list changed. Confirm no parameter, topic name, message field, tier value or number was invented or
  silently changed.
- Re-run the build and the proving command yourself; do not rely on the agent saying it passed.
  For tests, confirm they fail without the change and pass with it where practical. For tables or
  numbers derived from logs and bags, spot-check against the raw files.
- If the result is wrong or only partly right: fix it yourself if small; otherwise re-delegate once
  with a sharper brief; after one failed retry, do it yourself. Log the rejection.
- Never commit anything you have not read. Never merge agent work you have not verified.
- Cheap agents never decide anything about safety, GPS-loss behaviour, mode switching or anything
  that could affect hardware, and never push or touch real-hardware scripts.
- In docs/task2_run_log.md, for each delegation record: who, task, files, and the outcome
  (accepted / fixed by me / rejected, with the reason).

## Order of work (do not skip; stage gates apply)
0. In src/RobotX-SoS-communication-ITSN run git fetch and git pull --ff-only origin main (it is
   1 commit behind); rebuild rx_msgs. If this fails, log it and continue with the local copy.
1. W1: implement sim_gps_gate (sim only) and run the Pixhawk mode experiments M1-M6 from plan
   section 6, with the worst case (gps_loss_depth_m 0.1, reacquire delay 10 s, also 30 s).
   Write results to docs/task2_mode_experiments.md (per mode: held or cancelled, STATUSTEXT and
   /rosout messages, EKF status, 60 s depth and heading drift, clean return to GUIDED at the
   surface). Choose the underwater mode from the evidence; default ALT_HOLD + RC override.
2. W2-W3: mode switch in the BT and vehicle_execution, search prior from
   underwater_odometry_node, surface-to-fix branch (max 3 attempts).
3. Core stage (task2_tier 2): W4 (blue classifier and ring ROI hue topic), mock GCS
   tools/sos_integration/mock_gcs.py (scripted RUN-START, UAV-GO, DELIVERY, MISSION-DONE with
   increasing command_seq, auto-relay of UUV ResourceDelivery to CMD_DELIVERY, mock UAV echo +
   heartbeat) and the red-ring matrix R1-R8, R9-R16 from section 14, plus S1-S5, S7-S10, S12.
4. Advanced stage (task2_tier 3): single-colour flash decoding (ring_code_decoder node, W5),
   hold-and-decode step in the BT (W6), ResourceDelivery with resource_color COLOR_ANY.
   Re-run the Core matrix as a regression.
5. Disruptive stage (task2_tier 4): two-colour decoding, all 9 tin x circle pairs (S6), R11
   flash-not-a-new-red-ring check. Re-run Core and Advanced as a regression.
6. W7 pulse detector parameters (pinger_freq_hz, pulse_repetition_s, bandwidth_hz) and the
   hydrophone on/off/dropout cases S2, S3. S11 (reduced visibility) is reported as a limit, not
   pass/fail.
7. Update docs: plan status, section 9 with measured values, docs/task2_sim_results.md.

## Stage gate and failure handling
- A stage is done when its test cases pass, or after 3 documented fix attempts per failing case.
  If a case still fails, record it as a FAILED limitation with the cause and the evidence
  (bag path, log excerpt) and move on to the next stage. Do not loop forever on one case.
- Per sim run, cap wall time at 20 minutes; per stage, cap at about 3 hours. If a cap is hit,
  record it and continue.
- If a tool or permission is denied, choose an alternative and log it. Never retry the same
  denied command unchanged.
- If something is truly impossible without me (missing hardware data, external access), write
  it under "Open items" in docs/task2_run_log.md and continue with the plan's assumption.
- Unit tests: gtest for decoder and pulse detector with synthetic traces; run existing tests
  (banyu_vehicle_execution, banyu_communication) as regression.

## Logging
- Keep docs/task2_run_log.md as a running log: timestamp, step, decision, result, next. Append
  after every step so progress survives interruptions.
- Per run record: tier, red node, active end, seed, time to acquire, time to repair, decoded
  tin/circle, report order, pass or fail. Keep bag paths out of git.

## Final deliverable (when everything is done or capped)
Write docs/task2_feasibility_verdict.md: per tier (Core, Advanced, Disruptive), success rate per
case, the step where it fails, why, which sensor limitation caused it (plan section 13), and a
yes/no/partial feasibility conclusion with the sensors available. Then commit locally and stop.
Final chat reply: 10 lines max, listing the branch, the verdict per tier, the delegation summary
(how many jobs, how many rejected), and the open items.
```

## Notes for the operator (not part of the prompt)

- `agy` and Haiku subagents were tested on this machine: both run unattended. `agy` needs
  `--print='prompt'` (with the equals sign); the first form tried, `--print <prompt>` after other flags, fails.
- `--dangerously-skip-permissions` on `agy` disables its permission checks. The prompt confines it to an
  isolated worktree or copy for that reason.
- Nothing is pushed. All results stay on the local branch `feat/task2-core-disruptive-sim`.
