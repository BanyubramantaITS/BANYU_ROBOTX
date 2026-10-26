# Task 2 run log

Running log for the autonomous run of `docs/task2_core_disruptive_plan.md` (branch
`feat/task2-core-disruptive-sim`). One entry per step: decision, result, next.

## Step 0: rx_msgs sync
- `git pull --ff-only` in `RobotX-SoS-communication-ITSN` took 1 commit (`cc7d9fa`): `PipelineBuoys`
  now uses the plain default QoS (reliable, volatile, keep_last 10), repeated about 1 Hz.
- Finding: `pipeline_buoys_node` subscribed with reliable + transient_local (depth 1). A transient_local
  subscription is incompatible with a volatile publisher, so it would have received nothing from the real USV.
- Decision: subscribe with `QoS(10).reliable()` (volatile; compatible with both kinds of publisher). Update
  `dds_relay.py` (route is VOLATILE) and `mock_usv.py` (publishes default QoS). `rx_msgs` rebuilt,
  `banyu_communication` rebuilt.
- Open item: `/system/mission/course` is still subscribed latched on the UUV side while the spec says plain
  default QoS. Not changed here; the mock GCS will republish `Course` periodically.
