# SoS integration test tools

Run the UUV stack end to end against the GCS reporter and RoboCommand's test client on one machine.
These are test tools, not part of the vehicle software. Interface spec: `docs/gcs_ros_bridge_spec.md`.

| Tool | Domain / env | Purpose |
|---|---|---|
| `dds_relay.py` | ROS, domains 40 ⇄ 10 | Stand-in for the DDS Router: forwards only the `/system/*` topics, including the USV's `PipelineBuoys` down to the UUV |
| `gcs_ros_bridge.py` | GCS repo venv + ROS, domain 10 | Reference GCS bridge around `GcsReporter` (commands, stale heartbeats, persisted seqs, de-duplication per declared run, `/gcs_ros_bridge/new_run`) |
| `rc_observer.py` | GCS repo venv | Decodes everything crossing the broker into JSONL (RoboCommand's view) |
| `sim_supervisor.py` | ROS, domain 40 | Sim-only neutral supervisor RC (`/sim/supervisor_rc`) and lifecycle/course/command/buoy log |
| `mock_gcs.py` | ROS, domain 10 | ROS-only GCS and UAV stand-in for Task 2: scripted `rx_msgs/Command`s (`run-start`, `uav-go`, `delivery:RED:GREEN`, `mission-done`, `command_seq` always increasing), logs the UUV's survey and `ResourceDelivery`, relays it to the UAV as `CMD_DELIVERY`, echoes it as a mock UAV, and checks it against `--expect-*` (exit code 0 = pass) |
| `mock_usv.py` | ROS, domain 10 (or 40) | Stand-in for the USV's Task 2 `rx_msgs/PipelineBuoys` feed (staged buoy A, then both) |
| `mock_uav_safe_passage.py` | ROS, domain 40 | Sim stand-in for the UAV's Task 1 `rx_msgs/SafePassage` buoy field (the 2026-11-12 trial's first message, 8 buoys; publishes the later 5- and 6-buoy subsets mid-transit like the real UAV); prints the UUV's closest clearance per buoy |
| `rc_host_scenario.py` | ROS + robocommand-rx-ui-itsn | Presses the RoboCommand host console's buttons: course, two runs, End run, GCS `new_run`, **Reset run ID & seq**; screenshots and JSONL |

The GCS venv needs PyYAML for `rclpy` (`pip install pyyaml`); numpy is taken from the system.
`GCS_REPO` defaults to `robocommand-rx-ui-itsn` checked out next to the colcon workspace
(`<ros2_ws_robotx>/../robocommand-rx-ui-itsn`); set `GCS_REPO` to override.

## Procedure

Run from the colcon workspace root (`ros2_ws_robotx/`).

```bash
# 0. MQTT broker on 127.0.0.1:1883 (docker compose in robocommand/RobotX_2026, or any mosquitto)

# 1. GCS side
python src/BANYU_ROBOTX/tools/sos_integration/rc_observer.py mqtt.jsonl                       # GCS venv
ROS_DOMAIN_ID=10 python src/BANYU_ROBOTX/tools/sos_integration/gcs_ros_bridge.py --team ITSN --state-file gcs_state.json
python3 src/BANYU_ROBOTX/tools/sos_integration/dds_relay.py 40 10

# 2. UUV side (use_sim:=True already points the status node at /sim/supervisor_rc and starts
#    sim_rc_deadman_node; sim_supervisor.py adds the lifecycle/course/command/buoy log)
ROS_DOMAIN_ID=40 python3 src/BANYU_ROBOTX/tools/sos_integration/sim_supervisor.py uuv.jsonl
ROS_DOMAIN_ID=40 ros2 launch banyu_bringup main_launch.launch.py mode:=task2
# On PRIME on-demand laptops add: __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia

# 3. RoboCommand (official test client, interactive mode keeps the connection open)
cd robocommand-rx-ui-itsn/robocommand/RobotX_2026/rc_test
BROKER_HOST=127.0.0.1 PYTHONPATH=../gen/python python test_client.py
robotx> course --course-id ITSN-SIM-A --pinger-freq-hz 25000 --corner-lats 33.8093 33.8093 33.8113 33.8113 33.8093 --corner-lngs -118.3949 -118.3929 -118.3929 -118.3949 -118.3949

# 4. Operator prepares the UUV, then RoboCommand starts the run
ROS_DOMAIN_ID=40 ros2 service call /banyu/mission/prepare_autonomy std_srvs/srv/Trigger
robotx> run-start --team ITSN --declaration-seq 1 --run-id 1
```

Check `mqtt.jsonl` for `heartbeat` (2 Hz, `current_task`), `incident_ack` and `pipeline_survey`.

### USV buoy feed and run ID reset (issue #16)

In the simulator `task2_sim_provider.py` already publishes `PipelineBuoys` like the USV. Outside
the sim, or to try other buoy cases, publish it from the GCS side and let the relay forward it:

```bash
ROS_DOMAIN_ID=10 python3 src/BANYU_ROBOTX/tools/sos_integration/mock_usv.py \
  --a 33.810438 -118.393392 --b 33.810438 -118.393272 --active a --second-after-s 5
ROS_DOMAIN_ID=40 ros2 topic echo --qos-durability transient_local /banyu/task2/buoy_target
```

The whole host-side flow, including RoboCommand's **Reset run ID & seq**, can be driven with the
real host console instead of `test_client.py` (system Python with PyQt5, paho-mqtt and protobuf):

```bash
python3 src/BANYU_ROBOTX/tools/sos_integration/rc_host_scenario.py out/
```

Between runs the operator ends the run on the host, asks the GCS for a new declaration
(`ros2 service call /gcs_ros_bridge/new_run std_srvs/srv/Trigger`, domain 10), resets run ID and
seq on the host, and calls `prepare_autonomy` again: the UUV then accepts run 1 / seq 1 again.
The official client's one-shot mode disconnects before flushing; use the interactive mode above.
