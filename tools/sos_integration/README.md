# SoS integration test tools

Run the UUV stack end to end against the GCS reporter and RoboCommand's test client on one machine.
These are test tools, not part of the vehicle software. Interface spec: `docs/gcs_ros_bridge_spec.md`.

| Tool | Domain / env | Purpose |
|---|---|---|
| `dds_relay.py` | ROS, domains 40 ⇄ 10 | Stand-in for the DDS Router: forwards only the `/system/*` topics |
| `gcs_ros_bridge.py` | GCS repo venv + ROS, domain 10 | Reference GCS bridge around `GcsReporter` (commands, stale heartbeats, persisted seqs, de-duplication) |
| `rc_observer.py` | GCS repo venv | Decodes everything crossing the broker into JSONL (RoboCommand's view) |
| `sim_supervisor.py` | ROS, domain 40 | Sim-only neutral supervisor RC (`/sim/supervisor_rc`) and lifecycle/course/command log |

The GCS venv needs PyYAML for `rclpy` (`pip install pyyaml`); numpy is taken from the system.
`GCS_REPO` defaults to `~/Desktop/robocommand-rx-ui-itsn`.

## Procedure

```bash
# 0. MQTT broker on 127.0.0.1:1883 (docker compose in robocommand/RobotX_2026, or any mosquitto)

# 1. GCS side
python tools/sos_integration/rc_observer.py mqtt.jsonl                       # GCS venv
ROS_DOMAIN_ID=10 python tools/sos_integration/gcs_ros_bridge.py --team ITSN --state-file gcs_state.json
python3 tools/sos_integration/dds_relay.py 40 10

# 2. UUV side (headless SITL: point the status node at the supervisor RC, sim only)
sed 's#^\(\s*rc_topic:\s*\).*#\1"/sim/supervisor_rc"#' \
  install/banyu_bringup/share/banyu_bringup/config/task2_params.yaml > /tmp/task2_sim_rc.yaml
ROS_DOMAIN_ID=40 python3 tools/sos_integration/sim_supervisor.py uuv.jsonl
ROS_DOMAIN_ID=40 ros2 launch banyu_bringup main_launch.launch.py mode:=task2 \
  task2_params_file:=/tmp/task2_sim_rc.yaml
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
The official client's one-shot mode disconnects before flushing; use the interactive mode above.
