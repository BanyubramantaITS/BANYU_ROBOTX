# banyu_robocommand

RoboCommand (RobotX 2026) bridge. Runs on the **topside OCS host only** — §3.4
forbids vehicles connecting to the broker. Background and citations:
`src/docs/robocommand-integration-research.md`.

## Why it needs a venv

The schema is `edition = "2024"`; jammy's `python3-protobuf` 3.12.4 cannot import
the generated code and `protoc` 31.1 cannot regenerate it. Only
`protobuf==7.35` works, so `ros2 run` (system python3) will **not** run this node.

```bash
python3 -m venv --system-site-packages venv        # once, at workspace root
./venv/bin/pip install "protobuf==7.35" "paho-mqtt==2.0"
```

## Run

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=$PWD/src/banyu_robocommand:$PYTHONPATH ./venv/bin/python -m banyu_robocommand.rc_bridge \
  --ros-args -p team_id:=ITSN -p vehicle_id:=UUV1 -p broker_host:=192.168.65.2
```

`broker_host` is the course RoboCommand address = the DHCP default gateway
(Alpha .65.2 / Bravo .66.2 / Charlie .67.2 / Delta .68.2, §3.4.2).

Run-start sequence (§3.4.10) — declare, then let both vehicles report
`STATE_AUTO`; `RunStart` arrives on `/banyu/robocommand/run_start`:

```bash
ros2 service call /banyu/robocommand/declare_run std_srvs/srv/Trigger
```

Kill-switch override, when the hardware kill cuts thrusters without disarming:

```bash
ros2 param set /rc_bridge state_override killed     # '' returns to auto-derive
```

## Parameters

| Param | Default | Note |
|---|---|---|
| `team_id` | `ITSN` | **unconfirmed** — the published list maps `ITSN` to "Barunastra ITS" |
| `vehicle_id` | `UUV1` | must match the topic segment |
| `broker_host` / `broker_port` | `localhost` / `1883` | |
| `heartbeat_hz` | `2.0` | §3.4.11; total cap is 5 msg/s per system |
| `depth_sign` | `-1.0` | `depth_m = depth_sign * rel_alt`; sign convention unconfirmed upstream |
| `state_override` | `''` | `killed` / `manual` / `auto` |
| `run_vehicle_ids` | `[USV1, UUV1]` | PoR needs ≥ 2 |
| `run_tiers` | `[none, core, none, none]` | tasks 1-4 |

## Topics

Out: `banyu/robocommand/pinger_freq_hz` (`UInt32`, latched, from `RxCourse`),
`banyu/robocommand/run_start` (`Bool`, latched).
In: six MAVROS topics plus `banyu/mission/task` (`UInt8`, RxTask enum, published
by `banyu_mission`).

## Checks

```bash
./venv/bin/python src/banyu_robocommand/test/test_messages.py     # wire contract

# end-to-end against RoboNation's own server (also produces the PoR log):
git clone https://github.com/robonation/robocommand && cd robocommand/RobotX_2026
docker compose up -d --build            # broker + rc-test server, logs to ./logs
```

Communications Proof of Readiness (§3.1.5, window closes **31 Aug 2026**) is
satisfied by submitting that `logs/robocommand_*.log` showing a run-start
sequence with ≥2 vehicles declared.
