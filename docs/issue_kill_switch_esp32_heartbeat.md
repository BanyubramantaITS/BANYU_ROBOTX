# Title
[Feature] Implement Bidirectional ESP32 Safety Bridge with Heartbeat Watchdog & 3-Mode Kill Switch


> [!NOTE]
> **Proposal, not the protocol the robot runs today.** `sub_bringup.launch.py` starts `banyu_teleop`,
> which reads SB from `/mavros/rc/in` and writes a single byte `'0'`/`'1'`/`'2'` to the ESP32 when
> SB changes, with no heartbeat (`banyu_teleop/teleop_transmitter.md`). `esp32_bridge_node` below
> implements this heartbeat design and is off by default (`enable_esp32_bridge:=False`). If the
> ESP32 firmware enforces the 500 ms heartbeat watchdog described here, teleop alone would trip it
> in mode `2`. Evidence in this repo points to the single-byte firmware: the acceptance criteria
> below are all unchecked, `teleop_transmitter.md` documents the byte protocol with the ESP32
> echoing monitor lines, and `tools/scripts/debug/esp32_kill_cmd.py` notes the board keeps its state only
> while a program holds the port open. The firmware source is not in any team repository, so
> confirm on the bench once: SB Up, wait 5 s, the relay must stay on.

---

## 1. Context & Motivation
Under [RobotX 2026 Handbook Section 5.3.2 (UUV Requirements)](https://robonation.gitbook.io/robotx-2026-team-handbook/section-5-rules-and-requirements/5.3-system-requirements/5.3.2-uuv-requirements) and Section 5.2.3 (Kill Switch / Emergency Stop):
- The UUV must feature a remote emergency cutoff (Kill Switch) and a 3-state visual feedback indicator (LED system):
  - **State 0 (RED)**: Disarmed / Killed. Propulsion PDB is unpowered; Intel NUC remains powered on.
  - **State 1 (YELLOW)**: Manual teleoperation via RadioMaster TX16S.
  - **State 2 (GREEN)**: Autonomous mission mode.

An on-board **ESP32** handles the physical PDB disconnect relay and visual status LEDs based on the RadioMaster TX16S link. To prevent runaway sub hazards if the Intel NUC crashes or hangs during an autonomous run, a **bidirectional serial heartbeat watchdog** must be established between the NUC and ESP32.

---

## 2. Hardware Topology & Communication Architecture

```
[ RadioMaster TX16S ]
       │  (RF Link)
       ▼
 [ RC Receiver ]
       │  (SBUS / CRSF / PWM)
       ▼
 [ ESP32 Safety Controller ] ────(GPIO)────► [ Thruster PDB Relay ] (Cuts thruster power, NUC stays on)
       │                    ────(GPIO)────► [ Visual Status LEDs ] (RED, YELLOW, GREEN)
       │
       │  USB Serial (/dev/ttyUSB0 @ 115200 baud)
       │  ESP32 -> NUC: '0', '1', or '2' @ 10 Hz
       │  NUC -> ESP32: 'H\n' (Heartbeat) @ 10 Hz
       ▼
[ Intel NUC Mini-PC ]
       │
 [ esp32_bridge_node (rclcpp) ]
       │
       ▼  (/banyu/vehicle_status & MAVROS /mavros/cmd/arming)
 [ banyu_vehicle_interface ] ──► [ banyu_vehicle_execution ] & [ banyu_mission ]
```

---

## 3. Serial Protocol Specification

- **Baud Rate**: 115200 baud, 8N1.
- **ESP32 $\rightarrow$ NUC (10 Hz)**:
  - Streams ASCII characters with newline: `'0\n'`, `'1\n'`, or `'2\n'`.
    - `'0'`: DISARMED / KILLED (PDB Relay OFF, LED RED).
    - `'1'`: MANUAL (PDB Relay ON, LED YELLOW).
    - `'2'`: AUTONOMOUS (PDB Relay ON, LED GREEN).
- **NUC $\rightarrow$ ESP32 (10 Hz)**:
  - Streams ASCII heartbeat: `'H\n'`.
- **ESP32 Autonomous Watchdog**:
  - While in mode `2` (AUTONOMOUS), the ESP32 maintains a watchdog timer.
  - If no `'H\n'` is received from the NUC for **> 500 ms**, the ESP32 immediately trips the failsafe:
    - Drops PDB relay (cuts thruster power).
    - Transitions internal state to `0` (DISARMED).
    - Switches LED to solid RED (or fast-blinking RED indicating NUC watchdog fault).
    - Reports `'0\n'` back to NUC.

---

## 4. Software Implementation Plan

### A. ESP32 Firmware Requirements
- Read RC mode channel from the RadioMaster TX16S receiver.
- Read incoming serial bytes: on `'H'`, reset `last_nuc_heartbeat = millis()`.
- If `mode == 2 && (millis() - last_nuc_heartbeat > 500)`: Force `mode = 0`.
- Control relay and LEDs according to `mode`.
- Stream current `mode` over serial at 10 Hz.

### B. C++ ROS 2 Node: `esp32_bridge_node` (`banyu_vehicle_interface`)
Per repository rules (`AGENTS.md`), implement in **C++ (`rclcpp`)**:
- Uses POSIX asynchronous serial I/O (`termios`).
- Parameters:
  - `port`: Serial path (default `"/dev/ttyUSB0"`).
  - `baud`: Baud rate (default `115200`).
  - `heartbeat_rate_hz`: Frequency of heartbeat transmission (default `10.0`).
  - `serial_timeout_s`: Max allowed silence from ESP32 before flagging hardware fault (default `0.5`).
- Publishers:
  - `/banyu/hardware/kill_switch` (`std_msgs/msg/UInt8` or custom contract).
- Timers:
  - Heartbeat timer (10 Hz): Writes `'H\n'` to serial descriptor.
  - Read timer / async thread: Ingests incoming lines and updates state.

### C. Vehicle Interface Integration (`banyu_vehicle_interface`)
- Update `banyu_msgs/msg/VehicleStatus.msg`:
  - `uint8 operational_mode` (`0=DISARMED`, `1=MANUAL`, `2=AUTONOMOUS`).
  - `bool hardware_kill_active`.
- In `VehicleStatusTracker`:
  - Map incoming ESP32 state to `authority`:
    - Mode `0` $\rightarrow$ `"DISARMED"`.
    - Mode `1` $\rightarrow$ `"MANUAL"`.
    - Mode `2` $\rightarrow$ `"AUTONOMY"` (if FCU flight mode is `GUIDED`).
  - On transition to `0` (Disarmed): Asynchronously call `/mavros/cmd/arming` (`arm=false`).

### D. Execution & Mission Layer Gating
- `banyu_vehicle_execution` (`vehicle_execution_node.cpp`):
  - Mark safety fault immediately if `operational_mode != 2` while autonomous setpoints are requested.
  - Invalidate readiness and halt setpoint publisher if mode drops out of `2`.
- `banyu_mission` (`MissionState`):
  - `AutonomyInterruptedCondition` triggers immediately when `operational_mode` transitions to `1` or `0`, terminating active BehaviorTree actions and returning to standby.

### E. Configuration & Bringup
- Add `esp32_bridge_node` parameters to:
  - `banyu_bringup/config/por_params.yaml`
  - `banyu_bringup/config/task2_params.yaml`
- Update `banyu_bringup/launch/mission.launch.py` to start `esp32_bridge_node` when `use_sim:=False`.

---

## 5. Acceptance Criteria
- [ ] ESP32 successfully parses `'H\n'` and resets watchdog timestamp.
- [ ] In Autonomous mode (`2`), killing `esp32_bridge_node` on the NUC causes the ESP32 to trip the PDB relay to `DISARMED` and set LED to RED within 500 ms.
- [ ] Switching RadioMaster TX16S to `DISARM` immediately trips the PDB relay and calls MAVROS disarm.
- [ ] Switching RadioMaster TX16S to `MANUAL` while an autonomous BehaviorTree mission is running aborts the mission and yields stick teleoperation.
- [ ] Serial disconnect or cable unplug triggers `operational_mode = 0` (DISARMED) on the NUC.
- [ ] Unit tests for `VehicleStatusTracker` covering serial mode ingestion and timeout failsafes.
- [ ] Config parameters added to `por_params.yaml` and `task2_params.yaml`.
