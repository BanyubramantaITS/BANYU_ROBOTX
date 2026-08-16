# RoboCommand integration research — BANYU_ROBOTX

Research date: 2026-08-16. Every claim below carries an inline citation to a primary source
(RoboNation's own handbook, RoboNation's own GitHub repo + file/line, or a locally executed
verification). Anything not confirmed from a primary source is marked **UNVERIFIED**.

Primary sources are RoboNation's `robonation.gitbook.io` handbooks (the **live** version — see
§2.4, the PDF is stale on this exact topic) and <https://github.com/robonation/robocommand>
(Apache-2.0, RobotX schema committed `f6457fa` "Add RobotX 2026 message schema and test server",
2026-08-06). A local clone was made and executed; results are marked "verified locally".

---

## Bottom line

1. **The competition assumption is correct. This *is* RobotX, not RoboSub.** RobotX 2026 is no
   longer a surface-only competition: it is explicitly a multi-domain System of Systems, Mission
   Task 2 lists *"Minimum Systems Required: UUV, UAV"*
   ([§3.3.3](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.3-mission-task-descriptions/3.3.3-mission-task-2-infrastructure-survey-and-repair)), there is a dedicated
   [§5.3.2 UUV Requirements](https://robonation.gitbook.io/robotx-2026-team-handbook/section-5-rules-and-requirements/5.3-system-requirements/5.3.2-uuv-requirements) page, and the protobuf schema has a first-class
   `TYPE_UUV` and a UUV-only `depth_m` heartbeat field
   ([`rx_common.proto` L29-L34](https://github.com/robonation/robocommand/blob/main/RobotX_2026/proto/robotx/rx_common.proto#L29-L34), [`rx_reports.proto` L56](https://github.com/robonation/robocommand/blob/main/RobotX_2026/proto/robotx/rx_reports.proto#L56)).
   **RoboSub 2026 does not mandate RoboCommand at all** (§1.4). Clincher: this repo's
   `proof_of_readiness` mission tree is a line-by-line implementation of the RobotX
   [§3.1.3 UUV Proof of Readiness](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.1-proof-of-readiness) criteria (§2.3).
2. **RoboCommand is RoboNation's course-management server**, and for RobotX 2026 the interface is
   **MQTT over plain TCP port 1883 carrying Protocol Buffers binary payloads** — no HTTP, no
   WebSocket, no REST, no API key. It replaced the 2022 *"Technical Director Network"* raw-TCP
   NMEA-like `$RXHRB` heartbeat (§1.2).
3. **The only team system permitted to connect is the OCS.** *"Only the OCS connects to
   RoboCommand. Team vehicles must not connect directly to the RoboCommand network or MQTT
   broker."* ([§3.4](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.4-communications-protocol)). **This single rule decides the whole design**: a
   `banyu_robocommand` node must run on the topside OCS host, never on the vehicle (§4.1).
4. **You cannot start a run without it.** RoboCommand withholds `RunStart` until every declared
   vehicle heartbeats `STATE_AUTO`; the reference server implements exactly that
   ([`test_server.py` L118-L149](https://github.com/robonation/robocommand/blob/main/RobotX_2026/rc_test/test_server.py#L118-L149)), and §3.6 says *"Once the RoboCommand system confirms all
   systems are ready, it will activate the course."* A broken RoboCommand link is not a lost
   bonus — it is a run that never begins.
5. **There is a hard deadline 15 days from this research date.** Communications Proof of Readiness
   closes **31 August 2026**, and it is satisfied by submitting **`rc-test` logs showing a run
   start sequence with at least two vehicles in the `run_declaration`**
   ([§3.1.5](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.1-proof-of-readiness), §3.1 submission windows). **That deliverable needs zero
   ROS code** — `docker compose up` plus the vendor's own `test_client.py` produces it (§4.4
   step 0). Do that first, this week.
6. **Verified landmine: the schemas use `edition = "2024"`, which nothing in this Jammy toolchain
   can read.** Locally: `protoc` 31.1 refuses (*"is a file using edition 2024, which is later than
   the protoc maximum supported edition 2023"*) and `python3-protobuf` 3.12.4 cannot even import
   the vendor's generated classes (`ImportError: cannot import name 'runtime_version'`). The
   checked-in `gen/python` works only against `protobuf==7.35`, which I confirmed in a venv (§3.8).
   **Do not add a `protoc` step to `colcon`; vendor the vendor's `gen/python` and pin the runtime.**
7. **The stack already publishes 7 of the 11 heartbeat fields** via MAVROS, with near-exact type
   matches (`sensor_msgs/NavSatFix.altitude` is height-above-ellipsoid, which is literally
   `altitude_hae_m`). The two real gaps are `current_task` (nothing in `banyu_mission` publishes a
   task enum) and, on hardware, **a global position at all** — which is the same GPS-denied problem
   the DVL research note is about (§4.2, §4.3).
8. **Lazy recommendation: one Python node, ~150 lines, in a new `banyu_robocommand` ament_python
   package, running on the OCS.** Subscribe to five existing MAVROS topics plus one new
   `std_msgs/UInt8` task topic, publish an `RxReport` heartbeat at 2 Hz. Do not build a framework,
   do not build a task-report abstraction, do not use `mqtt_client` (§4.4, §4.6).

---

## 1. What RoboCommand is

### 1.1 Publisher, scope, and what it does

RoboCommand is published by **RoboNation** at <https://github.com/robonation/robocommand>,
Apache-2.0. Its own README defines it as *"Course management software for autonomous vehicle
competitions. Each competition has its own folder with its own protocol, message schemas, and
tooling."* ([README.md](https://github.com/robonation/robocommand/blob/main/README.md))

For RobotX the handbook definition is: RoboCommand *"exchanges course information, run-control
commands, vehicle status, and task reports"* with the team's Operator Control Station, and the OCS
relays to the team's own vehicles over the team's own comms
([§3.4](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.4-communications-protocol)). So it covers, in one channel, all five of the things the
question asked about:

| Function | Mechanism |
|---|---|
| Telemetry | `Heartbeat` inside `RxReport`, 2 Hz per vehicle |
| Kill-switch / mode state | `RobotState` = `STATE_KILLED` / `STATE_MANUAL` / `STATE_AUTO` |
| Run start | `RunDeclaration` (OCS) → `RunStart` (RoboCommand) |
| Task declaration | `RunDeclaration.taskN_tier`, plus `Heartbeat.current_task` transitions |
| Judging / task credit | `SafePassageReport`, `PipelineSurveyReport`, `DockingReport`, … |
| Live re-tasking | `RxCommand`: `AssistanceRequest`, `KeepOutZone`, `AllClear`, `MovingObjectAlert` |

Note there is **no run-stop message**. A task attempt ends by transitioning
`Heartbeat.current_task` to `TASK_NONE`
([`rx_common.proto` L5-L14](https://github.com/robonation/robocommand/blob/main/RobotX_2026/proto/robotx/rx_common.proto#L5-L14)).

### 1.2 What it replaced

The predecessor, verbatim from the **2022 RobotX Team Handbook v3.0, Appendix C** ([PDF, pp. 48-49](https://robonation.org/app/uploads/sites/2/2022/10/2022-RobotX_Team-Handbook_v3.0.pdf)):

- Network was the **"Technical Director's network"**, RJ-45, DHCP client, then *"establish a TCP
  connection to a server with an address and port number, correlating to the selected course"*
  (C.1). Address and port were handed out on-site.
- Wire format was **NMEA-like ASCII**: `$` + 5-char message type + comma-delimited fields + `*` +
  two-hex-digit XOR checksum + `<CR><LF>` (C.2).
- The heartbeat was `$RXHRB` at *"exactly a frequency of 1 Hz"*, fields: AEDT date `ddmmyy`, AEDT
  time `hhmmss`, latitude decimal degrees, N/S, longitude decimal degrees, E/W, 5-character Team ID
  *"assigned by Technical Director"*, System Mode (`1`=Remote Operated, `2`=Autonomous,
  `3`=Killed), UAV Status (`1`=Stowed, `2`=Deployed, `3`=Faulted), checksum. Example:
  `$RXHRB,111221,161229,21.31198,N,157.88972,W,ROBOT,2,1*11` (Table 15, C.3).

Everything in that list has a direct 2026 descendant: `$RXHRB` → `Heartbeat`; System Mode →
`RobotState`; 5-char Team ID → `team_id` (still 4-5 chars, still assigned by RoboNation, §3.1);
per-task NMEA sentences → the `oneof body` task reports; ASCII+XOR → protobuf; raw TCP → MQTT.

The 2022 scoring made this worth real points — *"100 points for transmitting first heartbeat
message"*, *"100 points for transmitting a heartbeat message reporting the ENTRY gate"*, and so on
per task (same PDF, Autonomy Challenge scoring). The 2026 points table is not published yet (§2.2).

### 1.3 "RoboCommand" is not one protocol — RoboBoat 2026 is different

The same repo also carries `RoboBoat_2026/`, which is **not** MQTT. RoboBoat uses raw TCP to
**10.10.10.1:50000**, with the framing *"Header(2 bytes) + Length(1 byte) + Payload(Length bytes) +
Footer(2 bytes)"* — ASCII `$R` … `!!` — a 1 Hz heartbeat, ≤5 Hz cap, and an NTP server on
10.10.10.1 ([RoboBoat Resources §3.3](https://robonation.gitbook.io/roboboat-resources/section-3-autonomy-challenge/3.3-communications-protocol)).

**This matters because it is the most likely way to waste a week.** The one third-party
implementation I found — [`MShandilya707/AAV_OCS_Robocommand`](https://github.com/MShandilya707/AAV_OCS_Robocommand) (NTU "Archimedes Autonomous
Vehicles" = team ID `NYTU1` on the RobotX team list) — is a ROS 2 node called `robocommand_hub`
that advertises *"the required `$R...!!` TCP framing"* and RoboBoat message names (`Report`,
`SoundSignal`, `ObjectDetected`, `PatrolBoat`). It was last pushed 2026-06-26, six weeks **before**
RoboNation published the RobotX MQTT schema. **SECONDARY, and now wrong for RobotX** — useful only
as corroboration that the "one ROS node subscribing to `/usv/odom`, `/uuv/odom`, `/uav/odom` on the
OCS" shape is what other teams landed on too.

### 1.4 RoboSub does not use RoboCommand

Checked directly: the [RoboSub 2026 handbook index](https://robonation.gitbook.io/robosub-resources/llms.txt) has **no communications-protocol
section at all** (its §3.x is Mandatory Activities / Task Descriptions / Vehicle Operations /
Sequence of Events). I fetched the six most likely RoboSub pages — 3.1, 3.3, 3.4, 5.3, 5.4, What's
New for 2026 — and grepped for `robocommand|mqtt|protobuf|heartbeat`: **zero matches**.

So if anyone on the team believes "RoboCommand is a RoboSub thing", they are wrong, and if anyone
believes "RobotX is surface-only so RoboCommand can't apply to our AUV", they are also wrong.

---

## 2. Why it is required

### 2.1 The rules that mandate it

Three separate handbook sections, all of which use "must":

- **[§3.2 Mandatory Activities](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.2-mandatory-activities)** — *"Any system operating in autonomy courses must
  report status and other required data via the RoboCommand system as specified in Section 3.4
  Communications Protocol."*
- **[§3.3.1 Cross-cutting Tasks](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.3-mission-task-descriptions/3.3.1-cross-cutting-tasks)** — *"RoboCommand: The System of Systems (SoS)
  must provide updates and reporting for each vehicle operating during any run in the autonomy
  challenge courses via the RoboCommand system."* It is listed alongside Obstacle Avoidance as a
  cross-cutting requirement, i.e. it applies to every task, not one of them.
- **[§3.4](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.4-communications-protocol)** itself — *"Every published message must use the correct topic,
  envelope type, `team_id`, sequence number, timestamp, and—where applicable—`vehicle_id`."*

For the UUV specifically, [§3.3.3 Task 2](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.3-mission-task-descriptions/3.3.3-mission-task-2-infrastructure-survey-and-repair) makes the report **the deliverable**: *"The minimum
response at the Core Tier is a report of the pipeline status to the RoboCommand system… The UUV may
surface to complete this reporting."* And at Advanced/Disruptive: *"the UUV and UAV must report
their status and progress in accordance with the RoboCommand messages described in Section 3.4."*
The `PipelineSurveyReport` comment states its receipt *"satisfies the Advanced tier's task
completion report"* ([`rx_reports.proto` L87-L101](https://github.com/robonation/robocommand/blob/main/RobotX_2026/proto/robotx/rx_reports.proto#L87-L101)).

### 2.2 What actually depends on it

- **The run does not start.** [§3.6 Scoring Rounds](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/hidden-3.6-scoring-rounds): *"Once the RoboCommand system confirms
  all systems are ready, it will activate the course and transmit the command allowing the
  uncrewed system of systems to begin mission execution."* Reference implementation: the server
  tracks the declared vehicle set and only publishes `RunStart` when
  `pending["auto_vehicles"] == pending["vehicle_ids"]`
  ([`test_server.py` L118-L149](https://github.com/robonation/robocommand/blob/main/RobotX_2026/rc_test/test_server.py#L118-L149)). And §3.4.10 instructs: *"All vehicles should
  position-hold after switching into Autonomous Mode. Only when the OCS receives and verifies the
  RunStart should any vehicle begin their run."*
- **Task credit.** Every task's scoring evidence is a RoboCommand message (§2.1).
- **Task 4 is impossible without the receive path** — it is defined entirely as command/ack chains
  (`AssistanceRequest → IncidentAck → ReadinessReport → ReadinessConfirm`, §3.4.12).
- **Deploy eligibility** via Proof of Readiness (§2.3).
- **Points table: not yet published.** [§4.2 Autonomy Challenge Scoring](https://robonation.gitbook.io/robotx-2026-team-handbook/hidden-section-4-scoring-and-awards) literally contains the
  placeholder `INSERT AUTONOMY TABLE OF POINTS` and says *"Qualifying Round and task completion
  criteria will be shared in the next version of the Team Handbook."* **So the exact per-message
  point values and any explicit penalty for a missing heartbeat are UNVERIFIED** — they do not
  exist in published form yet.

### 2.3 The deadline, and why this repo is already the right shape

[§3.1.5 Communications Proof of Readiness](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.1-proof-of-readiness):

> Teams must demonstrate their ability to send and receive messages in compliance with the
> Communications Protocol prior to arrival on site. This is accomplished by submitting *rc-test*
> logs that show a run start sequence as detailed in Section 3.4.10. The run start sequence must be
> performed with at least two vehicles in the *run_declaration* message.

Submission window (§3.1): **01 April – 31 August 2026** for Communications PoR and for the
mandatory two systems. Today is 2026-08-16. Failing PoR means *"teams will not be permitted to
deploy the system at competition"*, and passing it is also a precondition for the travel/shipping
stipend.

And the UUV autonomy PoR criteria (§3.1.3), verbatim:

> In a fully autonomous run, the UUV must start 3m behind the Gate, submerge, pass through the
> Gate, circle around the Marker, and then pass back through the Gate.

Compare `AGENTS.md`: *"The sub spawns 3 m behind the gate"*, and `bt/mission_tree.xml` =
`NavToGate → GateAlign → GatePass → SearchMarker → MarkerAlign → MarkerOrbit → ReturnToStart`
(`TurnAround → SearchGate → GateAlign → ReturnGatePass → Surface`). That is a one-to-one
implementation of the RobotX UUV PoR course. This repo is already a RobotX UUV; only the comms
layer is missing.

### 2.4 The PDF handbook is stale on exactly this topic — do not use it

The downloadable [Team Handbook PDF (`…20260330.pdf`)](https://robonation.org/app/uploads/sites/2/2025/10/Team-Handbook_RobotX-2026-20260330.pdf) still has §3.4 as a four-bullet stub
ending *"Additional details to be provided in a future iteration of the Handbook."* The
[Change Log](https://robonation.gitbook.io/robotx-2026-team-handbook/introduction-and-table-of-contents/change-log) shows the real content landed on **2026-08-07** (*"Published Communications
Proof of Readiness requirements; Published Communications Protocol and Team ID List"*), one day
after the schema was committed to GitHub. **Everything in §3 below comes from the live GitBook,
not the PDF.** GitBook serves a Markdown view of any page by appending `.md` to the URL, and an
index at [`/llms.txt`](https://robonation.gitbook.io/robotx-2026-team-handbook/llms.txt) — that is the reliable way to re-check this before competition.

---

## 3. The interface contract

### 3.1 Identity and network

**Team ID.** Assigned by RoboNation and published as a handbook page: the
[Team ID List](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.4-communications-protocol/team-id-list) has 25 entries. **`ITSN` = INSTITUT TEKNOLOGI SEPULUH
NOPEMBER, team name "Barunastra ITS".** There is no separate "Banyubramanta" entry. §3.4.8 says
*"Use the assigned `team_id` exactly as issued"*, and §3.4.14 warns *"Never use another team's
identifier for testing."* See open question O1 — this needs a human answer before anything is
published to a real broker.

**Vehicle IDs.** Team-chosen, but must be *"stable, unique"*, listed in `RunDeclaration`, and
identical in both the MQTT topic and `RxReport.vehicle_id` (§3.4.8). The vendor's examples use
`USV1`, `UUV1`, `UAV1` ([`RobotX_2026/README.md`](https://github.com/robonation/robocommand/blob/main/RobotX_2026/README.md)). One non-obvious behaviour: the reference
server enforces the geofence rule only for IDs that literally start with `"UAV"`
(`any(vid.startswith("UAV") for vid in decl.vehicle_ids)`, [`test_server.py` L104-L108](https://github.com/robonation/robocommand/blob/main/RobotX_2026/rc_test/test_server.py#L104-L108)) — so
the naming convention is load-bearing in the test tooling at least.

**Networks** (§3.4.2). Four isolated course networks plus a Team Village demo VLAN. RJ-45, DHCP
client mandatory, no Internet:

| Location | Team subnet | OCS address | RoboCommand address |
|---|---|---|---|
| Course Alpha | 192.168.65.0/24 | DHCP 192.168.65.15–254 | **192.168.65.2** |
| Course Bravo | 192.168.66.0/24 | DHCP 192.168.66.15–254 | **192.168.66.2** |
| Course Charlie | 192.168.67.0/24 | DHCP 192.168.67.15–254 | **192.168.67.2** |
| Course Delta | 192.168.68.0/24 | DHCP 192.168.68.15–254 | **192.168.68.2** |
| Team Village demo | Demo network, VLAN 70 | DHCP | *"Provided with the demo connection instructions"* |

The broker address equals the DHCP-assigned default gateway (§3.4.3: *"Confirm that the assigned
default gateway is the course RoboCommand address shown in the table above."*). §3.4.3 also
forbids static addressing, bridging, ICS, and running a DHCP server on that interface, and forbids
routing the vehicle-side network through it.

### 3.2 Transport

- **MQTT over TCP, port 1883** — §3.4.5: *"The current RobotX communications package uses MQTT over
  TCP port 1883."* No TLS mentioned. Broker is **Eclipse Mosquitto 2.x** (`eclipse-mosquitto:2` in
  [`docker-compose.yml`](https://github.com/robonation/robocommand/blob/main/RobotX_2026/docker-compose.yml)).
- **Payload** = raw protobuf `SerializeToString()` bytes. No framing, no length prefix, no
  checksum — MQTT supplies message boundaries. (Contrast RoboBoat's `$R`/`!!` framing, §1.3.)
- **Auth.** The test broker config is two lines: `listener 1883` / `allow_anonymous true`
  ([`mosquitto/mosquitto.conf`](https://github.com/robonation/robocommand/blob/main/RobotX_2026/mosquitto/mosquitto.conf)), and the client connects with a bare
  `client.connect(host, port)` — no username, password, TLS, or client-ID scheme
  ([`test_client.py` L64-L66](https://github.com/robonation/robocommand/blob/main/RobotX_2026/rc_test/test_client.py#L64-L66)). The handbook never mentions credentials; the
  troubleshooting section's only auth-adjacent advice is *"Confirm that local firewall rules allow
  the configured MQTT connection."* **Whether the on-course brokers require credentials is
  UNVERIFIED** — plan for anonymous, keep the username/password as node parameters anyway.
- **QoS and retain.** Not specified anywhere in the handbook. The vendor client publishes with
  paho's defaults (**QoS 0, retain false**) and sets `retain=True` only for `RxCourse`
  ([`test_client.py` L69-L72, L346-L347](https://github.com/robonation/robocommand/blob/main/RobotX_2026/rc_test/test_client.py#L69-L72)). Any specific QoS requirement is **UNVERIFIED**;
  QoS 0 is what the reference implementation does.
- **Connection discipline** (§3.4.5): *"maintain one OCS connection and use it for all required
  subscriptions and publications. If the connection is interrupted, the OCS should reconnect
  automatically and restore its subscriptions."*
- **Clock** (§3.4.8): *"keep the OCS clock synchronized."* Unlike RoboBoat, **no NTP server address
  is given for RobotX** — **UNVERIFIED** whether one is provided on the course network.

### 3.3 Topics

Root `robocommand/robotx` (§3.4.6, and [`RobotX_2026/README.md` L18-L23](https://github.com/robonation/robocommand/blob/main/RobotX_2026/README.md)):

| Purpose | Topic | Direction | Notes |
|---|---|---|---|
| Course configuration | `robocommand/robotx/course` | RC → OCS | **retained** |
| Vehicle-attributed report | `robocommand/robotx/<team_id>/<vehicle_id>/report` | OCS → RC | one per vehicle |
| Team request | `robocommand/robotx/<team_id>/request` | OCS → RC | |
| Team command | `robocommand/robotx/<team_id>/command` | RC → OCS | one per team, not per vehicle |

Example from the handbook: `robocommand/robotx/RNRX/USV1/report`. The OCS subscribes to
`robocommand/robotx/course` and `robocommand/robotx/<team_id>/command`, and *"must not subscribe to
or act on another team's topic namespace."*

### 3.4 Message envelopes, field by field

Four top-level envelopes; three carry a `oneof body` of which *"Only one body type is valid in each
envelope instance."*

**`RxReport`** — OCS → RC ([`rx_reports.proto` L24-L40](https://github.com/robonation/robocommand/blob/main/RobotX_2026/proto/robotx/rx_reports.proto#L24-L40)):

| # | Field | Type | Notes |
|---|---|---|---|
| 1 | `team_id` | string | e.g. `RNRX` |
| 2 | `vehicle_id` | string | must match the topic segment |
| 3 | `seq` | uint32 | **monotonic per vehicle**, not per team |
| 4 | `sent_at` | `google.protobuf.Timestamp` | time the OCS publishes |
| 10 | `heartbeat` | `Heartbeat` | |
| 11 | `safe_passage` | `SafePassageReport` | Task 1 |
| 12 | `pipeline_survey` | `PipelineSurveyReport` | Task 2 |
| 13 | `resource_delivery` | `ResourceDeliveryRequest` | Task 2/3 Adv+ |
| 14 | `docking` | `DockingReport` | Task 3 |
| 15 | `firefighting` | `FirefightingReport` | Task 3 |
| 16 | `incident_ack` | `IncidentAck` | Task 4 |
| 17 | `readiness` | `ReadinessReport` | Task 4 |

**`Heartbeat`** ([`rx_reports.proto` L42-L60](https://github.com/robonation/robocommand/blob/main/RobotX_2026/proto/robotx/rx_reports.proto#L42-L60)) — this is the message this repo has to produce:

| # | Field | Type | Comment in schema |
|---|---|---|---|
| 1 | `state` | `RobotState` | `STATE_UNKNOWN/KILLED/MANUAL/AUTO` = 0/1/2/3 |
| 2 | `position` | `LatLng` | decimal degrees, lat ±90, lng ±180 |
| 3 | `spd_mps` | float | |
| 4 | `heading_deg` | float | |
| 5 | `roll_deg` | float | |
| 6 | `pitch_deg` | float | |
| 7 | `altitude_hae_m` | float | *"altitude (height above elipsoid) in meters"* [sic] |
| 8 | `depth_m` | float | **`// UUV only`** |
| 9 | `current_task` | `RxTask` | *"used to track vehicle tasking for course activation"* |
| 10 | `vehicle_type` | `VehicleType` | `TYPE_USV/UUV/UAV` = 1/2/3 |
| 11 | `flight_phase` | `FlightPhase` | UAV only; others leave `FLIGHT_PHASE_UNKNOWN` |

Enums ([`common.proto`](https://github.com/robonation/robocommand/blob/main/RobotX_2026/proto/common.proto), [`rx_common.proto`](https://github.com/robonation/robocommand/blob/main/RobotX_2026/proto/robotx/rx_common.proto)):

- `RobotState`: `STATE_UNKNOWN=0, STATE_KILLED=1, STATE_MANUAL=2, STATE_AUTO=3` (L15-L20).
- `TaskTier`: `TIER_UNKNOWN=0, TIER_NONE=1, TIER_CORE=2, TIER_ADVANCED=3, TIER_DISRUPTIVE=4` (L30-L36).
- `RxTask`: `TASK_UNKNOWN=0, TASK_NONE=1, TASK_SAFE_PASSAGE=2, TASK_INFRA_SURVEY_REPAIR=3,
  TASK_COORDINATED_LOGISTICS=4, TASK_DYNAMIC_INCIDENT=5` (L15-L22). The schema is emphatic:
  *"`TASK_UNKNOWN` must never be read as standing down"*, and a transition into a task value
  **starts the attempt**, `TASK_NONE` **ends it**. This field is the scoring clock — treat it as
  such.
- `VehicleType`: `TYPE_UNKNOWN=0, TYPE_USV=1, TYPE_UUV=2, TYPE_UAV=3`; *"A UUV is always TYPE_UUV,
  even while surfaced — type reflects vehicle class, not current depth."* (L24-L34).

**`RxRequest`** — OCS → RC ([`rx_requests.proto` L17-L44](https://github.com/robonation/robocommand/blob/main/RobotX_2026/proto/robotx/rx_requests.proto#L17-L44)): `team_id`(1), `seq`(2),
`sent_at`(3), `oneof body { RunDeclaration run_declaration = 10 }`.
`RunDeclaration`: `vehicle_ids`(1, repeated string), `task1_tier`(2) … `task4_tier`(5),
`uav_geofence`(6, repeated `LatLng`, *"The first and last points must be the same"*, *"requested by
Garuda Robotics"*, validated against the course boundary before `RunStart`).

**`RxCommand`** — RC → OCS ([`rx_commands.proto` L27-L108](https://github.com/robonation/robocommand/blob/main/RobotX_2026/proto/robotx/rx_commands.proto#L27-L108)): `team_id`(1), `seq`(2),
`sent_at`(3), `vehicle_id`(4, optional — unset means "all vehicles of the named type"), then
`run_start`(10) `RunStart{declaration_seq, run_id}`, `assistance_request`(11)
`{LatLng position, VehicleType}`, `keep_out_zone`(12) `{LatLng center, float radius_m,
VehicleType}`, `all_clear`(13) `{VehicleType}`, `moving_object_alert`(14)
`{LatLng position, float heading_deg, float speed_mps, repeated VehicleType affected_vehicle_types}`,
`readiness_confirm`(15) `{uint32 report_seq, string vehicle_id}`.

**`RxCourse`** — RC → OCS, retained ([`rx_course.proto` L24-L29](https://github.com/robonation/robocommand/blob/main/RobotX_2026/proto/robotx/rx_course.proto#L24-L29)):
`course_id`(1, string), **`pinger_freq_hz`(2, uint32 — *"acoustic pinger frequency in Hz (for UUV
homing)"*)**, `corners`(3, repeated `LatLng`, closed polygon), `sent_at`(4).

`pinger_freq_hz` is the one field of `RxCourse` that a UUV genuinely needs at runtime. Note it is
distinct from the UUV safety localization beacon, which is a separate RJE ULB-350 at 37/45 kHz
provided by the event team ([§5.3.2](https://robonation.gitbook.io/robotx-2026-team-handbook/section-5-rules-and-requirements/5.3-system-requirements/5.3.2-uuv-requirements)).

**UUV-relevant task reports:**

- `PipelineSurveyReport{ LatLng active_buoy_position = 1; repeated PipelineSegmentStatus segments = 2 }`
  — segments ordered *"from the active (GREEN) buoy end"*, `PIPELINE_SEGMENT_INTACT=1` (GREEN) /
  `PIPELINE_SEGMENT_DAMAGED=2` (RED). Sent **once**, after survey and repair, and *"it can't
  transmit while submerged, so this typically isn't sent until it surfaces"*
  ([`rx_reports.proto` L87-L101](https://github.com/robonation/robocommand/blob/main/RobotX_2026/proto/robotx/rx_reports.proto#L87-L101)).
- `ResourceDeliveryRequest{ RxTask task; Color resource_color; Color delivery_circle_color }` —
  *"Field order matches the flash signal order"* (L103-L122).
- `IncidentAck{uint32 command_seq}` and `ReadinessReport{uint32 command_seq}` for Task 4.

### 3.5 Rates

- **Heartbeat: 2 Hz per vehicle.** §3.4.11: *"The OCS publishes a separate `RxReport` heartbeat on
  behalf of each active vehicle at 2Hz"*. (Compare 1 Hz in 2022 and 1 Hz in RoboBoat 2026 — do not
  copy either.)
- **Cap: ≤5 messages/second per system.** §3.4.5: *"The team should publish no more than five(5)
  messages per second per system unless otherwise specified."* With 2 Hz of heartbeat that leaves
  ~3 msg/s of headroom per vehicle for task reports.
- No timeout / miss-tolerance is specified. **UNVERIFIED** what happens if heartbeats stop
  mid-run.

### 3.6 Run sequence (§3.4.10)

1. OCS connects to the course broker.
2. OCS subscribes to `robocommand/robotx/course` and `robocommand/robotx/<team_id>/command`.
3. OCS receives and validates the retained `RxCourse`.
4. OCS publishes **one** `RxRequest{RunDeclaration}` — vehicle IDs, four task tiers, UAV geofence.
   *"Use `TIER_NONE` when the team is not attempting a task. Do not use `TIER_UNKNOWN` as a
   substitute."*
5. OCS publishes heartbeats for **all** declared vehicles; each reports `STATE_AUTO` when that
   vehicle is autonomous and ready.
6. RoboCommand publishes `RunStart` once the run-start conditions are satisfied.
7. OCS verifies `RunStart.declaration_seq` matches its declaration, records `run_id`, and only then
   do vehicles begin. Until then they **position-hold**.

Then during the run: `Heartbeat.current_task` transitions mark attempts, task reports are published
per §3.4.12, and Task 4 command/ack chains are:
`AssistanceRequest → IncidentAck → ReadinessReport → ReadinessConfirm` (Core);
`KeepOutZone → IncidentAck → AllClear → IncidentAck` (Advanced);
`MovingObjectAlert` with no ack (Disruptive).

§3.4.13 also asks that the application *"Prevent messages from a previous run from being reused in
a new run"* and *"Maintain independent report sequence counters for each vehicle"*.

### 3.7 SDK, languages, tooling

There is **no client library** — only schemas and generated code. The repo ships:

- `proto/` — `common.proto` + `robotx/{rx_common,rx_reports,rx_requests,rx_commands,rx_course}.proto`.
- `gen/python/` — `common_pb2.py`, `robotx/*_pb2.py` (+ `.pyi` stubs).
- `gen/cpp/` — `common.pb.{h,cc}`, `robotx/*.pb.{h,cc}`.
- `rc_test/` — `test_server.py` (subscribes `robocommand/robotx/#`, decodes, logs to
  `./logs/robocommand_<YYYYMMDD>_<HHMMSS>.log`) and `test_client.py` (a full OCS simulator with a
  REPL).
- `docker-compose.yml` + `mosquitto/`.

So: **Python and C++ out of the box** — and the handbook says teams may run `protoc` themselves for
*"Python/C++/Go/etc."* Dependencies for the Python path: `paho-mqtt==2.0`, `protobuf==7.35`
([`rc_test/requirements.txt`](https://github.com/robonation/robocommand/blob/main/RobotX_2026/rc_test/requirements.txt)).

Hard rule from §3.4.4: *"Use the competition-provided schema release without changing package
names, field numbers, enum values, or MQTT topic names. Protocol Buffer field numbers are part of
the wire format."*

### 3.8 The protobuf edition-2024 landmine (verified locally, 2026-08-16)

Every `.proto` starts with `edition = "2024";`, and the README states *"The checked-in `gen/`
output was produced with `protoc`/protobuf runtime **v7.35.0**."* On this machine:

```
$ protoc --version                    # /usr/local/bin/protoc
libprotoc 31.1
$ protoc -I proto --python_out=… proto/common.proto
common.proto: is a file using edition 2024, which is later than the protoc maximum supported edition 2023.

$ python3 -c "import common_pb2"      # system python3-protobuf 3.12.4 (jammy)
ImportError: cannot import name 'runtime_version' from 'google.protobuf'
```

and with the pinned runtime:

```
$ python3 -m venv venv && venv/bin/pip install protobuf==7.35
$ venv/bin/python -c "…RxReport(team_id='ITSN', vehicle_id='UUV1', seq=1); hb.state=STATE_AUTO; hb.vehicle_type=TYPE_UUV; hb.depth_m=2.0…"
OK serialized 25 bytes: 0a044954534e12045555563118015209080345000000405002
runtime 7.35.0
```

**Consequences for this workspace, stated plainly:**

- `apt install protobuf-compiler` / `python3-protobuf` on Jammy is useless here. So is the newer
  `protoc` already on `/usr/local/bin`.
- Do **not** add a `protoc` invocation to any `CMakeLists.txt` or `setup.py`. Vendor
  `gen/python/` verbatim (Apache-2.0 permits it) and pin `protobuf==7.35` in a venv or via `pip`
  outside `rosdep`.
- The **C++** path has the same problem, worse: it would need libprotobuf 7.35 headers and runtime
  alongside ROS Humble's own protobuf dependencies. This is a strong argument for keeping the
  RoboCommand node in **Python**, isolated from the C++ mission stack.

---

## 4. Integrating into this stack

### 4.1 The constraint that decides the design

> *"Only the OCS connects to RoboCommand. Team vehicles must not connect directly to the
> RoboCommand network or MQTT broker."* — [§3.4](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.4-communications-protocol)

Plus §3.4.3: other OCS interfaces *"must not route, bridge, or expose those networks through the
RoboCommand-facing interface."*

So the node **must run on the topside OCS host**, on a machine with two NICs (one to RoboCommand,
one to the vehicle link), with no IP forwarding between them. If the ROS 2 graph already spans the
tether — which for a RobotX UUV it can, since §5.3.2 permits a ≤6 m positively-buoyant tether to
the USV — then a plain ROS 2 subscriber running topside *is* the OCS-side telemetry path, and
nothing on the vehicle changes at all. That is the cheapest possible answer.

Everything else about OCS↔vehicle comms is explicitly out of scope (§3.4.17: the protocol *"does
not prescribe… Vehicle-to-OCS radios or network protocols… Vehicle control architecture"*).

### 4.2 What this stack already publishes

MAVROS 2.14.0, and none of these plugins are in `banyu_bringup/config/mavros_config.yaml`'s
denylist:

| Heartbeat field | Source topic | Type | Fit |
|---|---|---|---|
| `position` | `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | direct — `latitude`/`longitude` are decimal degrees ([`global_position.cpp` L123](https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/plugins/global_position.cpp#L123)) |
| `altitude_hae_m` | same message, `.altitude` | float64 | **exact** — `NavSatFix.altitude` is defined as metres above the WGS-84 ellipsoid, which is what "height above ellipsoid" means |
| `heading_deg` | `/mavros/global_position/compass_hdg` | `std_msgs/Float64` | direct, degrees ([L126](https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/plugins/global_position.cpp#L126)) |
| `depth_m` | `/mavros/global_position/rel_alt` | `std_msgs/Float64` | `depth = -rel_alt`; or `-pose.position.z` from `/mavros/local_position/pose` (already consumed by `banyu_mission.cpp` L104-L106) |
| `spd_mps` | `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | norm of `twist.linear` ([`local_position.cpp` L86-L87](https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/plugins/local_position.cpp#L86-L87)) |
| `roll_deg`, `pitch_deg` | `/mavros/imu/data` | `sensor_msgs/Imu` | quaternion → RPY; or reuse the pose quaternion already in `MissionContext::current_pose` |
| `state` | `/mavros/state` | `mavros_msgs/State` | `armed` + `mode`; already subscribed (`banyu_mission.cpp` L107-L108) |
| `vehicle_type` | — | — | constant `TYPE_UUV` |
| `flight_phase` | — | — | leave `FLIGHT_PHASE_UNKNOWN` (schema says non-UAVs should) |
| `current_task` | **nothing** | — | see §4.3 |

### 4.3 What is missing, honestly

1. **`current_task` has no publisher.** `MissionContext` carries `mission_active` / `aborted`
   atomics and a BT tree, but nothing exposes a task enum
   (`banyu_mission/include/banyu_mission/mission_context.hpp` L37-L54). For a UUV whose only task
   is Task 2, the minimum honest answer is: publish `TASK_INFRA_SURVEY_REPAIR` while
   `mission_active` and the tree is past its first phase, `TASK_NONE` otherwise. One
   `std_msgs/UInt8` publisher in `banyu_mission.cpp`, ~5 lines.
2. **`state` mapping is a judgement call, and the kill switch may be invisible to ROS.** §5.3.2
   requires a physical kill switch that *"instantaneously disconnect[s] power from all propulsion
   systems and actuators"* but explicitly *"does not have to kill the computer."* If it cuts
   thruster power without disarming the autopilot, `/mavros/state.armed` will still read true and
   the heartbeat will lie about `STATE_KILLED`. Decide the mapping deliberately; the safe default
   is `armed && mode=="GUIDED" → STATE_AUTO`, `armed && !GUIDED → STATE_MANUAL`,
   `!armed → STATE_KILLED`, plus a manual operator override on the OCS.
3. **`position` does not exist underwater on hardware.** This is the same finding as the DVL note:
   in SITL, `SIM_GPS1_ENABLE` synthesises a GPS from Gazebo ground truth so
   `/mavros/global_position/global` is populated and everything works; on the real vehicle there is
   no GPS below the surface. The 2 Hz heartbeat still has to carry *something*. Options, none of
   which the handbook adjudicates: hold the last surface fix; dead-reckon topside from the surface
   fix + EKF/DVL local position; or let the tethered USV supply the fix. **Whether RoboCommand
   tolerates a stale or zero `position` in a UUV heartbeat is UNVERIFIED** (open question O4).
4. **The sign convention of `depth_m` is UNVERIFIED.** The schema says only `// UUV only`. Positive-
   down is the overwhelmingly likely intent (it is called *depth*, and `altitude_hae_m` already
   covers signed altitude), but confirm with the Technical Director rather than guess.
5. **`PipelineSurveyReport` has no data source.** It needs the active buoy's lat/lng and an ordered
   segment status list. `banyu_detection` is WIP per `AGENTS.md` and has no pipeline classifier.
   This is a Task-2 perception project, not a comms project — do not let it block §4.4.
6. **`RunDeclaration` requires ≥2 vehicles for PoR** (§2.3) and a UAV geofence. A UUV-only team
   cannot satisfy the run-start sequence alone. See open question O1.

### 4.4 Recommendation — smallest thing that satisfies the rules

**Step 0 (this week, no ROS code, unblocks the 31 Aug deadline).** Clone the repo, `docker compose
up --build`, and drive `test_client.py` through the §3.4.10 sequence with two vehicles:

```bash
docker compose run --rm rc-test python test_client.py run-declaration \
  --team <ASSIGNED_ID> --vehicles USV1 UUV1 --task1 none --task2 core --task3 none --task4 none
docker compose run --rm rc-test python test_client.py heartbeat --team <ASSIGNED_ID> --vehicle USV1 --state auto
docker compose run --rm rc-test python test_client.py heartbeat --team <ASSIGNED_ID> --vehicle UUV1 --state auto
```

The server auto-publishes `RunStart` when both declared vehicles report `STATE_AUTO`
([`test_server.py` L130-L149](https://github.com/robonation/robocommand/blob/main/RobotX_2026/rc_test/test_server.py#L130-L149)); the resulting `./logs/robocommand_*.log` **is** the PoR
artefact §3.1.5 asks for. Note: with a `UAV*` vehicle ID the server demands a ≥4-point geofence, so
declaring `USV1 UUV1` is the path of least resistance for the log — but check O1 first, because the
declaration must reflect what you will actually field.

**Step 1 (the actual integration).** One new package, `banyu_robocommand`, **ament_python**, one
node, `rc_bridge`:

- Vendor `RobotX_2026/gen/python/{common_pb2.py, robotx/}` into
  `banyu_robocommand/banyu_robocommand/rc_proto/` unmodified, with a `VENDORED.md` recording the
  upstream commit (`f6457fa`) and the Apache-2.0 notice. Re-vendor when RoboNation bumps the
  schema; never hand-edit.
- Runtime deps `paho-mqtt` and `protobuf==7.35` come from a venv or `pip --user`, **not** from
  `rosdep`/apt (§3.8). Keep this package out of the default `colcon build --packages-select` line
  in `AGENTS.md` if the venv is not sourced.
- Parameters: `team_id`, `vehicle_id` (default `UUV1`), `broker_host` (default the course gateway),
  `broker_port` (1883), `heartbeat_hz` (2.0), plus optional `mqtt_username`/`mqtt_password` left
  empty.
- Subscribe: the six topics in §4.2 (all `SensorDataQoS` except `/mavros/state`, matching how
  `banyu_mission.cpp` already does it) plus `/banyu/mission/task` (`std_msgs/UInt8`).
- One `create_timer(0.5)` builds and publishes the `RxReport{heartbeat}` with a per-vehicle
  `seq` counter and `sent_at = GetCurrentTime()`.
- Subscribe on MQTT to `robocommand/robotx/course` and `…/<team_id>/command`; decode; log; publish
  `RxCourse.pinger_freq_hz` onto a latched ROS topic. Do **not** wire commands into the BT yet.
- `RunDeclaration` and `RunStart` handling: expose `RunDeclaration` as a ROS service or a one-shot
  CLI flag on the same node, and gate `banyu_mission`'s existing `/banyu/mission/start` service on
  having seen a matching `RunStart`. That reuses machinery that already exists
  (`banyu_mission.cpp` L125-L147) instead of inventing a run-state machine.

**Step 2 (only when Task 2 perception exists).** Add `PipelineSurveyReport`. One more method on the
same node, fed by a new `banyu_msgs` message. Not before.

### 4.5 Sketch

```python
# banyu_robocommand/banyu_robocommand/rc_bridge.py  (elided; ~150 lines total)
class RcBridge(Node):
    def __init__(self):
        super().__init__('rc_bridge')
        # ... declare_parameter for team_id / vehicle_id / broker_host / broker_port ...
        self.mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt.on_message = self._on_mqtt
        self.mqtt.connect(self.broker_host, self.broker_port)   # reconnect handled by loop_start()
        self.mqtt.subscribe(f'robocommand/robotx/{self.team}/command')
        self.mqtt.subscribe('robocommand/robotx/course')
        self.mqtt.loop_start()

        self.create_subscription(NavSatFix,    'mavros/global_position/global',     self._fix,   SensorDataQoS())
        self.create_subscription(Float64,      'mavros/global_position/compass_hdg',self._hdg,   SensorDataQoS())
        self.create_subscription(Float64,      'mavros/global_position/rel_alt',    self._relalt,SensorDataQoS())
        self.create_subscription(TwistStamped, 'mavros/local_position/velocity_local', self._vel, SensorDataQoS())
        self.create_subscription(Imu,          'mavros/imu/data',                   self._imu,   SensorDataQoS())
        self.create_subscription(State,        'mavros/state',                      self._state, 10)
        self.create_subscription(UInt8,        'banyu/mission/task',                self._task,  10)
        self.create_timer(1.0 / self.hz, self._beat)          # 2 Hz per §3.4.11

    def _beat(self):
        self.seq += 1
        r = rx_reports_pb2.RxReport(team_id=self.team, vehicle_id=self.vid, seq=self.seq)
        r.sent_at.GetCurrentTime()
        hb = r.heartbeat                                       # sets the oneof
        hb.state = self._robot_state()                         # see §4.3 item 2
        hb.position.latitude, hb.position.longitude = self.lat, self.lon
        hb.altitude_hae_m = self.alt_hae
        hb.depth_m        = -self.rel_alt                      # sign UNVERIFIED, see §4.3 item 4
        hb.spd_mps, hb.heading_deg = self.speed, self.hdg
        hb.roll_deg, hb.pitch_deg  = self.roll, self.pitch
        hb.current_task = self.task
        hb.vehicle_type = rx_common_pb2.TYPE_UUV
        self.mqtt.publish(f'robocommand/robotx/{self.team}/{self.vid}/report', r.SerializeToString())
```

The `oneof` is set simply by touching `r.heartbeat` — that is why `WhichOneof("body")` on the
server side returns `"heartbeat"` ([`test_server.py` L162](https://github.com/robonation/robocommand/blob/main/RobotX_2026/rc_test/test_server.py#L162)).

### 4.6 What not to build

- **Not `mqtt_client`.** [`ika-rwth-aachen/mqtt_client`](https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml) is released for Humble (2.5.0-1), but it
  bridges *ROS message serialization* over MQTT. RoboCommand wants *protobuf* bytes on
  *specific* topics. You would be fighting it. `paho-mqtt` + the vendored `_pb2` modules is
  strictly less code.
- **Not a C++ node.** §3.8: libprotobuf 7.35 next to ROS Humble is a dependency fight with no
  upside; the heartbeat is 2 Hz.
- **Not a generic "task report framework".** There is exactly one report this UUV can produce
  (`PipelineSurveyReport`) and it does not exist yet.
- **Not a re-implementation of `rc-test`.** The PoR artefact is *their* server's log; running your
  own would defeat the point.
- **Do not touch the mission tree for this.** The heartbeat is read-only telemetry. The only edit
  inside `banyu_mission` should be the one-topic `current_task` publisher (§4.3 item 1) and gating
  the existing start service on `RunStart`.

---

## 5. Open questions / what I could not verify

**Things only a human or a login can settle:**

- **O1 — Which team, and which `team_id`?** The published
  [Team ID List](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.4-communications-protocol/team-id-list) has one ITS entry: **`ITSN` — INSTITUT TEKNOLOGI SEPULUH
  NOPEMBER — "Barunastra ITS"**. There is no "Banyubramanta" row. Either Banyubramanta is the UUV
  inside the `ITSN` SoS (in which case `team_id="ITSN"` and vehicle IDs are shared with Barunastra
  and must be coordinated with them), or the registration is elsewhere. This must be settled before
  anything is published to a real broker — §3.4.14 forbids using another team's identifier even in
  the Team Village demo.
- **O2 — Has the Communications PoR already been submitted by the USV side?** Window closes
  31 Aug 2026 (§2.3). If Barunastra already submitted `rc-test` logs for `ITSN`, this may be done.
- **O3 — Where is the OCS, physically and in the ROS graph?** The design in §4 assumes a topside
  host with two NICs, in the same ROS 2 domain as the vehicle over the tether. If the ROS graph
  currently runs only on the vehicle's companion computer, that is the real integration work, not
  the MQTT part.
- **O4 — What position does a submerged UUV report?** Unanswered by the handbook; ask the Technical
  Director whether stale/zero `LatLng` in a UUV heartbeat is acceptable or penalised (§4.3 item 3).
- **O5 — Sign convention of `Heartbeat.depth_m`** (§4.3 item 4).
- **O6 — Broker authentication and QoS on the actual course brokers.** The reference broker is
  anonymous QoS 0; the handbook is silent (§3.2).
- **O7 — Is there an NTP source on the course network?** RoboBoat has one at 10.10.10.1; RobotX
  §3.4 only says "keep the OCS clock synchronized" (§3.2).
- **O8 — Team Village demo broker address.** §3.4.2 says it is *"Provided with the demo connection
  instructions"* — a document not published on GitBook.
- **O9 — Heartbeat loss tolerance and the points table.** §4.2 of the handbook is still a
  placeholder (§2.2); the per-message scoring and any missed-heartbeat penalty are unknowable until
  the next handbook revision.

**Documents I could not obtain:**

- **Team Village demo connection instructions** (O8) — handed out on site / by the organisers.
- **The next Team Handbook revision** carrying §4.2's autonomy points table and the Qualifying
  Round criteria. Watch the [Change Log](https://robonation.gitbook.io/robotx-2026-team-handbook/introduction-and-table-of-contents/change-log) — comms content moved on 2026-08-07 and could
  move again.
- **The team portal / Proof of Readiness submission form** at
  <https://robotx.org/proof-of-readiness> — login-gated, not fetched.
- **`RobotX-2026-RF_Info-TEMPLATE.xlsx`** (RF emitter declaration, part of the same §3.1.5 PoR) —
  a binary XLSX, not read.
- I did **not** run the stack against the vendor's broker end to end from ROS. The protobuf
  round-trip was verified (§3.8); the ROS wiring in §4.4/§4.5 is a design, not a tested artefact.

**No third-party RobotX-2026-MQTT implementation exists to corroborate against.** GitHub repo
search for `robocommand`, `robotx heartbeat`, `roboboat protobuf` returns only RoboNation's own
repo, two 2022-era NMEA tools, and the RoboBoat-framing NTU repo of §1.3. The schema is 10 days old.

---

## Appendix — source inventory

**RoboNation RobotX 2026 Team Handbook** (live GitBook, <https://robonation.gitbook.io/robotx-2026-team-handbook>, fetched 2026-08-16;
Markdown views via the `.md` suffix, index at `/llms.txt`):
`introduction-and-table-of-contents/change-log`,
`section-3-autonomy-challenge/3.1-proof-of-readiness`,
`section-3-autonomy-challenge/3.2-mandatory-activities`,
`section-3-autonomy-challenge/3.3-mission-task-descriptions/3.3.1-cross-cutting-tasks`,
`…/3.3.3-mission-task-2-infrastructure-survey-and-repair`,
`section-3-autonomy-challenge/3.4-communications-protocol` (+ `…/team-id-list`),
`section-3-autonomy-challenge/hidden-3.5-qualifying-round`, `…/hidden-3.6-scoring-rounds`,
`hidden-section-4-scoring-and-awards`,
`section-5-rules-and-requirements/5.3-system-requirements/5.3.2-uuv-requirements`

**RoboNation PDFs**
[`Team-Handbook_RobotX-2026-20260330.pdf`](https://robonation.org/app/uploads/sites/2/2025/10/Team-Handbook_RobotX-2026-20260330.pdf) (stale §3.4, see §2.4);
[`2022-RobotX_Team-Handbook_v3.0.pdf`](https://robonation.org/app/uploads/sites/2/2022/10/2022-RobotX_Team-Handbook_v3.0.pdf) Appendix C (the `$RXHRB` predecessor)

**RoboNation RoboBoat Resources** [`section-3-autonomy-challenge/3.3-communications-protocol`](https://robonation.gitbook.io/roboboat-resources/section-3-autonomy-challenge/3.3-communications-protocol) (the *other* RoboCommand protocol)

**RoboNation RoboSub Resources** <https://robonation.gitbook.io/robosub-resources> — index + six
pages checked, no RoboCommand (§1.4)

**[`robonation/robocommand`](https://github.com/robonation/robocommand)** @ `f6457fa` (2026-08-06), Apache-2.0, cloned and executed locally:
`README.md`, `RobotX_2026/README.md`,
`RobotX_2026/proto/common.proto`,
`RobotX_2026/proto/robotx/{rx_common,rx_reports,rx_requests,rx_commands,rx_course}.proto`,
`RobotX_2026/gen/{python,cpp}/`,
`RobotX_2026/rc_test/{test_server.py,test_client.py,requirements.txt}`,
`RobotX_2026/{docker-compose.yml,mosquitto/mosquitto.conf}`

**MAVROS** `mavlink/mavros` tag `2.14.0`: `mavros/src/plugins/{global_position,local_position}.cpp`;
local `/opt/ros/humble/share/mavros/mavros_plugins.xml`

**This repo** `src/AGENTS.md`, `src/banyu_bringup/config/mavros_config.yaml`,
`src/banyu_mission/src/banyu_mission.cpp`,
`src/banyu_mission/include/banyu_mission/mission_context.hpp`,
`src/docs/dvl-integration-research.md`

**Local toolchain checks (2026-08-16)** `protoc` 31.1 (`/usr/local/bin/protoc`);
`python3-protobuf` 3.12.4 (jammy); `python3-paho-mqtt` candidate 1.5.1-1 (jammy, not installed);
venv `protobuf==7.35.0` round-trip verified

**SECONDARY (labelled as such in text)** [`MShandilya707/AAV_OCS_Robocommand`](https://github.com/MShandilya707/AAV_OCS_Robocommand) (NTU Archimedes,
RoboBoat framing, pre-dates the RobotX schema); [`ika-rwth-aachen/mqtt_client`](https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml) 2.5.0-1 for Humble
