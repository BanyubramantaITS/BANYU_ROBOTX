# banyu_robocommand

OCS-side relay from the ROS graph to RoboCommand (RobotX 2026 handbook 3.4). Runs on the topside
Operator Control Station only: vehicles never connect to RoboCommand themselves.

- `report_bridge`: `/banyu/task2/pipeline_survey` (`banyu_msgs/PipelineSurveyReport`, published by
  the surfaced UUV) -> `RxReport{pipeline_survey}` on `robocommand/robotx/<team_id>/<vehicle_id>/report`.
- `rc_proto/`: RoboNation's generated protobuf classes, unchanged (see its README for the release).

Not rebuilt yet (the old `rc_bridge` did these; its source is lost): 2 Hz heartbeats per vehicle,
`RunDeclaration`, `RxCourse`/command subscriptions. They share `RxReport.seq` with the report.

## Run

The generated classes need protobuf 7.35, newer than Ubuntu's `python3-protobuf`, so use a venv
that still sees the ROS packages:

```bash
python3 -m venv --system-site-packages ~/rc_venv
~/rc_venv/bin/pip install -r src/banyu_robocommand/requirements.txt
source install/setup.bash
~/rc_venv/bin/python -m banyu_robocommand.report_bridge --ros-args \
  -p team_id:=ITSN -p vehicle_id:=UUV1 -p broker_host:=<course broker>
```

Local test broker + decoding server: `docker compose up --build` in RoboNation's
`robocommand/RobotX_2026`, then `broker_host:=localhost`. Tests: `~/rc_venv/bin/python -m pytest src/banyu_robocommand/test`.
