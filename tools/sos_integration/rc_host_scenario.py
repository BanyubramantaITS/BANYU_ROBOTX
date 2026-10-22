#!/usr/bin/env python3
"""Drive the RoboCommand host console through two runs with a run ID / seq reset in between.

Runs the real robocommand-rx-ui-itsn MainWindow (Qt) against the broker and presses its buttons
in order, while watching the UUV over ROS:

  Course tab: publish the course -> Run start: Allow start (run 1) -> UUV run SUCCEEDED and
  RxReport.pipeline_survey received -> End run -> GCS /gcs_ros_bridge/new_run ->
  Reset run ID & seq -> prepare_autonomy -> Allow start (run 1 again, seq 1) -> second survey.

Needs the procedure in README.md running (broker, gcs_ros_bridge.py, dds_relay.py, sim). Run with
ROS sourced; GCS_REPO points at the robocommand-rx-ui-itsn checkout:
  python3 rc_host_scenario.py OUT_DIR   # GCS_REPO defaults to <workspace>/../robocommand-rx-ui-itsn
"""
import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path

# Default: robocommand-rx-ui-itsn checked out next to the colcon workspace.
_DEFAULT_GCS_REPO = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', '..',
    'robocommand-rx-ui-itsn'))
GCS_REPO = os.environ.get('GCS_REPO', _DEFAULT_GCS_REPO)
sys.path.insert(0, GCS_REPO)
os.environ.setdefault('ROBOCOMMAND_MAP_TILES', '0')

from PyQt5.QtCore import QTimer  # noqa: E402
from PyQt5.QtWidgets import QApplication, QMessageBox  # noqa: E402

import rclpy  # noqa: E402
from rclpy.context import Context  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from banyu_msgs.msg import RunLifecycle, Task2BuoyTarget  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
PHASES = ['IDLE', 'PREPARING', 'READY', 'INITIALIZING', 'ACTIVE', 'SURVEY_COMPLETE',
          'SURFACED', 'SUCCEEDED', 'FAILED', 'INTERRUPTED']
EVENTS = ['SNAPSHOT', 'START_ACCEPTED', 'START_DUPLICATE', 'START_REJECTED', 'TERMINAL']


def ros_side(domain_id, name):
    context = Context()
    rclpy.init(context=context, domain_id=domain_id)
    node = Node(name, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    return node


class Scenario:
    def __init__(self, args, window):
        self.args = args
        self.win = window
        self.log_file = open(os.path.join(args.out, 'scenario.jsonl'), 'a', buffering=1)
        self.lifecycle = None
        self.lifecycle_events = []
        self.targets = []
        self.results = {'runs': []}
        self.uuv = ros_side(args.uuv_domain, 'rc_host_scenario_uuv')
        self.gcs = ros_side(args.gcs_domain, 'rc_host_scenario_gcs')
        self.uuv.create_subscription(RunLifecycle, '/banyu/mission/run_lifecycle',
                                     self.on_lifecycle, LATCHED)
        self.uuv.create_subscription(Task2BuoyTarget, '/banyu/task2/buoy_target',
                                     lambda m: self.targets.append(m), LATCHED)
        self.prepare = self.uuv.create_client(Trigger, '/banyu/mission/prepare_autonomy')
        self.new_run = self.gcs.create_client(Trigger, '/gcs_ros_bridge/new_run')
        self.steps = self.run()
        self.waiting = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(200)

    def record(self, kind, **data):
        data.update(kind=kind, t=time.time())
        self.log_file.write(json.dumps(data) + '\n')
        print(f'[scenario] {kind} {data}', flush=True)

    def on_lifecycle(self, m):
        key = (m.phase, m.event, m.run_id, m.epoch, m.reason)
        if self.lifecycle is None or key != (self.lifecycle.phase, self.lifecycle.event,
                                             self.lifecycle.run_id, self.lifecycle.epoch,
                                             self.lifecycle.reason):
            self.lifecycle_events.append(m)
            self.record('lifecycle', phase=PHASES[m.phase], event=EVENTS[m.event],
                        run_id=m.run_id, epoch=m.epoch, seq=m.command_seq, reason=m.reason,
                        requested=m.requested_run_id)
        self.lifecycle = m

    def screenshot(self, name):
        path = os.path.join(self.args.out, f'{name}.png')
        self.win.grab().save(path)
        self.record('screenshot', path=path)

    def team(self):
        return self.win.fleet.teams.get(self.args.team)

    def surveys(self):
        track = self.win.fleet.vehicles.get((self.args.team, self.args.vehicle))
        return track.reports.get('pipeline_survey', 0) if track else 0

    def call(self, client, label):
        if not client.wait_for_service(timeout_sec=2.0):
            self.record('service_unavailable', service=label)
            return None
        future = client.call_async(Trigger.Request())
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        response = future.result() if future.done() else None
        self.record('service', service=label, success=bool(response and response.success),
                    message=response.message if response else 'timeout')
        return response

    def wait(self, label, predicate, timeout_s):
        """Yield until predicate() is true; raise on timeout."""
        deadline = time.time() + timeout_s
        self.record('waiting', what=label)
        while not predicate():
            if time.time() > deadline:
                raise TimeoutError(label)
            yield
        self.record('done', what=label)

    def run(self):
        args = self.args
        win = self.win
        yield from self.wait('broker connected', lambda: win.mqtt.is_connected, 15)

        # Course tab: the operator types the course and presses Publish.
        win.tabs.setCurrentWidget(win.course_tab)
        win.course_tab.set_fields(args.course_id, args.pinger_hz, args.corners)
        win.course_tab.publish_btn.click()
        self.record('pressed', button='Publish course (retained)', course_id=args.course_id)
        yield from self.wait('team DECLARED', lambda: self.team() is not None and
                             self.team().run_state == 'DECLARED', 30)
        self.screenshot('01_course_declared')

        for index in (1, 2):
            yield from self.one_run(index)
            if index == 1:
                yield from self.between_runs()
        self.record('scenario_complete', results=self.results)
        QApplication.instance().quit()

    def one_run(self, index):
        args = self.args
        win = self.win
        surveys_before = self.surveys()
        epoch_before = self.lifecycle.epoch if self.lifecycle else 0

        # Operator prepares the vehicle (position hold), retried while the sim settles.
        deadline = time.time() + args.prepare_timeout_s
        while True:
            response = self.call(self.prepare, 'prepare_autonomy')
            if response and response.success:
                break
            if time.time() > deadline:
                raise TimeoutError('prepare_autonomy')
            pause = time.time() + 3.0
            while time.time() < pause:
                yield
        yield from self.wait('UUV READY', lambda: self.lifecycle is not None and
                             self.lifecycle.phase == RunLifecycle.READY and
                             self.lifecycle.autonomous_ready, args.prepare_timeout_s)

        # Run start tab: select the team, Run ID 1, Allow start.
        win.tabs.setCurrentWidget(win.run_start_tab)
        win.run_start_tab.refresh(win.fleet)
        win.run_start_tab.select_team(args.team)
        win.run_start_tab.run_id.setValue(args.run_id)
        next_seq = win.sender.next_seq(args.team)
        win.run_start_tab.start_btn.click()
        self.record('pressed', button='Allow start', run_id=args.run_id, command_seq=next_seq)
        self.screenshot(f'0{index + 1}_run{index}_started')

        yield from self.wait(
            'UUV accepted the start',
            lambda: self.lifecycle.epoch > epoch_before and self.lifecycle.run_id == args.run_id
            and self.lifecycle.phase >= RunLifecycle.INITIALIZING, 30)
        epoch = self.lifecycle.epoch
        rejected = [e for e in self.lifecycle_events if e.event == RunLifecycle.START_REJECTED and
                    e.requested_run_id == args.run_id and e.epoch >= epoch_before]
        yield from self.wait('run terminal', lambda: self.lifecycle.epoch == epoch and
                             self.lifecycle.phase >= RunLifecycle.SUCCEEDED, args.run_timeout_s)
        outcome = PHASES[self.lifecycle.phase]
        if self.lifecycle.phase == RunLifecycle.SUCCEEDED:
            yield from self.wait('RxReport.pipeline_survey at RoboCommand',
                                 lambda: self.surveys() > surveys_before, 30)
        team = self.team()
        target = self.targets[-1] if self.targets else None
        self.results['runs'].append({
            'run': index, 'run_id': args.run_id, 'command_seq': next_seq, 'epoch': epoch,
            'outcome': outcome, 'reason': self.lifecycle.reason,
            'start_rejections_before_accept': len(rejected),
            'surveys_at_robocommand': self.surveys(),
            'pipeline_report': None if team.pipeline is None else {
                'active_buoy': list(team.pipeline[0]), 'segments': list(team.pipeline[1])},
            'buoy_target': None if target is None else {
                'seq': target.source_seq, 'lat': target.latitude, 'lon': target.longitude,
                'heading_enu_rad': target.heading_enu_rad, 'locked': target.locked}})
        self.record('run_result', **self.results['runs'][-1])
        self.screenshot(f'0{index + 1}_run{index}_report')
        if outcome != 'SUCCEEDED':
            raise RuntimeError(f'run {index} ended {outcome}: {self.lifecycle.reason}')

    def between_runs(self):
        win = self.win
        args = self.args
        win.tabs.setCurrentWidget(win.run_start_tab)
        win.run_start_tab.select_team(args.team)
        win.run_start_tab.end_btn.click()
        self.record('pressed', button='End run')
        yield from self.wait('team ENDED', lambda: self.team().run_state == 'ENDED', 10)
        declaration = self.team().declaration_seq
        self.call(self.new_run, 'gcs new_run')
        yield from self.wait('new declaration', lambda: self.team().run_state == 'DECLARED' and
                             self.team().declaration_seq != declaration, 15)
        win.run_start_tab.refresh(win.fleet)
        win.run_start_tab.select_team(args.team)
        win.run_start_tab.reset_btn.click()
        self.record('pressed', button='Reset run ID & seq',
                    next_run_id=win.run_start_tab.run_id.value(),
                    next_seq=win.sender.next_seq(args.team))
        self.screenshot('04_after_reset')

    def tick(self):
        try:
            next(self.steps)
        except StopIteration:
            self.timer.stop()
        except Exception as error:      # report and stop the scenario
            self.timer.stop()
            self.record('scenario_failed', error=repr(error), results=self.results)
            self.screenshot('99_failure')
            QApplication.instance().exit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('out', help='directory for scenario.jsonl and screenshots')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--team', default='ITSN')
    parser.add_argument('--vehicle', default='UUV1')
    parser.add_argument('--run-id', type=int, default=1)
    parser.add_argument('--course-id', default='ITSN-SIM-A')
    parser.add_argument('--pinger-hz', type=int, default=25000)
    parser.add_argument('--uuv-domain', type=int, default=40)
    parser.add_argument('--gcs-domain', type=int, default=10)
    parser.add_argument('--prepare-timeout-s', type=float, default=120.0)
    parser.add_argument('--run-timeout-s', type=float, default=600.0)
    args = parser.parse_args()
    args.corners = [(33.8093, -118.3949), (33.8093, -118.3929), (33.8113, -118.3929),
                    (33.8113, -118.3949)]
    os.makedirs(args.out, exist_ok=True)

    from robocommand_ui.main_window import MainWindow
    app = QApplication(sys.argv)
    window = MainWindow(run_log_dir=Path(args.out) / 'runs',
                        presets_path=Path(args.out) / 'presets.json')
    # Scripted operator: answer the console's confirmation dialogs with Yes, never block.
    window._confirm = lambda title, text: True
    QMessageBox.warning = staticmethod(lambda *a, **k: print('[console warning]', a[2:]))
    window.show()
    window._on_connect(args.host, args.port)
    scenario = Scenario(args, window)
    code = app.exec_()
    window.mqtt.stop()
    window.mqtt.wait(3000)
    print(json.dumps(scenario.results, indent=2))
    sys.exit(code)


if __name__ == '__main__':
    main()
