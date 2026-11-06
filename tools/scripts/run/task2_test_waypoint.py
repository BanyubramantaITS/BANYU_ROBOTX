#!/usr/bin/env python3
"""Send the Task 2 test mission its buoys and start it (launch_task2.sh --test-mission).

Stands in for the GCS and the USV on the AUV's domain, so after SB Up (auto) one command starts the run:

  1. waits until /banyu/vehicle_status reports SB Up (operational_mode 2),
  2. publishes the GCS course (a square around the buoys, ~1 Hz) unless --no-course,
  3. calls /banyu/mission/prepare_autonomy, waits for READY and sends CMD_RUN_START (as the GCS does) unless --no-start,
  4. publishes the buoys as the USV's rx_msgs/PipelineBuoys (~1 Hz): LAT LON = the start (active) buoy A, --end LAT LON =
     the end buoy B. With both the AUV dives midway between them facing the end buoy (the real midpoint strategy);
     with the start buoy only it dives inside it. Nothing is published without LAT LON (the USV sends them),
  5. prints the run lifecycle and the UUV heartbeat, then the PipelineSurvey the AUV sends after surfacing.

The link drops while the AUV is under water; the tool keeps going and shows the report once it is back.
In the sim leave out LAT LON: task2_sim_provider.py publishes the world's two buoys, as the USV does.

Run on the laptop (or the Pi) in the UUV domain:
  ROS_DOMAIN_ID=40 ./tools/scripts/run/task2_test_waypoint.py -7.2834567 112.7948123 --end -7.2834567 112.7948823
"""
import argparse
import math
import os
import sys
import time

import rclpy
from banyu_msgs.msg import RunLifecycle, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rx_msgs.msg import Command, Course, Heartbeat, LatLng, PipelineBuoys, PipelineSurvey, VehicleType
from std_srvs.srv import Trigger

SYSTEM_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
LATCHED_10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
PHASES = ['IDLE', 'PREPARING', 'READY', 'INITIALIZING', 'ACTIVE', 'SURVEY_COMPLETE', 'SURFACED', 'SUCCEEDED',
          'FAILED', 'INTERRUPTED']
EVENTS = ['SNAPSHOT', 'START_ACCEPTED', 'START_DUPLICATE', 'START_REJECTED', 'TERMINAL']
SEGMENTS = {0: 'UNKNOWN', 1: 'GREEN', 2: 'RED'}


class TestWaypoint(Node):
    def __init__(self, args):
        super().__init__('task2_test_waypoint')
        self.args = args
        self.status = None
        self.lifecycle = None
        self.heartbeat = None
        self.survey = None
        self.start_event = None
        self.started_at = time.time()
        self.course_pub = self.create_publisher(Course, '/system/mission/course', SYSTEM_QOS)
        self.cmd_pub = self.create_publisher(Command, '/system/mission/command', SYSTEM_QOS)
        self.buoys_pub = self.create_publisher(PipelineBuoys, '/system/vehicle/usv/task2/pipeline_buoys', SYSTEM_QOS)
        self.prepare = self.create_client(Trigger, '/banyu/mission/prepare_autonomy')
        self.create_subscription(VehicleStatus, '/banyu/vehicle_status', self.on_status, 10)
        self.create_subscription(RunLifecycle, '/banyu/mission/run_lifecycle', self.on_lifecycle, LATCHED)
        self.create_subscription(Heartbeat, '/system/vehicle/uuv/heartbeat', self.on_heartbeat, SYSTEM_QOS)
        self.create_subscription(PipelineSurvey, '/system/vehicle/uuv/task2/pipeline_survey', self.on_survey,
                                 LATCHED_10)
        self.course_on = False
        self.buoys_on = False
        self.expect_survey = False
        self.create_timer(1.0, self.tick)

    def log(self, text):
        print(f'[{time.time() - self.started_at:6.1f} s] {text}', flush=True)

    def on_status(self, msg):
        self.status = msg

    def on_lifecycle(self, msg):
        previous = self.lifecycle
        self.lifecycle = msg
        if msg.event in (RunLifecycle.START_ACCEPTED, RunLifecycle.START_REJECTED, RunLifecycle.START_DUPLICATE):
            self.start_event = msg
        if previous is None or previous.phase != msg.phase or msg.event != RunLifecycle.SNAPSHOT:
            reason = f' ({msg.reason})' if msg.reason else ''
            self.log(f'run {msg.run_id} epoch {msg.epoch}: {PHASES[msg.phase] if msg.phase < len(PHASES) else msg.phase}'
                     f' {EVENTS[msg.event] if msg.event < len(EVENTS) else msg.event}{reason}')

    def on_heartbeat(self, msg):
        self.heartbeat = msg

    def on_survey(self, msg):
        # The topic is latched: the report of an earlier run arrives as soon as we subscribe.
        if not self.expect_survey:
            self.log('(ignoring the report of an earlier run)')
            return
        self.survey = msg
        segments = ' '.join(SEGMENTS.get(s, str(s)) for s in msg.segments)
        self.log(f'PIPELINE SURVEY received: active buoy {msg.active_buoy_position.latitude:.7f}, '
                 f'{msg.active_buoy_position.longitude:.7f}, segments [{segments}]')

    def tick(self):
        if self.course_on:
            self.publish_course()
        if self.buoys_on:
            self.publish_buoys()

    def publish_course(self):
        lat, lon, half = self.args.course_lat, self.args.course_lon, self.args.half_size_m
        m_lat, m_lon = 111320.0, 111320.0 * math.cos(math.radians(lat))
        corners = [LatLng(latitude=lat + y * half / m_lat, longitude=lon + x * half / m_lon)
                   for x, y in [(-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1)]]
        course = Course()
        course.course_id = self.args.course_id
        course.pinger_freq_hz = self.args.pinger_hz
        course.boundary = corners
        course.uav_geofence = corners
        self.course_pub.publish(course)

    def publish_buoys(self):
        msg = PipelineBuoys()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.course_id = self.args.course_id
        msg.source_vehicle = VehicleType.TYPE_USV
        msg.seq = self.args.buoy_seq
        msg.buoy_a = LatLng(latitude=self.args.lat, longitude=self.args.lon)
        if self.args.end is not None:
            msg.buoy_b = LatLng(latitude=self.args.end[0], longitude=self.args.end[1])
        msg.buoy_b_valid = self.args.end is not None
        msg.active_buoy = PipelineBuoys.ACTIVE_A
        msg.uncertainty_m = self.args.uncertainty_m
        self.buoys_pub.publish(msg)

    def spin_for(self, seconds):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_until(self, condition, timeout_s, waiting_text, every_s=5.0):
        end = time.time() + timeout_s
        last = 0.0
        while rclpy.ok() and time.time() < end:
            if condition():
                return True
            if time.time() - last >= every_s:
                self.log(waiting_text())
                last = time.time()
            rclpy.spin_once(self, timeout_sec=0.1)
        return condition()

    def status_text(self):
        s = self.status
        if s is None:
            return 'waiting for /banyu/vehicle_status (stack running? ROS_DOMAIN_ID 40?)'
        mode = {0: 'KILL (SB Down)', 1: 'MANUAL (SB Mid)', 2: 'AUTO (SB Up)'}.get(s.operational_mode, '?')
        return (f'waiting for SB Up: switch {mode}, armed {s.armed}, FCU mode {s.mode}, rc_fresh {s.rc_fresh}, '
                f'gps {s.gps_valid and s.gps_fresh}')

    def auto_ready(self):
        s = self.status
        return s is not None and s.operational_mode == 2 and not s.hardware_kill_active

    def prepare_and_start(self):
        if not self.prepare.wait_for_service(timeout_sec=10.0):
            self.log('ERROR: /banyu/mission/prepare_autonomy not available (stack not running in mode:=task2?)')
            return False
        for attempt in range(1, self.args.prepare_attempts + 1):
            future = self.prepare.call_async(Trigger.Request())
            while rclpy.ok() and not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
            response = future.result()
            self.log(f'prepare_autonomy try {attempt}: {response.message}')
            if response.success and self.wait_until(
                    lambda: self.lifecycle is not None and self.lifecycle.phase == RunLifecycle.READY, 35.0,
                    lambda: 'preparing (GUIDED, position hold)...'):
                break
            self.spin_for(3.0)
        else:
            self.log('ERROR: the AUV did not become READY; check SB Up, GPS fix and the mission log')
            return False
        self.spin_for(1.0)
        cmd = Command()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'gcs'
        cmd.command_type = Command.CMD_RUN_START
        cmd.command_seq = self.args.command_seq
        cmd.run_id = self.args.run_id
        cmd.target_vehicle = VehicleType.TYPE_UUV
        self.start_event = None
        self.cmd_pub.publish(cmd)
        self.log(f'CMD_RUN_START sent: run {cmd.run_id}, seq {cmd.command_seq}')
        if not self.wait_until(lambda: self.start_event is not None, 10.0, lambda: 'waiting for the start reply...'):
            self.log('ERROR: no reply to CMD_RUN_START')
            return False
        if self.start_event.event == RunLifecycle.START_REJECTED:
            self.log(f'ERROR: run start rejected: {self.start_event.reason}')
            return False
        return True

    def run(self):
        domain = os.environ.get('ROS_DOMAIN_ID', '0')
        if domain != '40':
            self.log(f'Warning: ROS_DOMAIN_ID is {domain}; the UUV side runs on 40')
        if self.args.lat is not None:
            self.log(f'start buoy {self.args.lat:.7f}, {self.args.lon:.7f} (active)')
            if self.args.end is not None:
                self.log(f'end buoy {self.args.end[0]:.7f}, {self.args.end[1]:.7f}: dive midway, facing the end buoy')
            else:
                self.log('no end buoy (--end): the AUV dives inside the start buoy')
        if not self.wait_until(self.auto_ready, self.args.auto_timeout_s, self.status_text):
            self.log('ERROR: SB never went to AUTO (Up); nothing was started')
            return 1
        self.log('SB Up (AUTO)')
        if self.args.course:
            self.course_on = True
            self.publish_course()
            self.log(f'publishing course {self.args.course_id} ({self.args.half_size_m:.0f} m square)')
            self.spin_for(2.0)
        if self.args.start and not self.prepare_and_start():
            return 1
        self.expect_survey = True
        if self.args.lat is not None:
            self.buoys_on = True
            self.publish_buoys()
            self.log('publishing the buoys (PipelineBuoys ~1 Hz) until the report arrives')
        last_hb = 0.0
        end = time.time() + self.args.timeout_s
        while rclpy.ok() and time.time() < end and self.survey is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.heartbeat is not None and time.time() - last_hb >= self.args.heartbeat_every_s:
                hb = self.heartbeat
                self.log(f'UUV heartbeat: {hb.position.latitude:.7f}, {hb.position.longitude:.7f}, '
                         f'depth {hb.depth_m:.2f} m, heading {hb.heading_deg:.0f} deg, task {hb.current_task}')
                last_hb = time.time()
            if self.lifecycle is not None and self.lifecycle.phase in (
                    RunLifecycle.FAILED, RunLifecycle.INTERRUPTED):
                self.log(f'run ended without a report: {PHASES[self.lifecycle.phase]} ({self.lifecycle.reason})')
                return 1
        if self.survey is None:
            self.log('ERROR: no PipelineSurvey within the timeout')
            return 1
        self.spin_for(2.0)
        return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('lat', type=float, nargs='?', help='start (active) buoy latitude (deg); omit to leave the buoys to the USV')
    ap.add_argument('lon', type=float, nargs='?', help='start buoy longitude (deg)')
    ap.add_argument('--end', type=float, nargs=2, metavar=('LAT', 'LON'),
                    help='end buoy: the AUV then dives midway between the buoys, facing the end buoy')
    ap.add_argument('--no-course', dest='course', action='store_false',
                    help='do not publish the course (the real GCS does)')
    ap.add_argument('--no-start', dest='start', action='store_false',
                    help='do not prepare/start the run (the real GCS sends CMD_RUN_START)')
    ap.add_argument('--course-id', default='BANYU-TEST')
    ap.add_argument('--course-center', type=float, nargs=2, metavar=('LAT', 'LON'),
                    help='centre of the course square (default: the waypoint)')
    ap.add_argument('--half-size-m', type=float, default=300.0, help='half side of the course square (m)')
    ap.add_argument('--pinger-hz', type=int, default=25000, help='course pinger frequency (must be non-zero)')
    ap.add_argument('--run-id', type=int, default=1)
    ap.add_argument('--command-seq', type=int, default=1,
                    help='prepare_autonomy forgets earlier runs, so run 1 / seq 1 works every time')
    ap.add_argument('--buoy-seq', type=int, default=1)
    ap.add_argument('--uncertainty-m', type=float, default=0.5, help='reported buoy position uncertainty (m)')
    ap.add_argument('--prepare-attempts', type=int, default=3)
    ap.add_argument('--auto-timeout-s', type=float, default=600.0, help='how long to wait for SB Up')
    ap.add_argument('--timeout-s', type=float, default=1800.0, help='how long to wait for the report')
    ap.add_argument('--heartbeat-every-s', type=float, default=5.0)
    args = ap.parse_args()
    if (args.lat is None) != (args.lon is None):
        ap.error('give both LAT and LON, or neither')
    if args.lat is not None and not (abs(args.lat) <= 90.0 and abs(args.lon) <= 180.0):
        ap.error('LAT/LON out of range')
    if args.end is not None and args.lat is None:
        ap.error('--end needs the start buoy LAT LON')
    if args.end is not None and not (abs(args.end[0]) <= 90.0 and abs(args.end[1]) <= 180.0):
        ap.error('--end LAT/LON out of range')
    if args.lat is None:
        center = args.course_center
    elif args.end is None:
        center = args.course_center or (args.lat, args.lon)
    else:
        center = args.course_center or (0.5 * (args.lat + args.end[0]), 0.5 * (args.lon + args.end[1]))
    if args.course and center is None:
        ap.error('without LAT LON the course needs --course-center LAT LON (or --no-course)')
    if center is not None:
        args.course_lat, args.course_lon = center

    rclpy.init()
    node = TestWaypoint(args)
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
