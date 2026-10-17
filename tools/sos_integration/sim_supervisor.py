#!/usr/bin/env python3
"""Simulation-only operator stand-in for headless SITL runs (UUV domain 40).

ArduSub SITL reports RC_CHANNELS with zero channels, so /mavros/rc/in never makes rc_fresh true.
This publishes a neutral supervisor RC stream on /sim/supervisor_rc (point the vehicle status
node's rc_topic at it, never on hardware) and logs RunLifecycle transitions, the received Course,
incoming /system/mission/command messages, the USV PipelineBuoys feed and the resulting Task 2
buoy target to JSONL.

Usage: ROS_DOMAIN_ID=40 python3 sim_supervisor.py OUT.jsonl
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from banyu_msgs.msg import RunLifecycle, Task2BuoyTarget
from mavros_msgs.msg import RCIn
from rx_msgs.msg import Command, Course, PipelineBuoys

PHASES = ['IDLE', 'PREPARING', 'READY', 'INITIALIZING', 'ACTIVE', 'SURVEY_COMPLETE',
          'SURFACED', 'SUCCEEDED', 'FAILED', 'INTERRUPTED']
EVENTS = ['SNAPSHOT', 'START_ACCEPTED', 'START_DUPLICATE', 'START_REJECTED', 'TERMINAL']
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SimSupervisor(Node):
    def __init__(self, log_path):
        super().__init__('sim_supervisor')
        self.out = open(log_path, 'a', buffering=1)
        self.rc = self.create_publisher(RCIn, '/sim/supervisor_rc', 10)
        self.create_timer(0.1, lambda: self.rc.publish(RCIn(channels=[1500] * 8)))
        self.last = None
        self.create_subscription(RunLifecycle, '/banyu/mission/run_lifecycle', self.on_lifecycle,
                                 LATCHED)
        self.create_subscription(Course, '/system/mission/course', self.on_course, LATCHED)
        self.create_subscription(Command, '/system/mission/command', self.on_command, 10)
        self.last_buoys = None
        self.create_subscription(PipelineBuoys, '/system/vehicle/usv/task2/pipeline_buoys',
                                 self.on_buoys, LATCHED)
        self.create_subscription(Task2BuoyTarget, '/banyu/task2/buoy_target', self.on_target,
                                 LATCHED)

    def record(self, kind, **data):
        data.update(kind=kind, t=time.time())
        self.out.write(json.dumps(data) + '\n')
        print(f'[uuv] {kind} {data}', flush=True)

    def on_lifecycle(self, m):
        key = (m.phase, m.event, m.reason, m.run_id)
        if key != self.last:
            self.last = key
            self.record('lifecycle', phase=PHASES[m.phase], event=EVENTS[m.event], run_id=m.run_id,
                        epoch=m.epoch, seq=m.command_seq, task=m.current_task, reason=m.reason,
                        requested=m.requested_run_id)

    def on_course(self, m):
        self.record('course_received', course_id=m.course_id, pinger=m.pinger_freq_hz,
                    corners=len(m.boundary))

    def on_command(self, m):
        self.record('command_received', type=m.command_type, seq=m.command_seq, run_id=m.run_id,
                    target=m.target_vehicle)

    def on_buoys(self, m):
        if (m.course_id, m.seq) != self.last_buoys:
            self.last_buoys = (m.course_id, m.seq)
            self.record('usv_pipeline_buoys', course_id=m.course_id, seq=m.seq,
                        active=m.active_buoy, buoy_b_valid=m.buoy_b_valid,
                        a=[m.buoy_a.latitude, m.buoy_a.longitude],
                        b=[m.buoy_b.latitude, m.buoy_b.longitude])

    def on_target(self, m):
        self.record('buoy_target', seq=m.source_seq, lat=m.latitude, lon=m.longitude,
                    heading_valid=m.heading_valid, heading=m.heading_enu_rad, locked=m.locked)


def main():
    rclpy.init()
    log_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/sim_supervisor.jsonl'
    print(f'[sim_supervisor] Logging to {log_path}')
    try:
        rclpy.spin(SimSupervisor(log_path))
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
