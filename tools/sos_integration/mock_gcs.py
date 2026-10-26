#!/usr/bin/env python3
"""Mock GCS and mock UAV for Task 2, ROS only (no MQTT, no RoboCommand).

Stands in for the team GCS (domain 10) so the UUV's Task 2 report path can be tested end to end:

  * Scripted commands on /system/mission/command (rx_msgs/Command). command_seq increases on every
    command this process sends, as the team format requires:
        run-start           CMD_RUN_START   (run_id, target UUV)
        uav-go              CMD_GO          (target UAV)
        delivery:RED:GREEN  CMD_DELIVERY    task 3, tin colour, circle colour (target UAV)
        mission-done        CMD_MISSION_DONE (run_id, target UUV)
  * Observer: logs the UUV's heartbeat, PipelineSurvey and ResourceDelivery as JSONL.
  * Auto-relay: when the UUV publishes ResourceDelivery, send CMD_DELIVERY to the UAV, like the real GCS.
  * Mock UAV: on CMD_DELIVERY (target UAV) echo ResourceDelivery on /system/vehicle/uav/task2/delivery
    and publish heartbeats whose current_task shows the tasking was received.
  * Checks: --expect-* options compare what arrived with what the sim flashed; exit code 0 = all pass.

Run with ROS sourced, in the GCS domain, next to dds_relay.py (domains 40 <-> 10):
  ROS_DOMAIN_ID=10 python3 mock_gcs.py --send run-start --serve --run-id 1 \\
      --expect-circle GREEN --expect-tin RED --expect-segments GREEN GREEN GREEN RED --timeout 900
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rx_msgs.msg import (Color, Command, Heartbeat, PipelineSegmentStatus, PipelineSurvey,
                         ResourceDelivery, RobotState, RxTask, VehicleType)

VOLATILE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
COLOURS = {'RED': Color.COLOR_RED, 'GREEN': Color.COLOR_GREEN, 'BLUE': Color.COLOR_BLUE,
           'ANY': Color.COLOR_ANY}
COLOUR_NAME = {v: k for k, v in COLOURS.items()}
SEGMENTS = {'GREEN': PipelineSegmentStatus.PIPELINE_SEGMENT_INTACT,
            'RED': PipelineSegmentStatus.PIPELINE_SEGMENT_DAMAGED,
            'UNKNOWN': PipelineSegmentStatus.PIPELINE_SEGMENT_UNKNOWN}
SEGMENT_NAME = {v: k for k, v in SEGMENTS.items()}


class MockGcs(Node):
    def __init__(self, args):
        super().__init__('mock_gcs')
        self.args = args
        self.seq = args.first_seq - 1
        self.out = open(args.log, 'a') if args.log else None
        self.survey = None
        self.deliveries = []          # ResourceDelivery from the UUV
        self.uav_echoes = []          # ResourceDelivery echoed by the mock UAV
        self.relayed = []             # CMD_DELIVERY sent to the UAV
        self.uav_tasked = False
        self.uuv_hb_state = None

        self.cmd_pub = self.create_publisher(Command, '/system/mission/command', VOLATILE)
        self.uav_delivery_pub = self.create_publisher(
            ResourceDelivery, '/system/vehicle/uav/task2/delivery', LATCHED)
        self.uav_hb_pub = self.create_publisher(Heartbeat, '/system/vehicle/uav/heartbeat', VOLATILE)

        self.create_subscription(PipelineSurvey, '/system/vehicle/uuv/task2/pipeline_survey',
                                 self.on_survey, LATCHED)
        self.create_subscription(ResourceDelivery, '/system/vehicle/uuv/task2/delivery',
                                 self.on_uuv_delivery, LATCHED)
        self.create_subscription(Heartbeat, '/system/vehicle/uuv/heartbeat', self.on_uuv_hb, VOLATILE)
        if args.uav:
            self.create_subscription(Command, '/system/mission/command', self.on_command, VOLATILE)
            self.create_timer(0.5, self.uav_heartbeat)

    # ---- logging
    def log(self, kind, **kw):
        row = {'t': round(self.get_clock().now().nanoseconds * 1e-9, 2), 'kind': kind, **kw}
        line = json.dumps(row)
        print(line, flush=True)
        if self.out:
            self.out.write(line + '\n')
            self.out.flush()

    # ---- commands
    def send(self, name):
        cmd = Command()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'gcs'
        self.seq += 1
        cmd.command_seq = self.seq
        base, *rest = name.split(':')
        if base == 'run-start':
            cmd.command_type = Command.CMD_RUN_START
            cmd.target_vehicle = self.args.uuv_target
            cmd.run_id = self.args.run_id
        elif base == 'uav-go':
            cmd.command_type = Command.CMD_GO
            cmd.target_vehicle = VehicleType.TYPE_UAV
        elif base == 'delivery':
            cmd.command_type = Command.CMD_DELIVERY
            cmd.target_vehicle = VehicleType.TYPE_UAV
            cmd.task = RxTask.TASK_INFRA_SURVEY_REPAIR
            cmd.resource_color = COLOURS[rest[0].upper()]
            cmd.delivery_circle_color = COLOURS[rest[1].upper()]
        elif base == 'mission-done':
            cmd.command_type = Command.CMD_MISSION_DONE
            cmd.target_vehicle = self.args.uuv_target
            cmd.run_id = self.args.run_id
        else:
            raise SystemExit(f'unknown command {name!r}')
        # The command topic is volatile: a command sent before a reader is matched is lost for that reader.
        # Wait for the expected readers (mission manager and uuv_comm_node), then let discovery settle.
        end = time.time() + 10.0
        while self.cmd_pub.get_subscription_count() < self.args.min_subs and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        settle = time.time() + 1.5
        while time.time() < settle:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.cmd_pub.publish(cmd)
        self.log('command_sent', name=name, command_seq=cmd.command_seq,
                 command_type=cmd.command_type, target_vehicle=cmd.target_vehicle, run_id=cmd.run_id)

    # ---- observers
    def on_uuv_hb(self, m):
        state = (m.state, m.current_task)
        if state != self.uuv_hb_state:
            self.uuv_hb_state = state
            self.log('uuv_heartbeat', state=m.state, current_task=m.current_task, depth_m=m.depth_m)

    def on_survey(self, m):
        self.survey = m
        self.log('uuv_pipeline_survey',
                 active_buoy=[m.active_buoy_position.latitude, m.active_buoy_position.longitude],
                 segments=[SEGMENT_NAME.get(s, s) for s in m.segments])

    def on_uuv_delivery(self, m):
        self.deliveries.append(m)
        self.log('uuv_resource_delivery', task=m.task, tin=COLOUR_NAME.get(m.resource_color),
                 circle=COLOUR_NAME.get(m.delivery_circle_color))
        if self.args.relay and len(self.deliveries) == 1:
            self.send(f'delivery:{COLOUR_NAME.get(m.resource_color, "ANY")}:'
                      f'{COLOUR_NAME.get(m.delivery_circle_color, "RED")}')
            self.relayed.append(m)

    # ---- mock UAV
    def on_command(self, m):
        if m.target_vehicle != VehicleType.TYPE_UAV:
            return
        if m.command_type == Command.CMD_DELIVERY:
            echo = ResourceDelivery()
            echo.header.stamp = self.get_clock().now().to_msg()
            echo.task = m.task
            echo.resource_color = m.resource_color
            echo.delivery_circle_color = m.delivery_circle_color
            self.uav_delivery_pub.publish(echo)
            self.uav_echoes.append(echo)
            self.uav_tasked = True
            self.log('uav_tasking_received', tin=COLOUR_NAME.get(m.resource_color),
                     circle=COLOUR_NAME.get(m.delivery_circle_color), command_seq=m.command_seq)
        elif m.command_type in (Command.CMD_GO, Command.CMD_MISSION_DONE):
            self.log('uav_command', command_type=m.command_type, command_seq=m.command_seq)

    def uav_heartbeat(self):
        hb = Heartbeat()
        hb.header.stamp = self.get_clock().now().to_msg()
        hb.state = RobotState.STATE_AUTO
        hb.vehicle_type = VehicleType.TYPE_UAV
        hb.current_task = RxTask.TASK_INFRA_SURVEY_REPAIR if self.uav_tasked else RxTask.TASK_NONE
        self.uav_hb_pub.publish(hb)

    # ---- checks
    def check(self):
        a = self.args
        results = {}
        if a.expect_segments is not None:
            got = [SEGMENT_NAME.get(s, s) for s in self.survey.segments] if self.survey else None
            results['survey_segments'] = (got == [s.upper() for s in a.expect_segments], got)
        if a.expect_circle:
            got = COLOUR_NAME.get(self.deliveries[0].delivery_circle_color) if self.deliveries else None
            results['uuv_circle'] = (got == a.expect_circle.upper(), got)
            echo = COLOUR_NAME.get(self.uav_echoes[0].delivery_circle_color) if self.uav_echoes else None
            results['uav_circle'] = (echo == a.expect_circle.upper(), echo)
        if a.expect_tin:
            got = COLOUR_NAME.get(self.deliveries[0].resource_color) if self.deliveries else None
            results['uuv_tin'] = (got == a.expect_tin.upper(), got)
        return results


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--send', nargs='*', default=[], help='commands: run-start uav-go delivery:RED:GREEN mission-done')
    ap.add_argument('--delay-s', type=float, default=1.0, help='pause between scripted commands')
    ap.add_argument('--serve', action='store_true', help='keep running: observe, relay, mock UAV')
    ap.add_argument('--relay', action='store_true', default=True, help='relay UUV ResourceDelivery to the UAV')
    ap.add_argument('--no-relay', dest='relay', action='store_false')
    ap.add_argument('--uav', action='store_true', default=True, help='run the mock UAV')
    ap.add_argument('--no-uav', dest='uav', action='store_false')
    ap.add_argument('--run-id', type=int, default=1)
    ap.add_argument('--first-seq', type=int, default=1)
    ap.add_argument('--uuv-target', type=int, default=VehicleType.TYPE_UUV, help='target for run-start / mission-done')
    ap.add_argument('--min-subs', type=int, default=2,
                    help='readers of /system/mission/command to wait for before the first send')
    ap.add_argument('--timeout', type=float, default=900.0)
    ap.add_argument('--log', default=None, help='JSONL log file')
    ap.add_argument('--expect-tin')
    ap.add_argument('--expect-circle')
    ap.add_argument('--expect-segments', nargs='*')
    args = ap.parse_args()

    rclpy.init()
    node = MockGcs(args)
    try:
        for name in args.send:
            node.send(name)
            end = time.time() + args.delay_s
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        if args.serve:
            need_survey = args.expect_segments is not None
            need_delivery = bool(args.expect_circle or args.expect_tin)
            end = time.time() + args.timeout
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.2)
                if (not need_survey or node.survey) and (not need_delivery or node.uav_echoes):
                    if need_survey or need_delivery:
                        break
            results = node.check()
            node.log('checks', **{k: {'pass': v[0], 'got': v[1]} for k, v in results.items()})
            if results:
                sys.exit(0 if all(v[0] for v in results.values()) else 1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
