#!/usr/bin/env python3
"""Allowlist relay between the UUV domain (40) and the GCS domain (10).

Test stand-in for the team's DDS Router: only the /system/... topics of the RobotX SoS
interface cross, each in its documented direction. The USV's Task 2 PipelineBuoys feed and the UAV's
Task 1 SafePassage buoy field are forwarded from the GCS domain (where the USV/UAV or mock_usv.py publishes them) to the UUV. /banyu, /mavros, cameras and /tf stay
private to the UUV domain.

Usage: python3 dds_relay.py [uuv_domain=40] [gcs_domain=10]
"""
import sys
import threading

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rx_msgs.msg import (Command, Course, Heartbeat, IncidentAck, MissionStatus, PipelineBuoys,
                         PipelineSurvey, ReadinessReport, ResourceDelivery, SafePassage)

VOLATILE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

UUV_TO_GCS = [
    ('/system/vehicle/uuv/heartbeat', Heartbeat, VOLATILE),
    ('/system/mission/status', MissionStatus, VOLATILE),
    ('/system/vehicle/uuv/task2/pipeline_survey', PipelineSurvey, LATCHED),
    ('/system/vehicle/uuv/task2/delivery', ResourceDelivery, LATCHED),
    ('/system/vehicle/uuv/task4/incident_ack', IncidentAck, VOLATILE),
    ('/system/vehicle/uuv/task4/readiness', ReadinessReport, VOLATILE),
]
GCS_TO_UUV = [
    ('/system/mission/course', Course, VOLATILE),
    ('/system/mission/command', Command, VOLATILE),
    ('/system/vehicle/usv/task2/pipeline_buoys', PipelineBuoys, VOLATILE),
    ('/system/vehicle/uav/task1/safe_passage', SafePassage, VOLATILE),
]


def make_side(domain_id, name):
    context = Context()
    rclpy.init(context=context, domain_id=domain_id)
    node = Node(name, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    return node


def bridge(source, destination, routes, label):
    counts = {}
    for topic, msg_type, qos in routes:
        publisher = destination.create_publisher(msg_type, topic, qos)
        counts[topic] = 0

        def forward(msg, publisher=publisher, topic=topic):
            counts[topic] += 1
            if counts[topic] == 1:
                print(f'[relay {label}] first message on {topic}', flush=True)
            publisher.publish(msg)
        source.create_subscription(msg_type, topic, forward, qos)
    return counts


def main():
    uuv_domain = int(sys.argv[1]) if len(sys.argv) > 1 else 40
    gcs_domain = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    uuv = make_side(uuv_domain, 'sos_relay_uuv')
    gcs = make_side(gcs_domain, 'sos_relay_gcs')
    up = bridge(uuv, gcs, UUV_TO_GCS, f'{uuv_domain}->{gcs_domain}')
    down = bridge(gcs, uuv, GCS_TO_UUV, f'{gcs_domain}->{uuv_domain}')
    print(f'[relay] UUV domain {uuv_domain} <-> GCS domain {gcs_domain}: '
          f'{len(UUV_TO_GCS)} up, {len(GCS_TO_UUV)} down', flush=True)
    stop = threading.Event()
    try:
        while not stop.wait(30):
            print(f'[relay] counts up={up} down={down}', flush=True)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
