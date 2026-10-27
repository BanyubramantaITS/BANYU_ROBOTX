#!/usr/bin/env python3
"""SIMULATION ONLY: record every accepted ring of a Task 2 run together with the vehicle's Gazebo ground truth.

Writes JSONL: one `accepted_ring` row per message (node id, colour, progress, EKF pose, Gazebo pose of the
vehicle) and one `evidence` row whenever the survey evidence changes. The Gazebo pose comes from
`gz topic /world/ocean/dynamic_pose/info`, sampled in a background thread at about 2 Hz.
  ROS_DOMAIN_ID=40 python3 ring_recorder.py out.jsonl 600
"""
import json
import re
import subprocess
import sys
import threading
import time

import rclpy
from banyu_msgs.msg import Task2RingRecord, Task2SurveyEvidence
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

truth = {'xyz': None, 't': 0.0}


def sample_truth(stop):
    while not stop.is_set():
        try:
            out = subprocess.run(['gz', 'topic', '-e', '-n', '1', '-t', '/world/ocean/dynamic_pose/info'],
                                 capture_output=True, text=True, timeout=6).stdout
            m = re.search(r'name: "bluerov2_heavy".*?position \{(.*?)\}', out, re.S)
            if m:
                v = dict(re.findall(r'([xyz]): (-?[0-9.eE+-]+)', m.group(1)))
                truth['xyz'] = [round(float(v.get(k, 0.0)), 2) for k in 'xyz']
                truth['t'] = time.time()
        except Exception:
            pass
        time.sleep(0.3)


def main():
    out = open(sys.argv[1], 'w')
    duration = float(sys.argv[2])
    rclpy.init()
    node = Node('ring_recorder')
    pose = [None]

    def write(kind, **kw):
        out.write(json.dumps({'t': round(time.time(), 2), 'kind': kind, 'ekf_pose': pose[0],
                              'gz_pose': truth['xyz'], 'gz_age_s': round(time.time() - truth['t'], 1),
                              **kw}) + '\n')
        out.flush()

    node.create_subscription(
        PoseStamped, '/mavros/local_position/pose',
        lambda m: pose.__setitem__(0, [round(m.pose.position.x, 2), round(m.pose.position.y, 2),
                                       round(m.pose.position.z, 2)]), qos_profile_sensor_data)
    node.create_subscription(
        Task2RingRecord, '/banyu/pipeline/accepted_ring',
        lambda m: write('accepted_ring', node_id=m.node_id, color=m.color,
                        progress=round(m.pipeline_progress_m, 2)),
        QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
    last = [None]

    def evidence(m):
        key = (str([(r.node_id, r.color) for r in m.rings]), m.traversal_complete, m.repair_verified,
               m.resource_code_complete, m.resource_color, m.delivery_circle_color)
        if key != last[0]:
            last[0] = key
            write('evidence', rings=[(r.node_id, r.color, round(r.pipeline_progress_m, 2)) for r in m.rings],
                  traversal_complete=m.traversal_complete, repair_verified=m.repair_verified,
                  code_complete=m.resource_code_complete, tin=m.resource_color, circle=m.delivery_circle_color)
    node.create_subscription(
        Task2SurveyEvidence, '/banyu/pipeline/survey_evidence', evidence,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    stop = threading.Event()
    threading.Thread(target=sample_truth, args=(stop,), daemon=True).start()
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    stop.set()


if __name__ == '__main__':
    main()
