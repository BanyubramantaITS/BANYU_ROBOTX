#!/usr/bin/env python3
"""Mock UAV for the sim: publishes a Task 1 buoy field (rx_msgs/SafePassage) across the UUV's surface transit.

The field is the first message the UAV sent on the 2026-11-12 trial (docs/task2_surface_planning_trial.md): 8 of the
9 real buoys (the UAV never reported the ninth), placed at --east/--north metres from the GPS home, with 0.1 m position
jitter like the real reports. --sequence trial (default) publishes like the UAV did: the 8-buoy message once, then
nothing until the UUV's NavigateSurfaceGps goal runs, then the subsets the UAV sent later (5 buoys for --phase-s, then
6, every second): only the UUV's memory of the first message keeps all 8 as obstacles. The UAV publishes with volatile
QoS, so the first message waits for a subscriber; --no-wait sends it at once, as a UAV that mapped the field before the
UUV stack started (the UUV never hears it). --sequence all publishes all 8 every second. There are no physical buoys:
the clearance is judged from the UUV's GPS track. On exit (--timeout or Ctrl-C) a summary is printed and written as
JSON to --out.

Run in the UUV domain (40) with ROS sourced, before the run starts. --domain joins it without ROS_DOMAIN_ID in the
environment, so task2_sim_cleanup.sh (which stops every process carrying ROS_DOMAIN_ID=40) leaves it running across
the e2e runner's relaunches. Keep discovery on this machine: a real UUV on the network would get the field.
  ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST tools/sos_integration/mock_uav_safe_passage.py --domain 40 --out /tmp/f.json
"""
import argparse
import json
import math
import random
import time

import rclpy
from banyu_msgs.action import NavigateSurfaceGps
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rx_msgs.msg import BuoyDetection, LatLng, SafePassage
from sensor_msgs.msg import NavSatFix

# The trial's first message, ENU metres (B0..B7, the six seen in most messages placed as before). B2 was in that
# message only (also its exit position), B5 in the 5-buoy minutes only.
TRIAL_FIELD = [(-3.7, -4.4), (-0.47, 4.5), (0.19, -8.97), (3.42, -2.06), (3.31, 0.94), (7.76, 3.61), (-3.59, 4.06),
               (3.31, 4.39)]
# Buoys in the later messages: 09:39 (5 buoys), from 09:48 (6 buoys).
TRIAL_SUBSETS = [(0, 1, 3, 5, 6), (0, 1, 3, 4, 6, 7)]
R = 6378137.0


class MockUav(Node):
    def __init__(self, a):
        super().__init__('mock_uav_safe_passage')
        self.a = a
        self.m_lat = math.radians(1) * R
        self.m_lon = math.radians(1) * R * math.cos(math.radians(a.home_lat))
        self.field = [(a.east + x, a.north + y) for x, y in TRIAL_FIELD]
        self.first_sent = False
        self.pub = self.create_publisher(SafePassage, a.topic, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.on_fix, qos_profile_sensor_data)
        self.create_subscription(
            NavigateSurfaceGps.Impl.FeedbackMessage, '/banyu/actions/navigate_surface_gps/_action/feedback',
            self.on_feedback, 10)
        self.rng = random.Random(1)
        self.navigating_since = None
        self.track = []
        self.min_clear = [math.inf] * len(self.field)
        self.last_fb = None
        self.create_timer(1.0, self.publish)

    def latlon(self, e, n):
        return LatLng(latitude=self.a.home_lat + n / self.m_lat, longitude=self.a.home_lon + e / self.m_lon)

    def enu(self, lat, lon):
        return ((lon - self.a.home_lon) * self.m_lon, (lat - self.a.home_lat) * self.m_lat)

    def subset(self):
        if self.a.sequence == 'all' or not self.first_sent:
            return range(len(self.field))
        if self.navigating_since is None:
            return None
        return TRIAL_SUBSETS[0 if time.monotonic() - self.navigating_since < self.a.phase_s else 1]

    def publish(self):
        if not self.first_sent and not self.a.no_wait and self.pub.get_subscription_count() == 0:
            return
        subset = self.subset()
        if subset is None:
            return
        if not self.first_sent:
            self.get_logger().info(f'First SafePassage: all {len(self.field)} buoys')
        self.first_sent = True
        msg = SafePassage()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in subset:
            e, n = self.field[i]
            b = BuoyDetection()
            b.position = self.latlon(e + self.rng.gauss(0, 0.1), n + self.rng.gauss(0, 0.1))
            b.state = 2
            msg.buoys.append(b)
        msg.entry_position = msg.buoys[0].position if msg.buoys else LatLng()
        self.pub.publish(msg)

    def on_feedback(self, msg):
        if self.navigating_since is None:
            self.navigating_since = time.monotonic()
            self.get_logger().info('NavigateSurfaceGps running: publishing the later subsets')
        self.last_fb = msg.feedback.distance_remaining_m

    def on_fix(self, msg):
        if self.navigating_since is None or msg.status.status < 0:
            return
        p = self.enu(msg.latitude, msg.longitude)
        if not self.track or math.dist(p, self.track[-1]) > 0.05:
            self.track.append(p)
        for i, b in enumerate(self.field):
            self.min_clear[i] = min(self.min_clear[i], math.dist(p, b))

    def summary(self):
        s = {
            'field_enu': self.field, 'sequence': self.a.sequence, 'navigated': self.navigating_since is not None,
            'min_clearance_m': [round(c, 2) if math.isfinite(c) else None for c in self.min_clear],
            'closest_m': round(min(self.min_clear), 2) if self.track else None,
            'track_points': len(self.track), 'track_start': self.track[0] if self.track else None,
            'track_end': self.track[-1] if self.track else None,
            'path_length_m': round(sum(math.dist(p, q) for p, q in zip(self.track, self.track[1:])), 1),
            'last_distance_remaining_m': self.last_fb,
            'track': [(round(x, 2), round(y, 2)) for x, y in self.track],
        }
        return s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--home-lat', type=float, default=33.810313, help='sim GPS home (the world origin)')
    ap.add_argument('--home-lon', type=float, default=-118.393866)
    ap.add_argument('--east', type=float, default=0.0, help='field centre east of home, m')
    ap.add_argument('--north', type=float, default=-8.0, help='field centre north of home, m')
    ap.add_argument('--sequence', choices=('trial', 'all'), default='trial', help='which buoys each message holds')
    ap.add_argument('--phase-s', type=float, default=20.0, help='trial sequence: seconds of 5-buoy messages')
    ap.add_argument('--no-wait', action='store_true', help='send the first message without waiting for a subscriber')
    ap.add_argument('--topic', default='/system/vehicle/uav/task1/safe_passage')
    ap.add_argument('--timeout', type=float, default=900.0, help='stop after this many seconds')
    ap.add_argument('--out', default='', help='write the summary JSON here')
    ap.add_argument('--domain', type=int, default=None, help='ROS domain (default: ROS_DOMAIN_ID)')
    a = ap.parse_args()
    rclpy.init(domain_id=a.domain) if a.domain is not None else rclpy.init()
    node = MockUav(a)
    end = time.monotonic() + a.timeout
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    s = node.summary()
    print(json.dumps({k: v for k, v in s.items() if k != 'track'}))
    if a.out:
        with open(a.out, 'w') as f:
            json.dump(s, f)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
