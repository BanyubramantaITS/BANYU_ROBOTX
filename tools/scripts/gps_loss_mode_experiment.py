#!/usr/bin/env python3
"""
SIMULATION ONLY: what does each Pixhawk mode do when the GPS disappears under water?

Run it against the Task 2 sim started with `gps_gate:=True` (sim_gps_gate cuts SIM_GPS1_ENABLE below
`gps_loss_depth_m`). For each mode it dives, observes for --hold-s seconds and logs one JSON line per sample
plus a summary, then surfaces and checks that GUIDED can be re-entered after the GPS comes back.

    ROS_DOMAIN_ID=40 python3 gps_loss_mode_experiment.py --modes GUIDED ALT_HOLD STABILIZE MANUAL POSHOLD \
        --out gps_loss_modes.jsonl

Modes: GUIDED streams velocity setpoints; ALT_HOLD/STABILIZE/MANUAL/POSHOLD use /mavros/rc/override sticks.
Never run on hardware: it arms, dives and cuts nothing, but it commands thrusters.
"""
import argparse
import json
import math
import time

import rclpy
from banyu_msgs.msg import WaterDepth
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OverrideRCIn, PositionTarget, State, StatusText
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu

CH_LATERAL, CH_FORWARD, CH_THROTTLE, CH_YAW = 0, 1, 2, 3   # RCMAP 1/2/3/4 (0-based here)
NEUTRAL = 1500
BEST_EFFORT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Experiment(Node):
    def __init__(self, args):
        super().__init__('gps_loss_mode_experiment',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args = args
        self.out = open(args.out, 'a')
        self.state = State()
        self.depth = None
        self.yaw = None
        self.pose_stamp = None
        self.texts = []
        self.create_subscription(State, '/mavros/state', lambda m: setattr(self, 'state', m), 10)
        self.create_subscription(WaterDepth, '/banyu/vehicle/water_depth', self.on_depth, 10)
        self.create_subscription(Imu, '/mavros/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.on_pose,
                                 qos_profile_sensor_data)
        self.create_subscription(StatusText, '/mavros/statustext/recv',
                                 lambda m: self.texts.append((self.now_s(), m.text)), 50)
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.sp_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')

    # ---- plumbing
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_depth(self, m):
        if m.valid:
            self.depth = m.depth_m

    def on_imu(self, m):
        self.yaw = yaw_of(m.orientation)

    def on_pose(self, m):
        self.pose_stamp = self.now_s()

    def wait(self, seconds, tick=None):
        end = self.now_s() + seconds
        while self.now_s() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if tick:
                tick()

    def call(self, client, req, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def set_mode(self, mode):
        r = self.call(self.mode_cli, SetMode.Request(custom_mode=mode))
        self.wait(1.0)
        return bool(r and r.mode_sent), self.state.mode

    def arm(self, value=True):
        r = self.call(self.arm_cli, CommandBool.Request(value=value))
        self.wait(1.0)
        return bool(r and r.success)

    def rc(self, forward=NEUTRAL, throttle=NEUTRAL, yaw=NEUTRAL, lateral=NEUTRAL):
        m = OverrideRCIn()
        m.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        m.channels[CH_LATERAL], m.channels[CH_FORWARD] = lateral, forward
        m.channels[CH_THROTTLE], m.channels[CH_YAW] = throttle, yaw
        self.rc_pub.publish(m)

    def release_rc(self):
        m = OverrideRCIn()
        m.channels = [OverrideRCIn.CHAN_RELEASE] * 18
        self.rc_pub.publish(m)

    def velocity_setpoint(self, vx=0.0, vy=0.0, vz=0.0):
        m = PositionTarget()
        m.header.stamp = self.get_clock().now().to_msg()
        m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        m.type_mask = 0x5C7   # velocity only, explicit zero yaw rate (AGENTS.md)
        m.velocity.x, m.velocity.y, m.velocity.z = vx, vy, vz
        m.yaw_rate = 0.0
        self.sp_pub.publish(m)

    def log(self, **kw):
        kw['t'] = round(self.now_s(), 2)
        self.out.write(json.dumps(kw) + '\n')
        self.out.flush()

    def sample(self, mode, phase):
        pose_age = None if self.pose_stamp is None else round(self.now_s() - self.pose_stamp, 2)
        self.log(kind='sample', mode_under_test=mode, phase=phase, fcu_mode=self.state.mode,
                 armed=self.state.armed, depth=self.depth, yaw=self.yaw, pose_age_s=pose_age)

    # ---- phases
    def ensure_armed(self):
        for _ in range(5):
            if self.state.armed:
                return True
            self.set_mode('ALT_HOLD')
            self.arm(True)
        return self.state.armed

    def dive(self, mode, target_depth, timeout):
        """Descend with the mode under test when it can, with ALT_HOLD sticks otherwise."""
        end = self.now_s() + timeout
        while self.now_s() < end and (self.depth is None or self.depth < target_depth):
            if mode == 'GUIDED' and self.state.mode == 'GUIDED':
                self.velocity_setpoint(vz=-0.3)
            else:
                self.rc(throttle=1400)
            self.wait(0.1)
            self.sample(mode, 'dive')
        return self.depth is not None and self.depth >= target_depth

    def hold(self, mode, seconds):
        end = self.now_s() + seconds
        while self.now_s() < end:
            if mode == 'GUIDED':
                self.velocity_setpoint()
            elif mode in ('ALT_HOLD', 'STABILIZE', 'MANUAL', 'POSHOLD'):
                self.rc()
            self.wait(0.1)
            self.sample(mode, 'hold')

    def surface(self, timeout=90.0):
        end = self.now_s() + timeout
        self.set_mode('ALT_HOLD')
        while self.now_s() < end and (self.depth is None or self.depth > 0.05):
            self.rc(throttle=1600)
            self.wait(0.1)
            self.sample('-', 'surfacing')
        self.rc()
        return self.depth is not None and self.depth <= 0.05

    def recover_guided(self, timeout):
        """Seconds until GUIDED is accepted again after surfacing, or None."""
        t0 = self.now_s()
        while self.now_s() - t0 < timeout:
            self.release_rc()
            self.set_mode('ALT_HOLD')
            ok, mode = self.set_mode('GUIDED')
            if self.state.mode == 'GUIDED':
                return round(self.now_s() - t0, 1)
            self.wait(2.0)
            self.sample('-', 'recover')
        return None

    def trial(self, mode):
        n_text = len(self.texts)
        self.log(kind='trial_start', mode=mode)
        if not self.ensure_armed():
            self.log(kind='trial_end', mode=mode, result='NOT_ARMED')
            return
        self.set_mode('GUIDED' if mode == 'GUIDED' else 'ALT_HOLD')
        reached = self.dive(mode, self.args.depth, self.args.dive_timeout)
        accepted, fcu_mode = self.set_mode(mode)
        self.log(kind='mode_request', mode=mode, reached_depth=reached, accepted=accepted,
                 fcu_mode=fcu_mode, depth=self.depth)
        d0, y0 = self.depth, self.yaw
        self.hold(mode, self.args.hold_s)
        d1, y1 = self.depth, self.yaw
        drift_yaw = None if y0 is None or y1 is None else \
            round(math.degrees(math.atan2(math.sin(y1 - y0), math.cos(y1 - y0))), 1)
        surfaced = self.surface()
        guided_s = self.recover_guided(self.args.recover_timeout)
        self.log(kind='trial_end', mode=mode, reached_depth=reached, accepted=accepted,
                 fcu_mode_after_hold=fcu_mode, depth_drift_m=None if None in (d0, d1) else round(d1 - d0, 2),
                 yaw_drift_deg=drift_yaw, surfaced=surfaced, guided_back_after_s=guided_s,
                 statustext=[t for _, t in self.texts[n_text:]][-20:])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--modes', nargs='+', default=['GUIDED', 'ALT_HOLD'])
    ap.add_argument('--depth', type=float, default=1.5)
    ap.add_argument('--hold-s', type=float, default=60.0)
    ap.add_argument('--dive-timeout', type=float, default=60.0)
    ap.add_argument('--recover-timeout', type=float, default=90.0)
    ap.add_argument('--out', default='gps_loss_modes.jsonl')
    args = ap.parse_args()
    rclpy.init()
    node = Experiment(args)
    try:
        node.wait(3.0)
        for mode in args.modes:
            node.trial(mode)
    finally:
        node.release_rc()
        node.arm(False)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
