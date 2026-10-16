#!/usr/bin/env python3
"""
Bridge the Gazebo DVL sensor into ArduSub as VISION_POSITION_DELTA.

Why this exists rather than a ros_gz_bridge + MAVROS chain:

- ros_gz_bridge on Humble cannot carry gz.msgs.DVLVelocityTracking at all; the
  mapping only exists from Jazzy. So we read the gz topic directly with
  gz-transport's Python bindings.
- MAVROS cannot emit VISION_POSITION_DELTA (40 plugins, none does), and that is
  the *only* message that bootstraps EKF3 aiding from a DVL alone:
      VISION_POSITION_DELTA -> writeBodyFrameOdom() -> readyToUseBodyOdm()
                            -> AID_RELATIVE
  VISION_SPEED_ESTIMATE and ODOMETRY-twist reach writeExtNavVelData(), which is
  in neither AID_NONE exit branch (AP_NavEKF3_Control.cpp:284-299).

So this speaks MAVLink straight to SITL on its own endpoint. Nothing here goes
through MAVROS, which means this script owns the *entire* frame conversion.

Run alongside the sim; see proof_of_readiness.launch.py.
"""

import argparse
import math
import os
import sys

# VISION_POSITION_DELTA lives in the ardupilotmega dialect, not common, and
# pymavlink binds its dialect at IMPORT time -- calling mavutil.set_dialect()
# afterwards leaves the already-built MAVLink class without
# vision_position_delta_send(). So this must come before the import below.
os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")
# ...and MAVLink *2*: the v1.0 ardupilotmega dialect has no VISION_POSITION_DELTA
# either, so without this pymavlink silently resolves to dialects.v10 and the
# send function is still missing.
os.environ.setdefault("MAVLINK20", "1")

from gz.transport13 import Node as GzNode
from gz.msgs10.dvl_velocity_tracking_pb2 import DVLVelocityTracking
# The target-type enum hangs off DVLTrackingTarget, not off the parent message.
from gz.msgs10.dvl_tracking_target_pb2 import DVLTrackingTarget
from pymavlink import mavutil

# Frame mapping, established EMPIRICALLY (do not "fix" from first principles).
# Driving an unrotated probe along each world axis and reading the sensor gave:
#     world +X 0.5 -> dvl reports y = +0.500
#     world +Y 0.5 -> dvl reports x = -0.500
#     world +Z 0.5 -> dvl reports z = +0.490
# i.e. the sensor frame is yawed +90 deg from the body frame, which is the
# <reference_frame>0 0 0 0 0 -1.570796</reference_frame> in the DVL model.
#
# Gazebo body frame is z-UP (x fwd, y left, z up).
# ArduPilot body frame is FRD (x fwd, y starboard, z down).
#   body_x =  dvl_y      FRD_x =  body_x =  dvl_y
#   body_y = -dvl_x  =>  FRD_y = -body_y =  dvl_x
#   body_z =  dvl_z      FRD_z = -body_z = -dvl_z
def dvl_to_frd(v):
    return (v.y, v.x, -v.z)


# Confidence -> the EKF's velocity error, via
#   velErr = EK3_VIS_VERR_MIN + (MAX-MIN)*(1 - quality/100)
#          = 0.1 + 0.8*(1 - quality/100)          (AP_NavEKF3_Measurements.cpp:138)
# Defaults put velErr in 0.1..0.9, always under readyToUseBodyOdm()'s < 1.0 gate,
# so confidence cannot *block* fusion -- it only scales how far the EKF trusts
# us. Send something honest anyway.
COV_FULL_DISTRUST = 0.5  # m/s stddev at which we report zero confidence


def confidence_from_cov(cov):
    if len(cov) < 9:
        return 0.0
    worst_var = max(cov[0], cov[4], cov[8])
    if worst_var <= 0.0 or not math.isfinite(worst_var):
        return 0.0
    sigma = math.sqrt(worst_var)
    return max(0.0, min(100.0, 100.0 * (1.0 - sigma / COV_FULL_DISTRUST)))


class Bridge:
    def __init__(self, mav, verbose=False):
        self.mav = mav
        self.verbose = verbose
        self.prev_stamp_us = None
        self.sent = 0
        self.skipped_no_lock = 0

    def on_dvl(self, msg: DVLVelocityTracking):
        # Bottom lock only. Water-track velocity is relative to a moving medium
        # and would poison a dead-reckoned position delta.
        if msg.target.type != DVLTrackingTarget.DVL_TARGET_BOTTOM:
            self.skipped_no_lock += 1
            self.prev_stamp_us = None  # gap: next delta would span the dropout
            return

        stamp_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nsec // 1000
        prev = self.prev_stamp_us
        self.prev_stamp_us = stamp_us
        if prev is None:
            return  # need two samples to form a delta

        dt_us = stamp_us - prev
        if dt_us <= 0:
            return
        dt = dt_us * 1e-6

        vx, vy, vz = dvl_to_frd(msg.velocity.mean)
        conf = confidence_from_cov(list(msg.velocity.covariance))

        self.mav.mav.vision_position_delta_send(
            stamp_us,
            dt_us,
            # No angle delta: this bridge has no attitude source. The EKF then
            # assumes zero body rotation over the interval, which costs a small
            # lever-arm error while yawing (VISO_POS_Z is 0.10 m). Acceptable
            # for now; revisit if orbit-phase drift shows up.
            [0.0, 0.0, 0.0],
            [vx * dt, vy * dt, vz * dt],
            conf,
        )
        self.sent += 1
        if self.verbose and self.sent % 20 == 1:
            print(
                f"[dvl] sent={self.sent} skipped={self.skipped_no_lock} "
                f"dt={dt:.3f}s v_frd=({vx:+.3f},{vy:+.3f},{vz:+.3f}) conf={conf:.0f}",
                flush=True,
            )


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gz-topic", default="/dvl/velocity")
    # ArduSub must be told to stream here too, e.g.
    #   sim_vehicle.py --out=udp:127.0.0.1:14552
    # We bind and wait for its heartbeat so we learn where to send back.
    ap.add_argument("--mavlink", default="udpin:127.0.0.1:14552")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    print(f"[dvl] connecting MAVLink on {args.mavlink} ...", flush=True)
    mav = mavutil.mavlink_connection(
        args.mavlink, source_system=1, source_component=197, dialect="ardupilotmega"
    )
    mav.wait_heartbeat()
    print(f"[dvl] heartbeat from sys={mav.target_system} comp={mav.target_component}", flush=True)

    bridge = Bridge(mav, verbose=args.verbose)

    gz = GzNode()
    if not gz.subscribe(DVLVelocityTracking, args.gz_topic, bridge.on_dvl):
        print(f"[dvl] ERROR: could not subscribe to gz topic {args.gz_topic}", file=sys.stderr)
        return 1
    print(f"[dvl] subscribed to gz {args.gz_topic}; forwarding VISION_POSITION_DELTA", flush=True)

    # Keep the MAVLink link alive and drain inbound so the socket peer stays fresh.
    try:
        while True:
            mav.recv_match(blocking=True, timeout=1.0)
    except KeyboardInterrupt:
        pass
    print(f"[dvl] stopping; sent={bridge.sent} skipped_no_lock={bridge.skipped_no_lock}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
