#!/usr/bin/env python3
"""
SIMULATION ONLY: cut the SITL GPS while the vehicle is submerged, like the real AUV.

The real RTK GPS only works with the antenna above water: it is lost as soon as the body goes under and
needs a reacquire delay after surfacing. SITL synthesises GPS from Gazebo ground truth and keeps it
alive underwater, which hides the position loss Task 2 has on the robot (docs/task2_underwater_position.md).
This node reads the calibrated depth and toggles SIM_GPS1_ENABLE through MAVROS.

    depth > loss_depth_m                   -> GPS off immediately
    depth <= resurface_depth_m for
      reacquire_delay_s without a dive     -> GPS on again

Never run this on hardware (it writes a SITL-only parameter). Self-check: `./sim_gps_gate.py --self-check`
"""

import sys

PARAM_ID = "SIM_GPS1_ENABLE"


def next_state(depth, gps_on, surfaced_since, now, loss_depth, resurface_depth, reacquire_delay):
    """
    Return (gps_on, surfaced_since) for one depth sample.

    `surfaced_since` is the time the vehicle first came back to <= resurface_depth, or None.
    Between resurface_depth and loss_depth the current state is held (hysteresis).
    """
    if depth > loss_depth:
        return False, None
    if depth <= resurface_depth:
        if gps_on:
            return True, None
        start = now if surfaced_since is None else surfaced_since
        if now - start >= reacquire_delay:
            return True, None
        return False, start
    # Inside the band: keep the state, and restart the reacquire timer if GPS is still off.
    return gps_on, (surfaced_since if not gps_on else None)


def rebase(baseline, last, last_t, depth, t, min_step=0.05, max_rate=0.8):
    """
    Return the new surface baseline. Arming zeroes the baro and a new run recalibrates the depth datum, so
    the reading jumps at the surface. The vehicle cannot descend faster than max_rate (m/s), so a step of at
    least min_step metres that fast shifts the baseline instead of counting as a dive.
    """
    if last is None or last_t is None:
        return baseline
    dt = max(t - last_t, 1e-3)
    step = depth - last
    if abs(step) > min_step and abs(step) / dt > max_rate:
        return baseline + step
    return baseline


def self_check():
    loss, res, delay = 0.1, 0.05, 10.0
    on, since = next_state(0.0, True, None, 0.0, loss, res, delay)
    assert on and since is None, "stays on at the surface"
    on, since = next_state(0.3, True, None, 1.0, loss, res, delay)
    assert not on, "cut as soon as it is past the loss depth"
    on, since = next_state(0.0, False, None, 5.0, loss, res, delay)
    assert not on and since == 5.0, "reacquire timer starts on surfacing"
    on, since = next_state(0.0, False, 5.0, 14.9, loss, res, delay)
    assert not on, "still reacquiring"
    on, since = next_state(0.0, False, 5.0, 15.0, loss, res, delay)
    assert on and since is None, "back on after the delay"
    on, since = next_state(0.08, False, 5.0, 8.0, loss, res, delay)
    assert not on and since == 5.0, "inside the band the timer keeps running"
    on, since = next_state(0.08, True, None, 8.0, loss, res, delay)
    assert on, "inside the band an enabled GPS stays on"
    on, since = next_state(0.3, False, 5.0, 8.0, loss, res, delay)
    assert not on and since is None, "a new dive resets the timer"
    # Datum jump at arming: the baseline follows it, a normal descent does not.
    assert abs(rebase(0.0, 0.0, 0.0, 0.15, 0.05) - 0.15) < 1e-9, "a 0.15 m jump in 50 ms is a datum jump"
    assert abs(rebase(0.0, 0.0, 0.0, -0.4, 0.1) + 0.4) < 1e-9
    assert rebase(0.0, 0.5, 0.0, 0.53, 0.1) == 0.0, "0.3 m/s is a dive"
    assert rebase(0.0, 0.0, 0.0, 0.03, 0.01) == 0.0, "noise below min_step is ignored"
    assert rebase(0.0, 0.0, 0.0, 0.3, 1.0) == 0.0, "0.3 m in a second is a dive, even with sparse samples"
    assert rebase(0.0, None, None, 1.0, 1.0) == 0.0
    print("sim_gps_gate: self-check OK")


def main():
    import rclpy
    from banyu_msgs.msg import WaterDepth
    from mavros_msgs.srv import ParamSetV2
    from rcl_interfaces.msg import ParameterType, ParameterValue
    from rclpy.node import Node

    class SimGpsGate(Node):
        def __init__(self):
            super().__init__("sim_gps_gate")
            self.loss_depth = float(self.declare_parameter("loss_depth_m", 0.1).value)
            self.resurface_depth = float(self.declare_parameter("resurface_depth_m", 0.05).value)
            self.reacquire_delay = float(self.declare_parameter("reacquire_delay_s", 10.0).value)
            depth_topic = self.declare_parameter(
                "depth_topic", "/banyu/vehicle/water_depth").value
            # Calibrated depth drifts 0.1-0.4 m at the surface in SITL (AGENTS.md), so the thresholds are
            # relative to the depth at start-up (vehicle on the surface), unless surface_depth_m is set.
            self.surface_depth = float(self.declare_parameter("surface_depth_m", float("nan")).value)
            self.last_depth = None
            self.last_t = None
            self.baseline_samples = []
            self.gps_on = True
            self.surfaced_since = None
            self.pending = None
            self.cli = self.create_client(ParamSetV2, "/mavros/param/set")
            # sensors:=minimal: the mission has no depth topic from the barometer; the world's depth for the gate then
            # comes straight from the SITL barometer (rel_alt), which the mission never reads.
            rel_alt_topic = self.declare_parameter("rel_alt_topic", "").value
            if rel_alt_topic and rel_alt_topic != "none":
                from std_msgs.msg import Float64
                from rclpy.qos import qos_profile_sensor_data

                class _Depth:
                    def __init__(self, d):
                        self.valid, self.depth_m = True, d
                self.create_subscription(
                    Float64, rel_alt_topic, lambda m: self.on_depth(_Depth(-m.data)), qos_profile_sensor_data)
            else:
                self.create_subscription(WaterDepth, depth_topic, self.on_depth, 10)
            self.get_logger().info(
                f"GPS gate armed: off below {self.loss_depth} m, on after "
                f"{self.reacquire_delay} s at <= {self.resurface_depth} m")

        def on_depth(self, msg):
            if not msg.valid:
                return
            if self.surface_depth != self.surface_depth:   # NaN: learn it from the first samples
                self.baseline_samples.append(msg.depth_m)
                if len(self.baseline_samples) < 20:
                    return
                self.surface_depth = sorted(self.baseline_samples)[len(self.baseline_samples) // 2]
                self.get_logger().info(f"surface depth baseline {self.surface_depth:.3f} m")
            now = self.get_clock().now().nanoseconds * 1e-9
            self.surface_depth = rebase(
                self.surface_depth, self.last_depth, self.last_t, msg.depth_m, now)
            self.last_depth, self.last_t = msg.depth_m, now
            depth = msg.depth_m - self.surface_depth
            gps_on, since = next_state(
                depth, self.gps_on, self.surfaced_since, now,
                self.loss_depth, self.resurface_depth, self.reacquire_delay)
            self.surfaced_since = since
            if gps_on != self.gps_on and gps_on != self.pending:
                self.send(gps_on, depth)

        def send(self, enable, depth):
            if not self.cli.service_is_ready():
                self.get_logger().warn("/mavros/param/set not up yet; will retry")
                return
            req = ParamSetV2.Request()
            req.param_id = PARAM_ID
            req.force_set = False
            req.value = ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=1 if enable else 0)
            self.pending = enable
            self.get_logger().info(f"depth {depth:.2f} m -> GPS {'ON' if enable else 'OFF'}")

            def done(fut):
                self.pending = None
                try:
                    if fut.result().success:
                        self.gps_on = enable
                        self.get_logger().info(f"{PARAM_ID} set to {int(enable)}")
                    else:
                        self.get_logger().error(f"{PARAM_ID} write rejected; retrying")
                except Exception as exc:
                    self.get_logger().error(f"param set call failed: {exc}; retrying")

            self.cli.call_async(req).add_done_callback(done)

    rclpy.init()
    node = SimGpsGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    if "--self-check" in sys.argv:
        self_check()
        sys.exit(0)
    sys.exit(main())
