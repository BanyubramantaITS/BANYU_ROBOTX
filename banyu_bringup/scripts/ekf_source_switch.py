#!/usr/bin/env python3
"""Switch ArduSub's EKF3 source set on depth: GPS on the surface, DVL below.

EKF3 carries three source sets (EK3_SRC1_*, EK3_SRC2_*, EK3_SRC3_*) and can be
told which is active over MAVLink, Lua or an RC aux switch -- but *nothing in
ArduPilot switches on depth by itself*. This node is that decision.

    SRC1  surface  GPS position + velocity   (ArduPilot's shipped defaults)
    SRC2  dive     DVL body-frame velocity, no absolute position

Ordering is the whole point: setOrigin() rejects a second origin
(AP_NavEKF3_Control.cpp:698), so a surface GPS fix anchors the EKF origin once
and the DVL then dead-reckons from a georeferenced point instead of from
nowhere. Surfacing again re-acquires absolute position.

Runs in sim and on hardware alike -- this is vehicle behaviour, not sim
plumbing (unlike dvl_mavlink_bridge.py, which is sim-only).

Self-check: `./ekf_source_switch.py --self-check` (no ROS needed).
"""

import sys

# ArduPilot-specific, from ardupilotmega.xml:144. param1 is 1-based (1..3);
# GCS_Common.cpp:5121 converts to the 0-based AHRS interface.
MAV_CMD_SET_EKF_SOURCE_SET = 42007

SET_SURFACE = 1  # -> EK3_SRC1_*
SET_DIVE = 2  # -> EK3_SRC2_*


def desired_set(depth, current, dive_depth, surface_depth):
    """Which source set should be active at this depth.

    `depth` is metres below the surface, positive down. The two thresholds form
    a hysteresis band; inside it the current set is held, so a vehicle bobbing
    at the waterline does not flap the EKF between sources.
    """
    if depth >= dive_depth:
        return SET_DIVE
    if depth <= surface_depth:
        return SET_SURFACE
    return current


def self_check():
    d, s = 0.5, 0.2
    assert desired_set(0.0, SET_SURFACE, d, s) == SET_SURFACE
    assert desired_set(2.0, SET_SURFACE, d, s) == SET_DIVE, "must dive past the threshold"
    assert desired_set(0.0, SET_DIVE, d, s) == SET_SURFACE, "must resurface"
    # Inside the band, whichever set is active stays active. This is the bit
    # that stops the flapping, and the only reason `current` is a parameter.
    assert desired_set(0.35, SET_SURFACE, d, s) == SET_SURFACE
    assert desired_set(0.35, SET_DIVE, d, s) == SET_DIVE
    # Thresholds themselves are inclusive on both sides.
    assert desired_set(0.5, SET_SURFACE, d, s) == SET_DIVE
    assert desired_set(0.2, SET_DIVE, d, s) == SET_SURFACE
    print("ekf_source_switch: self-check OK")


def main():
    import rclpy
    from mavros_msgs.srv import CommandLong
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float64

    class EkfSourceSwitch(Node):
        def __init__(self):
            super().__init__("ekf_source_switch")
            self.declare_parameter("dive_depth", 0.5)
            self.declare_parameter("surface_depth", 0.2)
            self.dive_depth = self.get_parameter("dive_depth").value
            self.surface_depth = self.get_parameter("surface_depth").value

            # Start assuming surface: on the deck, unarmed, that is true, and if
            # it is not the first depth sample past dive_depth corrects it.
            self.active = SET_SURFACE
            self.pending = None  # set we have commanded but not seen accepted

            self.cli = self.create_client(CommandLong, "/mavros/cmd/command")

            # rel_alt, NOT local_position/pose. local_position z is relative to
            # the EKF ORIGIN -- wherever the vehicle happened to be when the
            # origin was set, which in sim is on the pool floor. Measured with
            # it, a sub sitting 2.5 m down reads a depth of 0.07 m and the
            # switch never fires. rel_alt is relative to home, and ArduSub pins
            # home altitude to the water surface, so it is depth (negative down).
            #
            # Still no feedback loop: EK3_SRC*_POSZ is BARO in both source sets,
            # so this number does not depend on the horizontal source we are
            # choosing here.
            #
            # ponytail: home-relative, so it inherits ~0.1 m of home-fix error
            # (measured 0.138 m in SITL). Well inside the hysteresis band. If a
            # real Bar30 shows worse, add an explicit surface-offset param
            # rather than narrowing the band.
            self.create_subscription(
                Float64,
                "/mavros/global_position/rel_alt",
                self.on_depth,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            self.get_logger().info(
                f"depth switch armed: dive>={self.dive_depth} m, surface<={self.surface_depth} m"
            )

        def on_depth(self, msg):
            depth = -msg.data  # rel_alt is negative below the surface
            want = desired_set(depth, self.active, self.dive_depth, self.surface_depth)
            if want == self.active or want == self.pending:
                return
            self.send(want, depth)

        def send(self, source_set, depth):
            if not self.cli.service_is_ready():
                self.get_logger().warn("/mavros/cmd/command not up yet; will retry")
                return
            req = CommandLong.Request()
            req.command = MAV_CMD_SET_EKF_SOURCE_SET
            req.param1 = float(source_set)
            self.pending = source_set
            name = "SRC1/GPS" if source_set == SET_SURFACE else "SRC2/DVL"
            self.get_logger().info(f"depth {depth:.2f} m -> {name}")

            def done(fut):
                # Clearing `pending` on failure is what makes this retry: the
                # next pose sample sees want != active != pending and resends.
                self.pending = None
                try:
                    if fut.result().success:
                        self.active = source_set
                        self.get_logger().info(f"EKF source set {source_set} accepted")
                    else:
                        self.get_logger().error(
                            f"EKF source set {source_set} REJECTED "
                            f"(result={fut.result().result}); retrying"
                        )
                except Exception as exc:  # service call itself failed
                    self.get_logger().error(f"set-source call failed: {exc}; retrying")

            self.cli.call_async(req).add_done_callback(done)

    rclpy.init()
    node = EkfSourceSwitch()
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
