"""RoboCommand bridge — runs on the topside OCS host, never on the vehicle.

Handbook §3.4: "Only the OCS connects to RoboCommand. Team vehicles must not
connect directly to the RoboCommand network or MQTT broker." So this node sits
in the ROS graph with the vehicle, and it alone speaks MQTT.

Publishes an RxReport/Heartbeat per vehicle at 2 Hz (§3.4.11), subscribes to the
retained RxCourse and the team command topic, and can emit the one-shot
RunDeclaration that starts the §3.4.10 run-start sequence.

See src/docs/robocommand-integration-research.md for every citation.
"""

import math

import paho.mqtt.client as mqtt
import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float64, UInt8, UInt32
from std_srvs.srv import Trigger

from . import messages
from .rc_proto import common_pb2, rx_commands_pb2, rx_common_pb2, rx_course_pb2

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

STATE_BY_NAME = {
    'killed': common_pb2.STATE_KILLED,
    'manual': common_pb2.STATE_MANUAL,
    'auto': common_pb2.STATE_AUTO,
}


def quat_to_roll_pitch(x, y, z, w):
    """Roll/pitch in degrees. Yaw comes from the compass, not from here."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    return math.degrees(roll), math.degrees(pitch)


class RcBridge(Node):

    def __init__(self):
        super().__init__('rc_bridge')

        # O1 in the research note: the published Team ID list has ITSN =
        # "Barunastra ITS" and no Banyubramanta row. Confirm before publishing
        # to a real broker — §3.4.14 forbids using another team's identifier.
        self.team = self.declare_parameter('team_id', 'ITSN').value
        self.vehicle = self.declare_parameter('vehicle_id', 'UUV1').value
        self.broker_host = self.declare_parameter('broker_host', 'localhost').value
        self.broker_port = self.declare_parameter('broker_port', 1883).value
        hz = self.declare_parameter('heartbeat_hz', 2.0).value
        # depth_m sign is UNVERIFIED upstream (schema says only "// UUV only").
        # Positive-down is the likely intent; keep the knob rather than bake it in.
        self.depth_sign = self.declare_parameter('depth_sign', -1.0).value
        # The hardware kill switch cuts propulsion power but need not disarm the
        # autopilot (§5.3.2), so /mavros/state can lie. Operator override:
        #   ros2 param set /rc_bridge state_override killed
        self.declare_parameter('state_override', '')
        # Run declaration, published by the declare_run service.
        self.declare_parameter('run_vehicle_ids', ['USV1', 'UUV1'])
        self.declare_parameter('run_tiers', ['none', 'core', 'none', 'none'])

        self.seq = 0
        self.request_seq = 0
        self.lat = self.lon = self.alt_hae = 0.0
        self.rel_alt = 0.0
        self.speed = self.heading = self.roll = self.pitch = 0.0
        self.task = rx_common_pb2.TASK_NONE
        self.state = None

        self.pinger_pub = self.create_publisher(UInt32, 'banyu/robocommand/pinger_freq_hz', LATCHED)
        self.run_start_pub = self.create_publisher(Bool, 'banyu/robocommand/run_start', LATCHED)

        self.create_subscription(NavSatFix, 'mavros/global_position/global', self._on_fix, qos_profile_sensor_data)
        self.create_subscription(Float64, 'mavros/global_position/compass_hdg', self._on_hdg, qos_profile_sensor_data)
        self.create_subscription(Float64, 'mavros/global_position/rel_alt', self._on_rel_alt, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, 'mavros/local_position/velocity_local', self._on_vel, qos_profile_sensor_data)
        self.create_subscription(Imu, 'mavros/imu/data', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(State, 'mavros/state', self._on_state, 10)
        self.create_subscription(UInt8, 'banyu/mission/task', self._on_task, 10)

        self.create_service(Trigger, 'banyu/robocommand/declare_run', self._on_declare_run)

        self.mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt.on_connect = self._on_connect
        self.mqtt.on_message = self._on_mqtt
        # connect_async + loop_start so a broker that is not up yet (or drops)
        # is retried instead of killing the node — §3.4.5 wants automatic
        # reconnect with subscriptions restored, which _on_connect does.
        self.mqtt.connect_async(self.broker_host, self.broker_port)
        self.mqtt.loop_start()

        self.create_timer(1.0 / hz, self._beat)
        self.get_logger().info(
            f'rc_bridge: team={self.team} vehicle={self.vehicle} '
            f'broker={self.broker_host}:{self.broker_port} @ {hz} Hz')

    # --- MQTT ---------------------------------------------------------------

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code != 0:
            self.get_logger().error(f'MQTT connect failed: {reason_code}')
            return
        # Re-subscribed on every (re)connect, per §3.4.5.
        client.subscribe(messages.COURSE_TOPIC)
        client.subscribe(messages.command_topic(self.team))
        self.get_logger().info('MQTT connected; subscriptions restored')

    def _on_mqtt(self, client, userdata, msg):
        try:
            if msg.topic == messages.COURSE_TOPIC:
                course = rx_course_pb2.RxCourse()
                course.ParseFromString(msg.payload)
                self.get_logger().info(
                    f'RxCourse {course.course_id}: pinger {course.pinger_freq_hz} Hz, '
                    f'{len(course.corners)} corners')
                self.pinger_pub.publish(UInt32(data=course.pinger_freq_hz))
            else:
                command = rx_commands_pb2.RxCommand()
                command.ParseFromString(msg.payload)
                body = command.WhichOneof('body')
                self.get_logger().info(f'RxCommand {body} seq={command.seq}')
                if body == 'run_start':
                    self.get_logger().warn(
                        f'RunStart: declaration_seq={command.run_start.declaration_seq} '
                        f'run_id={command.run_start.run_id}')
                    self.run_start_pub.publish(Bool(data=True))
                # ponytail: other commands are logged only. Task 4 ack chains
                # (§3.4.12) need a UUV that can act on them first.
        except Exception as exc:  # a malformed payload must not kill the loop
            self.get_logger().error(f'undecodable message on {msg.topic}: {exc}')

    # --- ROS callbacks ------------------------------------------------------

    def _on_fix(self, msg):
        # Last fix is held while submerged — there is no GPS underwater on
        # hardware (same gap as the DVL note). O4: whether a stale LatLng is
        # acceptable is unconfirmed with the Technical Director.
        self.lat, self.lon, self.alt_hae = msg.latitude, msg.longitude, msg.altitude

    def _on_hdg(self, msg):
        self.heading = msg.data

    def _on_rel_alt(self, msg):
        self.rel_alt = msg.data

    def _on_vel(self, msg):
        v = msg.twist.linear
        self.speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    def _on_imu(self, msg):
        q = msg.orientation
        self.roll, self.pitch = quat_to_roll_pitch(q.x, q.y, q.z, q.w)

    def _on_state(self, msg):
        self.state = msg

    def _on_task(self, msg):
        self.task = msg.data

    def _on_declare_run(self, request, response):
        tiers = [messages.TIER_BY_NAME[t] for t in self.get_parameter('run_tiers').value]
        self.request_seq += 1
        req = messages.build_run_declaration(
            team_id=self.team,
            seq=self.request_seq,
            vehicle_ids=list(self.get_parameter('run_vehicle_ids').value),
            tiers=tiers,
        )
        self.mqtt.publish(messages.request_topic(self.team), req.SerializeToString())
        response.success = True
        response.message = f'RunDeclaration seq={self.request_seq} published'
        self.get_logger().info(response.message)
        return response

    # --- heartbeat ----------------------------------------------------------

    def _robot_state(self):
        override = self.get_parameter('state_override').value
        if override:
            return STATE_BY_NAME.get(override, common_pb2.STATE_UNKNOWN)
        if self.state is None:
            return common_pb2.STATE_UNKNOWN
        if not self.state.armed:
            return common_pb2.STATE_KILLED
        return common_pb2.STATE_AUTO if self.state.mode == 'GUIDED' else common_pb2.STATE_MANUAL

    def _beat(self):
        self.seq += 1
        report = messages.build_heartbeat(
            team_id=self.team,
            vehicle_id=self.vehicle,
            seq=self.seq,
            state=self._robot_state(),
            lat=self.lat,
            lon=self.lon,
            altitude_hae_m=self.alt_hae,
            depth_m=self.depth_sign * self.rel_alt,
            spd_mps=self.speed,
            heading_deg=self.heading,
            roll_deg=self.roll,
            pitch_deg=self.pitch,
            current_task=self.task,
        )
        self.mqtt.publish(messages.report_topic(self.team, self.vehicle), report.SerializeToString())


def main(args=None):
    rclpy.init(args=args)
    node = RcBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt.loop_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
