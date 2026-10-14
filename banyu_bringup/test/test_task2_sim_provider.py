import os
import sys
import time

from banyu_msgs.msg import PipelineRingState
import pytest
import rclpy
from rclpy.parameter import Parameter

# Add scripts directory to path
scripts_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
if scripts_path not in sys.path:
    sys.path.insert(0, scripts_path)

from task2_sim_provider import Task2SimProvider  # noqa: E402


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_default_ring_colors_has_four_nodes(rclpy_init):
    node = Task2SimProvider()
    try:
        colors = node.get_parameter('ring_colors').value
        assert len(colors) == 4
        assert colors == ['green', 'green', 'green', 'red']
        assert node.ring_colors == ['green', 'green', 'green', 'red']
    finally:
        node.destroy_node()


def test_custom_four_ring_colors(rclpy_init):
    param = Parameter('ring_colors', Parameter.Type.STRING_ARRAY,
                      ['red', 'green', 'green', 'green'])
    node = Task2SimProvider(parameter_overrides=[param])
    try:
        assert len(node.ring_colors) == 4
        assert node.ring_colors == ['red', 'green', 'green', 'green']
    finally:
        node.destroy_node()


def test_reject_five_ring_colors(rclpy_init):
    param = Parameter('ring_colors', Parameter.Type.STRING_ARRAY,
                      ['green', 'green', 'green', 'red', 'green'])
    with pytest.raises(ValueError, match='four entries'):
        Task2SimProvider(parameter_overrides=[param])


def test_reject_three_ring_colors(rclpy_init):
    param = Parameter('ring_colors', Parameter.Type.STRING_ARRAY,
                      ['green', 'green', 'red'])
    with pytest.raises(ValueError, match='four entries'):
        Task2SimProvider(parameter_overrides=[param])


def test_reject_invalid_color_entry(rclpy_init):
    param = Parameter('ring_colors', Parameter.Type.STRING_ARRAY,
                      ['green', 'green', 'green', 'yellow'])
    with pytest.raises(ValueError, match='Invalid Task 2 ring color'):
        Task2SimProvider(parameter_overrides=[param])


def test_publish_fixture_publishes_four_ring_states(rclpy_init):
    node = Task2SimProvider()
    published_states = []
    sub = node.create_subscription(
        PipelineRingState,
        '/banyu/debug/task2/ring_state',
        lambda msg: published_states.append(msg),
        10
    )
    try:
        node._publish_fixture()
        start = time.monotonic()
        while len(published_states) < 4 and time.monotonic() - start < 1.0:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert len(published_states) == 4
        indices = [msg.index for msg in published_states]
        colors = [msg.color for msg in published_states]
        assert indices == [0, 1, 2, 3]
        assert colors == [
            PipelineRingState.COLOR_GREEN,
            PipelineRingState.COLOR_GREEN,
            PipelineRingState.COLOR_GREEN,
            PipelineRingState.COLOR_RED
        ]
    finally:
        node.destroy_subscription(sub)
        node.destroy_node()


def test_end_candidate_only_inside_end_window(rclpy_init):
    node = Task2SimProvider()
    try:
        point = type('Point', (), {})
        def at(x, y):
            p = point()
            p.x, p.y = x, y
            return p
        assert not node._near_pipeline_end(at(47.2, 13.85))
        assert not node._near_pipeline_end(at(50.5, 13.85))
        assert node._near_pipeline_end(at(51.2, 13.85))
        assert node._near_pipeline_end(at(51.9, 13.85))
        assert not node._near_pipeline_end(at(51.2, 15.5))
        assert not node._near_pipeline_end(at(53.0, 13.85))
    finally:
        node.destroy_node()


def test_run_fixtures_are_run_tagged_and_need_active_run(rclpy_init):
    from banyu_msgs.msg import PipelineEndCandidate, RunLifecycle
    from geometry_msgs.msg import PoseStamped
    node = Task2SimProvider()
    candidates = []
    sub = node.create_subscription(PipelineEndCandidate, '/banyu/pipeline/end_candidate_state',
                                   candidates.append, 10)
    try:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = 51.3, 13.85
        pose.pose.orientation.w = 1.0
        node._latest_vehicle_pose = pose
        node._publish_run_fixtures()
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert not candidates
        node._lifecycle = RunLifecycle(run_id=7, epoch=2, phase=RunLifecycle.ACTIVE)
        node._publish_run_fixtures()
        node._publish_run_fixtures()
        start = time.monotonic()
        while len(candidates) < 2 and time.monotonic() - start < 1.0:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert candidates and all(c.candidate and c.run_id == 7 for c in candidates)
    finally:
        node.destroy_subscription(sub)
        node.destroy_node()


def _collect_buoys(node, buoys, count):
    start = time.monotonic()
    while len(buoys) < count and time.monotonic() - start < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)


def test_publishes_usv_pipeline_buoys_like_the_usv(rclpy_init):
    from rx_msgs.msg import Course, PipelineBuoys, VehicleType
    from sensor_msgs.msg import NavSatFix
    node = Task2SimProvider(parameter_overrides=[
        Parameter('pipeline_buoys_second_delay_s', Parameter.Type.DOUBLE, 0.4)])
    buoys = []
    sub = node.create_subscription(
        PipelineBuoys, '/system/vehicle/usv/task2/pipeline_buoys', buoys.append, 10)
    try:
        node._start_buoy_fix = NavSatFix(latitude=33.810438, longitude=-118.393392)
        node._end_buoy_fix = NavSatFix(latitude=33.810438, longitude=-118.393272)
        node._publish_pipeline_buoys()
        _collect_buoys(node, buoys, 1)
        assert not buoys  # the USV copies course_id, so nothing before the course
        node._course = Course(course_id='ITSN-SIM-A', pinger_freq_hz=25000)
        node._publish_pipeline_buoys()
        node._publish_pipeline_buoys()  # unchanged content is repeated at ~1 Hz, not per tick
        _collect_buoys(node, buoys, 1)
        assert len(buoys) == 1
        first = buoys[0]
        assert first.course_id == 'ITSN-SIM-A'
        assert first.source_vehicle == VehicleType.TYPE_USV
        assert first.seq == 1 and not first.buoy_b_valid
        assert first.active_buoy == PipelineBuoys.ACTIVE_A
        assert first.buoy_a.longitude == -118.393392
        time.sleep(0.45)
        node._publish_pipeline_buoys()
        _collect_buoys(node, buoys, 2)
        second = buoys[1]
        assert second.seq == 2 and second.buoy_b_valid
        assert second.buoy_b.longitude == -118.393272
        # A jittering surfaced fix does not move the reported buoys.
        node._start_buoy_fix = NavSatFix(latitude=33.82, longitude=-118.38)
        node._usv_buoys_sent_s = 0.0
        node._publish_pipeline_buoys()
        _collect_buoys(node, buoys, 3)
        assert buoys[2].seq == 2 and buoys[2].buoy_a.latitude == 33.810438
    finally:
        node.destroy_subscription(sub)
        node.destroy_node()


def test_start_buoy_can_be_labelled_b_or_reported_unknown(rclpy_init):
    from rx_msgs.msg import Course, PipelineBuoys
    from sensor_msgs.msg import NavSatFix
    for label, report, expected in (('b', True, PipelineBuoys.ACTIVE_B),
                                    ('a', False, PipelineBuoys.ACTIVE_UNKNOWN)):
        node = Task2SimProvider(parameter_overrides=[
            Parameter('pipeline_buoys_start_label', Parameter.Type.STRING, label),
            Parameter('pipeline_buoys_report_active', Parameter.Type.BOOL, report)])
        buoys = []
        sub = node.create_subscription(
            PipelineBuoys, '/system/vehicle/usv/task2/pipeline_buoys', buoys.append, 10)
        try:
            node._course = Course(course_id='ITSN-SIM-A')
            node._start_buoy_fix = NavSatFix(latitude=33.810438, longitude=-118.393392)
            node._end_buoy_fix = NavSatFix(latitude=33.810438, longitude=-118.393272)
            node._publish_pipeline_buoys()
            _collect_buoys(node, buoys, 1)
            assert buoys[0].active_buoy == expected and buoys[0].buoy_b_valid
            start = buoys[0].buoy_b if label == 'b' else buoys[0].buoy_a
            assert start.longitude == -118.393392
        finally:
            node.destroy_subscription(sub)
            node.destroy_node()
