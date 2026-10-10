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
    from banyu_msgs.msg import ActiveBuoyFix, PipelineEndCandidate, RunLifecycle
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import NavSatFix
    node = Task2SimProvider()
    buoys, candidates = [], []
    subs = [
        node.create_subscription(ActiveBuoyFix, '/banyu/task2/active_buoy_fix',
                                 buoys.append, 10),
        node.create_subscription(PipelineEndCandidate, '/banyu/pipeline/end_candidate_state',
                                 candidates.append, 10),
    ]
    try:
        node._start_buoy_fix = NavSatFix(latitude=33.81, longitude=-118.39)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = 51.3, 13.85
        pose.pose.orientation.w = 1.0
        node._latest_vehicle_pose = pose
        node._publish_run_fixtures()
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert not buoys and not candidates
        node._lifecycle = RunLifecycle(run_id=7, epoch=2, phase=RunLifecycle.ACTIVE)
        node._publish_run_fixtures()
        node._publish_run_fixtures()
        start = time.monotonic()
        while (not buoys or len(candidates) < 2) and time.monotonic() - start < 1.0:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert len(buoys) == 1  # republished at ~1 Hz, not every tick
        assert (buoys[0].run_id, buoys[0].epoch) == (7, 2)
        assert buoys[0].fix.header.stamp == buoys[0].header.stamp
        assert buoys[0].green_confirmed and not buoys[0].pinger_confirmed
        assert buoys[0].source_id == 'uuv_buoy_localizer'
        assert candidates and all(c.candidate and c.run_id == 7 for c in candidates)
    finally:
        for sub in subs:
            node.destroy_subscription(sub)
        node.destroy_node()
