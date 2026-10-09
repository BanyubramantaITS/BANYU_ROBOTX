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
