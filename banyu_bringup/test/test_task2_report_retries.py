from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from launch import LaunchContext


_LAUNCH_PATH = Path(__file__).parents[1] / "launch" / "mission.launch.py"
_SPEC = spec_from_file_location("banyu_bringup_mission_launch", _LAUNCH_PATH)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)


def _max_attempts(requested_retries):
    context = LaunchContext()
    context.launch_configurations["task2_report_max_retries"] = str(requested_retries)
    return int(_MODULE._task2_report_max_attempts_substitution().perform(context))


def test_zero_requested_retries_still_sends_initial_report():
    assert _max_attempts(0) == 1


def test_default_three_retries_produces_four_attempts():
    assert _max_attempts(3) == 4


def test_maximum_requested_retries_saturates_attempts_at_255():
    assert _max_attempts(255) == 255
