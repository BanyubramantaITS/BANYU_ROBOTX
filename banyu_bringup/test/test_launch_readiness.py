import importlib.util
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest
import yaml


BRINGUP_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
LAUNCH_DIR = os.path.join(BRINGUP_ROOT, "launch")
CONFIG_DIR = os.path.join(BRINGUP_ROOT, "config")


def _load_launch(filename):
    path = os.path.join(LAUNCH_DIR, filename)
    spec = importlib.util.spec_from_file_location(filename.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _declared_args(ld):
    args = {}
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            text = "".join(getattr(s, "text", "") for s in entity.default_value)
            args[entity.name] = text
    return args


def _arg_default(ld, name, **configs):
    context = LaunchContext()
    context.launch_configurations.update(configs)
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.name == name:
            return perform_substitutions(context, entity.default_value)
    raise KeyError(name)


def test_sub_bringup_defaults_are_hardware_safe():
    ld = _load_launch("sub_bringup.launch.py")
    args = _declared_args(ld)
    assert "use_sim" not in args
    assert "use_dvl" not in args
    # POR runs GPS-denied; Task 2 needs the surface GPS leg.
    assert _arg_default(ld, "use_gps", mode="por") == "False"
    assert _arg_default(ld, "use_gps", mode="task2") == "True"
    assert args["enable_teleop"] == "True"
    assert args["enable_esp32_bridge"] == "False"
    # MAVROS runs on its own (tools/scripts/run/launch_mavros.sh), never from a launch file.
    assert "fcu_url" not in args
    assert "launch_mavros" not in args
    assert args["enable_cameras"] == "True"
    assert args["record"] == "True"


def test_sub_bringup_contains_usb_cam_nodes():
    ld = _load_launch("sub_bringup.launch.py")
    nodes = [e for e in ld.entities if isinstance(e, Node)]
    assert len(nodes) == 3
    for node in nodes:
        assert node.condition is not None


def test_gazebo_por_nodes_are_all_sim_gated():
    try:
        pytest.importorskip("xacro")
        get_package_share_directory("banyu_simulation")
        get_package_share_directory("bluerov2_gz")
    except PackageNotFoundError:
        pytest.skip("Simulation packages not installed on this machine")
    ld = _load_launch("gazebo_por.launch.py")
    nodes = [e for e in ld.entities if isinstance(e, Node)]
    assert len(nodes) > 0
    for node in nodes:
        assert node.condition is not None


def test_yaml_configs_are_valid_and_free_of_dead_fixture_keys():
    for fname in (
        "por_params_timed_safe.yaml",
        "task2_params.yaml",
        "hardware_params.yaml",
        "front_camera_info.yaml",
        "bottom_camera_info.yaml",
    ):
        with open(os.path.join(CONFIG_DIR, fname), "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        assert isinstance(data, dict)

    with open(os.path.join(CONFIG_DIR, "task2_params.yaml"), "r", encoding="utf-8") as f:
        task2 = yaml.safe_load(f)
    mission_params = task2["banyu_mission"]["ros__parameters"]
    assert "task2_follow_source" not in mission_params
    assert mission_params["topics"]["pipeline_observation"] == "/banyu/pipeline/observation"
    assert mission_params["topics"]["ring_observation"] == "/banyu/pipeline/ring_observation"

    with open(os.path.join(CONFIG_DIR, "hardware_params.yaml"), "r", encoding="utf-8") as f:
        hw = yaml.safe_load(f)
    assert hw["front_camera"]["ros__parameters"]["camera_info_url"].endswith(
        "front_camera_info.yaml"
    )
    assert hw["bottom_camera"]["ros__parameters"]["camera_info_url"].endswith(
        "bottom_camera_info.yaml"
    )

    with open(os.path.join(CONFIG_DIR, "por_params_timed_safe.yaml"), "r", encoding="utf-8") as f:
        por = yaml.safe_load(f)
    assert por["por_detection"]["ros__parameters"]["image_topic"] == "/front_camera/image_raw"


def test_package_xml_has_no_sim_build_depends():
    tree = ET.parse(os.path.join(BRINGUP_ROOT, "package.xml"))
    root = tree.getroot()
    hard_depends = [el.text for el in root.findall("depend")]
    assert "banyu_simulation" not in hard_depends
    assert "ros_gz_bridge" not in hard_depends
    exec_depends = [el.text for el in root.findall("exec_depend")]
    assert "usb_cam" in exec_depends


def test_main_launch_declares_headless_argument():
    ld = _load_launch("main_launch.launch.py")
    args = _declared_args(ld)
    assert "headless" in args
    assert args["headless"] == "False"
    assert args["mode"] == "por"
    assert args["use_sim"] == "True"
    assert "fcu_url" not in args

