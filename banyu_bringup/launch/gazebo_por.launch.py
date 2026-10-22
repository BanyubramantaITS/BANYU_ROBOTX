import os
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        name="use_sim",
        default_value="True",
        description="Whether to launch the Gazebo simulation environment",
        choices=["True", "False"],
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        choices=["True", "False"],
    )

    headless_arg = DeclareLaunchArgument(
        name="headless",
        default_value="False",
        description="Run the Gazebo server only (no GUI): faster real-time factor for test runs.",
        choices=["True", "False"],
    )

    # SetUseSimTime only configures launch's own node; SetParameter reaches every launched node.
    set_use_sim_time = SetParameter(
        name="use_sim_time",
        value=True,
        condition=IfCondition(
            AndSubstitution(
                EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
                EqualsSubstitution(LaunchConfiguration("use_sim_time"), "True"),
            )
        ),
    )

    _banyu_sim_pkg_share = get_package_share_directory("banyu_simulation")
    _bluerov2_gz_pkg_share = get_package_share_directory("bluerov2_gz")

    _world_file = os.path.join(
        _banyu_sim_pkg_share,
        "worlds",
        "proof_of_readiness.world",
    )

    # `gz sim` shares its transport with any other running instance, so `create` must be
    # told which world to spawn into instead of letting it pick the first world it finds.
    _world_name = ET.parse(_world_file).getroot().find("world").get("name")

    # Gazebo resolves `model://<pkg>/...` and `model://<model>/...` mesh URIs through this
    # variable; set it here so it cannot go stale with the caller's shell environment.
    _gz_resource_path = os.pathsep.join(
        filter(
            None,
            [
                os.path.dirname(_banyu_sim_pkg_share),
                os.path.join(_banyu_sim_pkg_share, "models"),
                os.path.dirname(_bluerov2_gz_pkg_share),
                os.path.join(_bluerov2_gz_pkg_share, "models"),
                os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
            ],
        )
    )

    _bluerov2_heavy_file = os.path.join(
        _bluerov2_gz_pkg_share, "models", "bluerov2_heavy", "model.xacro"
    )

    bluerov2_heavy_xml = xacro.process(_bluerov2_heavy_file)

    spawn_bluerov2_heavy = Node(
        executable="create",
        package="ros_gz_sim",
        name="spawn_bluerov2_heavy",
        arguments=[
            "-world",
            _world_name,
            "-string",
            bluerov2_heavy_xml,
            "-name",
            "bluerov2_heavy",
            "-x",
            "-7",
            # Start at the surface like the real vehicle: ArduSub zeroes rel_alt (depth) on arming.
            "-z",
            "-0.1",
        ],
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": bluerov2_heavy_xml}],
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    ros_gz_sim_launch = ExecuteProcess(
        cmd=["gz", "sim", "-v3", "-r", _world_file],
        additional_env={"GZ_SIM_RESOURCE_PATH": _gz_resource_path},
        output="both",
        condition=IfCondition(
            AndSubstitution(
                EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
                EqualsSubstitution(LaunchConfiguration("headless"), "False"),
            )
        ),
    )

    ros_gz_sim_headless_launch = ExecuteProcess(
        cmd=["gz", "sim", "-v3", "-r", "-s", _world_file],
        additional_env={"GZ_SIM_RESOURCE_PATH": _gz_resource_path},
        output="both",
        condition=IfCondition(
            AndSubstitution(
                EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
                EqualsSubstitution(LaunchConfiguration("headless"), "True"),
            )
        ),
    )

    _por_pool_file = os.path.join(
        _banyu_sim_pkg_share, "models", "por_pool", "model.xacro"
    )
    _por_pool_xml = xacro.process(_por_pool_file)
    spawn_por_pool = Node(
        executable="create",
        package="ros_gz_sim",
        name="spawn_por_pool",
        arguments=[
            "-world",
            _world_name,
            "-z",
            "-2.5",
            "-string",
            _por_pool_xml,
            "-name",
            "pool",
        ],
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    _gate_file = os.path.join(_banyu_sim_pkg_share, "models", "Gate", "model.xacro")
    _gate_xml = xacro.process(_gate_file)
    spawn_gate = Node(
        executable="create",
        package="ros_gz_sim",
        name="spawn_gate",
        arguments=["-world", _world_name, "-string", _gate_xml, "-name", "gate", "-x", "-4"],
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    _buoy_file = os.path.join(
        _banyu_sim_pkg_share, "models", "yellow_buoy", "model.xacro"
    )
    _buoy_xml = xacro.process(_buoy_file)
    spawn_buoy = Node(
        executable="create",
        package="ros_gz_sim",
        name="spawn_buoy",
        arguments=[
            "-world",
            _world_name,
            "-string",
            _buoy_xml,
            "-name",
            "yellow_buoy",
            "-x",
            "6",
            "-z",
            "-2.1",
        ],
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    gazebo_ros_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/front_camera/image_raw"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    bottom_camera_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/bottom_camera/image_raw"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    # Gazebo sim time for every node running with use_sim_time (sensor stamps use it too).
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_info_bridge",
        arguments=["/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    bottom_camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bottom_camera_info_bridge",
        arguments=[
            "/bottom_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    return LaunchDescription(
        [
            use_sim_arg,
            use_sim_time_arg,
            headless_arg,
            set_use_sim_time,
            clock_bridge,
            ros_gz_sim_launch,
            ros_gz_sim_headless_launch,
            spawn_por_pool,
            spawn_gate,
            spawn_buoy,
            spawn_bluerov2_heavy,
            robot_state_publisher,
            gazebo_ros_image_bridge,
            bottom_camera_image_bridge,
            camera_info_bridge,
            bottom_camera_info_bridge,
        ]
    )
