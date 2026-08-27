import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetUseSimTime


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

    set_use_sim_time = SetUseSimTime(
        True, condition=IfCondition(LaunchConfiguration("use_sim_time"))
    )

    _banyu_sim_pkg_share = get_package_share_directory("banyu_simulation")
    _bluerov2_gz_pkg_share = get_package_share_directory("bluerov2_gz")

    _world_file = os.path.join(
        _banyu_sim_pkg_share,
        "worlds",
        "proof_of_readiness.world",
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
            "-string",
            bluerov2_heavy_xml,
            "-name",
            "bluerov2_heavy",
            "-x",
            "-7",
            "-z",
            "-2",
        ],
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": bluerov2_heavy_xml}],
    )

    ros_gz_sim_launch = ExecuteProcess(
        cmd=["gz", "sim", "-v3", "-r", _world_file],
        output="both",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    _por_pool_file = os.path.join(
        _banyu_sim_pkg_share, "models", "por_pool", "model.xacro"
    )
    _por_pool_xml = xacro.process(_por_pool_file)
    spawn_por_pool = Node(
        executable="create",
        package="ros_gz_sim",
        name="spawn_por_pool",
        arguments=["-z", "-2.5", "-string", _por_pool_xml, "-name", "pool"],
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    _gate_file = os.path.join(_banyu_sim_pkg_share, "models", "Gate", "model.xacro")
    _gate_xml = xacro.process(_gate_file)
    spawn_gate = Node(
        executable="create",
        package="ros_gz_sim",
        name="spawn_gate",
        arguments=["-string", _gate_xml, "-name", "gate", "-x", "-4"],
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
    )

    camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_info_bridge",
        arguments=["/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    return LaunchDescription(
        [
            use_sim_arg,
            use_sim_time_arg,
            set_use_sim_time,
            ros_gz_sim_launch,
            spawn_por_pool,
            spawn_gate,
            spawn_buoy,
            spawn_bluerov2_heavy,
            robot_state_publisher,
            gazebo_ros_image_bridge,
            camera_info_bridge,
        ]
    )
