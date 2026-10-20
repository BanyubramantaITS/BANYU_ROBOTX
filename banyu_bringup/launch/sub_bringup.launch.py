import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    EqualsSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, SetParameter


def _make_camera_node(name, image_topic, info_topic, hardware_params, enable_cameras, target_mode, mode):
    return Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name=name,
        output="screen",
        parameters=[hardware_params],
        remappings=[
            ("image_raw", image_topic),
            ("camera_info", info_topic),
        ],
        condition=IfCondition(
            AndSubstitution(
                EqualsSubstitution(enable_cameras, "True"),
                EqualsSubstitution(mode, target_mode),
            )
        ),
    )


def generate_launch_description():
    banyu_bringup_share = get_package_share_directory("banyu_bringup")

    default_por_params = os.path.join(
        banyu_bringup_share, "config", "por_params.yaml"
    )
    default_task2_params = os.path.join(
        banyu_bringup_share, "config", "task2_params.yaml"
    )
    default_hardware_params = os.path.join(
        banyu_bringup_share, "config", "hardware_params.yaml"
    )

    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="por",
        description="Which competition mission to launch on the real AUV: 'por' or 'task2'.",
        choices=["por", "task2"],
    )

    fcu_url_arg = DeclareLaunchArgument(
        name="fcu_url",
        default_value="udp://127.0.0.1:14551@",
        description=(
            "FCU connection URL for the physical Pixhawk "
            "(udp://127.0.0.1:14551@ when using tools/scripts/run_mavproxy.sh, "
            "or /dev/ttyACM0:921600 for direct USB serial)."
        ),
    )

    use_dvl_arg = DeclareLaunchArgument(
        name="use_dvl",
        default_value="False",
        description="Enable DVL source switching when a DVL is fitted on the AUV.",
        choices=["True", "False"],
    )

    use_gps_arg = DeclareLaunchArgument(
        name="use_gps",
        default_value="False",
        description=(
            "Whether GPS is available on the AUV. False (default for POR) "
            "forces EKF3 SRC2 (baro + compass) and runs nogps_odom_beacon when use_dvl is False."
        ),
        choices=["True", "False"],
    )

    debug_arg = DeclareLaunchArgument(
        name="debug",
        default_value="False",
        description="Enable OpenCV debug windows (only when a display is attached).",
        choices=["True", "False"],
    )

    enable_teleop_arg = DeclareLaunchArgument(
        name="enable_teleop",
        default_value="True",
        description="Whether to launch banyu_teleop for TX16S RC switches and ESP32 kill relay.",
        choices=["True", "False"],
    )

    enable_mock_auto_arg = DeclareLaunchArgument(
        name="enable_mock_auto",
        default_value="True",
        description="banyu_teleop land-test mode: SB up arms, sets MANUAL and spins all thrusters "
        "slowly. Set False for a real POR run so SB up means AUTONOMOUS for the mission.",
        choices=["True", "False"],
    )

    enable_esp32_bridge_arg = DeclareLaunchArgument(
        name="enable_esp32_bridge",
        default_value="False",
        description="Whether to launch esp32_bridge_node (keep False when enable_teleop is True).",
        choices=["True", "False"],
    )

    enable_esp32_trigger_arg = DeclareLaunchArgument(
        name="enable_esp32_trigger",
        default_value="False",
        description="Whether to launch the ESP32 trigger node.",
        choices=["True", "False"],
    )

    enable_cameras_arg = DeclareLaunchArgument(
        name="enable_cameras",
        default_value="True",
        description="Whether to launch local V4L2 USB camera driver nodes.",
        choices=["True", "False"],
    )

    por_params_arg = DeclareLaunchArgument(
        name="por_params_file",
        default_value=default_por_params,
        description="Path to ROS 2 parameter YAML file for POR nodes.",
    )

    task2_params_arg = DeclareLaunchArgument(
        name="task2_params_file",
        default_value=default_task2_params,
        description="Path to ROS 2 parameter YAML file for Task 2 nodes.",
    )

    hardware_params_arg = DeclareLaunchArgument(
        name="hardware_params_file",
        default_value=default_hardware_params,
        description="Path to ROS 2 parameter YAML file for hardware camera/sensor nodes.",
    )

    set_wall_time = SetParameter(name="use_sim_time", value=False)

    mode = LaunchConfiguration("mode")
    enable_cameras = LaunchConfiguration("enable_cameras")
    por_params = LaunchConfiguration("por_params_file")
    task2_params = LaunchConfiguration("task2_params_file")
    hardware_params = LaunchConfiguration("hardware_params_file")

    active_params = PythonExpression([
        "'", por_params, "' if '", mode, "' == 'por' else '", task2_params, "'"
    ])

    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(banyu_bringup_share, "launch", "sitl_mavros.launch.py")
        ),
        launch_arguments={
            "use_sim": "False",
            "use_sim_time": "False",
            "use_ardusub_sitl": "False",
            "fcu_url": LaunchConfiguration("fcu_url"),
            "use_dvl": LaunchConfiguration("use_dvl"),
            "use_gps": LaunchConfiguration("use_gps"),
        }.items(),
    )

    nogps_odom_beacon = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "banyu_bringup",
            "nogps_odom_beacon.py",
            "--mavlink",
            "udpin:127.0.0.1:14552",
        ],
        name="nogps_odom_beacon",
        output="screen",
        condition=IfCondition(
            AndSubstitution(
                EqualsSubstitution(LaunchConfiguration("use_gps"), "False"),
                EqualsSubstitution(LaunchConfiguration("use_dvl"), "False"),
            )
        ),
    )

    teleop_node = Node(
        package="banyu_teleop",
        executable="banyu_teleop",
        name="banyu_teleop",
        output="screen",
        parameters=[
            active_params,
            hardware_params,
            {"enable_mock_auto": ParameterValue(
                LaunchConfiguration("enable_mock_auto"), value_type=bool)},
        ],
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("enable_teleop"), "True")
        ),
    )

    front_camera_node = _make_camera_node(
        name="front_camera",
        image_topic="/front_camera/image_raw",
        info_topic="/camera/camera_info",
        hardware_params=hardware_params,
        enable_cameras=enable_cameras,
        target_mode="por",
        mode=mode,
    )

    bottom_camera_node = _make_camera_node(
        name="bottom_camera",
        image_topic="/bottom_camera/image_raw",
        info_topic="/bottom_camera/camera_info",
        hardware_params=hardware_params,
        enable_cameras=enable_cameras,
        target_mode="task2",
        mode=mode,
    )

    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(banyu_bringup_share, "launch", "mission.launch.py")
        ),
        launch_arguments={
            "mode": mode,
            "use_sim": "False",
            "use_sim_time": "False",
            "debug": LaunchConfiguration("debug"),
            "enable_esp32_trigger": LaunchConfiguration("enable_esp32_trigger"),
            "enable_esp32_bridge": LaunchConfiguration("enable_esp32_bridge"),
            "por_params_file": por_params,
            "task2_params_file": task2_params,
            "hardware_params_file": hardware_params,
        }.items(),
    )

    return LaunchDescription(
        [
            mode_arg,
            fcu_url_arg,
            use_dvl_arg,
            use_gps_arg,
            debug_arg,
            enable_teleop_arg,
            enable_mock_auto_arg,
            enable_esp32_bridge_arg,
            enable_esp32_trigger_arg,
            enable_cameras_arg,
            por_params_arg,
            task2_params_arg,
            hardware_params_arg,
            set_wall_time,
            mavros_launch,
            nogps_odom_beacon,
            teleop_node,
            front_camera_node,
            bottom_camera_node,
            mission_launch,
        ]
    )
