import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetUseSimTime


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        choices=["True", "False"],
    )

    set_use_sim_time = SetUseSimTime(
        True, condition=IfCondition(LaunchConfiguration("use_sim_time"))
    )

    debug_arg = DeclareLaunchArgument(
        name="debug",
        default_value="False",
        description="Set the debug flag for example (Detection Windows)",
        choices=["True", "False"],
    )

    # Which competition behaviour to run: 'por' or 'mission'. Forwarded to the
    # mission node so POR and main-mission behaviour can diverge later.
    mission_mode_arg = DeclareLaunchArgument(
        name="mission_mode",
        default_value="por",
        description="Which mission behaviour to run: 'por' (proof of readiness) or 'mission' (main mission).",
        choices=["por", "mission"],
    )

    mission_manager = Node(
        package="banyu_mission",
        executable="banyu_mission",
        name="banyu_mission",
        output="screen",
        parameters=[{"mission_mode": LaunchConfiguration("mission_mode")}],
    )

    detection_node = Node(
        package="banyu_detection",
        executable="por_detection",
        name="por_detection",
        parameters=[
            os.path.join(
                get_package_share_directory("banyu_detection"),
                "config",
                "por_detection_params.yaml",
            ),
            {"use_gui": LaunchConfiguration("debug_arg")},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            debug_arg,
            mission_mode_arg,
            set_use_sim_time,
            mission_manager,
            detection_node,
        ]
    )
