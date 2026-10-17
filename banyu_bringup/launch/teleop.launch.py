import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    banyu_teleop_share = get_package_share_directory("banyu_teleop")

    default_config_path = os.path.join(banyu_teleop_share, "config", "teleop_rcin.yaml")

    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        "teleop_config",
        default_value=default_config_path,
        description="Path to the teleop parameters YAML file",
    )

    teleop_node = Node(
        package="banyu_teleop",
        executable="banyu_teleop",
        name="banyu_teleop",
        parameters=[LaunchConfiguration("teleop_config")],
        output="screen",
    )

    return LaunchDescription([config_file_arg, teleop_node])
