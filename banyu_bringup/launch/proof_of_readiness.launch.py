import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, SetUseSimTime


def generate_launch_description():
    banyu_sim_pkg_share = get_package_share_directory("banyu_simulation")
    world_file = os.path.join(
        banyu_sim_pkg_share,
        "worlds",
        "proof_of_readiness.world",
    )
    por_pool_file = os.path.join(
        banyu_sim_pkg_share, "models", "por_pool", "model.xacro"
    )
    por_pool_xml = xacro.process(por_pool_file)

    set_use_sim_time = SetUseSimTime(True)

    ros_gz_sim_launch = ExecuteProcess(
        cmd=["gz", "sim", "-v3", "-r", world_file], output="both"
    )

    spawn_por_pool = Node(
        executable="create",
        package="ros_gz_sim",
        name="spawn_por_pool",
        arguments=["-z", "-2.0", "-string", por_pool_xml, "-name", "pool"],
    )

    return LaunchDescription(
        [
            set_use_sim_time,
            ros_gz_sim_launch,
            # spawn_por_pool,
        ]
    )
