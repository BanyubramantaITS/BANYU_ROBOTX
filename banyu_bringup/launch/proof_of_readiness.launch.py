import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, SetUseSimTime


def generate_launch_description():
    use_sim_time = True
    set_use_sim_time = SetUseSimTime(use_sim_time)

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

    bluerov2_heavy_file = os.path.join(
        banyu_sim_pkg_share, "models", "bluerov2_heavy", "model.urdf"
    )
    with open(bluerov2_heavy_file, "r") as f:
        bluerov2_heavy_xml = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": bluerov2_heavy_xml}],
    )

    ardusub_cmd = [
        "ardusub",
        "-w",
        "-M",
        "JSON",
        # "--defaults",
        # ardusub_params_file,
        "-I0",
        # "--home",
        # "33.810313,-118.39386700000001,0.0,0",
    ]
    # if use_sim_time:
    #     ardusub_cmd += [
    #         "--use_sim_time",
    #         "true",
    #     ]
    ardusub_launch = ExecuteProcess(
        cmd=ardusub_cmd,
        output="screen",
        log_cmd=True,
        # respawn=True
    )

    # ros_gz_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         # Add your required topic bridges here. Example below maps Odometry:
    #         "/model/bluerov2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    #     ],
    #     output="screen",
    # )

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
            spawn_por_pool,
            robot_state_publisher,
            ardusub_launch,
        ]
    )
