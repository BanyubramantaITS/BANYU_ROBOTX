import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetUseSimTime


def generate_launch_description():
    # ----------------------------------------------------
    # Launch Arguments
    # ----------------------------------------------------
    use_sim_arg = DeclareLaunchArgument(
        name="use_sim",
        default_value="True",
        description="Whether to run in simulation mode (Gazebo + SITL) or real ROV hardware mode",
        choices=["True", "False"],
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        choices=["True", "False"],
    )

    use_ardusub_sitl_arg = DeclareLaunchArgument(
        name="use_ardusub_sitl",
        default_value="True",
        description="Use sim_vehicle.py wrapper (True) or bare ardusub binary (False) for simulation",
        choices=["True", "False"],
    )

    fcu_url_arg = DeclareLaunchArgument(
        name="fcu_url",
        default_value="udp://127.0.0.1:14550@",
        description="FCU connection URL (udp://127.0.0.1:14550@ for SITL or udp://192.168.2.1:14550@ for real ROV)",
    )

    set_use_sim_time = SetUseSimTime(
        True, condition=IfCondition(LaunchConfiguration("use_sim_time"))
    )

    # ----------------------------------------------------
    # File Paths
    # ----------------------------------------------------
    banyu_sim_pkg_share = get_package_share_directory("banyu_simulation")
    bluerov2_gz_pkg_share = get_package_share_directory("bluerov2_gz")

    world_file = os.path.join(
        banyu_sim_pkg_share,
        "worlds",
        "proof_of_readiness.world",
    )

    bluerov2_heavy_file = os.path.join(
        bluerov2_gz_pkg_share, "models", "bluerov2_heavy", "model.urdf"
    )

    with open(bluerov2_heavy_file, "r") as f:
        bluerov2_heavy_xml = f.read()

    # ----------------------------------------------------
    # Nodes & Processes
    # ----------------------------------------------------

    # Robot State Publisher (publishes URDF robot_description)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": bluerov2_heavy_xml}],
    )

    # Bare ArduSub Process (sim mode when use_ardusub_sitl == False)
    ardusub_cmd = [
        "ardusub",
        "-w",
        "-M",
        "JSON",
        "-I0",
    ]
    ardusub_launch = ExecuteProcess(
        cmd=ardusub_cmd,
        name="ardusub_cmd",
        output="screen",
        log_cmd=True,
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("use_sim"),
                    "' == 'True' and '",
                    LaunchConfiguration("use_ardusub_sitl"),
                    "' == 'False'",
                ]
            )
        ),
    )

    # ArduSub SITL Process via sim_vehicle.py (sim mode when use_ardusub_sitl == True)
    ardusub_sitl_cmd = [
        "sim_vehicle.py",
        "-L",
        "RATBeach",
        "-v",
        "ArduSub",
        "-f",
        "vectored_6dof",
        "--model=JSON",
        "--out=udp:0.0.0.0:14550",
        "--console",
    ]
    ardusub_sitl_launch = ExecuteProcess(
        cmd=ardusub_sitl_cmd,
        name="ardusub_sitl",
        log_cmd=True,
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("use_sim"),
                    "' == 'True' and '",
                    LaunchConfiguration("use_ardusub_sitl"),
                    "' == 'True'",
                ]
            )
        ),
    )

    # Gazebo Sim Process (sim mode only)
    ros_gz_sim_launch = ExecuteProcess(
        cmd=["gz", "sim", "-v3", "-r", world_file],
        output="both",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    # Spawn Pool Node (sim mode only)
    por_pool_file = os.path.join(
        banyu_sim_pkg_share, "models", "por_pool", "model.xacro"
    )
    por_pool_xml = xacro.process(por_pool_file)
    spawn_por_pool = Node(
        executable="create",
        package="ros_gz_sim",
        name="spawn_por_pool",
        arguments=["-z", "-2.5", "-string", por_pool_xml, "-name", "pool"],
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    # Spawn Gate Node (sim mode only)
    gate_file = os.path.join(banyu_sim_pkg_share, "models", "Gate", "model.xacro")
    gate_xml = xacro.process(gate_file)
    spawn_gate = Node(
        executable="create",
        package="ros_gz_sim",
        name="spawn_gate",
        arguments=["-string", gate_xml, "-name", "gate", "-x", "-4"],
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    # Spawn Buoy Node (sim mode only)
    buoy_file = os.path.join(banyu_sim_pkg_share, "models", "white_buoy", "model.xacro")
    buoy_xml = xacro.process(buoy_file)
    spawn_buoy = Node(
        executable="create",
        package="ros_gz_sim",
        name="spawn_buoy",
        arguments=["-string", buoy_xml, "-name", "white_buoy", "-x", "6", "-z", "-2.1"],
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    return LaunchDescription(
        [
            use_sim_arg,
            use_sim_time_arg,
            use_ardusub_sitl_arg,
            fcu_url_arg,
            set_use_sim_time,
            ros_gz_sim_launch,
            spawn_por_pool,
            spawn_gate,
            spawn_buoy,
            robot_state_publisher,
            ardusub_launch,
            ardusub_sitl_launch,
        ]
    )
