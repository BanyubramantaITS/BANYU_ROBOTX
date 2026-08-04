import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    AndSubstitution,
    EqualsSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node, SetUseSimTime


def generate_launch_description():
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
        default_value="udp://127.0.0.1:14551@",
        description="FCU connection URL (udp://127.0.0.1:14551@ for SITL or udp://192.168.2.1:14550@ for real ROV). SITL keeps 14550 free for QGC.",
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
        _bluerov2_gz_pkg_share, "models", "bluerov2_heavy", "model.urdf"
    )

    with open(_bluerov2_heavy_file, "r") as f:
        bluerov2_heavy_xml = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": bluerov2_heavy_xml}],
    )

    _ardusub_cmd = [
        "ardusub",
        "-w",
        "-M",
        "JSON",
        "-I0",
    ]
    ardusub_launch = ExecuteProcess(
        cmd=_ardusub_cmd,
        name="ardusub_cmd",
        output="screen",
        log_cmd=True,
        condition=IfCondition(
            AndSubstitution(
                EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
                EqualsSubstitution(LaunchConfiguration("use_ardusub_sitl"), "False"),
            )
        ),
    )

    _ardusub_sitl_cmd = [
        "sim_vehicle.py",
        "-L",
        "RATBeach",
        "-v",
        "ArduSub",
        "-f",
        "vectored_6dof",
        "--model=JSON",
        "--out=udp:0.0.0.0:14550",
        "--out=udp:127.0.0.1:14551",
        "--console",
        "-w",
    ]
    ardusub_sitl_launch = ExecuteProcess(
        cmd=_ardusub_sitl_cmd,
        name="ardusub_sitl",
        output="screen",
        log_cmd=True,
        condition=IfCondition(
            AndSubstitution(
                EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
                EqualsSubstitution(LaunchConfiguration("use_ardusub_sitl"), "True"),
            )
        ),
    )

    _banyu_bringup_share = get_package_share_directory("banyu_bringup")
    _mavros_config_file = os.path.join(
        _banyu_bringup_share, "config", "mavros_config.yaml"
    )

    # Launch mavros_node via `ros2 run` (not via mavros' apm.launch) so we can
    # set system_id=255, which ArduSub requires to accept MAVLink MANUAL_CONTROL.
    # It MUST be via ExecuteProcess, NOT the launch Node action: launch injects
    # "-r __node:=mavros", which triggers a MAVROS bug (mavlink/mavros#1977) —
    # the remap also renames MAVROS' internal UAS node, causing an immediate
    # "create_subscription() ... incompatible type ... CompanionProcessStatus"
    # crash. `ros2 run` (no __node remap) avoids it.
    mavros_launch = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "mavros",
            "mavros_node",
            "--ros-args",
            "--params-file",
            _mavros_config_file,
            "-p",
            PythonExpression(
                [
                    "'fcu_url:=' + '",
                    LaunchConfiguration("fcu_url"),
                    "'",
                ]
            ),
            "--log-level",
            "mavros.param:=error",
            "--log-level",
            "mavros.rallypoint:=error",
            "--log-level",
            "mavros.geofence:=error",
        ],
        output="screen",
        name="mavros",
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
        _banyu_sim_pkg_share, "models", "white_buoy", "model.xacro"
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
            "white_buoy",
            "-x",
            "6",
            "-z",
            "-2.1",
        ],
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
            mavros_launch,
        ]
    )
