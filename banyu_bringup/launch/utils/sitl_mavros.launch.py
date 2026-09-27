import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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

    use_ardusub_sitl_arg = DeclareLaunchArgument(
        name="use_ardusub_sitl",
        default_value="True",
        description="Use sim_vehicle.py wrapper (True) or bare ardusub binary (False) for simulation",
        choices=["True", "False"],
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
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

    _banyu_bringup_share = get_package_share_directory("banyu_bringup")
    _banyu_defaults_file = os.path.join(
        _banyu_bringup_share, "config", "banyu_defaults.parm"
    )

    _ardusub_cmd = [
        "ardusub",
        "-w",
        "-M",
        "JSON",
        "-I0",
        "--defaults",
        _banyu_defaults_file,
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
        # Third endpoint for the banyu::DvlMavlink world system (main_world.world).
        # It cannot share MAVROS' 14551, and MAVROS cannot carry VISION_POSITION_DELTA.
        "--out=udp:127.0.0.1:14552",
        "--console",
        "-w",
        "--add-param-file=" + _banyu_defaults_file,
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

    _mavros_config_file = os.path.join(
        _banyu_bringup_share, "config", "mavros_config.yaml"
    )

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

    # GPS on the surface, DVL underwater. ArduPilot has the source sets and
    # three ways to select one, but switches on depth by itself in none of them
    # -- this node is that decision. Not sim-conditioned: unlike the DVL bridge
    # (banyu::DvlMavlink, a sim world system), this is vehicle behaviour and runs on hardware too.
    ekf_source_switch = Node(
        package="banyu_bringup",
        executable="ekf_source_switch.py",
        name="ekf_source_switch",
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_arg,
            use_ardusub_sitl_arg,
            use_sim_time_arg,
            fcu_url_arg,
            set_use_sim_time,
            ardusub_launch,
            ardusub_sitl_launch,
            mavros_launch,
            ekf_source_switch,
        ]
    )
