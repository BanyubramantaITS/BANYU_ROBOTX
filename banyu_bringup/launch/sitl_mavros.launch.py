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
from launch_ros.actions import Node, SetParameter


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

    use_dvl_arg = DeclareLaunchArgument(
        name="use_dvl",
        default_value="False",
        description=(
            "Enable DVL transport and source switching. False leaves the "
            "estimator on its configured source set without DVL aiding."
        ),
        choices=["True", "False"],
    )

    fcu_url_arg = DeclareLaunchArgument(
        name="fcu_url",
        default_value="udp://127.0.0.1:14551@",
        description="FCU connection URL (udp://127.0.0.1:14551@ for SITL or udp://192.168.2.1:14550@ for real ROV). SITL keeps 14550 free for QGC.",
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
        # Third endpoint for the DVL bridge. It cannot share MAVROS' 14551, and
        # MAVROS cannot carry VISION_POSITION_DELTA anyway.
        "--out=udp:127.0.0.1:14552",
        "--console",
        # MAVProxy requests telemetry at 4 Hz by default; mission readiness needs pose fresher than 1 s.
        "--mavproxy-args=--streamrate=10",
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
            "-p",
            PythonExpression(
                [
                    "'use_sim_time:=' + ('true' if '",
                    LaunchConfiguration("use_sim"),
                    "' == 'True' and '",
                    LaunchConfiguration("use_sim_time"),
                    "' == 'True' else 'false')",
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

    # DVL -> ArduSub. Deliberately NOT a MAVROS plugin and not a ros_gz_bridge:
    # MAVROS cannot emit VISION_POSITION_DELTA and ros_gz_bridge on Humble cannot
    # carry the gz DVL message. See banyu_bringup/README.md.
    # Must be ExecuteProcess, not Node: this is a plain process, not a ROS node,
    # and a launch Node action injects `--ros-args -r __node:=...` which it
    # rejects. Same reason mavros_node is an ExecuteProcess above.
    dvl_bridge = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "banyu_bringup",
            "dvl_mavlink_bridge.py",
            "--mavlink",
            "udpin:127.0.0.1:14552",
            "--verbose",
        ],
        name="dvl_mavlink_bridge",
        output="screen",
        condition=IfCondition(
            AndSubstitution(
                EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
                EqualsSubstitution(LaunchConfiguration("use_dvl"), "True"),
            )
        ),
    )

    dvl_health = Node(
        package="banyu_bringup",
        executable="dvl_health.py",
        name="dvl_health",
        output="screen",
        condition=IfCondition(
            AndSubstitution(
                EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
                EqualsSubstitution(LaunchConfiguration("use_dvl"), "True"),
            )
        ),
    )

    # GPS on the surface, DVL underwater. ArduPilot has the source sets and
    # three ways to select one, but switches on depth by itself in none of them
    # -- this node is that decision. It is only started when DVL is enabled, so
    # the no-DVL default cannot select the DVL source set.
    ekf_source_switch = Node(
        package="banyu_bringup",
        executable="ekf_source_switch.py",
        name="ekf_source_switch",
        output="screen",
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("use_dvl"), "True")
        ),
    )

    return LaunchDescription(
        [
            use_sim_arg,
            use_ardusub_sitl_arg,
            use_sim_time_arg,
            use_dvl_arg,
            fcu_url_arg,
            set_use_sim_time,
            ardusub_launch,
            ardusub_sitl_launch,
            mavros_launch,
            dvl_bridge,
            dvl_health,
            ekf_source_switch,
        ]
    )
