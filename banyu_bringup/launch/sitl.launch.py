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
from launch_ros.actions import SetParameter
from launch_ros.parameter_descriptions import ParameterValue


# ArduSub SITL only. MAVROS is not started by any launch file: run tools/scripts/run/launch_mavros.sh.
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

    use_gps_arg = DeclareLaunchArgument(
        name="use_gps",
        default_value="True",
        description=(
            "Whether GPS is available for EKF aiding. When False, the SITL GPS is "
            "removed from boot (sim_no_gps.parm); nogps_odom_beacon (started by the "
            "parent launch) then selects EKF3 source set 2 (baro + compass)."
        ),
        choices=["True", "False"],
    )

    extra_sitl_params_arg = DeclareLaunchArgument(
        name="extra_sitl_params",
        default_value="",
        description=(
            "Optional extra ArduPilot .parm file for sim_vehicle.py (fault injection, e.g. "
            "config/sim_faults/dead_thrusters_2_3.parm). Simulation only."
        ),
    )

    site_arg = DeclareLaunchArgument(
        name="site",
        default_value="",
        description="Simulation GPS home: empty = RATBeach (Long Beach), 'marina_bay' = Marina Bay, Singapore.",
    )

    headless_arg = DeclareLaunchArgument(
        name="headless",
        default_value="False",
        description="Run headless without MAVProxy GUI console",
        choices=["True", "False"],
    )

    # SetUseSimTime only configures launch's own node; SetParameter reaches every launched node.
    set_use_sim_time = SetParameter(
        name="use_sim_time",
        value=ParameterValue(
            AndSubstitution(
                EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
                EqualsSubstitution(LaunchConfiguration("use_sim_time"), "True"),
            ),
            value_type=bool,
        ),
    )

    _banyu_bringup_share = get_package_share_directory("banyu_bringup")
    _banyu_defaults_file = os.path.join(
        _banyu_bringup_share, "config", "banyu_defaults.parm"
    )

    # Sim-only tuning (attitude gains); never loaded on the robot.
    _sim_only_file = os.path.join(_banyu_bringup_share, "config", "sim_only.parm")
    _sim_defaults = _banyu_defaults_file + "," + _sim_only_file

    # use_gps:=False removes the simulated GPS from boot, like the real AUV.
    _no_gps_file = os.path.join(_banyu_bringup_share, "config", "sim_no_gps.parm")
    _no_gps = ["'", LaunchConfiguration("use_gps"), "'.lower() == 'false'"]

    _ardusub_cmd = [
        "ardusub",
        "-w",
        "-M",
        "JSON",
        "-I0",
        "--defaults",
        PythonExpression(
            ["'" + _sim_defaults + ",' + '" + _no_gps_file + "' if "] + _no_gps
            + [" else '" + _sim_defaults + "'"]
        ),
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

    _ardusub_sitl_base_cmd = [
        "sim_vehicle.py",
        PythonExpression(["'--custom-location=1.2816,103.8636,0,270' if '", LaunchConfiguration("site"),
                          "' == 'marina_bay' else '--location=RATBeach'"]),
        "-v",
        "ArduSub",
        "-f",
        "vectored_6dof",
        "--model=JSON",
        "--out=udp:0.0.0.0:14550",
        "--out=udp:127.0.0.1:14551",
        "--out=udp:127.0.0.1:14552",
        # 25 Hz of sim time: on a slow machine (0.34x real time on an iGPU) 10 Hz gave poses up to 0.6 s of wall time
        # apart, past vehicle execution's 0.5 s pose freshness for the position hold, and the readiness latched off.
        "--mavproxy-args=--streamrate=25",
        "-w",
        "--add-param-file=" + _banyu_defaults_file,
        "--add-param-file=" + _sim_only_file,
        PythonExpression(
            ["'--add-param-file=" + _no_gps_file + "' if "] + _no_gps
            + [" else '--add-param-file=" + _banyu_defaults_file + "'"]
        ),
        PythonExpression([
            "'--add-param-file=' + '", LaunchConfiguration("extra_sitl_params"),
            "' if '", LaunchConfiguration("extra_sitl_params"),
            "' else '--add-param-file=" + _banyu_defaults_file + "'",
        ]),
    ]
    ardusub_sitl_launch = ExecuteProcess(
        cmd=_ardusub_sitl_base_cmd + ["--console"],
        name="ardusub_sitl",
        output="screen",
        log_cmd=True,
        condition=IfCondition(
            AndSubstitution(
                AndSubstitution(
                    EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
                    EqualsSubstitution(LaunchConfiguration("use_ardusub_sitl"), "True"),
                ),
                EqualsSubstitution(LaunchConfiguration("headless"), "False"),
            )
        ),
    )
    ardusub_sitl_headless_launch = ExecuteProcess(
        cmd=_ardusub_sitl_base_cmd,
        name="ardusub_sitl_headless",
        output="screen",
        log_cmd=True,
        condition=IfCondition(
            AndSubstitution(
                AndSubstitution(
                    EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
                    EqualsSubstitution(LaunchConfiguration("use_ardusub_sitl"), "True"),
                ),
                EqualsSubstitution(LaunchConfiguration("headless"), "True"),
            )
        ),
    )

    return LaunchDescription(
        [
            site_arg,
            use_sim_arg,
            use_ardusub_sitl_arg,
            use_sim_time_arg,
            headless_arg,
            extra_sitl_params_arg,
            use_gps_arg,
            set_use_sim_time,
            ardusub_launch,
            ardusub_sitl_launch,
            ardusub_sitl_headless_launch,
        ]
    )
