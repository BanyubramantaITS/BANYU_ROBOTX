import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetUseSimTime


def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        name="headless",
        default_value="False",
        description="Whether to run Gazebo Sim in headless mode (-s)",
        choices=["True", "False"],
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        choices=["True", "False"],
    )

    banyu_sim_share = get_package_share_directory("banyu_simulation")
    world_file = os.path.join(banyu_sim_share, "worlds", "pipeline_lights.sdf")
    controller_script = os.path.join(banyu_sim_share, "scripts", "robotx_light_controller.py")

    # Gazebo simulation process
    gz_cmd = ["gz", "sim", "-r", world_file]
    # For headless, users can pass headless:=True (or use -s)
    # Python controller process
    controller_cmd = ["python3", "-u", controller_script, "pipeline_lights"]

    gz_sim_proc = ExecuteProcess(
        cmd=gz_cmd,
        output="screen",
    )

    controller_proc = ExecuteProcess(
        cmd=controller_cmd,
        output="screen",
    )

    return LaunchDescription(
        [
            headless_arg,
            use_sim_time_arg,
            SetUseSimTime(True),
            gz_sim_proc,
            controller_proc,
        ]
    )
