import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    if "sim:=ulab" in sys.argv[4:]:
        sim = True
        unitree = False
        config_file = "leg_param_sim.yaml"
    elif "sim:=go2" in sys.argv[4:] or "sim:=unitree" in sys.argv[4:]:
        sim = True
        unitree = True
        config_file = "leg_param_sim.yaml"
    elif "real:=ulab" in sys.argv[4:]:
        sim = False
        unitree = False
        config_file = "leg_param_real_ulab.yaml"
    elif "real:=go2" in sys.argv[4:] or "real:=unitree" in sys.argv[4:]:
        sim = False
        unitree = True
        config_file = "leg_param_real_go2.yaml"
    else:
        print("Please specify param 'sim' or 'real' and robot. E.g. 'sim:=ulab' or 'real:=go2'.")
        exit()

    pkg_drivers = get_package_share_directory("drivers")
    driver_config_path = os.path.join(pkg_drivers, "config", config_file)

    use_sim_time = LaunchConfiguration("use_sim_time", default=str(sim))
    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time",
        default_value=str(sim),
        description="Use simulation clock if true. Default is false.",
    )

    # Mjbots Motor Driver Launch
    leg_driver = Node(
        package="drivers",
        name="leg_driver",
        executable="leg_driver",
        parameters=[driver_config_path, {"use_sim_time": use_sim_time}],
        # prefix= ["sudo -E env \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\""],
        # remappings = [("/joint_states","/joint_states_hw")],
        shell=True,
        # emulate_tty=True,
        output="screen",
    )
    return LaunchDescription([leg_driver, declare_use_sim_time])
