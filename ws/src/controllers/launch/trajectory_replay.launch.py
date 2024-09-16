import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os
import sys

def generate_launch_description():
    if "sim:=ulab" in sys.argv[4:]:
        sim = True
        unitree = False
        config_file = "trajectory_replay_sim.yaml"
    elif "sim:=go2" in sys.argv[4:] or "sim:=unitree" in sys.argv[4:]:
        sim = True
        unitree = True
        config_file = "trajectory_replay_sim.yaml"
    elif "real:=ulab" in sys.argv[4:]:
        sim = False
        unitree = False
        config_file = "trajectory_replay_real.yaml"
    elif "real:=go2" in sys.argv[4:] or "real:=unitree" in sys.argv[4:]:
        sim = False
        unitree = True
        config_file = "trajectory_replay_real.yaml"
    else:
        print("Please specify param 'sim' or 'real' and robot. E.g. 'sim:=ulab' or 'real:=go2'.")
        exit()

    pkg_controllers = get_package_share_directory("controllers")
    controller_config_path = os.path.join(pkg_controllers, "config", config_file)
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value=str(sim),
        description="Use simulation clock if true. Default is false.",
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default=str(sim)) #This variable is during launch replaced with the respective LaunchArgument declared by DeclareLaunchArgument

    return LaunchDescription([
        Node(
            package='controllers',  # Replace with the actual package name
            executable='trajectory_replay',  # Replace with the executable name
            name='trajectory_replay',
            parameters=[controller_config_path, {"use_sim_time": use_sim_time}]
        ),declare_use_sim_time_cmd
    ])
