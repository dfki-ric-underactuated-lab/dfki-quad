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
        config_file = "visualizer_params_ulab.yaml"
    elif "sim:=go2" in sys.argv[4:] or "sim:=unitree" in sys.argv[4:]:
        sim = True
        unitree = True
        config_file = "visualizer_params_go2.yaml"
    elif "real:=ulab" in sys.argv[4:]:
        sim = False
        unitree = False
        config_file = "visualizer_params_ulab.yaml"
    elif "real:=go2" in sys.argv[4:] or "real:=unitree" in sys.argv[4:]:
        sim = False
        unitree = True
        config_file = "visualizer_params_go2.yaml"
    else:
        print("Please specify param 'sim' or 'real' and robot. E.g. 'sim:=ulab' or 'real:=go2'.")
        exit()

    remappings = []
    if "onboard:=true" in sys.argv[5:]:
        remappings = [
            ('/quad_state', '/quad_state/lf'),
        ]

    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="False",
        description="Use simulation clock if true. Default is false.",
    )
    use_sim_time = LaunchConfiguration("use_sim_time",
                                       default="False")  # This variable is during launch replaced with the respective LaunchArgument declared by DeclareLaunchArgument

    pkg_simulator = get_package_share_directory("simulator")
    visualizer_config_path = os.path.join(pkg_simulator, "config", config_file)

    visualizer = Node(
        package="simulator",
        name="drake_visualizer",
        executable="visualizer",
        parameters=[visualizer_config_path, {"use_sim_time": use_sim_time}],
        # prefix= ["sudo -E env \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\""],
        # remappings = [("/joint_states","/joint_states_hw")],
        shell=True,
        output="screen",
        remappings=remappings
    )
    return LaunchDescription([visualizer, declare_use_sim_time])
