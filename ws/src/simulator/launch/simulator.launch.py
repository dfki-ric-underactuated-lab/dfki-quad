import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    if "sim:=ulab" in sys.argv[4:]:
        unitree = False
        config_file = "simulator_params_ulab.yaml"
    elif "sim:=go2" in sys.argv[4:] or "sim:=unitree" in sys.argv[4:]:
        unitree = True
        config_file = "simulator_params_go2.yaml"
    else:
        print("Please specify param 'sim' with robot. E.g. 'sim:=ulab' or 'sim:=go2'.")
        exit()

    pkg_simulator = get_package_share_directory("simulator")
    sim_config_path = os.path.join(pkg_simulator, "config", config_file)

    # Mjbots Motor Driver Launch
    visualizer = Node(
        package="simulator",
        name="drake_simulator",
        executable="simulator",
        parameters=[sim_config_path],
        shell = True,
        output="screen",
    )
    return LaunchDescription([visualizer])

