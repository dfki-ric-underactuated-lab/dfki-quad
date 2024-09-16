import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    if "sim:=ulab" in sys.argv[4:]:
        print("No difference between sim and real here. Use 'real:=ulab' even in simulation.")
        exit()
    elif "sim:=go2" in sys.argv[4:]:
        print("No difference between sim and real here. Use 'real:=go2' even in simulation.")
        exit()
    elif "real:=ulab" in sys.argv[4:]:
        config_file = "sys_id_ulab.yaml"
    elif "real:=go2" in sys.argv[4:]:
        config_file = "sys_id_go2.yaml"
    else:
        print("Please specify param 'sim' or 'real' and robot. E.g. 'sim:=ulab' or 'real:=go2'.")
        exit()

    pkg_controllers = get_package_share_directory("controllers")
    config_path = os.path.join(pkg_controllers, "config", config_file)
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value=str(False),
        description="Use simulation clock if true. Default is false.",
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default=str(
        False))  # This variable is during launch replaced with the respective LaunchArgument declared by DeclareLaunchArgument

    return LaunchDescription([
        Node(
            package='controllers',
            executable='SysIDLegControllerNode',
            name='sysid_leg_controller_node',
            parameters=[config_path, {"use_sim_time": use_sim_time}],
            output='screen'
        )
    ])