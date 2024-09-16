import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_drivers = get_package_share_directory("drivers")
    driver_config_path = os.path.join(pkg_drivers, "config", "unitree_go2_param.yaml")


    # Unitree Motor Driver Launch
    unitree_ros2_motor_driver = Node(
        package="drivers",
        name="unitree_ros2_motor_driver",
        executable="unitree_ros2_motor_driver",
        parameters=[driver_config_path],
        shell = True,
        output="screen",
        arguments=['--ros-args', '--log-level',  ["unitree_ros2_motor_driver:=", "debug"]]
    )
    return LaunchDescription([unitree_ros2_motor_driver])