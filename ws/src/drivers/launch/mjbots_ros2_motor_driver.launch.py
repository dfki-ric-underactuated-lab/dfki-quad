import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dfki_quad_drivers = get_package_share_directory("drivers")
    driver_config_path = os.path.join(pkg_dfki_quad_drivers, "config", "mjbots_test_params.yaml")

    # Mjbots Motor Driver Launch
    mjbots_ros2_motor_driver = Node(
        package="drivers",
        name="mjbots_ros2_motor_driver",
        executable="mjbots_ros2_motor_driver",
        parameters=[driver_config_path],
        prefix=["sudo -E env \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\""],
        # remappings = [("/joint_states","/joint_states_hw")],
        shell=True,
        output="screen",
    )

    mjbots_ros2_motor_driver = Node(
        package="drivers",
        name="mjbots_ros2_motor_driver",
        executable="mjbots_ros2_motor_driver",
        parameters=[driver_config_path],
        shell=True,
        output="screen",
    )

    return LaunchDescription([mjbots_ros2_motor_driver])
