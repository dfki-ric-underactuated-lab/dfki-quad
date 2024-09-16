import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_drivers = get_package_share_directory("drivers")
    driver_config_path = os.path.join(pkg_drivers, "config", "test_params.yaml")

    # Test Abstract Driver Launch
    test_abstract_driver = Node(
        package="drivers",
        name="test_abstract_motor_driver",
        executable="test_abstract_motor_driver",
        parameters=[driver_config_path],
        output="screen",
    )
    return LaunchDescription([test_abstract_driver])
