import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    if "sim:=ulab" in sys.argv[4:]:
        unitree = False
    elif "sim:=go2" in sys.argv[4:] or "sim:=unitree" in sys.argv[4:]:
        unitree = True
    else:
        print("Please specify param 'sim' with robot. E.g. 'sim:=ulab' or 'sim:=go2'.")
        exit()

    # Generate nodes from package launch files
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("simulator"), "launch", "simulator.launch.py")),

    )

    leg_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("drivers"), "launch", "leg_driver_launch.py")),
        launch_arguments={"use_sim_time": "True"}.items(),
    )

    viz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("simulator"), "launch", "visualizer.launch.py")),
        launch_arguments={"use_sim_time": "False", }.items(),
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")  # This variable is during launch replaced with the respective LaunchArgument declared by DeclareLaunchArgument


    sim_disturber = Node(
        package="simulator",
        name="disturbance_node",
        executable="sim_disturber",
        parameters=[{"use_sim_time": use_sim_time}],
        shell = True,
        output="screen",
    )

    return LaunchDescription(
        [
            sim,
            leg_driver,
            viz,
            sim_disturber
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
