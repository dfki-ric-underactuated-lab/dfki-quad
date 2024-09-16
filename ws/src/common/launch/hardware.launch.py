import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    if "sim:=ulab" in sys.argv[4:]:
        print("Drivers only for real system!")
        exit()

    elif "sim:=go2" in sys.argv[4:] or "sim:=unitree" in sys.argv[4:]:
        print("Drivers only for real system!")
        exit()
    elif "real:=ulab" in sys.argv[4:]:
        sim = False
        unitree = False
        config_file_leg_driver = "leg_param_real_ulab.yaml"
        config_file_state_estimation = "state_estimation_ulab.yaml"
        if "onboard:=true" not in sys.argv[5:]:
            config_file_viz = "visualizer_params_ulab.yaml"

    elif "real:=go2" in sys.argv[4:] or "real:=unitree" in sys.argv[4:]:
        sim = False
        unitree = True
        config_file_leg_driver = "leg_param_real_go2.yaml"
        config_file_state_estimation = "state_estimation_go2_real.yaml"
        if "onboard:=true" not in sys.argv[5:]:
            config_file_viz = "visualizer_params_go2.yaml"
    else:
        print("Please specify param 'sim' or 'real' and robot. E.g. 'sim:=ulab' or 'real:=go2'.")
        exit()

    pkg_drivers = get_package_share_directory("drivers")
    driver_config_path = os.path.join(pkg_drivers, "config", config_file_leg_driver)
    pkg_state_estimation = get_package_share_directory("state_estimation")
    state_estimation_config_path = os.path.join(pkg_state_estimation, "config", config_file_state_estimation)

    if "onboard:=true" not in sys.argv[5:]:
        pkg_simulator = get_package_share_directory("simulator")
        visualizer_config_path = os.path.join(pkg_simulator, "config", config_file_viz)

    use_sim_time = False
    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time",
        default_value=str(sim),
        description="Use simulation clock if true. Default is false.",
    )

    # MOTOR DRIVERS
    mjbots_ros2_motor_driver = Node(
        package="drivers",
        name="mjbots_ros2_motor_driver",
        executable="mjbots_ros2_motor_driver",
        parameters=[os.path.join(pkg_drivers, "config", "mjbots_test_params.yaml")],
        prefix=["sudo -E env \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\""],
        shell=True,
        output="screen",
    )
    unitree_ros2_motor_driver = Node(
        package="drivers",
        name="unitree_ros2_motor_driver",
        executable="unitree_ros2_motor_driver",
        parameters=[os.path.join(pkg_drivers, "config", "unitree_go2_param.yaml")],
        shell=True,
        output="screen",
        arguments=['--ros-args', '--log-level', ["unitree_ros2_motor_driver:=", "debug"]]
    )
    # LEG DRIVER
    leg_driver = Node(
        package="drivers",
        name="leg_driver",
        executable="leg_driver",
        parameters=[driver_config_path, {"use_sim_time": use_sim_time}],
        shell=True,
        output="screen",
    )
    # STATE ESTIMATION
    state_estimation = Node(
        package="state_estimation",  # Replace with the actual package name
        executable="state_estimation",  # Replace with the executable name
        name="state_estimation_node",
        parameters=[state_estimation_config_path, {"use_sim_time": use_sim_time}],
        output='screen',
        arguments=['--ros-args', '--log-level', ["mit_controller_node:=", "info"]]
    )

    # VIZUALIZER
    if "onboard:=true" not in sys.argv[5:]:
        visualizer = Node(
            package="simulator",
            name="drake_visualizer",
            executable="visualizer",
            parameters=[visualizer_config_path, {"use_sim_time": use_sim_time}],
            arguments=['--ros-args', '--log-level', ["drake_visualizer:=", "info"]],
            shell=True,
            output="screen",
        )

    # GPS driver
    gps_driver = Node(
        package="nmea_navsat_driver",
        name="nmea_navsat_driver",
        executable="nmea_serial_driver",
        parameters=[{"use_sim_time": use_sim_time, "port": "/dev/ttyACM0"}],
        # arguments=['--ros-args', '--log-level', ["nmea_tcp_driver:=", "info"]],
        shell=True,
        output="screen",
    )

    hostname = '10.250.223.40'
    buffer_size = 10
    topic_namespace = 'vicon'
    # VICON driver
    vicon_driver = Node(
        package='vicon_receiver', executable='vicon_client', output='screen',
        parameters=[{'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace}]
    )

    if unitree:
        motor_driver = unitree_ros2_motor_driver
        if "onboard:=true" not in sys.argv[5:]:
            return LaunchDescription(
                [state_estimation, leg_driver, declare_use_sim_time, motor_driver, visualizer, gps_driver,
                 vicon_driver])
        else:
            return LaunchDescription(
                [state_estimation, leg_driver, declare_use_sim_time, motor_driver, gps_driver])
    else:
        if "onboard:=true" not in sys.argv[5:]:
            return LaunchDescription(
                [state_estimation, leg_driver, declare_use_sim_time, visualizer, gps_driver, vicon_driver])
        else:
            return LaunchDescription([state_estimation, leg_driver, declare_use_sim_time, gps_driver])
