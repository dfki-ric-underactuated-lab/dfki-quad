import os
import sys
import re

import numpy as np
import rclpy
import rclpy.time
from ament_index_python.packages import get_package_share_directory
from interfaces.msg import QuadState
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def safe_start():
    rclpy.init()
    node = rclpy.create_node("safe_start_launcher")
    node.get_logger().info("Safe start launcher is running")
    poses = []
    twists = []
    quad_state_subscription = node.create_subscription(QuadState, "/quad_state",
                                                       lambda state_msg: (
                                                           poses.append(
                                                               [state_msg.pose.pose.position.x,
                                                                state_msg.pose.pose.position.y,
                                                                state_msg.pose.pose.position.z,
                                                                state_msg.pose.pose.orientation.w,
                                                                state_msg.pose.pose.orientation.x,
                                                                state_msg.pose.pose.orientation.y,
                                                                state_msg.pose.pose.orientation.z]),
                                                           twists.append([state_msg.twist.twist.linear.x,
                                                                          state_msg.twist.twist.linear.y,
                                                                          state_msg.twist.twist.linear.z,
                                                                          state_msg.twist.twist.angular.x,
                                                                          state_msg.twist.twist.angular.y,
                                                                          state_msg.twist.twist.angular.z]
                                                                         )
                                                       ), 100)
    node.get_logger().info("Wait for quad_state message and collect it for 3 seconds")
    start_time = node.get_clock().now()
    end_time = start_time
    while ((end_time - start_time).nanoseconds <= rclpy.time.Time(seconds=3).nanoseconds):
        rclpy.spin_once(node, timeout_sec=1)
        if (len(poses) == 0):
            node.get_logger().warn("No quad state has been received yet", throttle_duration_sec=1)
            start_time = node.get_clock().now()
        end_time = node.get_clock().now()

    if (len(poses) <= 10 or len(twists) <= 10):
        node.get_logger().error('To less quad_states received: %d' % (len(poses)))
        exit(-1)

    poses = np.array(poses)
    twists = np.array(twists)
    poses_var = poses.var(0)
    twist_mean = twists.mean(0)
    if ((poses_var < 0.05).all()):
        node.get_logger().info("No pose drift: OK")
    else:
        node.get_logger().error("Pose drift in last 3 second quad state data to high: var=" + str(poses_var))
        exit(-1)

    if ((twist_mean < 0.05).all()):
        node.get_logger().info("No velocity: OK")
    else:
        node.get_logger().error("Velocity in last 3 second quad state data to high: mean=" + str(twist_mean < 0.1))
        exit(-1)

    node.get_logger().info("Safe standup successful, handing over to controller launch")


def generate_launch_description():
    if not "safe_start:=false" in sys.argv[4:]:
        safe_start()
    if "sim:=ulab" in sys.argv[4:]:
        sim = True
        unitree = False
        config_file = "mit_controller_sim_ulab.yaml"
    elif "sim:=go2" in sys.argv[4:] or "sim:=unitree" in sys.argv[4:]:
        sim = True
        unitree = True
        config_file = "mit_controller_sim_go2.yaml"
    elif "real:=ulab" in sys.argv[4:]:
        sim = False
        unitree = False
        config_file = "mit_controller_real_ulab.yaml"
    elif "real:=go2" in sys.argv[4:] or "real:=unitree" in sys.argv[4:]:
        sim = False
        unitree = True
        config_file = "mit_controller_real_go2.yaml"
    else:
        print("Please specify param 'sim' or 'real' and robot. E.g. 'sim:=ulab' or 'real:=go2'.")
        exit()

    if "mpc_solver:=PARTIAL_CONDENSING_HPIPM" in sys.argv[4:]:
        mpc_solver_param = ['-p', 'mpc_solver:=PARTIAL_CONDENSING_HPIPM']
    elif "mpc_solver:=FULL_CONDENSING_HPIPM" in sys.argv[4:]:
        mpc_solver_param = ['-p', 'mpc_solver:=FULL_CONDENSING_HPIPM']
    elif "mpc_solver:=PARTIAL_CONDENSING_OSQP" in sys.argv[4:]:
        mpc_solver_param = ['-p', 'mpc_solver:=PARTIAL_CONDENSING_OSQP']
    elif "mpc_solver:=FULL_CONDENSING_QPOASES" in sys.argv[4:]:
        mpc_solver_param = ['-p', 'mpc_solver:=FULL_CONDENSING_QPOASES']
    else:
        mpc_solver_param = []


    substring = "mpc_condensed_size:="
    condensed_param = [s for s in sys.argv[4:] if substring in s]

    if len(condensed_param) == 1:
        mpc_condensed_size_param = ['-p', condensed_param[0]]
        print(mpc_condensed_size_param)
    elif len(condensed_param) > 1:
        print("Error: Launch param \"mpc_condensed_size\" specified more than once.")
    else:
        mpc_condensed_size_param = []

    if "mpc_hpipm_mode:=SPEED_ABS" in sys.argv[4:]:
        mpc_hpipm_mode_param = ['-p', 'mpc_hpipm_mode:=SPEED_ABS']
    elif "mpc_hpipm_mode:=SPEED" in sys.argv[4:]:
        mpc_hpipm_mode_param = ['-p', 'mpc_hpipm_mode:=SPEED']
    elif "mpc_hpipm_mode:=BALANCE" in sys.argv[4:]:
        mpc_hpipm_mode_param = ['-p', 'mpc_hpipm_mode:=BALANCE']
    elif "mpc_hpipm_mode:=ROBUST" in sys.argv[4:]:
        mpc_hpipm_mode_param = ['-p', 'mpc_hpipm_mode:=ROBUST']
    else:
        mpc_hpipm_mode_param = []

    # if "mpc_condensed_size:=NONE" in sys.argv[4:]:
    #     mpc_condensed_size_param = ['-p', 'mpc_condensed_size:=20']
    # elif "mpc_condensed_size:=HALF" in sys.argv[4:]:
    #     mpc_condensed_size_param = ['-p', 'mpc_condensed_size:=10']
    # elif "mpc_condensed_size:=FULL" in sys.argv[4:]:
    #     mpc_condensed_size_param = ['-p', 'mpc_condensed_size:=1']


    pkg_controllers = get_package_share_directory("controllers")
    controller_config_path = os.path.join(pkg_controllers, "config", config_file)
    cpu_power_logging_config_path = os.path.join(pkg_controllers, "config", "cpu_power_logging.yaml")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value=str(sim),
        description="Use simulation clock if true. Default is false.",
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default=str(
        sim))  # This variable is during launch replaced with the respective LaunchArgument declared by DeclareLaunchArgument

    joy = Node(
        package="joy_linux",
        name="joy_linux_node",
        executable="joy_linux_node",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # joystick to goal traj node
    joy_to_target = Node(
        package="controllers",
        name="joy_to_target",
        executable="joy_to_target.py",
        parameters=[controller_config_path, {"use_sim_time": use_sim_time}],
        # parameters=[config["js2mpc"][mode]],
        output="screen",
    )

    return LaunchDescription([
        Node(
            package='controllers',  # Replace with the actual package name
            executable='mitcontrollernode',  # Replace with the executable name
            name='mit_controller_node',
            parameters=[controller_config_path, {"use_sim_time": use_sim_time}],
            output='screen',
            arguments=['--ros-args', '--log-level', ["mit_controller_node:=", "info"]] + mpc_solver_param + mpc_condensed_size_param + mpc_hpipm_mode_param
        ),
        Node(
            package='controllers',  # Replace with the actual package name
            executable='log_cpu_power',  # Replace with the executable name
            name='cpu_power_logging_node',
            parameters=[cpu_power_logging_config_path],
            output='screen'
        ),
        joy, joy_to_target, declare_use_sim_time_cmd])
