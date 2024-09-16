#!/usr/bin/env python3

import asyncio
import time

import numpy as np
import rclpy
import std_srvs.srv
from interfaces.msg import JointCmd, JointState
from interfaces.srv import KeepJointPositions, ChangeLegDriverMode
from rclpy.node import Node


class StandUpNode(Node):

    def __init__(self):
        super().__init__("stand_up_node")

        self.declare_parameter("dt", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("Kp", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("Kd", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("total_time", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("target_joint_positions", rclpy.Parameter.Type.DOUBLE_ARRAY)

        self.dt = self.get_parameter("dt").get_parameter_value().double_value
        self.Kp = self.get_parameter("Kp").get_parameter_value().double_value
        self.Kd = self.get_parameter("Kd").get_parameter_value().double_value

        desired_joint_pose = np.array(
            self.get_parameter("target_joint_positions").get_parameter_value().double_array_value)
        total_time = self.get_parameter("total_time").get_parameter_value().double_value
        self.keep_pose_client = self.create_client(KeepJointPositions, "keep_joint_pose")
        self.switch_damping_client = self.create_client(std_srvs.srv.Trigger, "set_damping_mode")
        self.switch_leg_mode_client = self.create_client(ChangeLegDriverMode, "switch_op_mode")
        self.joint_pub = self.create_publisher(JointCmd, "leg_joint_cmd", 0)

        # waiting for leg driver services
        super().get_logger().info("waiting for leg driver services to become active")
        self.keep_pose_client.wait_for_service()
        super().get_logger().info("- keep pose found")
        self.switch_damping_client.wait_for_service()
        super().get_logger().info("- set damping found")
        self.switch_leg_mode_client.wait_for_service()
        super().get_logger().info("- switch leg driver mode found")

        # waiting for init position
        super().get_logger().info("waiting for current joint states")
        joint_msg_recv = asyncio.get_event_loop().create_future()

        def joint_msg_callback(msg):
            if not joint_msg_recv.done():
                joint_msg_recv.set_result(msg)

        self.joint_state_sub = self.create_subscription(JointState, "joint_states", joint_msg_callback, 0)
        rclpy.spin_until_future_complete(self, joint_msg_recv)
        self.start_joint_pos = joint_msg_recv.result().position

        cmd_msg = JointCmd()
        super().get_logger().info("switching leg driver mode")
        req = ChangeLegDriverMode.Request()
        req.target_mode = 0  # joint control mode
        future = self.switch_leg_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if (future.result().success):
            super().get_logger().info("taking control")
        else:
            super().get_logger().error("could not switch mode of leg driver")
            exit(-1)
        # interpolate and send:
        for t in np.arange(0, total_time, self.dt):
            current_pose = self.start_joint_pos + (desired_joint_pose - self.start_joint_pos) * (t / total_time)
            cmd_msg.position = list(current_pose)
            cmd_msg.velocity = list(np.zeros((12,)))
            cmd_msg.effort = list(np.zeros((12,)))
            cmd_msg.kp = list(np.ones((12,)) * self.Kp)
            cmd_msg.kd = list(np.ones((12,)) * self.Kd)
            self.joint_pub.publish(cmd_msg)
            time.sleep(self.dt)  # TODO: replace

        super().get_logger().info("calling leg driver to keep pose")
        req = KeepJointPositions.Request()
        req.joint_positions = list(desired_joint_pose)  # joint control mode
        future = self.keep_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if (future.result().success):
            super().get_logger().info("leg driver has control")
        else:
            super().get_logger().error("could not call leg driver")
            exit(-1)


if __name__ == "__main__":
    rclpy.init()
    node = StandUpNode()
    node.destroy_node()
    rclpy.shutdown()
