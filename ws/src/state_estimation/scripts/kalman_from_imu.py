#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import numpy as np

class IMUIEKalmanFilterNode(Node):
    def __init__(self):
        super().__init__('imu_ie_kalman_filter_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'Kalman', 10)
        self.subscription = self.create_subscription(
            Imu,
            'imu_measurement',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialization of State variable and covariance matrix
        self.x = np.zeros((9, 9))   # State matrix [R, V, P, PC1, PC2, PC3, PC4]
        self.P = np.eye(9) * 0.01
    
    def listener_callback(self, msg):
        # Process incoming IMU measurements and perform prediction and update steps
        imu_measurement = np.array([
            msg.orientation.x, msg.orientation.y, msg.orientation.z,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        ])

        dt = 0.01 # time difference
        self.predict(dt)
        self.update(imu_measurement.reshape((9, 1)))  # Reshape to match the dimensions of the state matrix

        # Publish the estimated state
        msg = Float32MultiArray()
        msg.data = self.x.flatten().tolist()
        self.publisher_.publish(msg)

    def predict(self, dt):
        # State prediction 
        M = np.eye(9)          # State transition matrix
        Q = np.eye(9) * 0.01   # Noise covariance

        self.x = np.dot(M, self.x)
        self.P = np.dot(M , np.dot(self.P,M.T)) + Q

        # Print prediction values
        print("Prediction:")
        print("State matrix (x):")
        print(self.x)
        print("Covariance matrix (P):")
        print(self.P)

    def update(self, z):
        # Measurement update using IEKF
        H = np.eye(9)
        R = np.eye(9) * 0.1

        y = z - self.x
        S = np.dot(H, np.dot(self.P, H.T)) + R
        K = np.dot(self.P , np.dot(H.T, np.linalg.inv(S)))
        self.x += np.dot(K, y)
        self.P = np.dot((np.eye(9) - np.dot(K, H)), self.P)

        # Print update values
        print("Update:")
        print("State matrix (x):")
        print(self.x)
        print("Covariance matrix (P):")
        print(self.P)

def main():
    rclpy.init()
    robot_iekf = IMUIEKalmanFilterNode()
    rclpy.spin(robot_iekf)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
