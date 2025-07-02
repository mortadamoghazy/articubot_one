#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import math

class EKFOdometryFusionNode(Node):
    def __init__(self):
        super().__init__('ekf_odom_fusion_node')

        self.sub_odom = self.create_subscription(
            Odometry,
            '/diff_cont/odom',
            self.odom_callback,
            10
        )

        self.pub_fused = self.create_publisher(
            Odometry,
            '/odom_fused',
            10
        )

        # State vector: [x, y, theta, vx, vy, omega]
        self.x = np.zeros((6,1))
        self.P = np.eye(6) * 0.1  # Initial covariance
        self.Q = np.eye(6) * 0.01  # Process noise covariance
        self.R = np.eye(3) * 0.05  # Measurement noise covariance

        self.last_time = None

    def odom_callback(self, msg):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            # Initialize state from first measurement
            self.x[0,0] = msg.pose.pose.position.x
            self.x[1,0] = msg.pose.pose.position.y
            self.x[2,0] = self.quaternion_to_yaw(msg.pose.pose.orientation)
            self.x[3,0] = msg.twist.twist.linear.x
            self.x[4,0] = msg.twist.twist.linear.y
            self.x[5,0] = msg.twist.twist.angular.z
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 1.0:
            # Ignore invalid dt
            self.last_time = now
            return
        self.last_time = now

        # Prediction step
        F = np.eye(6)
        F[0,3] = dt
        F[1,4] = dt
        F[2,5] = dt

        # Control input is zero (no control input)
        u = np.zeros((6,1))

        self.x = F @ self.x + u
        self.P = F @ self.P @ F.T + self.Q

        # Measurement vector z: position (x,y) and orientation theta
        z = np.array([
            [msg.pose.pose.position.x],
            [msg.pose.pose.position.y],
            [self.quaternion_to_yaw(msg.pose.pose.orientation)]
        ])

        # Measurement matrix H (maps state to measurement)
        H = np.zeros((3,6))
        H[0,0] = 1
        H[1,1] = 1
        H[2,2] = 1

        y = z - (H @ self.x)
        y[2] = self.normalize_angle(y[2])

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

        # Publish fused odometry
        fused_msg = Odometry()
        fused_msg.header.stamp = now.to_msg()
        fused_msg.header.frame_id = 'odom'
        fused_msg.child_frame_id = 'base_link'

        fused_msg.pose.pose.position.x = float(self.x[0])
        fused_msg.pose.pose.position.y = float(self.x[1])
        fused_msg.pose.pose.orientation = self.yaw_to_quaternion(float(self.x[2]))

        fused_msg.twist.twist.linear.x = float(self.x[3])
        fused_msg.twist.twist.linear.y = float(self.x[4])
        fused_msg.twist.twist.angular.z = float(self.x[5])

        self.pub_fused.publish(fused_msg)

    def quaternion_to_yaw(self, q):
        # Convert quaternion to yaw angle
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def normalize_angle(self, angle):
        # Normalize angle to [-pi, pi]
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = EKFOdometryFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
