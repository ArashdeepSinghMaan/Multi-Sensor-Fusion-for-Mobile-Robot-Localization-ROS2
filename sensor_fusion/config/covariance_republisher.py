#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np

class CovarianceRepublisher(Node):
    def __init__(self):
        super().__init__('covariance_republisher')

        # Subscribers
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu_cov', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_cov', 10)

        # Define covariance values
        self.orientation_cov = np.diag([0.01, 0.01, 0.01]).flatten().tolist()
        self.angular_vel_cov = np.diag([0.001, 0.001, 0.001]).flatten().tolist()
        self.linear_acc_cov = np.diag([0.1, 0.1, 0.1]).flatten().tolist()

        self.pose_cov = np.diag([0.05, 0.05, 0.05, 0.1, 0.1, 0.1]).flatten().tolist()
        self.twist_cov = np.diag([0.1, 0.1, 0.1, 0.2, 0.2, 0.2]).flatten().tolist()

        self.get_logger().info('Covariance republisher started')

    def imu_callback(self, msg: Imu):
        msg_out = Imu()
        msg_out.header = msg.header
        msg_out.orientation = msg.orientation
        msg_out.angular_velocity = msg.angular_velocity
        msg_out.linear_acceleration = msg.linear_acceleration

        # Add covariance
        msg_out.orientation_covariance = self.orientation_cov
        msg_out.angular_velocity_covariance = self.angular_vel_cov
        msg_out.linear_acceleration_covariance = self.linear_acc_cov

        self.imu_pub.publish(msg_out)

    def odom_callback(self, msg: Odometry):
        msg_out = Odometry()
        msg_out.header = msg.header
        msg_out.child_frame_id = msg.child_frame_id
        msg_out.pose = msg.pose
        msg_out.twist = msg.twist

        # Add covariance
        msg_out.pose.covariance = self.pose_cov
        msg_out.twist.covariance = self.twist_cov

        self.odom_pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = CovarianceRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
