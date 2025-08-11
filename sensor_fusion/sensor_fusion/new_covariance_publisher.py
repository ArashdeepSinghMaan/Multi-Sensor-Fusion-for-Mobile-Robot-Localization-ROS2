#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np

class CovarianceRepublisher(Node):
    def __init__(self):
        super().__init__('covariance_republisher')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom_cov', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu_cov', 10)
        
        # Subscribers with larger queue sizes for reliability
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 20)
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 20)
        
        # Define covariance matrices
        self.setup_covariance_matrices()
        
        # Add timing diagnostics
        self.last_odom_time = self.get_clock().now()
        self.last_imu_time = self.get_clock().now()
        
    def setup_covariance_matrices(self):
        # Odometry covariance (6x6 for pose, 6x6 for twist)
        self.odom_pose_covariance = np.zeros((6, 6))
        self.odom_pose_covariance[0, 0] = 0.1    # x variance
        self.odom_pose_covariance[1, 1] = 0.1    # y variance  
        self.odom_pose_covariance[5, 5] = 0.2    # yaw variance
        # Set large values for unused dimensions
        for i in [2, 3, 4]:
            self.odom_pose_covariance[i, i] = 1e6
            
        self.odom_twist_covariance = np.zeros((6, 6))
        self.odom_twist_covariance[0, 0] = 0.05  # vx variance
        self.odom_twist_covariance[1, 1] = 0.05  # vy variance
        self.odom_twist_covariance[5, 5] = 0.1   # vyaw variance
        # Set large values for unused dimensions  
        for i in [2, 3, 4]:
            self.odom_twist_covariance[i, i] = 1e6
            
        # IMU covariance
        self.imu_orientation_covariance = np.zeros((3, 3))
        self.imu_orientation_covariance[0, 0] = 0.01  # roll
        self.imu_orientation_covariance[1, 1] = 0.01  # pitch
        self.imu_orientation_covariance[2, 2] = 0.1   # yaw (less certain)
        
        self.imu_angular_velocity_covariance = np.zeros((3, 3))
        self.imu_angular_velocity_covariance[0, 0] = 0.01  # wx
        self.imu_angular_velocity_covariance[1, 1] = 0.01  # wy
        self.imu_angular_velocity_covariance[2, 2] = 0.02  # wz (yaw rate)
        
        self.imu_linear_acceleration_covariance = np.zeros((3, 3))
        for i in range(3):
            self.imu_linear_acceleration_covariance[i, i] = 0.1

    def odom_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        
        if dt > 1.0:  # More than 1 second since last message
            self.get_logger().warn(f'Large gap in odometry messages: {dt:.2f}s')
        
        # Create new message with covariance
        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.child_frame_id = msg.child_frame_id
        odom_msg.pose.pose = msg.pose.pose
        odom_msg.twist.twist = msg.twist.twist
        
        # Add covariance
        odom_msg.pose.covariance = self.odom_pose_covariance.flatten().tolist()
        odom_msg.twist.covariance = self.odom_twist_covariance.flatten().tolist()
        
        self.odom_pub.publish(odom_msg)
        self.last_odom_time = current_time

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_imu_time).nanoseconds / 1e9
        
        if dt > 1.0:  # More than 1 second since last message
            self.get_logger().warn(f'Large gap in IMU messages: {dt:.2f}s')
        
        # Create new message with covariance
        imu_msg = Imu()
        imu_msg.header = msg.header
        imu_msg.orientation = msg.orientation
        imu_msg.angular_velocity = msg.angular_velocity
        imu_msg.linear_acceleration = msg.linear_acceleration
        
        # Add covariance
        imu_msg.orientation_covariance = self.imu_orientation_covariance.flatten().tolist()
        imu_msg.angular_velocity_covariance = self.imu_angular_velocity_covariance.flatten().tolist()
        imu_msg.linear_acceleration_covariance = self.imu_linear_acceleration_covariance.flatten().tolist()
        
        self.imu_pub.publish(imu_msg)
        self.last_imu_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = CovarianceRepublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
