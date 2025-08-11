import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import csv

class IMURecorder(Node):
    def __init__(self):
        super().__init__('imu_recorder')
        self.subscription = self.create_subscription(Imu, '/imu', self.listener_callback, 10)
        self.file = open('imu_data.csv', 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'time',
            'qx','qy','qz','qw',
            'ax','ay','az',
            'gx','gy','gz'
        ])
    
    def listener_callback(self, msg):
        self.writer.writerow([
            self.get_clock().now().nanoseconds * 1e-9,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

def main(args=None):
    rclpy.init(args=args)
    node = IMURecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.file.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
