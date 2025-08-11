import rclpy
from rclpy.node import Node
from ros_ign_interfaces.msg import PoseV
from geometry_msgs.msg import PoseStamped

class GroundTruth(Node):
    def __init__(self):
        super().__init__('ground_truth_node')
        self.subscription = self.create_subscription(PoseV, '/world/empty/dynamic_pose/info', self.callback, 10)
        self.pub = self.create_publisher(PoseStamped, '/ground_truth/pose', 10)

    def callback(self, msg: PoseV):
        for pose in msg.poses:
            if pose.name == 'husky':
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'map'
                pose_msg.pose = pose.pose
                self.pub.publish(pose_msg)
                break

def main():
    rclpy.init()
    node = GroundTruth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
