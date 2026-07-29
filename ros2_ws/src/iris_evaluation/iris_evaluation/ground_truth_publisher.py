# ROS2
import rclpy
from rclpy.node import Node

# msgs
from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import Path
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V


class GroundTruthPublisher(Node):

    def __init__(self):
        super().__init__('ground_truth_publisher')

        self.pub_pose_ = self.create_publisher(PoseStamped, '/ground_truth/pose', 10)
        # self.pub_path_ = self.create_publisher(Path, '/ground_truth/path', 10)

        # self.path_ = Path()
        # self.path_.header.frame_id = 'map'

        self.gz_node_ = GzNode()
        self.gz_node_.subscribe(
            Pose_V,
            '/world/default/pose/info',
            self.gz_callback
        )

        self.get_logger().info('ground_truth_publisher ready')
        self.get_logger().info('Publishing to: /ground_truth/pose')
        # self.get_logger().info('Publishing to: /ground_truth/pose, /ground_truth/path')

    def gz_callback(self, pose_v):
        for pose in pose_v.pose:
            if pose.name != 'iris':
                continue

            stamp = self.get_clock().now().to_msg()

            msg = PoseStamped()
            msg.header.stamp    = stamp
            msg.header.frame_id = 'map'
            msg.pose.position.x = pose.position.x
            msg.pose.position.y = pose.position.y
            msg.pose.position.z = pose.position.z
            msg.pose.orientation.x = pose.orientation.x
            msg.pose.orientation.y = pose.orientation.y
            msg.pose.orientation.z = pose.orientation.z
            msg.pose.orientation.w = pose.orientation.w

            self.pub_pose_.publish(msg)

            # self.path_.header.stamp = stamp
            # self.path_.poses.append(msg)
            # self.pub_path_.publish(self.path_)
            break


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()