# ROS2
import rclpy
from rclpy.node import Node

# QoS
from rclpy.qos import QoSProfile, ReliabilityPolicy

# msgs
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class Ekf3PathPublisher(Node):

    def __init__(self):
        super().__init__('ekf3_path_publisher')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub_ = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos
        )

        self.pub_ = self.create_publisher(Path, '/ekf3/path', 10)

        self.path_ = Path()
        self.path_.header.frame_id = 'map'

        self.get_logger().info('ekf3_path_publisher ready')

    def pose_callback(self, msg: PoseStamped):
        qw = msg.pose.orientation.w
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z

        # Normalize quaternion sign (EKF3 outputs w < 0)
        if qw < 0:
            qx, qy, qz, qw = -qx, -qy, -qz, -qw

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'map'
        pose.pose.position    = msg.pose.position
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.path_.header.stamp = msg.header.stamp
        self.path_.poses.append(pose)

        self.pub_.publish(self.path_)


def main(args=None):
    rclpy.init(args=args)
    node = Ekf3PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()