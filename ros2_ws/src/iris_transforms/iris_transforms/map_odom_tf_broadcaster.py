# ROS2
import rclpy
from rclpy.node import Node

# msgs
from geometry_msgs.msg import TransformStamped

# TF
from tf2_ros import TransformBroadcaster


class MapOdomTFBroadcaster(Node):
    """
    Broadcasts map -> odom TF as identity.

    Since /mavros/local_position/odom is published with frame_id 'map'
    (MAVROS tracks in map frame directly via vision pose input),
    map and odom are the same frame. Identity TF is correct.
    """

    def __init__(self):
        super().__init__('map_odom_tf_broadcaster')

        self.tf_broadcaster_ = TransformBroadcaster(self)

        # Publish at 50 Hz
        self.timer_ = self.create_timer(0.02, self._broadcast)

        self.get_logger().info('MapOdomTFBroadcaster node ready...')
        self.get_logger().info('Broadcasting TF: map -> odom (identity)')

    def _broadcast(self):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id  = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster_.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()