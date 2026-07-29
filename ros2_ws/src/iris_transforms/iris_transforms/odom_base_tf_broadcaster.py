# ROS2 
import rclpy
from rclpy.node import Node

# msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# qos
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# OdomBaseTFBroadcaster 
class OdomBaseTFBroadcaster(Node):

    def __init__(self):
        super().__init__('odom_base_tf_broadcaster')

        # creating parameters (considering future extension to multi-uav)
        self.declare_parameter('odom_topic', 'mavros/local_position/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # QOS profile for MAVROS topics (BEST_EFFORT reliability)
        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE,
            history = HistoryPolicy.KEEP_LAST,
            depth = 10
        )

        # subscriptions
        self.odom_sub = self.create_subscription(
            msg_type = Odometry,
            topic = self.odom_topic,
            callback = self.odom_callback,
            qos_profile = qos_profile
        )

        self.get_logger().info('OdomTFBroadcaster node ready...')
        self.get_logger().info(f'Subscribed to: {self.odom_topic}')
        self.get_logger().info(f'Publishing TF: {self.odom_frame} --> {self.base_frame}')

    
    # odom callback
    def odom_callback(self, msg: Odometry):
        t = TransformStamped()

        # time 
        t.header.stamp = msg.header.stamp

        # frames
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        # Translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Rotation — normalize quaternion hemisphere (EKF3 uses w<0 convention)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        if qw < 0:
            qx, qy, qz, qw = -qx, -qy, -qz, -qw
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        # Publish TF
        self.tf_broadcaster.sendTransform(t)

# main function

def main(args = None):
    # initialize the rclpy client
    rclpy.init(args = args)

    # initialize the node
    node = OdomBaseTFBroadcaster()

    # spin the node
    rclpy.spin(node)

    # destroy the node, if interrupted
    node.destroy_node()

    # shutdown client
    rclpy.shutdown()

if __name__ == "__main__":
    main()
