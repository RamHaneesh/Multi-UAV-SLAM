# ROS2
import rclpy
from rclpy.node import Node

# computation
import math

# msgs
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

# euler -> quaternion converter
from tf_transformations import quaternion_from_euler

class CameraBaseTFBroadcaster(Node):

    def __init__(self):
        super().__init__('camera_base_tf_broadcaster')

        # parameters (for future multi-uav extension)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('left_camera_physical_frame', 'camera_left_link')
        self.declare_parameter('right_camera_physical_frame', 'camera_right_link')
        self.declare_parameter('left_camera_optical_frame', 'camera_left_optical_link')
        self.declare_parameter('right_camera_optical_frame', 'camera_right_optical_link')

        self.base_frame = self.get_parameter('base_frame').value
        self.left_physical  = self.get_parameter('left_camera_physical_frame').value
        self.right_physical = self.get_parameter('right_camera_physical_frame').value
        self.left_optical   = self.get_parameter('left_camera_optical_frame').value
        self.right_optical  = self.get_parameter('right_camera_optical_frame').value

        # static TF broadcaster
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # publish once on startup
        self.publish_camera_transforms()

        # logging
        self.get_logger().info('CameraTFBroadcaster node ready...')
        self.get_logger().info(f'Publishing static TF: {self.base_frame} --> {self.left_physical} --> {self.left_optical}')
        self.get_logger().info(f'Publishing static TF: {self.base_frame} --> {self.right_physical} --> {self.right_optical}')


    def publish_camera_transforms(self):
        transforms = []

        camera_configs = [
            # (physical_frame,      optical_frame,        x,     y,      z  )
            (self.left_physical,  self.left_optical,  0.12,  0.06, 0.02),
            (self.right_physical, self.right_optical, 0.12, -0.06, 0.02),
        ]

        for physical_frame, optical_frame, x, y, z in camera_configs:

            # TF1: base_link → camera_left/right_link
            # physical placement + 15 deg downward tilt only
            t1 = TransformStamped()
            t1.header.stamp = self.get_clock().now().to_msg()
            t1.header.frame_id = self.base_frame
            t1.child_frame_id = physical_frame
            t1.transform.translation.x = x
            t1.transform.translation.y = y
            t1.transform.translation.z = z
            q1 = quaternion_from_euler(0.0, 0.2618, 0.0)  # 15 deg tilt only
            t1.transform.rotation.x = q1[0]
            t1.transform.rotation.y = q1[1]
            t1.transform.rotation.z = q1[2]
            t1.transform.rotation.w = q1[3]
            transforms.append(t1)

            # TF2: camera_left/right_link → camera_left/right_optical_link
            # pure OpenCV frame correction, no translation
            t2 = TransformStamped()
            t2.header.stamp = self.get_clock().now().to_msg()
            t2.header.frame_id = physical_frame
            t2.child_frame_id = optical_frame
            t2.transform.translation.x = 0.0
            t2.transform.translation.y = 0.0
            t2.transform.translation.z = 0.0
            q2 = quaternion_from_euler(-math.pi/2, 0.0, -math.pi/2)  # OpenCV correction only
            t2.transform.rotation.x = q2[0]
            t2.transform.rotation.y = q2[1]
            t2.transform.rotation.z = q2[2]
            t2.transform.rotation.w = q2[3]
            transforms.append(t2)

        self.static_tf_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = CameraBaseTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()