#!/usr/bin/env python3

# ROS2
import rclpy
from rclpy.node import Node

# msgs
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# math
import numpy as np


class SlamBridge(Node):
    """
    Subscribes to /orbslam3/pose (ORB-SLAM3 frame convention)
    Transforms to ArduPilot/MAVROS frame convention
    Publishes to /mavros/vision_pose/pose

    Frame convention difference:
        ORB-SLAM3 : X-right, Y-down,    Z-forward  (camera/OpenCV convention)
        ROS/MAVROS : X-forward, Y-left, Z-up       (ROS REP-103)

    Rotation to convert ORB-SLAM3 -> ROS:
        x_ros =  z_orb
        y_ros = -x_orb
        z_ros = -y_orb

    Note: map -> odom TF is broadcast by map_odom_tf_broadcaster (iris_transforms)
    """

    def __init__(self):
        super().__init__('slam_bridge')

        # Subscriber
        self.sub_pose_ = self.create_subscription(
            PoseStamped,
            '/orbslam3/pose',
            self.pose_callback,
            10
        )

        # Publisher
        self.pub_vision_pose_ = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )
        self.pub_path_ = self.create_publisher(Path, '/slam/path', 10)
        self.path_ = Path()
        self.path_.header.frame_id = 'map'


        self.get_logger().info('SlamBridge node ready...')
        self.get_logger().info('Subscribing to: /orbslam3/pose')
        self.get_logger().info('Publishing to:  /mavros/vision_pose/pose, /slam/path')

    def pose_callback(self, msg: PoseStamped):
        """
        Receives pose in ORB-SLAM3 frame, converts to ROS frame,
        publishes to /mavros/vision_pose/pose.
        map -> odom TF broadcasting is handled separately by map_odom_tf_broadcaster.
        """
        # Extract position and orientation from ORB-SLAM3 pose
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # --- SANITY CHECK ---
        # Reject poses where SLAM is not yet properly initialized.
        # Near-zero position + near-identity quaternion = uninitialized pose.
        pos_magnitude = (px**2 + py**2 + pz**2) ** 0.5
        if pos_magnitude < 1e-3 and abs(qw) > 0.99:
            self.get_logger().warning(
                'Rejecting uninitialized SLAM pose (near-zero)'
            )
            return
        # --- END SANITY CHECK ---

        # Convert position: ORB-SLAM3 (X-right, Y-down, Z-forward)
        #                -> ROS       (X-forward, Y-left, Z-up)
        ros_px =  pz
        ros_py = -px
        ros_pz = -py

        # Convert orientation quaternion using the same axis remapping
        # Apply rotation matrix R to quaternion:
        # R = [[0, 0, 1],[-1, 0, 0],[0, -1, 0]]
        ros_qx, ros_qy, ros_qz, ros_qw = self._convert_quaternion(
            qx, qy, qz, qw
        )

        # Apply +15° tilt correction around Y-axis (camera mounted with 15° downward pitch)
        tilt = 0.2618  # 15° in radians
        cos_t = np.cos(tilt)
        sin_t = np.sin(tilt)

        cx, cy, cz = ros_px, ros_py, ros_pz
        ros_px =  cos_t * cx + sin_t * cz
        ros_py =  cy
        ros_pz = -sin_t * cx + cos_t * cz

        tilt_qw = np.cos(tilt / 2.0)
        tilt_qy = np.sin(tilt / 2.0)

        ox, oy, oz, ow = ros_qx, ros_qy, ros_qz, ros_qw
        ros_qx =  tilt_qw*ox + tilt_qy*oz
        ros_qy =  tilt_qw*oy + tilt_qy*ow
        ros_qz =  tilt_qw*oz - tilt_qy*ox
        ros_qw =  tilt_qw*ow - tilt_qy*oy

        # Build converted PoseStamped
        converted = PoseStamped()
        converted.header.stamp    = msg.header.stamp
        converted.header.frame_id = 'map'

        converted.pose.position.x = ros_px
        converted.pose.position.y = ros_py
        converted.pose.position.z = ros_pz

        converted.pose.orientation.x = ros_qx
        converted.pose.orientation.y = ros_qy
        converted.pose.orientation.z = ros_qz
        converted.pose.orientation.w = ros_qw

        # Publish to MAVROS
        self.pub_vision_pose_.publish(converted)

        # Publish path
        self.path_.header.stamp = converted.header.stamp
        self.path_.poses.append(converted)
        self.pub_path_.publish(self.path_)
        

    def _convert_quaternion(self, qx, qy, qz, qw):
        """
        Converts quaternion from ORB-SLAM3 frame to ROS frame.

        The frame conversion rotation matrix is:
            R = [[0, 0, 1],
                 [-1, 0, 0],
                 [0, -1, 0]]

        New quaternion = R * q * R^T expressed as quaternion multiplication.
        We apply this by converting to rotation matrix, remapping, converting back.
        """
        # Quaternion to rotation matrix
        R = self._quat_to_rot(qx, qy, qz, qw)

        # Frame conversion matrix: ORB-SLAM3 -> ROS
        C = np.array([[ 0,  0,  1],
                      [-1,  0,  0],
                      [ 0, -1,  0]], dtype=float)

        # Apply: R_ros = C * R_orb * C^T
        R_ros = C @ R @ C.T

        # Rotation matrix back to quaternion
        return self._rot_to_quat(R_ros)

    def _quat_to_rot(self, qx, qy, qz, qw):
        """Quaternion to 3x3 rotation matrix."""
        R = np.array([
            [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
            [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ], dtype=float)
        return R

    def _rot_to_quat(self, R):
        """3x3 rotation matrix to quaternion (x, y, z, w)."""
        trace = R[0,0] + R[1,1] + R[2,2]

        if trace > 0:
            s  = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (R[2,1] - R[1,2]) * s
            qy = (R[0,2] - R[2,0]) * s
            qz = (R[1,0] - R[0,1]) * s
        elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            s  = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
            qw = (R[2,1] - R[1,2]) / s
            qx = 0.25 * s
            qy = (R[0,1] + R[1,0]) / s
            qz = (R[0,2] + R[2,0]) / s
        elif R[1,1] > R[2,2]:
            s  = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
            qw = (R[0,2] - R[2,0]) / s
            qx = (R[0,1] + R[1,0]) / s
            qy = 0.25 * s
            qz = (R[1,2] + R[2,1]) / s
        else:
            s  = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
            qw = (R[1,0] - R[0,1]) / s
            qx = (R[0,2] + R[2,0]) / s
            qy = (R[1,2] + R[2,1]) / s
            qz = 0.25 * s

        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = SlamBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()