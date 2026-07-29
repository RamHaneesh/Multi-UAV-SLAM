"""
    Author: Rudraksha Bandodkar
    License: Apache License 2.0

    KeyboardControl: Node to control UAV using keyboard
"""

# ROS2
import rclpy

# computations
import math

# geometry message for pose with time-stamp
from geometry_msgs.msg import PoseStamped

# Base Node for inheritance
from iris_control.utils.base import Base

# to monitor the current state of control
from enum import Enum

# to listen to keyboard
import sys, termios, tty, select


# terminal settings
settings = termios.tcgetattr(sys.stdin)

# key listener function

def get_key():
    tty.setraw(sys.stdin.fileno())

    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    key = ''

    if rlist:
        key = sys.stdin.read(1)

        # ensure that we interrupt when we press `ctrl + c`
        if key == '\x03':
            raise KeyboardInterrupt

        # handling arrow key
        if key == '\x1b':
            key += sys.stdin.read(2)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# STATE class
class State(Enum):
    TAKEOFF = 1
    LAND = 2

# CONSTANTS
TAKEOFF_HEIGHT = 2.5

# Keyboard Control Node
class KeyboardControl(Base):
    def __init__(self):
        super().__init__("keyboard_control_node")

        # state
        self.state = State.LAND

        # Control parameters (speeds + publish rate)
        self.v_speed = 2.0
        self.h_speed = 2.0
        self.turn_speed = 4.0
        self.publish_rate = 45.0

        # change in motion
        self.d_motion = [0.0, 0.0, 0.0, 0.0]

        # intialize target pose
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        # timer to continuously publish setpose_local (indirectly controls drone motion) 
        self.timer = self.create_timer(1.0 / self.publish_rate, self.move)

    # function to process pressed key
    def process_key(self, key):

        # TAKEOFF 
        if key == '1' and self.state == State.LAND:

            self.get_logger().info("Takeoff triggered")

            if not self.wait_for_connection():
                self.get_logger().error("Not connected to MAVROS")
                return

            if not self.set_mode("GUIDED"):  
                self.get_logger().error("Failed to set mode")
                return

            if not self.arm():
                self.get_logger().error("Failed to arm")
                return

            self.takeoff(TAKEOFF_HEIGHT)

            self.state = State.TAKEOFF

        # LAND 
        elif key == '2' and self.state == State.TAKEOFF:

            self.get_logger().info("Landing triggered")
            self.land()
            self.state = State.LAND


        # controlling the UAV when state = TAKEOFF
        elif self.state == State.TAKEOFF:
            
            # altitude/thrust control
            if key in ['w', 'W']:
                self.d_motion[2] = self.v_speed / self.publish_rate

            elif key in ['s', 'S']:
                self.d_motion[2] = - self.v_speed / self.publish_rate
            
            # pitch control
            elif key == '\x1b[A':  # Forward
                self.d_motion[0] = math.cos(self.yaw)*(self.h_speed / self.publish_rate)
                self.d_motion[1] = math.sin(self.yaw)*(self.h_speed / self.publish_rate)

            elif key == '\x1b[B':  # Backward
                self.d_motion[0] = - math.cos(self.yaw)*(self.h_speed / self.publish_rate)
                self.d_motion[1] = - math.sin(self.yaw)*(self.h_speed / self.publish_rate)

            # roll control
            elif key == '\x1b[D':  # Left
                self.d_motion[0] = math.cos(self.yaw + math.pi/2) * (self.h_speed / self.publish_rate)
                self.d_motion[1] = math.sin(self.yaw + math.pi/2) * (self.h_speed / self.publish_rate)

            elif key == '\x1b[C':  # Right
                self.d_motion[0] = math.cos(self.yaw - math.pi/2) * (self.h_speed / self.publish_rate)
                self.d_motion[1] = math.sin(self.yaw - math.pi/2) * (self.h_speed / self.publish_rate)

            # yaw control
            elif key in ['a', 'A']:
                self.d_motion[3] = self.turn_speed / self.publish_rate

            elif key in ['d', 'D']:
                self.d_motion[3] = - self.turn_speed / self.publish_rate

    # function which publishes setpose_local (indirect way to control motion)
    def move(self):
        # only control in air (UAV takeoff complete)
        if self.state != State.TAKEOFF:
            return

        if self.d_motion == [0.0, 0.0, 0.0, 0.0]:
            # sychronize target with actual pose
            self.x = self.current_x
            self.y = self.current_y
            self.z = self.current_z
            self.yaw = self.current_yaw
            return
        
        # Update x,y,z,yaw only if there is some motion
        self.x += self.d_motion[0]
        self.y += self.d_motion[1]
        self.z += self.d_motion[2]
        self.yaw = self.current_yaw + self.d_motion[3]


        # Create message
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z

        # Yaw → quaternion
        qw = math.cos(self.yaw / 2.0)
        qz = math.sin(self.yaw / 2.0)

        msg.pose.orientation.w = qw
        msg.pose.orientation.z = qz

        # publish setpoint
        self.setpoint_pub.publish(msg)

# main function
def main(args=None):
    rclpy.init(args=args)

    node = KeyboardControl()

    try:
        while rclpy.ok():

            # spin node 
            rclpy.spin_once(node, timeout_sec=0.01)

            # read keyboard
            key = get_key()

            if key:
                node.process_key(key)
            else:
                # reset motion every loop (IMPORTANT)
                node.d_motion = [0.0, 0.0, 0.0, 0.0]

    except KeyboardInterrupt:
        pass

    finally:
        # resetting the terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        # delete node and shutdown rclpy client
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()