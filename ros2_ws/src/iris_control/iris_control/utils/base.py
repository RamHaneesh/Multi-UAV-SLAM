"""
    Author: Rudraksha Bandodkar
    License: Apache License 2.0

    Base: node having the basic functionalities related to UAV control to be inherited by other nodes.
"""

# ROS2 
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# MAVROS 
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

# geometry_msgs
from geometry_msgs.msg import PoseStamped

# time
import time

# for computations
import math

# for quaternion to euler transformations
from tf_transformations import euler_from_quaternion

class Base(Node):

    # constructor
    def __init__(self,node_name:str):
        super().__init__(node_name)

        # state variables
        self.connected = False
        self.armed = False
        self.current_mode = ""

        # sensor check flag
        self.imu_ok = False
        self.baro_ok = False

        # tolerance for position to be reached/takeoff height
        self.tolerance = 0.5

        # current position (ENU frame)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        # QOS profile for MAVROS topics (BEST_EFFORT reliability)
        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE,
            history = HistoryPolicy.KEEP_LAST,
            depth = 10
        )

        # create subscriptions

        ### subscribe to UAV state
        self.state_sub = self.create_subscription(
            msg_type = State,
            topic = 'mavros/state',
            callback = self.state_callback,
            qos_profile = qos_profile
        )

        ### get current pose (local position)
        self.position_sub = self.create_subscription(
            msg_type = PoseStamped,
            topic = '/mavros/local_position/pose',
            callback = self.position_callback,
            qos_profile = qos_profile
        )

        ### publisher to publish position setpoints
        self.setpoint_pub = self.create_publisher(
            msg_type = PoseStamped,
            topic = '/mavros/setpoint_position/local',
            qos_profile = qos_profile
        )

        # create service clients
        self.arming_client = self.create_client(
            srv_type = CommandBool,
            srv_name = 'mavros/cmd/arming'
        )

        self.set_mode_client = self.create_client(
            srv_type = SetMode,
            srv_name = 'mavros/set_mode'
        )

        self.takeoff_client = self.create_client(
            srv_type = CommandTOL,
            srv_name = 'mavros/cmd/takeoff'
        )

        self.get_logger().info(f"{node_name} intiated...")

        self.wait_for_services()
    

    # wait for services function
    def wait_for_services(self):
        """
            Checks whether the ROS2 services exists, basically it blocks execution until MAVROS is launched and it advertises these services.
        """
        self.get_logger().info("Waiting for services...")
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.takeoff_client.wait_for_service()
        self.get_logger().info("All services ready")


    # wait for connection function
    def wait_for_connection(self, timeout = 30):
        """
            Wait for MAVROS connection
        """
        self.get_logger().info('Waiting to connect to MAVROS...')
        start_time = time.time()
        while not self.connected:
            if time.time() - start_time > timeout:
                self.get_logger().error('Connection timeout...')
                return False
            rclpy.spin_once(self, timeout_sec = 0.1) # keep timeout between 0.05 to 0.2 seconds
        self.get_logger().info('Connected to MAVROS')
        return True


    # callbacks

    ### state callback
    def state_callback(self,msg):
        """
            Updates the UAV's current state
        """
        self.connected = msg.connected
        self.armed = msg.armed
        self.current_mode = msg.mode


    ### position callback
    def position_callback(self,msg):
        """
            Updates UAV's position from local_position topic.
            Position is in ENU frame (East-North-Up)
        """
        # position
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z

        # orientation
        q = msg.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]

        # convert to euler to get the yaw
        _,_,yaw = euler_from_quaternion(quaternion)
        self.current_yaw = yaw




    # client functions

    ### set mode function
    def set_mode(self,mode, timeout = 10):
        """
            Change flight mode 
        """
        self.get_logger().info(f'Setting mode:{mode}')
        request = SetMode.Request()
        request.custom_mode = mode

        future = self.set_mode_client.call_async(request = request)
        rclpy.spin_until_future_complete(node = self,future = future)

        # if there is no response to the request from set mode service, return False
        if not future.result().mode_sent:
            return False
        
        # wait for the mode to be set to custom mode
        start_time = time.time()
        while self.current_mode != mode:
            if time.time() - start_time > timeout:
                return False
            rclpy.spin_once(self, timeout_sec = 0.1) # keep timeout between 0.05 to 0.2 seconds
        
        self.get_logger().info(f'Mode set to: {mode}.')
        return True


    ### arm function
    def arm(self, timeout = 10):
        """
        arm the UAV
        """
        self.get_logger().info('Arming...')
        request = CommandBool.Request()
        request.value = True 

        future = self.arming_client.call_async(request = request)
        rclpy.spin_until_future_complete(node = self, future = future)

        # if there is no response to the request from arming service, return False
        if not future.result().success:
            return False
        
        # wait for the UAV to be armed
        start_time = time.time()
        
        while not self.armed:
            if time.time() - start_time > timeout:
                return False
            rclpy.spin_once(node = self, timeout_sec = 0.1) # keep timeout between 0.05 to 0.2 seconds
        
        self.get_logger().info('UAV armed')
        return True
    

    ### takeoff command function
    def takeoff(self,altitude, timeout = 30):
        """
        Takeoff to specified altitude
        """
        self.get_logger().info(f'Takeoff to {altitude}m')
        request = CommandTOL.Request()
        request.altitude = altitude

        future = self.takeoff_client.call_async(request)
        rclpy.spin_until_future_complete(node = self, future = future)

        # if there is no response to the request from takeoff service, return False
        if not future.result().success:
            self.get_logger().info('Takeoff command not accepted.')
            return False
        
        # wait for altitude stabilization
        start_time = time.time()
        while True:
            # calculate distance to target
            dist2target = self.get_distance_from_target((self.current_x, self.current_y, altitude))

            # check if reached (within tolerance)
            if dist2target < self.tolerance:
                self.get_logger().info(f'Takeoff successful to ({altitude} m)')
                return True
            
            # check timeout
            if time.time() - start_time > timeout:
                self.get_logger().error(
                    f'Takeoff timeout.'
                )
                return False
            
            # update position feedback
            rclpy.spin_once(node = self, timeout_sec = 0.1) # keep timeout between 0.05 to 0.2 seconds

    ### land command
    def land(self, timeout = 30):
        '''
        land at the current location
        '''
        if not self.set_mode("LAND"):   # ArduPilot
            self.get_logger().error("Failed to switch to LAND mode")
            return
        
        self.get_logger().info(f'Landing...')

        # wait for UAV to land
        start_time = time.time()
        while True:
            # calculate distance to target
            dist2target = self.get_distance_from_target((self.current_x, self.current_y, 0.0))

            # check if reached (within tolerance)
            if dist2target < self.tolerance:
                self.get_logger().info(f'Successfully landed ...')
                return True
            
            # check timeout
            if time.time() - start_time > timeout:
                self.get_logger().error(
                    f'Takeoff timeout.'
                )
                return False
            
            # update position feedback
            rclpy.spin_once(node = self, timeout_sec = 0.1) # keep timeout between 0.05 to 0.2 seconds

    
    # publishing positions 
    def send_position(self,x,y,z):
        """
            Publishes PoseStamped() message containing waypoint position on the 
            `/mavros/setpoint_position/local` topic.
        
            Args: 
                x: Target X position (East) in meters
                y: Target Y position (North) in meters
                z: Target Z position (Up) in meters

            Note: Setpoints are latched - the drone will continue to target this position even if we stop publishing.
        """       

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # set target position (ENU frame)
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        # orientation (0,0,0,1 = no-rotation)
        msg.pose.orientation.w = 1.0

        self.setpoint_pub.publish(msg = msg )

    
    # get distance from target function
    def get_distance_from_target(self,target):
        """
            Calculates the Euclidean distance from UAV's current position to its target

            Args: 
                target: tuple of target's coordinate (x,y,z) in ENU frame
            Returns:
                (float): Distance in meters 
        """        
        dx = self.current_x - target[0]
        dy = self.current_y - target[1]
        dz = self.current_z - target[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    
    # wait for position to be reached function
    def wait_for_position(self, target, timeout = 30):
        """
        Wait until drone reaches target position.
        
        Args: 
            target: tuple of target's coordinate (x,y,z) in ENU frame.
            timeout: Maximum wait time in seconds.
        
        Returns:
            (bool): True if position is reached, False if timeout.
        """

        self.get_logger().info(f'Moving to ({target[0]:.2f},{target[1]:.2f},{target[2]:.2f})')

        start_time = time.time()
        while True:
            # calculate distance to target
            dist2target = self.get_distance_from_target(target)

            # check if reached (within tolerance)
            if dist2target < self.tolerance:
                self.get_logger().info(f'Position reached: ({self.current_x:.2f},{self.current_y:.2f},{self.current_z:.2f})')
                return True
            
            # check timeout
            if time.time() - start_time > timeout:
                self.get_logger().error(
                    f'Position timeout. Distance from target: {dist2target:.2f}m.'
                )
                return False
            
            # update position feedback
            rclpy.spin_once(node = self, timeout_sec = 0.1) # keep timeout between 0.05 to 0.2 seconds