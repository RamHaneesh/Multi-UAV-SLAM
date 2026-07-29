"""
    Author: Rudraksha Bandodkar
    License: Apache License 2.0

    Mission Planner: Node which will run a mission given waypoints (ENU).
"""

# ROS2
import rclpy

# Base Node for inheritance
from iris_control.utils.base import Base

# paths
import os
from ament_index_python.packages import get_package_share_directory

# json (to read mission file)
import json

# time (to hold at particular waypoint)
import time

# default mission filename
MISSION_FILENAME = 'mission'

class MissionPlanner(Base):
    def __init__(self):
        # inherit from base class 
        super().__init__(node_name = "Mission_Planner")

        # declaring parameters
        self.declare_parameter("m", value="")  # short mission filename (e.g. 'mission_1' → loads mission_1.json)
        self.declare_parameter('tolerance', value = 0.5)
        self.declare_parameter('home', value = [0.0,0.0,0.0])

        # get parameter values
        mission_filename = self.get_parameter("m").get_parameter_value().string_value
        self.tolerance = self.get_parameter("tolerance").get_parameter_value().double_value
        self.home_location = self.get_parameter("home").get_parameter_value().double_array_value

        # resolve mission file path: use provided filename or fall back to default
        m = mission_filename if mission_filename else MISSION_FILENAME
        mission_file = os.path.join(
            get_package_share_directory('iris_control'),
            'config',
            f'{m}.json'
        )

        # set parameter value (to use sim-time)
        # self.set_parameters([
        #     Parameter('use_sim_time', Parameter.Type.BOOL, True)
        # ])

        # get the mission parameters
        self.load_mission(mission_file)
    

    def execute_mission(self):
        """
            Flight plan:
                1. Take off to `takeoff height`.
                2. Traverse the given waypoints one by one (hold at each waypoint based on hold-time)
                3. If RTL requested, perform RTL
        """
        # initial logging
        self.get_logger().info('=== Starting the given mission ===')

        # step 1. connect to MAVROS
        if not self.wait_for_connection():
            return False
        
        # step 2. enter GUIDED mode
        if not self.set_mode('GUIDED'):
            return False
        
        # step 3. arm
        if not self.arm():
            return False
        
        # step 4. takeoff to takeoff_height
        if not self.takeoff(altitude = self.takeoff_height):
            return False
        
        # step 5. following the waypoints
        for i,(x,y,z) in enumerate(self.waypoints):
            # step 5.1 move to waypoint and wait until it is reached
            self.send_position(x,y,z)
            if not self.wait_for_position((x,y,z)):
                return False
            
            # step 5.2 hold position for hold_time
            if self.hold_time[i]!=0:
                self.get_logger().info(f'Holding position for {self.hold_time[i]} seconds')
                time.sleep(self.hold_time[i])

        # step 6. return to launch if requested
        if self.perform_rtl:
            # start RTL
            if not self.set_mode("RTL"):
                return False 

            # wait for UAV to reach home
            self.wait_for_position(self.home_location, timeout = 180)
        
        # step 7. land requested
        if self.perform_land:
            # start LAND
            if not self.set_mode("LAND"):
                return False
            self.get_logger().info('Landing...')

            # wait for UAV to land at the current position
            self.wait_for_position((x,y,0.0),timeout=180)

        # mission complete
        self.get_logger().info('=== Mission Complete ===')
        return True


    # load mission file function
    def load_mission(self,file_path):
        with open(file_path, 'r') as f:
            data = json.load(f)

        self.takeoff_height = data.get("takeoff_height")
        self.waypoints = data.get("waypoints", [])
        self.hold_time = data.get('hold_time', [])
        self.perform_rtl = True if data.get("rtl") == "true" else False
        self.perform_land = True if data.get("land") == "true" else False



def main(args = None):
    # init rclpy
    rclpy.init(args = args)

    # create mission planner node
    node = MissionPlanner()

    try:
        success = node.execute_mission()
        
        if success:
            node.get_logger().info("Mission completed successfully!")
        else:
            node.get_logger().info("Mission Failed!")

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')

    except Exception as e:
        node.get_logger().info(f'Error: {str(e)}')
    
    finally:
        # End of Communication
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()