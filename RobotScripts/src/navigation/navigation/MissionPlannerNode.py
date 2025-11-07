# Mission Planner Node
# Holds the data structure containing the GPS waypoint for a mission.
# Includes methods for adding, removing, and cycling through waypoints.
# Next waypoint is cycled to when the robot reaches a certain radius of proximity from the target waypoint.
# Current goal waypoint is published to a ROS topic /gps_goal for other nodes to access.

# Subscribed ROS Topics:
# /gps/fix [sensor_msgs/NavSatFix]: Current GPS location of the robot.

# Published ROS Topics:
# /gps_goal [sensor_msgs/NavSatFix]: Current target GPS waypoint for the mission.

# Key Data Structures:
# self.waypoints:  Python List of GPS waypoints

# Key Properties/Methods:
# detect_proximity(): Check if the robot is within a certain radius of the target waypoint.
# cycle_waypoint(): Move to the next waypoint in the list.

import rclpy
from sensor_msgs.msg import NavSatFix



PROXIMITY_RADIUS_M = 5.0  # Meters to consider "arrived" at waypoint


class MissionPlannerNode(Node):

    def __init__(self):
        
        self.logger = self.get_logger()
        super().__init__('mission_planner_node')
        self.logger.info("Mission Planner Node initialization started.")

        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10,
        )

        self.gps_publisher = self.create_publisher(NavSatFix, '/gps_goal', 10)

        self.waypoints = []
        self.current_waypoint = 0
        self.current_position = None
        self.navigating = False

        def gps_callback(self, msg):
            self.current_position = (msg.latitude, msg.longitude)
            
            if self.navigating:
                if self.detect_proximity():
                    self.cycle_waypoint()

 

    def destroy_node(self):
        self.logger.info("Shutting down Mission Planner Node.")
        super().destroy_node()

    
    def detect_proximity(self):
        if not self.current_position:
            return False

        target_lat, target_lon = self.waypoints[self.current_waypoint]
        current_lat, current_lon = self.current_position
        distance = self.simple_distance(current_lat, current_lon, target_lat, target_lon)
        return distance <= PROXIMITY_RADIUS_M
    
    def simple_distance(self, lat1, lon1, lat2, lon2):
        lat_diff = (lat2 - lat1) * 111000  # Approx. meters per degree latitude
        lon_diff = (lon2 - lon1) * 111000  # Approx. meters per degree longitude
        return (lat_diff**2 + lon_diff**2) ** 0.5 # L2 Norm

def main():
    rclpy.init() # Initialize ROS 2
    node = MissionPlannerNode()
    try:
        rclpy.spin(node) # Keep the node running
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Mission Planner Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
