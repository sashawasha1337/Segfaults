import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import NavSatFix
from rclpy.action import ActionClient
from pyproj import Proj, Transformer
import math

UTMZONE = 'epsg:32610' # Hard coded utm zone for part of California

TRANSFORMER = Transformer.from_crs('epsg:4326', UTMZONE, always_xy=True)
LAT0, LON0 = 0, 0 # GPS origin coordinates
X0, Y0 = TRANSFORMER.transform(LON0, LAT0)

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose') # ROS2 Action
        self.create_subscription(NavSatFix, '/gps_goal', self.gps_goal_callback, 10)

    def gps_conversion(lat, lon):
        x, y = TRANSFORMER.transform(lon, lat)
        return x - X0, y - Y0

    def send_goal(self, x, y, theta):
        # Initialize goal_msg
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0 # 2d movement

        # Convert euler angle to quarternion
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        # Wait until action server is available
        self._action_client.wait_for_server()
        goal_response = self._action_client.send_goal_async(goal_msg)
        goal_response.add_done_callback(self.goal_response_callback)

    # Periodical action server status update
    def goal_response_callback(self, response):
        goal_result = response.result()
        if not goal_result.accepted:
            self.get_logger().info('Destination not reached')
            return
        self.get_logger().info('Destination reached')
        goal_result.get_result_async().add_done_callback(self.get_result_callback)

    # Log outcome after navigation goal is reached
    def get_result_callback(self, response):
        result = response.result().result
        self.get_logger().info(f'Navigation result: {result}')

    def gps_goal_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        self.get_logger().info(f"Received GPS goal: Latitude = {lat}, Longitude = {lon}")
        x, y = gps_conversion(msg.latitude, msg.longitude)
        self.get_logger().info(f"Sending cartesian goal with coordinates: x = {x}, y = {y}")
        send_goal(x, y, 0)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        node.send_goal(2.0, 2.0, 0) # set test destination
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down navigation node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
