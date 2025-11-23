import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose') # ROS2 Action

        # Subscribe to the Path message from FirestoreGPSListener
        self.create_subscription(Path, '/gps_goal', self.path_callback, 10)

        # Retry settings
        self.max_retry_attempts = 3
        self.retry_delay = 2.0 # seconds
        self.retry_attempt = 0

        # Path state
        self.waypoints = []
        self.current_index = 0
        self.goal_msg = None

        self.get_logger().info("Navigation node started.")

    # Callback when a new path is published
    def path_callback(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("Received empty path. Ignoring.")
            return
        
        # Cancel any existing timers
        if hasattr(self, 'retry_timer'):
            self.retry_timer.cancel()
            del self.retry_timer
        
        self.waypoints = msg.poses
        self.current_index = 0
        self.get_logger().info(f"Received path with {len(self.waypoints)} waypoints")
        self.navigate_next_waypoint()

    # Navigate to the next waypoint
    def navigate_next_waypoint(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached.")
            self.goal_msg = None
            return
        
        pose = self.waypoints[self.current_index]
        self.goal_msg = NavigateToPose.Goal()
        self.goal_msg.pose = pose
        self.retry_attempt = 0
        
        self.get_logger().info(
            f"Sending waypoint {self.current_index + 1}/{len(self.waypoints)}: "
            f"x = {pose.pose.position.x:.2f}, y = {pose.pose.position.y:.2f}"
        )
        self.send_goal_with_retries(self.goal_msg)

    # Send goal with retry logic
    def send_goal_with_retries(self, msg):
        if self.retry_attempt >= self.max_retry_attempts:
            self.get_logger().error(f"Max retries reached for waypoint {self.current_index + 1}. Skipping to next waypoint.")
            self.current_index += 1
            self.navigate_next_waypoint()
            return
        
        self.retry_attempt += 1
        self.get_logger().info(f"Sending goal attempt #{self.retry_attempt}")
        
        # Wait until action server is available
        self._action_client.wait_for_server(timeout_sec=5.0)
        
        goal_response = self._action_client.send_goal_async(msg)
        goal_response.add_done_callback(self.goal_response_callback)

    # Check how nav2 responds to goal request
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn(f"Goal rejected, retrying in {self.retry_delay} seconds")

            # Cancel any existing timers
            if hasattr(self, 'retry_timer'):
                self.retry_timer.cancel()
                del self.retry_timer

            # one-shot timer
            def retry_callback():
                self.send_goal_with_retries(self.goal_msg)
                self.retry_timer.cancel()
                del self.retry_timer

            self.retry_timer = self.create_timer(self.retry_delay, retry_callback)
        else:
            self.get_logger().info('Goal accepted')
            self.retry_attempt = 0

            # Cancel retry timer if it exists
            if hasattr(self, 'retry_timer'):
                self.retry_timer.cancel()
                del self.retry_timer

            goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    # Log outcome after navigation goal is reached
    def get_result_callback(self, future):
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().info(f"Failed to get navigation result: {e}")
            result = None
        self.get_logger().info(f'Navigation result for waypoint {self.current_index + 1}: {result}')
        self.current_index += 1
        self.navigate_next_waypoint()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down navigation node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
