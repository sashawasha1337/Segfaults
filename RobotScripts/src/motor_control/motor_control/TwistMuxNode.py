import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TwistMux(Node):
    def __init__(self):
        super().__init__('twist_mux')

        # Parameters for YAML or command line configuration
        self.declare_parameter('teleop_active_duration', 0.5)       # Duration teleop is considered active
        self.declare_parameter('handover_delay', 1.0)               # Delay before returning back to autonomous movement
        self.teleop_active_duration = self.get_parameter('teleop_active_duration').get_parameter_value().double_value
        self.handover_delay = self.get_parameter('handover_delay').get_parameter_value().double_value

        # Publishers and subscriptions
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_teleop', self.teleop_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.nav_callback, 10)

        self.last_teleop_time = 0.0
        self.last_teleop_msg = Twist() # Initialize with empty twist message
        self.last_nav_msg = Twist()
        self.source = "nav"

        # Evaluate twist messages on a timer
        # timer variable needed so the timer isn't garbage collected
        self.timer = self.create_timer(0.05, self.publish_cmd)  # Every 50 ms = 20 Hz

    def teleop_callback(self, msg):
        self.last_teleop_msg = msg
        self.last_teleop_time = time.time()

    def nav_callback(self, msg):
        self.last_nav_msg = msg

    def publish_cmd(self):
        now = time.time()
        teleop_age = now - self.last_teleop_time
        cmd = Twist()

        # Case where teleop message has been active for less than teleop_active_duration
        if teleop_age < self.teleop_active_duration:
            cmd = self.last_teleop_msg
            self.source = "teleop"
        
        # Case where last teleop message was received less than handover_delay + teleop_active_duration
        elif self.source == "teleop" and teleop_age < (self.teleop_active_duration + self.handover_delay):
            # Robot will stand still for this duration (empty twist message)
            self.get_logger().debug(f"Handover delay active for {teleop_age} - waiting before nav resumes.")

        # Default to autonomous navigation twist messages when no teleop messages received
        else:
            cmd = self.last_nav_msg
            self.source = "nav"
        
        self.cmd_pub.publish(cmd)
        self.get_logger().debug(f"Publishing {source} cmd_vel")

def main(args=None):
    rclpy.init(args=args)
    node = TwistMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down twist mux node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()