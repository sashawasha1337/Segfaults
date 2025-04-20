# work in progress, just testing the lidar 

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import rclpy

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('avoid_obstacles')
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        valid = [r for r in msg.ranges if r > 0.0]
        if valid:
            closest = min(valid)
            self.get_logger().info(f'Closest object: {closest:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()