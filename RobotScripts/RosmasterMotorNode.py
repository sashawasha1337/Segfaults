# Node to handle the control of bi-directional PWM DC motors using lgpio

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import time


class RosmasterMotorNode(Node):
    def __init__(self):
        super().__init__('rosmaster_motor_node')
        self.get_logger().info("Beginning RosmasterMotorNode initialization.")

    def destroy_node(self):
        super().destroy_node()
        self.get_logger().info("RosmasterMotorNode has been shut down.")

def main(args=None):
    rclpy.init(args=args)
    node = RosmasterMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()