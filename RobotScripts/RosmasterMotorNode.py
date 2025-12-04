# Node to handle the control of bi-directional PWM DC motors using lgpio

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from Rosmaster_Lib import Rosmaster
import time


class RosmasterMotorNode(Node):

    def __init__(self):
        super().__init__('rosmaster_motor_node')
        self.get_logger().info("Beginning RosmasterMotorNode initialization.")

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10  # QoS depth
        )
        self.get_logger().info("Created cmd_vel subscription.")

        try:
            self.car = Rosmaster('/dev/ttyUSB0')  # Adjust port as necessary
            self.get_logger().info("Connected to Rosmaster motor controller")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Rosmaster: {e}")
            exit(1)

        self.car.set_car_type(1)  # Rosmaster X3 car type
        self.car.create_receive_threading()  # start receiving data from motor controller

        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.keep_alive)

    # Handle Motor Commands
    def cmd_callback(self, msg: Twist):
        self.get_logger().info(
            f"Received cmd_vel: linear_x={msg.linear.x}, linear_y={msg.linear.y}, angular_z={msg.angular.z}"
        )
        self.last_cmd_time = self.get_clock().now()
        self.car.set_car_motion(msg.linear.x, msg.linear.y, msg.angular.z)

    def keep_alive(self):
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds > 1e9:
            self.car.set_car_motion(0.0, 0.0, 0.0) #Keep connection alive

    def destroy_node(self):
        self.car.set_car_motion(0.0, 0.0, 0.0)  # Stop the car
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

if __name__ == "__main__":
    main()
