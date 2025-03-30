import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DriveForward(Node):
    def __init__(self):
        super().__init__('drive_foward')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(1)

    def move_forward(self, speed = 0.5, duration = 2.0):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(msg)

        self.get_logger().info('moving forward..')

        time.sleep(duration)

        msg.linear.x = 0.0
        
        self.publisher.publish(msg)
        self.get_logger().info('stopping')

def main(args=None):
    rclpy.init(args=args)
    node = DriveForward()
    node.move_forward()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()