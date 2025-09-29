#image based obstacle avoidance node
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.subscription = self.create_subscription(
            Detection2DArray,
            'detections',
            self.detection_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info("Obstacle Avoidance node initialized")

        #default camera res, adjust to actual camera res
        self.image_width = 640

    def detection_callback(self, msg: Detection2DArray):
        twist = Twist()

        if msg.detections:
            det = msg.detections[0]
            bbox = det.bbox

            x_center = bbox.center.x
            size_y = bbox.size_y

            if size_y > 200:
                self.get_logger().warn("Obstacle detected! Stopping.")
                twist.linear.x = 0.0
                twist.angular.z = 0.3 # Turn right
            else:
                if x_center < self.image_width / 3:
                    self.get_logger().info("Obstacle on left, turning right.")
                    twist.linear.x = 0.1
                    twist.angular.z = -0.3
                elif x_center > 2 * self.image_width / 3:
                    self.get_logger().info("Obstacle on right, turning left.")
                    twist.linear.x = 0.1
                    twist.angular.z = 0.3
                else:
                    self.get_logger().warn("Obstacle ahead, stopping.")
                    twist.linear.x = 0.0
                    twist.angular.z = 0.3 # Turn right
        else:
            self.get_logger().info("No obstacles detected, moving forward.")
            twist.linear.x = 0.2
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

def main():
    rclpy.init()
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()