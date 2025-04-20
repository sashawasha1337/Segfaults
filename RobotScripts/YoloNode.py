import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # Initialize YOLO model
        self.model = YOLO("yolo11s.pt")

        # Subscribe to camera feed
        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("YOLO node initialized")

    def image_callback(self, msg):
        # Convert ROS Image msg to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Run model on frame
        results = self.model(frame)

        # Display bounding box on frame
        annotated_frame = results[0].plot()

        # Test video (delete later)
        cv2.imshow("YOLO Object Detection", annotated_frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()