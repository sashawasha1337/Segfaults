import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO;
#imports for publishing detected objects
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D;

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.publisher_ = self.create_publisher(Detection2DArray, 'detections', 10)
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
        results = self.model(frame)[0]

        # Display bounding box on frame
        annotated_frame = results.plot()

        # Test video (delete later)
        cv2.imshow("YOLO Object Detection", annotated_frame)
        cv2.waitKey(1)
        
        det_array= Detection2DArray()
        det_array.header= msg.header
        #creates a detection2D object for each object detection which creates a publish
        for box in results.boxes:
            det= Detection2D()
            bbox= BoundingBox2D()
            bbox.center.x= (box.xyxy[0][0].item()+ box.xyxy[0][2].item())/2
            bbox.center.y= (box.xyxy[0][1].item()+ box.xyxy[0][3].item())/2
            bbox.size_x= box.xyxy[0][2].item()- box.xyxy[0][0].item()
            bbox.size_y= box.xyxy[0][3].item()- box.xyxy[0][1].item()
            det.bbox= bbox

            hypo= ObjectHypothesisWithPose()
            hypo.hypothesis.class_id= str(int(box.cls[0].item()))
            hypo.hypothesis.score= float(box.conf[0].item())
            det.results.append(hypo)

            det_array.detections.append(det)
        if det_array.detections:
            self.publisher_.publish(det_array)
            self.get_logger().info(f"Published {len(det_array.detections)} detections")

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