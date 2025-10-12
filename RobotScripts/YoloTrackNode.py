import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D

class YoloTrackNode(Node):
    def __init__(self):
        super().__init__('yolo_track_node')
        self.get_logger().info("Starting YOLO11 Tracking Node initialization.")

        # Parameters for setup
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('out_image_topic', '/tracking/image') # Where detections are published
        self.declare_parameter('weights', './Model/best.pt')
        self.declare_parameter('tracker', 'botsort.yaml') # Ultralytics tracking
        self.declare_parameter('conf', 0.3)

        # Read parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.out_image_topic = self.get_parameter('out_image_topic').value
        self.weights = self.get_parameter('weights').value
        self.tracker = self.get_parameter('tracker').value
        self.conf = self.get_parameter('conf').value

        # Seting up Model
        self.bridge = CvBridge()
        self.model = YOLO(self.weights)
        self.known_ids = set()  # store IDs of objects already seen

        # ROS interfaces
        self.sub = self.create_subscription(Image, self.image_topic, self.image_cb, 10) # Camera feed subscription
        self.get_logger().info(f"Subscribed to image topic: {self.image_topic}")
        self.pub_img = self.create_publisher(Image, self.out_image_topic, 10) # Annotated images published here
        self.get_logger().info(f"Publishing annotated images to: {self.out_image_topic}")
        self.pub_tracks = self.create_publisher(Detection2DArray, 'detections', 10) # Publishes new detection to here
        self.get_logger().info(f"YOLO11 tracking node initialized with {self.weights}, tracker={self.tracker}")

    def image_cb(self, msg: Image):
        # Convert ROS2 Image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO tracking
        results = self.model.track(
            source=frame,
            conf=self.conf,
            persist=True,
            tracker=self.tracker,
            verbose=False
        )

        # Annotated frame
        annotated = results[0].plot()

        # Prepare Detection2DArray
        det_array = Detection2DArray()
        det_array.header = msg.header

        boxes = results[0].boxes
        if boxes.id is not None:
            for box, tid, cls, conf in zip(boxes.xyxy, boxes.id, boxes.cls, boxes.conf):
                tid = int(tid.item())

                # Only publish new detections
                if tid not in self.known_ids:
                    self.known_ids.add(tid)

                    # Bounding box
                    x1, y1, x2, y2 = [float(v) for v in box.tolist()]
                    bbox = BoundingBox2D()
                    bbox.center.x = (x1 + x2) / 2
                    bbox.center.y = (y1 + y2) / 2
                    bbox.size_x = x2 - x1
                    bbox.size_y = y2 - y1

                    # Detection object
                    det = Detection2D()
                    det.bbox = bbox

                    # Class hypothesis
                    hypo = ObjectHypothesisWithPose()
                    hypo.hypothesis.class_id = str(int(cls.item()))
                    hypo.hypothesis.score = float(conf.item())
                    det.results.append(hypo)

                    det_array.detections.append(det)

        # Publish annotated image
        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        img_msg.header = msg.header
        self.pub_img.publish(img_msg)

        # Publish new detections if any
        if det_array.detections:
            self.pub_tracks.publish(det_array)

    def destroy_node(self):
        self.get_logger().info("Shutting down YOLO11 Tracking Node.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloTrackNode()
    try:
        node.get_logger().info("YOLO11 Tracking Node starting.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()