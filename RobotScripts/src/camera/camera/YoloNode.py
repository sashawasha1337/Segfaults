#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import json
import base64
from datetime import datetime


class YoloTrackNode(Node):
    def __init__(self):
        super().__init__('yolo_track_node')

        # Parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('out_image_topic', '/tracking/image')
        self.declare_parameter('weights', '~/home/robot/ros2_ws/Model/best.pt')
        self.declare_parameter('tracker', 'botsort.yaml')
        self.declare_parameter('conf', 0.3)

        self.image_topic = self.get_parameter('image_topic').value
        self.out_image_topic = self.get_parameter('out_image_topic').value
        self.weights = self.get_parameter('weights').value
        self.tracker = self.get_parameter('tracker').value
        self.conf = self.get_parameter('conf').value

        # YOLO model + bridge
        self.bridge = CvBridge()
        self.model = YOLO(self.weights)
        self.known_ids = set()

        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ROS publishers/subscribers
        self.sub = self.create_subscription(Image, self.image_topic, self.image_cb, qos)
        self.pub_img = self.create_publisher(Image, self.out_image_topic, 10)
        self.pub_tracks = self.create_publisher(String, 'detections', 10)

        self.get_logger().info(f"YOLO tracking node running with {self.weights}, tracker={self.tracker}")

    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Run YOLO tracker
        results = self.model.track(
            source=frame,
            conf=self.conf,
            persist=True,
            tracker=self.tracker,
            verbose=False
        )

        annotated = results[0].plot()
        boxes = results[0].boxes

        if boxes.id is not None:
            for box, tid, cls, conf in zip(boxes.xyxy, boxes.id, boxes.cls, boxes.conf):
                tid = int(tid.item())
                class_name = self.model.names[int(cls.item())]
                confidence = float(conf.item())

                # Only publish new detections
                if tid not in self.known_ids:
                    self.known_ids.add(tid)

                    # Encode annotated image to base64
                    _, buffer = cv2.imencode('.jpg', annotated)
                    img_base64 = base64.b64encode(buffer).decode('utf-8')

                    # Build JSON
                    detection_data = {
                        "timestamp": datetime.now().isoformat(),
                        "class": class_name,
                        "annotated_image": img_base64
                    }

                    msg_out = String()
                    msg_out.data = json.dumps(detection_data)
                    self.pub_tracks.publish(msg_out)

                    self.get_logger().info(f"New detection published: {class_name} (ID:{tid})")

        # Publish annotated frame to /tracking/image
        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        img_msg.header = msg.header
        self.pub_img.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

