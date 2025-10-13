#!/usr/bin/env python3
import uuid
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
import os
import firebase_admin
from firebase_admin import credentials, firestore, storage


FIREBASE_STORAGE_BUCKET = os.environ.get("FIREBASE_STORAGE_BUCKET","segfaults-database.appspot.com")



SERVICE_ACCOUNT_FILE = "robot-service-account.json"   
ROBOT_ID = os.environ.get("ROBOT_ID") or os.uname().nodename
COLLECTION_PATH = "events"    
PUBLISH_DEBUG_TOPIC = True
ROBOT_UID = os.environ.get("ROBOT_UID") or ROBOT_ID

class YoloTrackNode(Node):
    def __init__(self):
        
        super().__init__('yolo_track_node')

        # Parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('weights', os.path.expanduser('/home/ws/Segfaults/RobotScripts/Model/best.pt'))
        self.declare_parameter('tracker', 'botsort.yaml')
        self.declare_parameter('conf', 0.3)

        self.image_topic = self.get_parameter('image_topic').value
        self.weights = self.get_parameter('weights').value
        self.tracker = self.get_parameter('tracker').value
        self.conf = self.get_parameter('conf').value

        # YOLO model + bridge
        self.bridge = CvBridge()
        self.model = YOLO(self.weights)
        self.known_ids = set()
        
        # Firebase init
        script_dir = os.path.dirname(os.path.abspath(__file__))
        cred_path = os.path.join(script_dir, SERVICE_ACCOUNT_FILE)
        if not firebase_admin._apps:
            cred = credentials.Certificate(cred_path)
            firebase_admin.initialize_app(cred,{"storageBucket": FIREBASE_STORAGE_BUCKET})
        self.db = firestore.client()
        # end of firebase inmit

        self.bucket = storage.bucket()

        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ROS publishers/subscribers
        self.sub = self.create_subscription(Image, self.image_topic, self.image_cb, qos)

        self.get_logger().info(f"YOLO tracking node running with {self.weights}, tracker={self.tracker}")
    
    
    def synced_callback(self, image_msg: Image, det_msg: String):
        # Convert ROS Image msg to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        ts_ms = int(ros_time_to_seconds(image_msg.header.stamp) * 1000)
        image_storage_path, image_url = self.upload_image_to_firebase(frame, ts_ms)

        # Here you can process the frame and detection data into a json which will be published as an event

        events = {
            "category": json.loads(det_msg.data).get("class", ""),
            "location": "",  # gps data later
            "robot_id": ROBOT_ID,
            "time": json.loads(det_msg.data).get("timestamp", ""),
            # we need to change this part we have no need for users/need to redesign
            "users": [ROBOT_UID],
            "image_url": image_url
            # "timestamp": firestore.SERVER_TIMESTAMP
        }
        self.get_logger().info(f"EVENT: {events}  ")
        self.publisher_.publish(String(data=json.dumps(events)))

        # write to firestore
        try:
            ref = self.db.collection(COLLECTION_PATH).add(events)
            self.get_logger().info(f"Firestore write successful: {ref}")
        except Exception as e:
            self.get_logger().error(f"Error writing to Firestore: {e}")

    def upload_image_to_firebase(self, frame, ts_ms):
        ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        filename = f"{ROBOT_ID}_{ts_ms}_{uuid.uuid4().hex}.jpg"
        path = f"events/{ROBOT_ID}/{filename}"
        blob = self.bucket.blob(path)
        token = uuid.uuid4().hex
        blob.metadata = {"firebaseStorageDownloadTokens": token}
        blob.upload_from_string(buf.tobytes(), content_type='image/jpeg')
        fixedname = urllib.parse.quote(blob.name, safe="")
        url = f"https://firebasestorage.googleapis.com/v0/b/{self.bucket.name}/o/{fixedname}?alt=media&token={token}"
        return path, url
    
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
                        "image_stamp": {
                            "sec": int(msg.header.stamp.sec),
                            "nanosec": int(msg.header.stamp.nanosec)
                        }
                    }
                    

                    msg_out = String()
                    msg_out.data = json.dumps(detection_data)
                    msg_out.header = msg.header
               

                    self.get_logger().info(f"New detection published: {class_name} (ID:{tid})")

        # Publish annotated frame to /tracking/image
        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        

        
def ros_time_to_seconds(ros_time):
    return ros_time.sec + ros_time.nanosec * 1e-9


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

