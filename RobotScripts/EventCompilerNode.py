##each robot will need json credentials,

##the json file should be in the same directory as firebase publishing script

import os
import cv2
import uuid 
import urllib.parse
from platform import node
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
import json
from std_msgs.msg import String
import firebase_admin
from firebase_admin import credentials, firestore, storage


FIREBASE_STORAGE_BUCKET = os.environ.get("FIREBASE_STORAGE_BUCKET","segfaults-database.appspot.com")



SERVICE_ACCOUNT_FILE = "robot-service-account.json"   
ROBOT_ID = os.environ.get("ROBOT_ID") or os.uname().nodename
COLLECTION_PATH = "events"    
PUBLISH_DEBUG_TOPIC = True
ROBOT_UID = os.environ.get("ROBOT_UID") or ROBOT_ID


def ros_time_to_seconds(ros_time):
    return ros_time.sec + ros_time.nanosec * 1e-9

def detection_to_dict(detection):
    
    result = {
        "center": {
            "x": detection.bbox.center.x,
            "y": detection.bbox.center.y
        },
        "size": {
            "x": detection.bbox.size.x,
            "y": detection.bbox.size.y
        },
        "results": []
    }
    for res in detection.results:
        result["results"].append({
            "id": res.id,
            "score": res.score,
            "hypothesis": {
                "class_id": res.hypothesis.class_id,
                "class_name": res.hypothesis.class_name
            }
        })
    return result

class EventCompilerNode(Node):
    def __init__(self):
        super().__init__('event_compiler_node')

        # start of firebase inmit
        script_dir = os.path.dirname(os.path.abspath(__file__))
        cred_path = os.path.join(script_dir, SERVICE_ACCOUNT_FILE)
        if not firebase_admin._apps:
            cred = credentials.Certificate(cred_path)
            firebase_admin.initialize_app(cred,{"storageBucket": FIREBASE_STORAGE_BUCKET})
        self.db = firestore.client()
        # end of firebase inmit

        self.bucket = storage.bucket()

        self.subscription1 = Subscriber(self, Image, '/camera/image_raw')
        ##this subscribes to example gps topic, can change this to whatever it actually is 
        ##self.subscription2 = Subscriber(self, NavSatFix, '/gps/fix')
        self.subscription3 = Subscriber(self, Detection2DArray, 'detections')

        ##ensures that the time of the bounding box message and image are close
        self.ts = ApproximateTimeSynchronizer(
            [self.subscription1, self.subscription3], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synced_callback)
        self.bridge = CvBridge()
        ##publish compiled events to compiled_events topic
        self.publisher_ = self.create_publisher(String, 'compiled_events', 10)
        self.get_logger().info("Event Compiler node initialized")

    def synced_callback(self, image_msg:Image, det_msg:Detection2DArray):
        # Convert ROS Image msg to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        ts_ms = int(ros_time_to_seconds(image_msg.header.stamp) * 1000)
        image_storage_path, image_url = self.upload_image_to_firebase(frame,ts_ms)

        # Here you can process the frame and detection data into a json which will be published as an event

        events={
            "time": ros_time_to_seconds(image_msg.header.stamp),
            "frame_id": image_msg.header.frame_id,
            "detections": len(det_msg.detections),
            "robot_id": ROBOT_ID,
            "users": [ROBOT_UID],
            "detections_full": [detection_to_dict(d) for d in det_msg.detections],
            "image_storage_path": image_storage_path,
            "image_url": image_url
            #"timestamp": firestore.SERVER_TIMESTAMP
        }
        self.get_logger().info(f"EVENT: {events}  ")
        self.publisher_.publish(String(data=json.dumps(events)))

        #write to firestore
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
        return path,url

def main(args=None):
    rclpy.init(args=args)
    node = EventCompilerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



##sashas notes:
##https://firebase.google.com/docs/storage/admin/start?utm_source=chatgpt.com#node.js

##shows that I should use appspot.com for storage bucket

