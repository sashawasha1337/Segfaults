
"""
Simple version of EventCompiler that does not handle images.

Requires:
  pip install firebase-admin
  (and ROS 2 with vision_msgs installed)
"""

import os
import json
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String

import firebase_admin
from firebase_admin import credentials, firestore

SERVICE_ACCOUNT_FILE = "robot-service-account.json"
ROBOT_ID  = os.environ.get("ROBOT_ID")  or os.uname().nodename
ROBOT_UID = os.environ.get("ROBOT_UID") or ROBOT_ID
COLLECTION_PATH = "events"

def detection_to_dict(d):
    return {
        "center": {"x": d.bbox.center.x, "y": d.bbox.center.y},
        "size":   {"x": d.bbox.size.x,   "y": d.bbox.size.y},
        "results": [
            {
                "id": r.id,
                "score": r.score,
                "hypothesis": {
                    "class_id": r.hypothesis.class_id,
                    "class_name": r.hypothesis.class_name,
                },
            }
            for r in d.results
        ],
    }

class EventCompilerNode(Node):
    def __init__(self):
        super().__init__("event_compiler_node_no_image")

        # Firebase init
        cred_path = os.path.join(os.path.dirname(__file__), SERVICE_ACCOUNT_FILE)
        if not firebase_admin._apps:
            cred = credentials.Certificate(cred_path)
            firebase_admin.initialize_app(cred)
        self.db = firestore.client()

        # Subscribe only to detection_events
        self.create_subscription(
            Detection2DArray, "detection_events", self.detection_callback, 10
        )

        # Optional: republish JSON for debugging
        self.publisher_ = self.create_publisher(String, "compiled_events", 10)

        self.get_logger().info("Event Compiler (no image) node initialized")

    def detection_callback(self, det_msg: Detection2DArray):
        event = {
            "time": det_msg.header.stamp.sec + det_msg.header.stamp.nanosec * 1e-9,
            "frame_id": det_msg.header.frame_id,
            "detections": len(det_msg.detections),
            "robot_id": ROBOT_ID,
            "users": [ROBOT_UID],
            "detections_full": [detection_to_dict(d) for d in det_msg.detections],
            "timestamp": firestore.SERVER_TIMESTAMP,
        }

        self.get_logger().info(f"EVENT: {event}")
        self.publisher_.publish(String(data=json.dumps(event)))

        try:
            ref = self.db.collection(COLLECTION_PATH).add(event)
            self.get_logger().info(f"Firestore write successful: {ref}")
        except Exception as e:
            self.get_logger().error(f"Firestore write failed: {e}")

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

if __name__ == "__main__":
    main()
