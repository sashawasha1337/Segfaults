"""
Minimal ROS2 -> Firestore writer for top-level 'events' collection.
Only writes: robot_id, location, time, category, users.

Run example:'
    cd C:\pixi_ws
    pixi shell
    call C:\pixi_ws\ros2-windows\local_setup.bat
  set ROBOT_ID=RBT-01
  set ROBOT_UID= 1P9zyLnCuoRLiqIRWZub                 (this is Joey's robot UID)
  set EVENT_LOCATION= Test Park
  set EVENT_CATEGORY= Litter
  # ONE_SHOT=1 (default) sends a single event then exits; set to 0 to keep sending on every message
  set ONE_SHOT=1
  python3 SuperSimpleEventTest.py
"""

import os
import json
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray  
from std_msgs.msg import String               # optional debug publish

import firebase_admin
from firebase_admin import credentials, firestore

SERVICE_ACCOUNT_FILE = "robot-service-account.json"
ROBOT_NAME  = os.environ.get("ROBOT_ID")  or os.uname().nodename
ROBOT_UID = os.environ.get("ROBOT_UID") or ROBOT_ID
EVENT_LOCATION = os.environ.get("EVENT_LOCATION", "")
EVENT_CATEGORY = os.environ.get("EVENT_CATEGORY", "")
COLLECTION_PATH = "events"

ONE_SHOT = os.environ.get("ONE_SHOT", "1") == "1"  # default: send once then exit

class EventCompilerNode(Node):
    def __init__(self):
        super().__init__('event_compiler_node')

        # Firebase init
        cred_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), SERVICE_ACCOUNT_FILE)
        if not firebase_admin._apps:
            cred = credentials.Certificate(cred_path)
            firebase_admin.initialize_app(cred)
        self.db = firestore.client()

        # Subscribe to any trigger (detections)
        self.create_subscription(Detection2DArray, 'detection_events', self.on_trigger, 10)

        # Optional: publish the exact JSON we send (handy for testing)
        self.pub_debug = self.create_publisher(String, 'compiled_events', 10)

        self._sent = False
        self.get_logger().info(
            f"Minimal events writer ready. robot_id='{ROBOT_ID}', uid='{ROBOT_UID}', "
            f"location='{EVENT_LOCATION}', category='{EVENT_CATEGORY}', ONE_SHOT={ONE_SHOT}"
        )

    def on_trigger(self, _msg: Detection2DArray):
        if ONE_SHOT and self._sent:
            return

        # Build only the fields your schema needs
        event_doc = {
            "robotName": ROBOT_ID,
            "location": EVENT_LOCATION,
            "category": EVENT_CATEGORY,
            "robotID": [ROBOT_UID],
            "time": firestore.SERVER_TIMESTAMP,  # Firestore server time
        }

        # Optional: publish JSON to compiled_events for visibility
        self.pub_debug.publish(String(data=json.dumps(event_doc)))

        # Write to Firestore
        try:
            ref = self.db.collection(COLLECTION_PATH).add(event_doc)
            self.get_logger().info(f"Wrote events/{ref[1].id}: {event_doc}")
        except Exception as e:
            self.get_logger().error(f"Firestore write failed: {e}")

        if ONE_SHOT:
            self._sent = True
            self.get_logger().info("ONE_SHOT complete; shutting down.")
            # Delay shutdown slightly so logs/publish flush
            self.create_timer(0.05, lambda: rclpy.shutdown())

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