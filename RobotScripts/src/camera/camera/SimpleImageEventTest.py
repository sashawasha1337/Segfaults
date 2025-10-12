"""
Minimal ROS2 -> Firestore writer with optional image upload (CompressedImage).
Writes: robotId, location, time, category, users, and (if available) image_storage_path, image_url, frame_id.

Example (PowerShell / Windows):

cd C:\pixi_ws
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat

set ROBOT_ID=RBT-01
set ROBOT_UID=3wxSSldz5mhCJrOJiAGvzsnMGpn2
set EVENT_LOCATION=Test Park
set EVENT_CATEGORY=Litter
set ONE_SHOT=1
set FIREBASE_STORAGE_BUCKET=segfaults-database.firebasestorage.app
:: Optional: require an image before posting
set WAIT_FOR_IMAGE=1
:: Optional: change image topic
set IMAGE_TOPIC=/camera/image_raw/compressed

python testFeeder.py
python SimpleImageEventTest.py
"""

import os
import json
import uuid
import urllib.parse
import platform
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

import firebase_admin
from firebase_admin import credentials, firestore, storage

# ---------- Config ----------
SERVICE_ACCOUNT_FILE = "robot-service-account.json"
COLLECTION_PATH = "events"

ROBOT_ID = os.environ.get("ROBOT_ID") or platform.node() or socket.gethostname()
ROBOT_UID = os.environ.get("ROBOT_UID") or ROBOT_ID
EVENT_LOCATION = os.environ.get("EVENT_LOCATION", "")
EVENT_CATEGORY = os.environ.get("EVENT_CATEGORY", "")

# One-shot behavior: send once then exit (default: 1)
ONE_SHOT = os.environ.get("ONE_SHOT", "1") == "1"

# If true, wait until at least one CompressedImage is received before sending the event
WAIT_FOR_IMAGE = os.environ.get("WAIT_FOR_IMAGE", "0") == "1"

# Topic to listen for JPEG-compressed images
IMAGE_TOPIC = os.environ.get("out_image_topic','/tracking/image")

# Firebase Storage bucket (must be *.appspot.com for Admin SDK signed URLs)
FIREBASE_STORAGE_BUCKET = os.environ.get("FIREBASE_STORAGE_BUCKET", "segfaults-database.firebasestorage.app")
# ---------------------------


class SuperSimpleEventNode(Node):
    def __init__(self):
        super().__init__("super_simple_event_node")

        # --- Firebase init ---
        script_dir = os.path.dirname(os.path.abspath(__file__))
        cred_path = os.path.join(script_dir, SERVICE_ACCOUNT_FILE)
        if not os.path.exists(cred_path):
            raise FileNotFoundError(
                f"Service account file not found at {cred_path}. "
                f"Place {SERVICE_ACCOUNT_FILE} next to this script."
            )
        if not firebase_admin._apps:
            cred = credentials.Certificate(cred_path)
            firebase_admin.initialize_app(cred, {"storageBucket": FIREBASE_STORAGE_BUCKET})
        self.db = firestore.client()
        self.bucket = storage.bucket(FIREBASE_STORAGE_BUCKET)   
        # ---------------------

        # Debug publisher (what we write to Firestore)
        self.pub_debug = self.create_publisher(String, "compiled_events", 10)

        # Image buffer
        self._last_jpeg_bytes = None
        self._last_frame_id = None
        self._last_stamp = None

        # Subscribe to  images
        self._img_sub = self.create_subscription(Image, IMAGE_TOPIC, self._on_image, 10)

        self._sent = False
        self.get_logger().info(
            f"Minimal events writer ready.\n"
            f"  robot_id='{ROBOT_ID}', uid='{ROBOT_UID}', location='{EVENT_LOCATION}', category='{EVENT_CATEGORY}'\n"
            f"  ONE_SHOT={ONE_SHOT}, WAIT_FOR_IMAGE={WAIT_FOR_IMAGE}, IMAGE_TOPIC='{IMAGE_TOPIC}'\n"
            f"  FIREBASE_STORAGE_BUCKET='{FIREBASE_STORAGE_BUCKET}'"
        )

        # Fire once shortly after startup (or keep trying until image arrives if WAIT_FOR_IMAGE is set)
        self.create_timer(0.25, self._tick)

    # ---------- ROS callbacks ----------

    def _on_image(self, msg: Image):
        # Store the most recent JPEG bytes + metadata
        self._last_jpeg_bytes = bytes(msg.data)
        self._last_frame_id = msg.header.frame_id
        self._last_stamp = msg.header.stamp
        # Optional safety check
        if msg.format and "jpeg" not in msg.format.lower():
            self.get_logger().warning(f"Received Image with unexpected format '{msg.format}'")

    def _tick(self):
        if ONE_SHOT and self._sent:
            return

        if WAIT_FOR_IMAGE and self._last_jpeg_bytes is None:
            self.get_logger().info("Waiting for first image before posting event...")
            return

        self._send_event()

        if ONE_SHOT:
            self._sent = True
            self.get_logger().info("ONE_SHOT complete; shutting down.")
            self.create_timer(0.05, lambda: rclpy.shutdown())

    # ---------- Firebase helpers ----------

    def _upload_jpeg_bytes_to_firebase(self, jpeg_bytes: bytes, ts_ms: int):
        """Uploads JPEG bytes to Firebase Storage and returns (path, public_token_url)."""
        filename = f"{ROBOT_ID}_{ts_ms}_{uuid.uuid4().hex}.jpg"
        path = f"events/{ROBOT_ID}/{filename}"
        blob = self.bucket.blob(path)

        token = uuid.uuid4().hex
        blob.metadata = {"firebaseStorageDownloadTokens": token}
        blob.upload_from_string(jpeg_bytes, content_type="image/jpeg")

        fixedname = urllib.parse.quote(blob.name, safe="")
        url = f"https://firebasestorage.googleapis.com/v0/b/{self.bucket.name}/o/{fixedname}?alt=media&token={token}"
        return path, url

    # ---------- Event writer ----------

    def _send_event(self):
        image_storage_path = None
        image_url = None
        frame_id = self._last_frame_id
        ts_ms = None

        # If we have an image, upload it first
        if self._last_jpeg_bytes:
            # Compute ms timestamp from ROS2 stamp if available
            if self._last_stamp is not None:
                ts_ms = int(self._last_stamp.sec * 1000 + self._last_stamp.nanosec / 1e6)
            else:
                ts_ms = 0
            try:
                image_storage_path, image_url = self._upload_jpeg_bytes_to_firebase(self._last_jpeg_bytes, ts_ms)
                self.get_logger().info(f"Uploaded image to {image_storage_path}")
            except Exception as e:
                self.get_logger().error(f"Image upload failed: {e}")

        # Build the Firestore document (server timestamp for canonical event time)
        event_doc = {
            "robotId": ROBOT_ID,
            "location": EVENT_LOCATION,
            "category": EVENT_CATEGORY,
            "users": [ROBOT_UID],
            "time": firestore.SERVER_TIMESTAMP,
        }
        if image_url:
            event_doc["image_storage_path"] = image_storage_path
            event_doc["image_url"] = image_url
        if frame_id:
            event_doc["frame_id"] = frame_id

        # Debug publish (optional)
        debug_doc = dict(event_doc)
        if "time" in debug_doc:
            debug_doc["time"] = "SERVER_TIMESTAMP"
        try:
            self.pub_debug.publish(String(data=json.dumps(debug_doc)))
        except Exception as e:
            self.get_logger().warn(f"Debug publish failed: {e}")

        # Write to Firestore
        try:
            ref = self.db.collection(COLLECTION_PATH).add(event_doc)
            self.get_logger().info(f"Wrote events/{ref[1].id}: {event_doc}")
        except Exception as e:
            self.get_logger().error(f"Firestore write failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SuperSimpleEventNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
