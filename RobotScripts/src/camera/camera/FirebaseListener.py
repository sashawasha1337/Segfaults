#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import json
import base64
import uuid
import urllib.parse
import datetime as dt
import os

import firebase_admin
from firebase_admin import credentials, firestore, storage


# ----------------------------------------------------------
# ENV / CONSTANTS
# ----------------------------------------------------------
FIREBASE_STORAGE_BUCKET = os.environ.get(
    "FIREBASE_STORAGE_BUCKET",
    "segfaults-database.firebasestorage.app"
)

SERVICE_ACCOUNT_FILE = (
    os.path.expanduser("~/ros2_ws/src/camera/camera/robot-service-account.json")
)

ROBOT_ID = os.environ.get("ROBOT_ID") or os.uname().nodename
ROBOT_UID = os.environ.get("ROBOT_UID") or ROBOT_ID
COLLECTION_PATH = "events"


class FirebaseListener(Node):
    def __init__(self):
        super().__init__("firebase_listener")

        # Firebase initialization
        cred = credentials.Certificate(SERVICE_ACCOUNT_FILE)
        firebase_admin.initialize_app(cred, {
            "storageBucket": FIREBASE_STORAGE_BUCKET
        })

        self.db = firestore.client()
        self.bucket = storage.bucket(FIREBASE_STORAGE_BUCKET)

        # Subscribe to YOLO compiled events
        self.subscription = self.create_subscription(
            String,
            "/compiled_events",
            self.on_event,
            10
        )

        self.get_logger().info("Firebase listener ready (listening on /compiled_events)")


    def upload_jpeg_bytes(self, robot_id, ts_ms, jpeg_bytes):
        filename = f"{robot_id}_{ts_ms}_{uuid.uuid4().hex}.jpg"
        path = f"events/{robot_id}/{filename}"

        blob = self.bucket.blob(path)
        token = uuid.uuid4().hex

        blob.metadata = {"firebaseStorageDownloadTokens": token}
        blob.upload_from_string(jpeg_bytes, content_type="image/jpeg")

        fixedname = urllib.parse.quote(blob.name, safe="")
        url = (
            f"https://firebasestorage.googleapis.com/v0/b/"
            f"{self.bucket.name}/o/{fixedname}?alt=media&token={token}"
        )

        return path, url


    def publish_to_firebase(self, req):
        ts_ms = req.get("timestamp_ms")
        jpeg_b64 = req.get("image")
        label = req.get("label")
        conf = req.get("conf")

        image_url = None

        if jpeg_b64:
            jpeg_bytes = base64.b64decode(jpeg_b64.encode("ascii"))
            _, image_url = self.upload_jpeg_bytes(ROBOT_UID, ts_ms, jpeg_bytes)


        if ts_ms:
            iso = dt.datetime.utcfromtimestamp(ts_ms / 1000).isoformat() + "Z"
        else:
            iso = dt.datetime.utcnow().isoformat() + "Z"

        # Firestore event payload
        new_event = {
            "category": label,
            "location": "",
            "robotId": ROBOT_UID,
            "time": iso,
            "confidence": conf,
            "image_url": image_url,

            # *Extra Yolo metadata preserved*
            #"track_id": req.get("track_id"),
            #"bbox": req.get("bbox"),
            #"class_id": req.get("class_id"),
        }

        self.db.collection(COLLECTION_PATH).add(new_event)
        self.get_logger().info(f"Firebase listener Published:{label}")


    def on_event(self, msg):
        try:
            req = json.loads(msg.data)
            self.publish_to_firebase(req)
        except Exception as e:
            self.get_logger().error(f"Failed to process event: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FirebaseListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()