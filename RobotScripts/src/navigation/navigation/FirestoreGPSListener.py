#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from pyproj import Proj, Transformer

import firebase_admin
from firebase_admin import credentials, firestore

from datetime import datetime, timezone
import math
import os

SERVICE_ACCOUNT_FILE = os.path.expanduser(
    "~/ros2_ws/src/camera/camera/robot-service-account.json"
)

WAYPOINT_COLLECTION = "waypoints"       # Firestore collection name
PUBLISH_RATE = 2.0                      # seconds between reads

class FirestoreGPSListener(Node):
    def __init__(self):
        super().__init__("firestore_gps_listener")

        # Initialize Firebase Admin SDK
        cred = credentials.Certificate(SERVICE_ACCOUNT_FILE)
        if not firebase_admin._apps:
            firebase_admin.initialize_app(cred)
            self.db = firestore.client()

        # Publishers / subscribers
        self.publisher = self.create_publisher(Path, "/gps_goal", 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_origin_callback, 10)

        # UTM conversion
        self.UTMZONE = 'epsg:32610' # Hard coded utm zone for part of California
        self.TRANSFORMER = Transformer.from_crs('epsg:4326', self.UTMZONE, always_xy=True)
        self.LAT0, self.LON0 = None, None # GPS origin coordinates
        self.X0, self.Y0 = 0, 0

        # Timer to poll Firestore
        self.timer = self.create_timer(PUBLISH_RATE, self.read_and_publish)

        # Cache last document ID
        self.last_doc_id = None

        self.node_initialized_at = datetime.now(timezone.utc)

        self.get_logger().info("Firestore GPS Listener started.")

    def gps_origin_callback(self, msg):
        if (self.LAT0 is None and self.LON0 is None):
            self.LAT0 = msg.latitude
            self.LON0 = msg.longitude
            self.X0, self.Y0 = self.TRANSFORMER.transform(self.LON0, self.LAT0)
            self.get_logger().info(f"Set GPS origin at: Latitude = {self.LAT0}, Longitude = {self.LON0}")
            self.destroy_subscription(self.gps_sub)

    def read_and_publish(self):
        # # Check if gps origin has been set from gps module
        # if (self.LAT0 is None or self.LON0 is None):
        #     self.get_logger().warn("No reading from gps module, GPS origin not set.")
        #     return

        try:
            latest_doc = self.db.collection(WAYPOINT_COLLECTION) \
                .order_by("timestamp", direction=firestore.Query.DESCENDING) \
                .limit(1) \
                .get()

            # Case where path hasn't changed, skip publishing
            if latest_doc[0].id == self.last_doc_id:
                return

            # # Only process documents that have been added to firestore after node initialization
            # if latest_doc:
            #     data = latest_doc[0].to_dict()

            #     doc_ts = data.get("timestamp")
            #     if not doc_ts:
            #         return # No timestamp field, ignore
            #     # Ensure timezone awareness
            #     doc_dt = doc_ts.replace(tzinfo=timezone.utc) if doc_ts.tzinfo is None else doc_ts

            #     # Only process if document timezone is newer than initialization time
            #     if doc_dt > self.node_initialized_at:
            #         path_array = data.get("path", [])
            #     else:
            #         return # Ignore old data
            if latest_doc:
                data = latest_doc[0].to_dict()
                path_array = data.get("path", [])

            if not path_array:
                self.get_logger().warn(f"Document {latest_doc[0].id} has no path.")
                return
            
            path_msg = Path()
            path_msg.header.frame_id = "map"

            # Convert all GPS points in path to PoseStamped
            for i, point in enumerate(path_array):
                pose = PoseStamped()
                pose.header.frame_id = "map"

                # Convert lat/lon to UTM x/y
                x, y = self.TRANSFORMER.transform(point["lng"], point["lat"])
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0

                # Orientation
                if i < len(path_array) - 1:
                    next_point = path_array[i + 1]
                    next_x, next_y = self.TRANSFORMER.transform(next_point["lng"], next_point["lat"])
                    theta = math.atan2(next_y - y, next_x - x)
                else:
                    theta = 0.0  # last point
                
                qz = math.sin(theta / 2.0)
                qw = math.cos(theta / 2.0)
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw

                path_msg.poses.append(pose)

            self.publisher.publish(path_msg)
            self.last_doc_id = latest_doc[0].id
            self.get_logger().info(f"Published path from doc {latest_doc[0].id} with {len(path_msg.poses)} waypoints.")

        except Exception as e:
            self.get_logger().error(f"Error reading from Firestore: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FirestoreGPSListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down FirestoreGPSListener")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()