#!/usr/bin/env python3
import os
import math
import rclpy
from rclpy.node import Node
from firebase_admin import credentials, firestore, initialize_app
from threading import Thread
from .NavigationNode import NavigationNode

SERVICE_ACCOUNT_PATH = os.getenv("FIREBASE_KEY_PATH", "/root/.keys/firebase-adminsdk.json")

class FirestoreWaypointListener(Node):
    """
    Watches Firestore doc 'nav_goal/global' for {latitude, longitude, heading_deg}
    and sends a NavigateToPose goal through Nav2. Keeps a simple origin (lat0,lon0)
    from the first GPS fix seen in Firestore 'gps_data/global' (or falls back to params).
    """
    def __init__(self):
        super().__init__('firestore_waypoint_listener')

        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('origin_lat', 0.0)
        self.declare_parameter('origin_lon', 0.0)

        # Firebase init
        cred = credentials.Certificate(SERVICE_ACCOUNT_PATH)
        initialize_app(cred)
        self.db = firestore.client()

        # Establish a local origin: prefer the last known GPS doc; else param
        origin_lat = float(self.get_parameter('origin_lat').value)
        origin_lon = float(self.get_parameter('origin_lon').value)

        try:
            snap = self.db.collection("gps_data").document("global").get()
            if snap.exists:
                d = snap.to_dict()
                origin_lat = float(d.get('latitude', origin_lat))
                origin_lon = float(d.get('longitude', origin_lon))
        except Exception as e:
            self.get_logger().warn(f"Could not fetch origin from Firestore: {e}")

        # Child NavigationNode does UTM projection and NavigateToPose
        self.nav = NavigationNode(lat0=origin_lat, lon0=origin_lon)

        # Start listener thread (Firestore SDK is blocking)
        self._thread = Thread(target=self._listen_loop, daemon=True)
        self._thread.start()
        self.get_logger().info("Firestore waypoint listener started.")

    def _listen_loop(self):
        doc_ref = self.db.collection("nav_goal").document("global")
        def on_snapshot(doc_snapshot, changes, read_time):
            for doc in doc_snapshot:
                data = doc.to_dict() or {}
                try:
                    lat = float(data['latitude'])
                    lon = float(data['longitude'])
                    heading_deg = float(data.get('heading_deg', 0.0))
                except Exception:
                    self.get_logger().warn("nav_goal/global missing fields {latitude, longitude}")
                    continue

                x, y = self.nav.gps_conversion(lat, lon)
                theta = math.radians(heading_deg)
                self.get_logger().info(f"Received waypoint lat={lat:.6f}, lon={lon:.6f} -> x={x:.2f}, y={y:.2f}, θ={heading_deg:.1f}°")
                self.nav.send_goal(x, y, theta)

        doc_ref.on_snapshot(on_snapshot)

        import time
        while rclpy.ok():
            time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = FirestoreWaypointListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
