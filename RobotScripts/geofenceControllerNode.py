#!/usr/bin/env python3
import os
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import firebase_admin
from firebase_admin import credentials, firestore

SERVICE_ACCOUNT_PATH = os.getenv("FIREBASE_KEY_PATH", "/home/ugv/.keys/firebase-adminsdk.json")

GEO_CHECK_PERIOD = 2.0            # seconds
OUTSIDE_SPIN_DURATION = 2.0       # seconds
TURN_ANGULAR_SPEED = 0.8          # rad/s (positive = left, negative = right)
LINEAR_SPEED_WHEN_INSIDE = 0.0    # m/s
TEST_MODE = True

def haversine_m(lat1, lng1, lat2, lng2):
    r = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlng = math.radians(lng2 - lng1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlng/2)**2
    c = 2 * math.asin(math.sqrt(a))
    return r * c

class GeofenceController(Node):
    def __init__(self):
        super().__init__("geofence_controller_node")

        if not firebase_admin._apps:
            cred = credentials.Certificate(SERVICE_ACCOUNT_PATH)
            firebase_admin.initialize_app(cred)
        self.db = firestore.client()

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(GEO_CHECK_PERIOD, self.loop)

        self.get_logger().info("GeofenceController started Firestore → /cmd_vel.")

    def fetch_latest_gps(self):
        q = (
            self.db.collection("gps_data")
            .order_by("timestamp", direction=firestore.Query.DESCENDING)
            .limit(1)
        )
        docs = list(q.stream())
        if not docs:
            return None
        return docs[0].to_dict()

    def fetch_geofence(self):
        ref = self.db.collection("geofence").document("global")
        snap = ref.get()
        if not snap.exists:
            return None
        return snap.to_dict()

    def publish_cmd(self, linear_x: float, angular_z: float, duration: float=0.0):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)

        if TEST_MODE:
            self.get_logger().info(
                f"(TEST) Would publish /cmd_vel"
            )
            return

        time_end = time.time() + max(0.0, duration)
        if duration > 0:
            while time.time() < time_end and rclpy.ok():
                self.cmd_pub.publish(twist)
                time.sleep(0.05)
            self.stop()
        else:
            self.cmd_pub.publish(twist)

    def stop(self):
        if TEST_MODE:
            self.get_logger().info("(TEST) Would STOP motors")
            return
        twist = Twist()
        self.cmd_pub.publish(twist)

    def loop(self):
        try:
            gf = self.fetch_geofence()
            if not gf or not gf.get("enabled"):
                self.get_logger().debug("Geofence disabled or missing configuration, stopping...")
                self.stop()
                return

            center = gf.get("center")
            radius = float(gf.get("radius", 25.0))
            if not center:
                self.get_logger().warn("Geofence has no center, stopping...")
                self.stop()
                return

            gps = self.fetch_latest_gps()
            if not gps:
                self.get_logger().warn("No GPS data yet, stopping...")
                self.stop()
                return

            lat = float(gps["latitude"])
            lng = float(gps["longitude"])
            distance = haversine_m(lat, lng, center["lat"], center["lng"])
            self.get_logger().info(f"Distance from center: {distance:.1f} meters (radius {radius:.1f} meters)")

            if distance > radius:
                self.get_logger().info("Outside geofence, ugv will spin in place (U-turn).")
                self.publish_cmd(0.0, TURN_ANGULAR_SPEED, OUTSIDE_SPIN_DURATION)
            else:
                if LINEAR_SPEED_WHEN_INSIDE == 0.0:
                    self.stop()
                else:
                    self.publish_cmd(LINEAR_SPEED_WHEN_INSIDE, 0.0, 0.25)

        except Exception as e:
            self.get_logger().error(f"Geofence loop function error: {e}")
            self.stop()

def main(args=None):
    rclpy.init(args=args)
    node = GeofenceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()