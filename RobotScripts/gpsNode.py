#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import os
import time
import serial
import pynmea2
import firebase_admin
from firebase_admin import credentials, firestore

SERVICE_ACCOUNT_PATH = os.getenv("FIREBASE_KEY_PATH", "/home/ugv/.keys/firebase-adminsdk.json")
SERIAL_PORT = '/dev/ttyUSB0'          # Raspberry Pi or Jetson Nano use = '/dev/ttyUSB0'
BAUD_RATE = 9600                      # bits per second
DELTA_DEG = 0.000005                  # ~ 0.5 meters
LOCATION_CHECK_INTERVAL = 2.5         # seconds

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.timer = self.create_timer(LOCATION_CHECK_INTERVAL, self.run)

        if not firebase_admin._apps:
            cred = credentials.Certificate(SERVICE_ACCOUNT_PATH)
            firebase_admin.initialize_app(cred)

        self.db = firestore.client()
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        self.last_lat = None
        self.last_lng = None

    def has_moved(self, lat, lng):
        if self.last_lat is None or self.last_lng is None:
            return True
        return (abs(lat - self.last_lat) > DELTA_DEG or (abs(lng - self.last_lng) > DELTA_DEG))

    def save_detection(self, image_url, category, confidence, lat, lng, location, robot_id):
        image_data_collection = self.db.collection("trash_data")

        image_data_collection.add({
            "url": image_url,
            "category": category,
            "confidence": confidence,
            "latitude": lat,
            "longitude": lng,
            "location": location,
            "robotID": robot_id,
            "timestamp": firestore.SERVER_TIMESTAMP,
        })

    def run(self):
        try:
            line = self.ser.readline().decode("ascii", errors="ignore").strip()
            if not line.startswith("$"):
                return
            packet = pynmea2.parse(line)
            if isinstance(packet, pynmea2.types.talker.RMC) and getattr(packet, "status", None) == "A":
                latitude = round(float(packet.latitude), 5)
                longitude = round(float(packet.longitude), 5)

                msg = NavSatFix()
                msg.latitude = latitude
                msg.longitude = longitude
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'gps'
                self.publisher.publish(msg)
                self.get_logger().info(f"Published GPS: lat={latitude}, lng={longitude}")

                if self.has_moved(latitude, longitude):
                    self.last_lat = latitude
                    self.last_lng = longitude
                    self.db.collection("gps_data").document("global").set({
                        "latitude": latitude,
                        "longitude": longitude,
                        "timestamp": firestore.SERVER_TIMESTAMP,
                    })
                    self.get_logger().info("Uploaded GPS to Firestore")
        except Exception as e:
            self.get_logger().error(f"Failed to parse line: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down GPS node.")
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
