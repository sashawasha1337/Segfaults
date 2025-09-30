#!/usr/bin/env python3
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

if not firebase_admin._apps:
    cred = credentials.Certificate(SERVICE_ACCOUNT_PATH)
    firebase_admin.initialize_app(cred)

db = firestore.client()
gps_collection = db.collection("gps_data")

def has_moved(last_lat, last_lng, lat, lng):
    if last_lat is None or last_lng is None:
        return True
    return (abs(lat - last_lat) > DELTA_DEG or (abs(lng - last_lng) > DELTA_DEG))

def save_detection(image_url, category, confidence, lat, lng, location, robot_id):
    db = firestore.client()
    image_data_collection = db.collection("trash_data")

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

def run():
    print(f"\nListening for GPS data on serial port {SERIAL_PORT} ...")
    last_lat = None
    last_lng = None
    next_check_interval = 0.0

    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        while True:
            try:
                line_bytes = ser.readline()
                if not line_bytes:
                    continue
                now = time.time()
                if now < next_check_interval:
                    continue
                line = line_bytes.decode("ascii", errors="ignore").strip()
                if not line.startswith("$"):
                    continue
                try:
                    packet = pynmea2.parse(line)
                except pynmea2.ParseError:
                    continue

                if isinstance(packet, pynmea2.types.talker.RMC) and getattr(packet, "status", None) == "A":
                    latitude = round(float(packet.latitude), 5)
                    longitude = round(float(packet.longitude), 5)

                    if has_moved(last_lat, last_lng, latitude, longitude):
                        last_lat = latitude
                        last_lng = longitude
                        print(f"\nGPS fix received at: lat={latitude}, lng={longitude}")

                        data_point = {
                            "latitude": latitude,
                            "longitude": longitude,
                            "timestamp": firestore.SERVER_TIMESTAMP,
                        }
                        gps_collection.add(data_point)
                        print(f"Uploaded GPS data to Firebase: {data_point}")
                    else:
                        print(f"UGV has not moved")
                    next_check_interval = now + LOCATION_CHECK_INTERVAL
            except KeyboardInterrupt:
                print("\nStopping...")
                break
            except Exception as e:
                print(f"Failed to parse line: {e}")
                time.sleep(0.25)

if __name__ == "__main__":
    run()