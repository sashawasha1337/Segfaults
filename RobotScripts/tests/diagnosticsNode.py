#!/usr/bin/env python3
import json
import os
import socket
import time
from typing import Optional, Dict, Any

try:
    import psutil
except Exception:
    psutil = None

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Firestore check
_FIREBASE_ENABLED = False
_db = None
try:
    import firebase_admin
    from firebase_admin import credentials, firestore

    KEY = os.getenv("FIREBASE_KEY_PATH", "")
    if KEY and os.path.exists(KEY):
        if not firebase_admin._apps:
            firebase_admin.initialize_app(credentials.Certificate(KEY))
        _db = firestore.client()
        _FIREBASE_ENABLED = True
except Exception:
    _FIREBASE_ENABLED = False
    _db = None

class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__("diagnostics_node")
        self.hostname = socket.gethostname()
        self.timer_period = float(os.getenv("DIAG_PERIOD_SEC", "2.0"))

        # Updated state
        self.state: Dict[str, Any] = {
            "battery": None,        # voltage (float)
            "network": None,        # rssi (float)
            "camera": None,         # fps (float)
            "last_heartbeat": time.time(),
        }

        # Subscribers
        try:
            from sensor_msgs.msg import BatteryState
            self.create_subscription(BatteryState, "/battery_state", self._on_battery, 10)
        except Exception:
            self.get_logger().info("BatteryState not available; skipping ...")

        try:
            from std_msgs.msg import Int32
            self.create_subscription(Int32, "/network/rssi", self._on_rssi, 10)
        except Exception:
            self.get_logger().info("RSSI topic not available; skipping ...")

        try:
            from std_msgs.msg import Float32
            self.create_subscription(Float32, "/camera/fps", self._on_fps, 10)
        except Exception:
            self.get_logger().info("Camera FPS topic not available; skipping ...")

        # Publisher
        self.pub = self.create_publisher(String, "/diagnostics", 10)
        self.timer = self.create_timer(self.timer_period, self._tick)

        self.get_logger().info(
            f"DiagnosticsNode running on {self.hostname} | period={self.timer_period}s | firestore={_FIREBASE_ENABLED}"
        )

    # Callbacks
    def _on_battery(self, msg):
        self.state["battery"] = {"voltage": float(msg.voltage), "percent": float(msg.percentage)}
        self.state["last_heartbeat"] = time.time()

    def _on_rssi(self, msg):
        self.state["network"] = {"rssi": int(getattr(msg, "data", -99))}
        self.state["last_heartbeat"] = time.time()

    def _on_fps(self, msg):
        self.state["camera"] = {"fps": float(getattr(msg, "data", 0.0))}
        self.state["last_heartbeat"] = time.time()

    # Periodic tick
    def _collect_host_metrics(self) -> Dict[str, Any]:
        cpu = psutil.cpu_percent(interval=None) if psutil else None
        mem = psutil.virtual_memory()._asdict() if (psutil and hasattr(psutil, "virtual_memory")) else None
        temp = None
        if psutil and hasattr(psutil, "sensors_temperatures"):
            try:
                tmap = psutil.sensors_temperatures()
                if tmap:
                    first = next(iter(tmap.values()))
                    if first:
                        temp = first[0]._asdict()
            except Exception:
                temp = None
        return {"cpu_percent": cpu, "memory": mem, "temperature": temp}

    def _heartbeat_status(self) -> str:
        dt = time.time() - self.state.get("last_heartbeat", 0)
        if dt < self.timer_period * 2.5:
            return "OK"
        return "STALE"

    def _tick(self):
        payload = {
            "host": self.hostname,
            "ts": firestore.SERVER_TIMESTAMP,
            "heartbeat": self._heartbeat_status(),
            "battery": self.state["battery"],
            "network": self.state["network"],
            "camera": self.state["camera"],
            "host_metrics": self._collect_host_metrics(),
        }

        # Publish JSON
        msg = String()
        msg.data = json.dumps(payload, default=str)
        self.pub.publish(msg)

        # Mirror to firestore
        if _FIREBASE_ENABLED and _db is not None:
            try:
                _db.document("diagnostics", self.hostname).set(payload, merge=True)
            except Exception as e:
                self.get_logger().warn(f"Firestore write failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()