# Usage: Robot drives forward until it detects an object/obstacale in front of it. Then attempts to back and turn to go around
# Mainly a test/theory code, likely not for the final product  

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time
from Rosmaster_Lib import Rosmaster

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        # Initialize connection to the motor controller via serial
        try:
            self.car = Rosmaster('/dev/myserial')  # Assumes /dev/myserial is a symlink to /dev/ttyUSB0
            print("✅ Connected to Rosmaster via /dev/myserial")
        except Exception as e:
            print(f"Rosmaster connection failed: {e}")
            exit()

        # Set the car type to X3 (important for correct control)
        self.car.set_car_type(1)
        print("Car type set to X3")

        # Start background data reading from the Rosmaster board
        self.car.create_receive_threading()
        time.sleep(0.5)

        # Center the steering so the robot starts straight
        self.car.set_akm_steering_angle(0.0)
        print("Steering centered (angle = 0.0)")

        # Subscribe to LiDAR scan data on the /scan topic
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Timer to periodically send stop signals (prevents robot from drifting)
        self.timer = self.create_timer(0.5, self.keep_alive)

        # Flag to indicate if the robot is currently avoiding an obstacle
        self.avoiding = False

    def scan_callback(self, msg):
        # If we're in the middle of an avoidance maneuver, ignore new scan data
        if self.avoiding:
            return

        # Filter out invalid readings (like 0.0 or inf)
        valid = [r for r in msg.ranges if 0.1 < r < 3.0]
        if not valid:
            print("No valid LiDAR data")
            return

        # Find the closest valid obstacle
        closest = min(valid)
        print(f"Closest object at {closest:.2f} m")

        # If something is too close, perform avoidance
        if closest < 0.4:
            print("Obstacle detected — Backing up and turning")
            self.avoiding = True

            # Turn wheels slightly left
            self.car.set_akm_steering_angle(-0.5)
            time.sleep(0.2)

            # Reverse while turning to avoid obstacle
            self.car.set_car_motion(-0.3, 0.0, 0.3)
            print("🔄 Reversing with turn")
            time.sleep(1.5)

            # Stop after maneuver
            self.car.set_car_motion(0.0, 0.0, 0.0)
            self.car.set_akm_steering_angle(0.0)
            print("Evasion complete — Stopped")

            self.avoiding = False

        else:
            # Path is clear — drive forward with centered steering
            self.car.set_akm_steering_angle(0.0)
            print("Steering set to center")

            self.car.set_car_motion(0.3, 0.0, 0.0)
            print("Path is clear — Moving forward")

    def keep_alive(self):
        # Prevents the robot from drifting if no new commands come in
        if not self.avoiding:
            self.car.set_car_motion(0.0, 0.0, 0.0)
            print("Keep-alive: Stopping to prevent idle drift")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
