#RPLIDAR obstacle avoidance node
#make sure rosmaster driver is installed and running

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from Rosmaster_Lib import Rosmaster
import time

class RPLidarAvoidanceNode(Node):
    def __init__(self):
        super().__init__('rplidar_avoidance_node')

        #connect to motor controller
        try:
            self.car = Rosmaster('/dev/ttyUSB0') # Adjust port as necessary. Could be /dev/ttyACM0?
            self.get_logger().info("Connected to Rosmaster motor controller")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Rosmaster: {e}")
            exit(1)

        self.car.set_car_type(1) #Rosmaster X3 car type
        self.car.create_receive_threading() #start receiving data from motor controller
        self.car.set_akm_steering_angle(0.0) #straight steering

        #subscribe to LIDAR scan data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        #keep alive timer to prevent motor controller timeout
        self.timer = self.create_timer(0.5, self.keep_alive)

        self.avoiding = False

    def scan_callback(self, msg: LaserScan):
        if self.avoiding:
            return # Already avoiding an obstacle
            
        n = len(msg.ranges)
        #look at 15 degrees on either side of front
        #assumes msg.ranges has enough points, which it should
        #may want to check len(msg.ranges) to avoid index errors
        front_sector = msg.ranges[n // 2 - 15: n // 2 + 15] 

        #filer out invalid readings
        valid = [r for r in front_sector if 0.1 < r < msg.range_max]
        if not valid:
            self.get_logger().warn("No valid LIDAR readings")
            return
            
        closest = min(valid)
        self.get_logger().info(f"Closest obstacle at {closest:.2f} meters")

        #if obstacle closer than 0.5m
        #may need to adjust distance threshold (0.5) based on speed and LiDAR resolution
        if closest < 0.5: 
            self.get_logger().warn("Obstacle too close! Stopping and avoiding.")
            self.avoiding = True
            #turn wheel slightly
            self.car.set_akm_steering_angle(-0.5) 
            time.sleep(0.2)

            # Reverse while turning
            self.car.set_car_motion(-0.3, 0.0, 0.3)
            time.sleep(1.0)

            # Stop and realign
            self.car.set_car_motion(0.0, 0.0, 0.0)
            self.car.set_akm_steering_angle(0.0)

            self.avoiding = False
        else:
            self.car.set_akm_steering_angle(0.0)
            self.car.set_car_motion(0.3, 0.0, 0.0)
        
    def keep_alive(self):
        if not self.avoiding:
            self.car.set_car_motion(0.0, 0.0, 0.0) #stop moving if idle

def main():
    rclpy.init()
    node = RPLidarAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
