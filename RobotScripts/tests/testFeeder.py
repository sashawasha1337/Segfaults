import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

IMG = r"C:\pixi_ws\test.jpg"  # test jpg path here
class Feeder(Node):
    def __init__(self):
        super().__init__("feeder")
        self.pub = self.create_publisher(CompressedImage, "/camera/image_raw/compressed", 10)
        with open(IMG, "rb") as f: self.jpg = f.read()
        self.create_timer(0.5, self.tick)
    def tick(self):
        m = CompressedImage()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "camera"
        m.format = "jpeg"
        m.data = bytearray(self.jpg)
        self.pub.publish(m)
        self.get_logger().info("Published compressed image")

rclpy.init(); n=Feeder()
try: rclpy.spin(n)
except KeyboardInterrupt: pass
finally: n.destroy_node(); rclpy.shutdown()