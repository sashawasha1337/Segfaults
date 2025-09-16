import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")


        self.declare_parameter("camera_index", 0)
        self.declare_parameter("width",        640)
        self.declare_parameter("height",       480)
        self.declare_parameter("fps",          15.0)
        self.declare_parameter('camera_backend', 'auto')  # 'auto' | 'gstreamer' | 'v4l2'



        idx    = self.get_parameter("camera_index").value
        width  = self.get_parameter("width").value
        height = self.get_parameter("height").value
        self.fps = self.get_parameter("fps").value

        # OpenCV capture
        self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS,          self.fps)

        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera index {idx}")
            raise RuntimeError("Camera open failed")

        # ROS pub ‑ use best‑effort QoS for high‑rate sensor data
        qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1,
        )
        self.pub   = self.create_publisher(Image, "/camera/image_raw", qos)
        self.bridge = CvBridge()

        # Publish at the requested frame rate
        self.timer = self.create_timer(1.0 / self.fps, self._publish_frame)
        self.get_logger().info("Camera publisher started")





    def _publish_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warning("Frame grab failed")
            return

        self.get_logger().info(f"Raw frame shape: {frame.shape}, dtype: {frame.dtype}")

        width  = self.get_parameter("width").value
        height = self.get_parameter("height").value

        # Detect flattened buffer and reshape
        if frame.ndim == 2 and frame.shape[0] == 1:
            # Flatten then reshape
            flat = frame.flatten()
            expected = width * height * 3
            if flat.size == expected:
                frame = flat.reshape((height, width, 3))
                self.get_logger().info(f"Reshaped to: {frame.shape}")
            else:
                self.get_logger().error(f"Unexpected buffer size: {flat.size}, expected {expected}")
                return

        # Convert grayscale to BGR if needed
        if len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub.publish(msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()



def main():
    rclpy.init()
    node = None
    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
