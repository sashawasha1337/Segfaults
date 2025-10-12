import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        self.get_logger().info("Starting camera node initialization.")


        self.declare_parameter("camera_index", 0)
        self.declare_parameter("width",        640)
        self.declare_parameter("height",       480)
        self.declare_parameter("fps",          15.0)
        self.declare_parameter('camera_backend', 'auto')  # 'auto' | 'gstreamer' | 'v4l2'



        idx    = self.get_parameter("camera_index").value
        width  = self.get_parameter("width").value
        height = self.get_parameter("height").value
        self.fps = self.get_parameter("fps").value

        try:
            # OpenCV capture
            self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS,          self.fps)
        
        except Exception as e:
            self.get_logger().error(f"Exception opening camera index {idx}: {str(e)}")
            raise ConnectionError("Camera open failed")

        backend = self.get_parameter('camera_backend').value
        if backend != 'auto':
            self.get_logger().info(f"Camera backend detected: {backend}")
        else:
            self.get_logger().info(f"Camera backend: Auto-detected (using {cv2.CAP_V4L2})")

        try:
            # Log actual camera properties
            actual_width  = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps    = self.cap.get(cv2.CAP_PROP_FPS)

            self.get_logger().info(f"Actual camera properties: {actual_width}x{actual_height} @ {actual_fps} FPS")
        except Exception as e:
            self.get_logger().warning(f"Could not retrieve camera properties: {str(e)}")

        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera index {idx}")
            raise ConnectionError("Camera open failed")

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
        self.get_logger().info("Camera publisher started.")
        self.get_logger().info("Camera node initialized.")




    def _publish_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warning("Frame grab failed, retrying...", throttle_duration_sec=5.0)
            return

        self.get_logger().debug(f"Raw frame shape: {frame.shape}, dtype: {frame.dtype}", throttle_duration_sec=5.0)

        width  = self.get_parameter("width").value
        height = self.get_parameter("height").value

        # Detect flattened buffer and reshape
        if frame.ndim == 2 and frame.shape[0] == 1:
            # Flatten then reshape
            flat = frame.flatten()
            expected = width * height * 3
            if flat.size == expected:
                frame = flat.reshape((height, width, 3))
                #self.get_logger().info(f"Reshaped to: {frame.shape}")
            else:
                self.get_logger().error(f"Unexpected buffer size: {flat.size}, expected {expected}")
                return

        # Convert grayscale to BGR if needed
        if len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.pub.publish(msg)
            # Once-only debug log
            self.get_logger().info("Published first camera frame", once=True)
            self.get_logger().debug(f"Published frame shape: {frame.shape}, dtype: {frame.dtype}", throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f"Failed to publish camera frame: {str(e)}")

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()
        self.get_logger().info("Camera node shutting down.")



def main():
    rclpy.init()
    node = None
    try:
        node = CameraPublisher()
        node.get_logger().info("Camera node starting.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
