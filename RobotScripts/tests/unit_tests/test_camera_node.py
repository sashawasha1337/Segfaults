# tests/unit_tests/test_camera_node.py
import time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

@pytest.fixture(scope="session", autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture()
def frame_listener():
    class FrameListener(Node):
        def __init__(self):
            super().__init__('frame_listener')
            self.frames_received = 0
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
            self.create_subscription(Image, '/camera/image_raw', self.callback, qos)

        def callback(self, msg):
            self.frames_received += 1

    node = FrameListener()
    yield node
    node.destroy_node()

@pytest.mark.integration
def test_camera_publishes_frames(frame_listener):
    """Verify camera node publishes to /camera/image_raw"""
    timeout = 3.0
    start = time.time()
    while time.time() - start < timeout:
        rclpy.spin_once(frame_listener, timeout_sec=0.1)
        if frame_listener.frames_received > 0:
            break

    assert frame_listener.frames_received > 0, f"Expected frames but got {frame_listener.frames_received}"