import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'src')))

from unittest.mock import Mock, MagicMock, patch
import numpy as np
import pytest
import time

sys.modules["networking.status"] = MagicMock()
sys.modules["networking.webrtcpeer"] = MagicMock()
sys.modules["flask"] = MagicMock()
sys.modules["flask_cors"] = MagicMock()
sys.modules["flask_socketio"] = MagicMock()
sys.modules["cv_bridge"] = MagicMock()
sys.modules["cv2"] = MagicMock()

import rclpy
from sensor_msgs.msg import Image

try:
    from camera.camera.CameraNode import CameraPublisher
    from networking.networking.NetworkNode import NetworkNode
    _camera_module = "camera.camera.CameraNode"
except Exception:
    from camera.CameraNode import CameraPublisher
    from networking.NetworkNode import NetworkNode
    _camera_module = "camera.CameraNode"

@pytest.fixture(scope="session", autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_camera_publishes_to_network_receives():
    """Integration test: camera frame flows from CameraPublisher to NetworkNode"""
    
    # === ARRANGE ===
    with patch(_camera_module + '.cv2') as mock_cv2:
        mock_capture = MagicMock()
        mock_cv2.VideoCapture.return_value = mock_capture
        mock_capture.isOpened.return_value = True
        
        # Create a realistic test frame (480x640x3 BGR)
        test_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        mock_capture.read.return_value = (True, test_frame)
        
        camera_node = CameraPublisher()
        network_node = NetworkNode()
        
        # Mock the bridge so frames convert properly
        camera_node.bridge = Mock()
        camera_node.bridge.cv2_to_imgmsg.side_effect = lambda frame, encoding: Image()
        
        network_node.bridge = Mock()
        network_node.bridge.imgmsg_to_cv2.return_value = test_frame
        
        # === ACT ===
        # Trigger camera to read and publish frame
        camera_node._publish_frame()
        
        # Spin both nodes to let ROS2 message pass through subscriptions
        for _ in range(10):
            rclpy.spin_once(camera_node, timeout_sec=0.01)
            rclpy.spin_once(network_node, timeout_sec=0.01)
        
        # === ASSERT ===
        # NetworkNode should have received and stored the frame
        assert network_node.latest_frame is not None, "NetworkNode did not receive frame from CameraPublisher"
        assert np.array_equal(network_node.latest_frame, test_frame), "Frame data mismatch between camera and network"
        assert network_node.latest_frame.shape == (480, 640, 3), "Frame dimensions incorrect"
        assert network_node.latest_frame.dtype == np.uint8, "Frame dtype incorrect"
        
        # === CLEANUP ===
        camera_node.destroy_node()
        network_node.destroy_node()


def test_camera_network_multiple_frames():
    """Integration test: multiple frames flow correctly, latest frame is stored"""
    
    # === ARRANGE ===
    with patch(_camera_module + '.cv2') as mock_cv2:
        mock_capture = MagicMock()
        mock_cv2.VideoCapture.return_value = mock_capture
        mock_capture.isOpened.return_value = True
        
        camera_node = CameraPublisher()
        network_node = NetworkNode()
        
        # Store frames to simulate bridge conversion
        last_frame = {"data": None}
        
        def fake_cv2_to_imgmsg(frame, encoding):
            last_frame["data"] = np.copy(frame)
            return Image()
        
        def fake_imgmsg_to_cv2(msg, encoding):
            return last_frame["data"]
        
        camera_node.bridge = Mock()
        camera_node.bridge.cv2_to_imgmsg.side_effect = fake_cv2_to_imgmsg
        
        network_node.bridge = Mock()
        network_node.bridge.imgmsg_to_cv2.side_effect = fake_imgmsg_to_cv2
        
        # === ACT ===
        frames = []
        for i in range(3):
            # Create unique frame for each iteration (filled with different value)
            frame = np.full((480, 640, 3), i * 50, dtype=np.uint8)
            frames.append(frame)
            mock_capture.read.return_value = (True, frame)
            
            # Publish frame
            camera_node._publish_frame()
            
            # Spin to let message pass
            for _ in range(5):
                rclpy.spin_once(camera_node, timeout_sec=0.01)
                rclpy.spin_once(network_node, timeout_sec=0.01)
        
        # === ASSERT ===
        # Latest frame should be the last one published (frame[2], all pixels = 100)
        assert network_node.latest_frame is not None, "No frame received"
        assert np.all(network_node.latest_frame == 100), "NetworkNode did not store the latest frame"
        
        # === CLEANUP ===
        camera_node.destroy_node()
        network_node.destroy_node()


def test_camera_network_frame_size_variations():
    """Integration test: different frame sizes flow correctly"""
    
    # === ARRANGE ===
    with patch(_camera_module + '.cv2') as mock_cv2:
        mock_capture = MagicMock()
        mock_cv2.VideoCapture.return_value = mock_capture
        mock_capture.isOpened.return_value = True
        
        camera_node = CameraPublisher()
        network_node = NetworkNode()
        
        last_frame = {"data": None}
        
        def fake_cv2_to_imgmsg(frame, encoding):
            last_frame["data"] = np.copy(frame)
            return Image()
        
        def fake_imgmsg_to_cv2(msg, encoding):
            return last_frame["data"]
        
        camera_node.bridge = Mock()
        camera_node.bridge.cv2_to_imgmsg.side_effect = fake_cv2_to_imgmsg
        
        network_node.bridge = Mock()
        network_node.bridge.imgmsg_to_cv2.side_effect = fake_imgmsg_to_cv2
        
        # === ACT & ASSERT ===
        test_sizes = [
            (480, 640, 3),
            (720, 1280, 3),
            (360, 480, 3),
        ]
        
        for height, width, channels in test_sizes:
            frame = np.random.randint(0, 255, (height, width, channels), dtype=np.uint8)
            mock_capture.read.return_value = (True, frame)
            
            camera_node._publish_frame()
            
            for _ in range(5):
                rclpy.spin_once(camera_node, timeout_sec=0.01)
                rclpy.spin_once(network_node, timeout_sec=0.01)
            
            assert network_node.latest_frame is not None, f"Frame not received for size {height}x{width}"
            assert network_node.latest_frame.shape == (height, width, channels), f"Shape mismatch for {height}x{width}"
        
        # === CLEANUP ===
        camera_node.destroy_node()
        network_node.destroy_node()


def test_camera_network_handles_camera_failure():
    """Integration test: NetworkNode gracefully handles camera publish failure"""
    
    # === ARRANGE ===
    with patch(_camera_module + '.cv2') as mock_cv2:
        mock_capture = MagicMock()
        mock_cv2.VideoCapture.return_value = mock_capture
        mock_capture.isOpened.return_value = True
        
        camera_node = CameraPublisher()
        network_node = NetworkNode()
        
        camera_node.bridge = Mock()
        camera_node.bridge.cv2_to_imgmsg.side_effect = Exception("Bridge failed")
        
        network_node.bridge = Mock()
        network_node.bridge.imgmsg_to_cv2.side_effect = lambda msg, encoding: msg
        
        # === ACT ===
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        mock_capture.read.return_value = (True, frame)
        
        # Camera will fail to publish due to bridge exception
        camera_node._publish_frame()
        
        # Spin nodes
        for _ in range(5):
            rclpy.spin_once(camera_node, timeout_sec=0.01)
            rclpy.spin_once(network_node, timeout_sec=0.01)
        
        # === ASSERT ===
        # NetworkNode should still be healthy (no exception raised)
        # latest_frame will be None because no message was published
        assert network_node.latest_frame is None, "NetworkNode should not have a frame"
        
        # But the nodes should still be functional
        assert network_node is not None
        
        # === CLEANUP ===
        camera_node.destroy_node()
        network_node.destroy_node()