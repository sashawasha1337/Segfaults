# tests/unit_tests/test_camera_node.py
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'src')))

from unittest.mock import Mock, MagicMock, patch
import numpy as np
import pytest

from sensor_msgs.msg import Image

# Mock cv2 before importing CameraNode
sys.modules["cv2"] = MagicMock()

import rclpy

@pytest.fixture(scope="session", autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()

# Import the node under test; try both likely package layouts
try:
    from camera.camera.CameraNode import CameraPublisher
    _camera_module = "camera.camera.CameraNode"
except Exception:
    from camera.CameraNode import CameraPublisher
    _camera_module = "camera.CameraNode"

def test_camera_publishes_frames():
    """Test that CameraPublisher reads frames and publishes them."""
    
    # === ARRANGE ===
    # Mock cv2.VideoCapture to avoid actual camera access
    with patch(_camera_module + '.cv2') as mock_cv2:
        # Setup mock capture
        mock_capture = MagicMock()
        mock_cv2.VideoCapture.return_value = mock_capture
        mock_capture.isOpened.return_value = True
        
        # Mock frame reading: return (success, frame)
        frame_data = np.zeros((480, 640, 3), dtype=np.uint8)
        mock_capture.read.return_value = (True, frame_data)
        
        # Create the node
        camera_node = CameraPublisher()
        
        # Mock the publisher and bridge to avoid ROS/cv_bridge issues
        camera_node.pub = Mock()
        camera_node.bridge = Mock()
        camera_node.bridge.cv2_to_imgmsg.return_value = Image()
        
        # === ACT ===
        # Trigger frame publishing
        camera_node._publish_frame()
        
        # === ASSERT ===
        # Verify a frame was published
        assert camera_node.pub.publish.called, "Publisher did not publish a frame"
        
        # Verify bridge was used to convert frame
        camera_node.bridge.cv2_to_imgmsg.assert_called_once()
        call_args = camera_node.bridge.cv2_to_imgmsg.call_args[0]
        assert np.array_equal(call_args[0], frame_data), "Bridge should convert the actual frame"
        
        # === CLEANUP ===
        camera_node.destroy_node()

def test_camera_handles_frame_grab_failure():
    """Test that CameraPublisher handles frame grab failures gracefully."""
    
    # === ARRANGE ===
    with patch(_camera_module + '.cv2') as mock_cv2:
        mock_capture = MagicMock()
        mock_cv2.VideoCapture.return_value = mock_capture
        mock_capture.isOpened.return_value = True
        
        # Frame read fails
        mock_capture.read.return_value = (False, None)
        
        camera_node = CameraPublisher()
        camera_node.pub = Mock()
        
        # === ACT ===
        camera_node._publish_frame()
        
        # === ASSERT ===
        # Publisher should NOT be called when frame grab fails
        assert not camera_node.pub.publish.called, "Publisher should not publish on frame grab failure"
        
        # === CLEANUP ===
        camera_node.destroy_node()

def test_camera_converts_grayscale_to_bgr():
    """Test that CameraPublisher converts grayscale frames to BGR."""
    
    # === ARRANGE ===
    with patch(_camera_module + '.cv2') as mock_cv2:
        mock_capture = MagicMock()
        mock_cv2.VideoCapture.return_value = mock_capture
        mock_capture.isOpened.return_value = True
        
        # Return a grayscale frame (2D array)
        gray_frame = np.zeros((480, 640), dtype=np.uint8)
        mock_capture.read.return_value = (True, gray_frame)
        
        # Mock cv2.cvtColor to convert grayscale to BGR
        bgr_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        mock_cv2.cvtColor.return_value = bgr_frame
        mock_cv2.COLOR_GRAY2BGR = 6  # actual cv2 constant
        
        camera_node = CameraPublisher()
        camera_node.pub = Mock()
        camera_node.bridge = Mock()
        camera_node.bridge.cv2_to_imgmsg.return_value = Image()
        
        # === ACT ===
        camera_node._publish_frame()
        
        # === ASSERT ===
        # cvtColor should have been called for grayscale conversion
        mock_cv2.cvtColor.assert_called_once()
        call_args = mock_cv2.cvtColor.call_args[0]
        assert np.array_equal(call_args[0], gray_frame), "cvtColor called with original grayscale frame"
        
        # Frame should be published (converted version)
        assert camera_node.pub.publish.called, "Publisher should publish converted BGR frame"
        assert camera_node.bridge.cv2_to_imgmsg.called, "Bridge should convert the BGR frame"
        
        # === CLEANUP ===
        camera_node.destroy_node()
