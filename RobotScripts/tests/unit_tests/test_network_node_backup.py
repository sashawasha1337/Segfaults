# tests/unit_tests/test_network_node.py
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'src')))

from unittest.mock import Mock, MagicMock
import numpy as np
import pytest

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# Mock all heavy dependencies before importing NetworkNode
sys.modules["networking.status"] = MagicMock()
sys.modules["networking.webrtcpeer"] = MagicMock()
sys.modules["flask"] = MagicMock()
sys.modules["flask_cors"] = MagicMock()
sys.modules["flask_socketio"] = MagicMock()
sys.modules["cv_bridge"] = MagicMock()

# Import the node under test; try both likely package layouts
try:
    from networking.networking.NetworkNode import NetworkNode
except Exception:
    from networking.NetworkNode import NetworkNode

import rclpy

@pytest.fixture(scope="session", autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_networknode_camera():    
    # === ARRANGE ===
    network_node = NetworkNode()
    
    # Mock the bridge
    network_node.bridge = Mock()
    network_node.bridge.imgmsg_to_cv2.return_value = np.zeros((64, 64, 3), dtype=np.uint8)
    
    # === ACT ===
    img_msg = Image()
    network_node.camera_callback(img_msg)
    
    # === ASSERT ===
    assert network_node.latest_frame is not None, "Camera callback did not set latest_frame"
    network_node.bridge.imgmsg_to_cv2.assert_called_once_with(img_msg, "bgr8")

def test_networknode_cmdvel():
    # === ARRANGE ===
    network_node = NetworkNode()
    network_node.cmd_vel_publisher = Mock()

    # === ACT & ASSERT: Command execution ===
    network_node.execute_command("forward")
    
    # Verify publish was called
    network_node.cmd_vel_publisher.publish.assert_called_once()
    
    # Get the Twist message that was published
    published_twist = network_node.cmd_vel_publisher.publish.call_args[0][0]
    assert published_twist.linear.x > 0.0, "Forward command should have positive linear.x"
    assert published_twist.linear.x == 0.2, "Forward should be 0.2 m/s"

    # === ACT & ASSERT: Other commands ===
    network_node.cmd_vel_publisher.reset_mock()
    network_node.execute_command("back")
    back_twist = network_node.cmd_vel_publisher.publish.call_args[0][0]
    assert back_twist.linear.x == -0.2, "Back command should be -0.2 m/s"

    network_node.cmd_vel_publisher.reset_mock()
    network_node.execute_command("left")
    left_twist = network_node.cmd_vel_publisher.publish.call_args[0][0]
    assert left_twist.angular.z == 0.5, "Left command should have positive angular.z"

    network_node.cmd_vel_publisher.reset_mock()
    network_node.execute_command("right")
    right_twist = network_node.cmd_vel_publisher.publish.call_args[0][0]
    assert right_twist.angular.z == -0.5, "Right command should have negative angular.z"
