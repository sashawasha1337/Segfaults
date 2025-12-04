import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'src')))

from unittest.mock import Mock, MagicMock, patch
import numpy as np
import pytest

from sensor_msgs.msg import Image, BatteryState
from geometry_msgs.msg import Twist

sys.modules["networking.status"] = MagicMock()
sys.modules["networking.webrtcpeer"] = MagicMock()
sys.modules["flask"] = MagicMock()
sys.modules["flask_cors"] = MagicMock()
sys.modules["flask_socketio"] = MagicMock()
sys.modules["cv_bridge"] = MagicMock()

try:
    from networking.networking.NetworkNode import NetworkNode
    from networking.networking.status import StatusPusher
    _net_module = "networking.networking.NetworkNode"
    _status_module = "networking.networking.status"
except Exception:
    from networking.NetworkNode import NetworkNode
    from networking.status import StatusPusher
    _net_module = "networking.NetworkNode"
    _status_module = "networking.status"

import rclpy

@pytest.fixture(scope="session", autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


class TestNetworkNodeCommands:
    def test_execute_command_forward(self):
        node = NetworkNode()
        node.cmd_vel_publisher = Mock()
        
        node.execute_command("forward")
        
        twist = node.cmd_vel_publisher.publish.call_args[0][0]
        assert twist.linear.x == 0.2

    def test_execute_command_back(self):
        node = NetworkNode()
        node.cmd_vel_publisher = Mock()
        
        node.execute_command("back")
        
        twist = node.cmd_vel_publisher.publish.call_args[0][0]
        assert twist.linear.x == -0.2

    def test_execute_command_left(self):
        node = NetworkNode()
        node.cmd_vel_publisher = Mock()
        
        node.execute_command("left")
        
        twist = node.cmd_vel_publisher.publish.call_args[0][0]
        assert twist.angular.z == 0.5

    def test_execute_command_right(self):
        node = NetworkNode()
        node.cmd_vel_publisher = Mock()
        
        node.execute_command("right")
        
        twist = node.cmd_vel_publisher.publish.call_args[0][0]
        assert twist.angular.z == -0.5

    def test_execute_command_unknown_publishes_zero_twist(self):
        node = NetworkNode()
        node.cmd_vel_publisher = Mock()
        
        node.execute_command("invalid_command")
        
        twist = node.cmd_vel_publisher.publish.call_args[0][0]
        assert twist.linear.x == 0.0
        assert twist.linear.y == 0.0
        assert twist.angular.z == 0.0


class TestNetworkNodeCameraCallback:
    def test_camera_callback_stores_frame(self):
        node = NetworkNode()
        node.bridge = Mock()
        frame_data = np.zeros((480, 640, 3), dtype=np.uint8)
        node.bridge.imgmsg_to_cv2.return_value = frame_data
        
        msg = Image()
        node.camera_callback(msg)
        
        assert node.latest_frame is not None
        assert np.array_equal(node.latest_frame, frame_data)

    def test_camera_callback_error_handling(self):
        node = NetworkNode()
        node.bridge = Mock()
        node.bridge.imgmsg_to_cv2.side_effect = Exception("Bridge conversion failed")
        
        msg = Image()
        node.camera_callback(msg)
        
        assert node.latest_frame is None


class TestStatusPusher:
    def test_battery_callback_stores_voltage(self):
        node = NetworkNode()
        pusher = StatusPusher(node, lambda: None)
        
        battery_msg = BatteryState()
        battery_msg.voltage = 11.5
        
        pusher.battery_callback(battery_msg)
        
        assert node.battery_voltage == 11.5

    def test_wifi_strength_parsing_valid(self):
        pusher = StatusPusher(MagicMock(), lambda: None)
        
        iwconfig_output = """wlan0     IEEE 802.11bgn  ESSID:"MyNetwork"
                    Mode:Managed  Frequency:2.437 GHz  Access Point: AA:BB:CC:DD:EE:FF
                    Tx-Power=20 dBm
                    Signal level=-45 dBm
                    Link Quality=70/70  Signal level=-45 dBm"""
        
        with patch(_status_module + '.subprocess.run') as mock_run:
            mock_run.return_value = MagicMock(stdout=iwconfig_output, stderr="")
            strength = pusher.get_wifi_strength()
            assert strength == -45

    def test_wifi_strength_parsing_no_match(self):
        pusher = StatusPusher(MagicMock(), lambda: None)
        
        iwconfig_output = "wlan0     No WiFi signal found"
        
        with patch(_status_module + '.subprocess.run') as mock_run:
            mock_run.return_value = MagicMock(stdout=iwconfig_output, stderr="")
            strength = pusher.get_wifi_strength()
            assert strength is None

    def test_wifi_strength_subprocess_error(self):
        node = MagicMock()
        pusher = StatusPusher(node, lambda: None)
        
        with patch(_status_module + '.subprocess.run') as mock_run:
            mock_run.side_effect = Exception("iwconfig not found")
            strength = pusher.get_wifi_strength()
            assert strength is None
            node.get_logger().error.assert_called_once()

    def test_push_status_no_peer_session(self):
        node = MagicMock()
        pusher = StatusPusher(node, lambda: None)
        node.battery_voltage = 11.5
        
        pusher.push_status()
        
        node.get_logger().error.assert_called_once()
        assert "No active peer session" in node.get_logger().error.call_args[0][0]
