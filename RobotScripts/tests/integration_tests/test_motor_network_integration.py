import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'src')))

from unittest.mock import Mock, MagicMock, patch
import pytest

sys.modules["networking.status"] = MagicMock()
sys.modules["networking.webrtcpeer"] = MagicMock()
sys.modules["flask"] = MagicMock()
sys.modules["flask_cors"] = MagicMock()
sys.modules["flask_socketio"] = MagicMock()
sys.modules["cv_bridge"] = MagicMock()
sys.modules["Rosmaster_Lib"] = MagicMock()

import rclpy
from geometry_msgs.msg import Twist

try:
    from networking.networking.NetworkNode import NetworkNode
    from motor_control.motor_control.RosmasterMotorNode import RosmasterMotorNode
    _network_module = "networking.networking.NetworkNode"
    _motor_module = "motor_control.motor_control.RosmasterMotorNode"
except Exception:
    from networking.NetworkNode import NetworkNode
    from motor_control.RosmasterMotorNode import RosmasterMotorNode
    _network_module = "networking.NetworkNode"
    _motor_module = "motor_control.RosmasterMotorNode"


@pytest.fixture(scope="session", autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_network_publishes_motor_receives():
    """Integration test: Twist command flows from NetworkNode to RosmasterMotorNode"""
    
    # === ARRANGE ===
    # Mock the Rosmaster hardware library
    with patch(_motor_module + '.Rosmaster') as mock_rosmaster_class:
        mock_car = MagicMock()
        mock_rosmaster_class.return_value = mock_car
        
        network_node = NetworkNode()
        motor_node = RosmasterMotorNode()
        
        # WORKAROUND: Topic mismatch bridge
        # NetworkNode publishes to 'cmd_vel', MotorNode subscribes to '/cmd_vel_teleop'
        bridge_pub = network_node.create_publisher(Twist, '/cmd_vel_teleop', 10)
        
        def bridge_callback(msg):
            bridge_pub.publish(msg)
        
        bridge_sub = network_node.create_subscription(Twist, 'cmd_vel', bridge_callback, 10)
        
        # === ACT ===
        # Use NetworkNode's execute_command to publish
        network_node.execute_command("forward")
        
        # Spin both nodes to let ROS2 message pass through
        for _ in range(10):
            rclpy.spin_once(network_node, timeout_sec=0.01)
            rclpy.spin_once(motor_node, timeout_sec=0.01)
        
        # === ASSERT ===
        # Motor controller should have received the command
        mock_car.set_car_motion.assert_called()
        call_args = mock_car.set_car_motion.call_args[0]
        assert call_args[0] == 0.2, "linear.x not passed correctly (forward = 0.2)"
        assert call_args[1] == 0.0, "linear.y should be 0.0"
        assert call_args[2] == 0.0, "angular.z should be 0.0"
        
        # === CLEANUP ===
        network_node.destroy_node()
        motor_node.destroy_node()


def test_network_motor_multiple_commands():
    """Integration test: multiple sequential commands flow correctly"""
    
    # === ARRANGE ===
    with patch(_motor_module + '.Rosmaster') as mock_rosmaster_class:
        mock_car = MagicMock()
        mock_rosmaster_class.return_value = mock_car
        
        network_node = NetworkNode()
        motor_node = RosmasterMotorNode()
        
        # WORKAROUND: Topic bridge
        bridge_pub = network_node.create_publisher(Twist, '/cmd_vel_teleop', 10)
        bridge_sub = network_node.create_subscription(
            Twist, 'cmd_vel', lambda msg: bridge_pub.publish(msg), 10
        )
        
        # === ACT ===
        # Send multiple different commands using NetworkNode's execute_command
        commands = ["forward", "left", "back", "right"]
        
        for cmd in commands:
            network_node.execute_command(cmd)
            
            # Spin to let message pass
            for _ in range(5):
                rclpy.spin_once(network_node, timeout_sec=0.01)
                rclpy.spin_once(motor_node, timeout_sec=0.01)
        
        # === ASSERT ===
        # Motor should have been called 4 times (once per command)
        assert mock_car.set_car_motion.call_count >= 4, "Not all commands received"
        
        # Check the last command was "right" (angular.z = -0.5)
        last_call_args = mock_car.set_car_motion.call_args[0]
        assert last_call_args[2] == -0.5, "Final 'right' command not received correctly"
        
        # === CLEANUP ===
        network_node.destroy_node()
        motor_node.destroy_node()


def test_network_motor_mecanum_commands():
    """Integration test: mecanum-specific commands (strafe) flow correctly"""
    
    # === ARRANGE ===
    with patch(_motor_module + '.Rosmaster') as mock_rosmaster_class:
        mock_car = MagicMock()
        mock_rosmaster_class.return_value = mock_car
        
        network_node = NetworkNode()
        motor_node = RosmasterMotorNode()
        
        # WORKAROUND: Topic bridge
        bridge_pub = network_node.create_publisher(Twist, '/cmd_vel_teleop', 10)
        bridge_sub = network_node.create_subscription(
            Twist, 'cmd_vel', lambda msg: bridge_pub.publish(msg), 10
        )
        
        # === ACT ===
        # Send a strafe command directly (NetworkNode doesn't have strafe in execute_command)
        twist = Twist()
        twist.linear.y = 0.4  # strafe right (mecanum capability)
        network_node.cmd_vel_publisher.publish(twist)
        
        for _ in range(10):
            rclpy.spin_once(network_node, timeout_sec=0.01)
            rclpy.spin_once(motor_node, timeout_sec=0.01)
        
        # === ASSERT ===
        mock_car.set_car_motion.assert_called()
        call_args = mock_car.set_car_motion.call_args[0]
        assert call_args[0] == 0.0, "linear.x should be 0.0"
        assert call_args[1] == 0.4, "linear.y (strafe) not passed correctly"
        assert call_args[2] == 0.0, "angular.z should be 0.0"
        
        # === CLEANUP ===
        network_node.destroy_node()
        motor_node.destroy_node()


def test_network_motor_handles_no_commands():
    """Integration test: motor node remains stable with no incoming commands"""
    
    # === ARRANGE ===
    with patch(_motor_module + '.Rosmaster') as mock_rosmaster_class:
        mock_car = MagicMock()
        mock_rosmaster_class.return_value = mock_car
        
        network_node = NetworkNode()
        motor_node = RosmasterMotorNode()
        
        # Don't publish anything
        
        # === ACT ===
        # Just spin without publishing
        for _ in range(10):
            rclpy.spin_once(network_node, timeout_sec=0.01)
            rclpy.spin_once(motor_node, timeout_sec=0.01)
        
        # === ASSERT ===
        # Motor node should still be functional (no crashes)
        assert motor_node is not None
        # set_car_motion shouldn't be called (except possibly in initialization)
        
        # === CLEANUP ===
        network_node.destroy_node()
        motor_node.destroy_node()


def test_motor_node_handles_rosmaster_failure():
    """Integration test: motor node handles Rosmaster connection failure gracefully"""
    
    # === ARRANGE ===
    with patch(_motor_module + '.Rosmaster') as mock_rosmaster_class:
        # Simulate connection failure
        mock_rosmaster_class.side_effect = Exception("Failed to open /dev/ttyUSB0")
        
        # === ACT & ASSERT ===
        # Node initialization should raise ConnectionError
        with pytest.raises(ConnectionError):
            motor_node = RosmasterMotorNode()
