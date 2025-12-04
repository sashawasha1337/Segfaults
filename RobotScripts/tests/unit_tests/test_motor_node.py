import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'src')))

from unittest.mock import Mock, patch
import pytest
from geometry_msgs.msg import Twist

sys.modules["lgpio"] = Mock()

import rclpy

@pytest.fixture(scope="session", autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()

try:
    from motor_control.motor_control.MotorNode import MotorControlNode
    _motor_module = "motor_control.motor_control.MotorNode"
except Exception:
    from motor_control.MotorNode import MotorControlNode
    _motor_module = "motor_control.MotorNode"


@pytest.fixture
def motor_node():
    with patch(_motor_module + '.lgpio'):
        node = MotorControlNode()
        node.set_motor = Mock()
        yield node
        node.destroy_node()


class TestMotorSpeedCalculation:
    def test_forward_only(self, motor_node):
        msg = Twist()
        msg.linear.x = 0.5
        motor_node.cmd_callback(msg)
        motor_node.set_motor.assert_called_once_with(0.5, 0.5)

    def test_turn_right(self, motor_node):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.2
        motor_node.cmd_callback(msg)
        motor_node.set_motor.assert_called_once_with(0.3, 0.7)

    def test_turn_left(self, motor_node):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = -0.2
        motor_node.cmd_callback(msg)
        motor_node.set_motor.assert_called_once_with(0.7, 0.3)

    def test_backward(self, motor_node):
        msg = Twist()
        msg.linear.x = -0.4
        motor_node.cmd_callback(msg)
        motor_node.set_motor.assert_called_once_with(-0.4, -0.4)

    def test_spin_in_place(self, motor_node):
        msg = Twist()
        msg.angular.z = 0.6
        motor_node.cmd_callback(msg)
        motor_node.set_motor.assert_called_once_with(-0.6, 0.6)

    def test_clamp_left_speed(self, motor_node):
        msg = Twist()
        msg.linear.x = 0.8
        msg.angular.z = 0.5
        motor_node.cmd_callback(msg)
        motor_node.set_motor.assert_called_once_with(pytest.approx(0.3), 1.0)

    def test_clamp_right_speed(self, motor_node):
        msg = Twist()
        msg.linear.x = 0.9
        msg.angular.z = -0.4
        motor_node.cmd_callback(msg)
        motor_node.set_motor.assert_called_once_with(1.0, pytest.approx(0.5))

    def test_clamp_both_speeds(self, motor_node):
        msg = Twist()
        msg.linear.x = 1.5
        msg.angular.z = 0.8
        motor_node.cmd_callback(msg)
        motor_node.set_motor.assert_called_once_with(pytest.approx(0.7), 1.0)

    def test_stop(self, motor_node):
        msg = Twist()
        motor_node.cmd_callback(msg)
        motor_node.set_motor.assert_called_once_with(0.0, 0.0)
