import os
import sys
from unittest.mock import MagicMock, patch

import numpy as np  # not strictly needed but nice for future checks
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String
from rclpy.node import Node

# Make src/ importable like in camera/network integration test
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'src')))

# Stub external dependencies to keep the test lightweight 
sys.modules.setdefault("firebase_admin", MagicMock())
sys.modules.setdefault("firebase_admin.credentials", MagicMock())
sys.modules.setdefault("firebase_admin.firestore", MagicMock())
sys.modules.setdefault("serial", MagicMock())
sys.modules.setdefault("pynmea2", MagicMock())

# Stub nav2_msgs.action.NavigateToPose if not available
if "nav2_msgs.action" not in sys.modules:
    nav2_action_mod = MagicMock()

    class _DummyGoal:
        def __init__(self):
            self.pose = PoseStamped()

    class _DummyNavigateToPose:
        Goal = _DummyGoal

    nav2_action_mod.NavigateToPose = _DummyNavigateToPose
    sys.modules["nav2_msgs"] = MagicMock()
    sys.modules["nav2_msgs.action"] = nav2_action_mod

# rclpy init/shutdown fixture (session-wide) 
@pytest.fixture(scope="session", autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()

# Import NavigationNode (support both layouts) 
try:
    from navigation.navigation.NavigationNode import NavigationNode
    _nav_module = "navigation.navigation.NavigationNode"
except Exception:
    from navigation.NavigationNode import NavigationNode
    _nav_module = "navigation.NavigationNode"


# Fake ActionClient that simulates Nav2 and publishes cmd_vel + telemetry 
class FakeGoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted

    def get_result_async(self):
        future = MagicMock()

        def add_done_callback(cb):
            class _ResultFuture:
                def result(self_inner):
                    class _R:
                        result = "OK"
                    return _R()
            cb(_ResultFuture())

        future.add_done_callback = add_done_callback
        return future


class FakeActionClient:
    """
    Fake ActionClient used by NavigationNode.
    When a goal is sent:
      - record it
      - publish a cmd_vel Twist (to simulate Nav2 driving the robot)
      - publish a simple telemetry String
      - immediately call goal/ result callbacks
    """
    def __init__(self, node: Node, action_type, name: str):
        self.node = node
        self.action_type = action_type
        self.name = name
        self.sent_goals = []

        # Publishers to simulate Nav2 output
        self.cmd_vel_pub = node.create_publisher(Twist, "cmd_vel", 10)
        self.telemetry_pub = node.create_publisher(String, "nav_telemetry", 10)

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal_msg):
        self.sent_goals.append(goal_msg)

        # Simulate Nav2 generating a cmd_vel
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

        # Simulate Nav2 publishing some telemetry string
        status = String()
        status.data = "goal_sent"
        self.telemetry_pub.publish(status)

        # Now emulate the async goal_response_callback / get_result_callback chain
        future = MagicMock()

        def add_done_callback(cb):
            goal_handle = FakeGoalHandle(accepted=True)
            cb(MagicMock(result=lambda: goal_handle))

        future.add_done_callback = add_done_callback
        return future


# Simple fake Motor + Network nodes that subscribe to above topics 
class FakeMotorNode(Node):
    def __init__(self):
        super().__init__("fake_motor_node")
        self.last_cmd = None
        self.create_subscription(Twist, "cmd_vel", self._cmd_cb, 10)

    def _cmd_cb(self, msg: Twist):
        self.last_cmd = msg


class FakeNetworkNode(Node):
    def __init__(self):
        super().__init__("fake_network_node")
        self.last_telemetry = None
        self.create_subscription(String, "nav_telemetry", self._telemetry_cb, 10)

    def _telemetry_cb(self, msg: String):
        self.last_telemetry = msg


# INTEGRATION TEST

def test_navigation_pipeline_sends_goal_and_drives_motor_and_telemetry():
    """
    Integration-style test:

    - A Path with one waypoint is fed to NavigationNode (standing in for 'GPS->UTM->Path').
    - NavigationNode uses FakeActionClient (mock Nav2) to send a NavigateToPose goal.
    - FakeActionClient publishes cmd_vel and telemetry.
    - FakeMotorNode receives cmd_vel.
    - FakeNetworkNode receives telemetry.
    """

    with patch(_nav_module + ".ActionClient", new=FakeActionClient):
        nav_node = NavigationNode()
        motor_node = FakeMotorNode()
        net_node = FakeNetworkNode()

        # Build a single-waypoint Path
        path = Path()
        p = PoseStamped()
        p.pose.position.x = 10.0
        p.pose.position.y = 20.0
        path.poses.append(p)

        # Feed the path into NavigationNode
        nav_node.path_callback(path)

        # Spin all three nodes for a short while to let messages flow
        for _ in range(20):
            rclpy.spin_once(nav_node, timeout_sec=0.01)
            rclpy.spin_once(motor_node, timeout_sec=0.01)
            rclpy.spin_once(net_node, timeout_sec=0.01)

        ac: FakeActionClient = nav_node._action_client  # type: ignore

        # NavigationNode should have sent exactly one goal
        assert len(ac.sent_goals) == 1, "NavigationNode should send one NavigateToPose goal"
        goal_pose = ac.sent_goals[0].pose.pose.position
        assert goal_pose.x == pytest.approx(10.0)
        assert goal_pose.y == pytest.approx(20.0)

        # Motor node should have received cmd_vel
        assert motor_node.last_cmd is not None, "FakeMotorNode did not receive cmd_vel"
        assert motor_node.last_cmd.linear.x > 0

        # Network node should have received telemetry
        assert net_node.last_telemetry is not None, "FakeNetworkNode did not receive nav telemetry"
        assert net_node.last_telemetry.data == "goal_sent"

        nav_node.destroy_node()
        motor_node.destroy_node()
        net_node.destroy_node()
