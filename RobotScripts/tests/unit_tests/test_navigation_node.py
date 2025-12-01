import os
import sys
from unittest.mock import MagicMock, patch

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# Make src/ importable
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'src')))

# Stub nav2_msgs.action.NavigateToPose if not available 
from unittest.mock import MagicMock as _MM

if "nav2_msgs.action" not in sys.modules:
    nav2_action_mod = _MM()
    # Minimal dummy NavigateToPose with Goal inner class
    class _DummyGoal:
        def __init__(self):
            self.pose = PoseStamped()

    class _DummyNavigateToPose:
        Goal = _DummyGoal

    nav2_action_mod.NavigateToPose = _DummyNavigateToPose
    sys.modules["nav2_msgs"] = _MM()
    sys.modules["nav2_msgs.action"] = nav2_action_mod

# rclpy init/shutdown fixture (session-wide) 
@pytest.fixture(scope="session", autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()

# Import NavigationNode, supporting both layouts 
try:
    from navigation.navigation.NavigationNode import NavigationNode
    _nav_module = "navigation.navigation.NavigationNode"
except Exception:
    from navigation.NavigationNode import NavigationNode
    _nav_module = "navigation.NavigationNode"


# Fake ActionClient used to avoid real Nav2
class FakeGoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted

    def get_result_async(self):
        """Return a fake future whose callback immediately calls back with a result object."""
        future = MagicMock()

        def add_done_callback(cb):
            class _ResultFuture:
                def result(self_inner):
                    class _R:
                        # NavigationNode only logs this; could be anything
                        result = "OK"
                    return _R()
            cb(_ResultFuture())

        future.add_done_callback = add_done_callback
        return future


class FakeActionClient:
    """Synchronous fake ActionClient that records sent goals and calls callbacks immediately."""
    def __init__(self, node, action_type, name):
        self.node = node
        self.action_type = action_type
        self.name = name
        self.sent_goals = []

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal_msg):
        self.sent_goals.append(goal_msg)
        future = MagicMock()

        def add_done_callback(cb):
            # future.result() inside NavigationNode should return a goal_handle
            goal_handle = FakeGoalHandle(accepted=True)
            cb(MagicMock(result=lambda: goal_handle))

        future.add_done_callback = add_done_callback
        return future


# TESTS

def test_navigation_ignores_empty_path():
    """Empty Path should be ignored and not trigger any goals."""
    with patch(_nav_module + ".ActionClient", new=FakeActionClient):
        node = NavigationNode()

        # Pre-populate state to ensure it's not modified by empty path
        node.waypoints = [PoseStamped()]
        node.current_index = 42
        ac = node._action_client

        empty_path = Path()
        node.path_callback(empty_path)

        # No goals sent and state unchanged
        assert ac.sent_goals == []
        assert len(node.waypoints) == 1
        assert node.current_index == 42

        node.destroy_node()


def test_navigation_sends_one_goal_per_waypoint_and_advances():
    """
    When a Path with multiple waypoints is received, NavigationNode should:
    - send exactly one NavigateToPose goal per waypoint
    - advance current_index as each goal completes
    """
    with patch(_nav_module + ".ActionClient", new=FakeActionClient):
        node = NavigationNode()
        ac: FakeActionClient = node._action_client  # type: ignore

        # Build a Path with two waypoints
        path = Path()
        p1 = PoseStamped()
        p1.pose.position.x = 1.0
        p1.pose.position.y = 2.0
        path.poses.append(p1)

        p2 = PoseStamped()
        p2.pose.position.x = 3.0
        p2.pose.position.y = 4.0
        path.poses.append(p2)

        # ACT: deliver the path via the callback
        node.path_callback(path)

        # ASSERT: FakeActionClient should have one goal per waypoint
        assert len(ac.sent_goals) == 2, "Expected one goal per waypoint"

        coords = [
            (g.pose.pose.position.x, g.pose.pose.position.y)
            for g in ac.sent_goals
        ]
        assert coords == [(1.0, 2.0), (3.0, 4.0)], "Goal poses should match waypoints"

        # After both goals are 'completed', current_index should be 2 and goal_msg cleared
        assert node.current_index == 2
        assert node.goal_msg is None

        node.destroy_node()
