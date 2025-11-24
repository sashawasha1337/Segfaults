import sys
import os
import pytest
from unittest.mock import MagicMock, patch
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'src')))

sys.modules.setdefault("firebase_admin", MagicMock())
sys.modules.setdefault("firebase_admin.credentials", MagicMock())
sys.modules.setdefault("firebase_admin.firestore", MagicMock())
sys.modules.setdefault("serial", MagicMock())

@pytest.fixture
def mock_firestore(monkeypatch):
    """Mock Firestore client to return fixed GPS waypoints."""
    mock_db = MagicMock()
    mock_collection = MagicMock()

    # Setup Firestore collection call chain
    mock_collection.order_by.return_value.limit.return_value.get.return_value = []
    mock_db.collection.return_value = mock_collection

    # Patch firestore.client() to return the mock
    monkeypatch.setattr("firebase_admin.firestore.client", lambda: mock_db)
    return mock_db

@pytest.fixture
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()

try:
    from navigation.navigation.FirestoreGPSListener import FirestoreGPSListener
    _fgps_module = "navigation.navigation.FirestoreGPSListener"
except Exception:
    from navigation.FirestoreGPSListener import FirestoreGPSListener
    _fgps_module = "navigation.FirestoreGPSListener"

# Test that FirestoreGPSListener publishes a Path message from Firestore waypoints.
def test_read_and_publish(mock_firestore, rclpy_init_shutdown):
    node = FirestoreGPSListener()

    node.db = mock_firestore

    # Set fake GPS origin for UTM conversion
    node.LAT0, node.LON0 = 37.7740, -122.4190
    node.X0, node.Y0 = node.TRANSFORMER.transform(node.LON0, node.LAT0)

    # Spy on publisher to capture published messages
    published_msgs = []
    def capture_publish(msg):
        published_msgs.append(msg)

    node.publisher.publish = capture_publish

    mock_doc = MagicMock()

    # Simulated Firestore document
    mock_doc.id = "doc123"
    # Two GPS waypoionts added for route
    mock_doc.to_dict.return_value = {
        "path": [
            {"lat": 37.7749, "lng": -122.4194},
            {"lat": 37.7750, "lng": -122.4180}
        ]
    }
    
    mock_firestore.collection.return_value.order_by.return_value.limit.return_value.get.return_value = [mock_doc]

    # Call the method that reads Firestore and publishes Path
    node.read_and_publish()

    # Assertions
    assert len(published_msgs) == 1
    path_msg = published_msgs[0]
    assert isinstance(path_msg, Path)
    assert path_msg.header.frame_id == "map"
    assert len(path_msg.poses) == 2 # Two gps waypoint PoseStamped messages are added to Path

    for pose in path_msg.poses:
        assert isinstance(pose, PoseStamped)
        assert pose.header.frame_id == "map"
        # Check that coordinates are floats
        assert isinstance(pose.pose.position.x, float)
        assert isinstance(pose.pose.position.y, float)
        assert isinstance(pose.pose.position.z, float)
        # Check that orientation are floats
        assert isinstance(pose.pose.orientation.z, float)
        assert isinstance(pose.pose.orientation.w, float)

    invalid_doc = MagicMock()
    
    invalid_doc.id = "doc_invalid"
    invalid_doc.to_dict.return_value = {
        "dummy": [
            {"lat": 11.1111, "lng": -111.1111},
            {"lat": 22.2222, "lng": -122.2222}
        ]
    }
    
    mock_firestore.collection.return_value.order_by.return_value.limit.return_value.get.return_value = [invalid_doc]
    
    node.read_and_publish()
    
    assert len(published_msgs) == 1
    assert len(published_msgs[0].poses) == 2

    node.destroy_node()