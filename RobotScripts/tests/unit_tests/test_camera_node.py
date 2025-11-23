'''
Unit test for ROS2 Camera Node
'''

import pytest


def test_camera_node_initialization(mock_cv2):
    # Arrange
    import rclpy
    rclpy.init()
    from camera.camera.CameraNode import CameraPublisher

    try:
        # Act
        camera_node = CameraPublisher()

        assert camera_node.get_parameter("camera_index").value == 0
        assert camera_node.get_parameter("width").value == 640
        assert camera_node.get_parameter("height").value == 480
        assert camera_node.get_parameter("fps").value == 15.0
        assert camera_node.get_parameter('camera_backend').value == 'auto'
    finally:
        rclpy.shutdown()


def test_camera_node_camera_open_error(mock_cv2_broken):
    # Arrange
    import rclpy
    rclpy.init()
    from camera.camera.CameraNode import CameraPublisher


    # Act & Assert
    try:
        # Act & Assert
        with pytest.raises(ConnectionError, match="Camera open failed"):
            camera_node = CameraPublisher()
    finally:
        rclpy.shutdown()

def test_camera_releases_on_destroy(mock_cv2):
    # Arrange
    import rclpy
    rclpy.init()
    from camera.camera.CameraNode import CameraPublisher

    try:
        # Act
        camera_node = CameraPublisher()
        cap = camera_node.cap

        # Assert camera is opened
        assert cap.isOpened() is True

        # Destroy node
        camera_node.destroy_node()

        # Assert camera is released
        assert cap.isOpened() is False
    finally:
        rclpy.shutdown()


def test_process_frame_reshaping(mock_cv2):
    # Arrange
    import rclpy
    rclpy.init()
    from camera.camera.CameraNode import CameraPublisher
    import numpy as np

    try:
        camera_node = CameraPublisher()

        flattened = np.zeros((1, 640 * 480 * 3), dtype=np.uint8)

        # Act
        processed_frame = camera_node.process_frame(flattened, 640, 480)

        # Assert
        assert processed_frame.shape == (480, 640, 3)
    finally:
        rclpy.shutdown()