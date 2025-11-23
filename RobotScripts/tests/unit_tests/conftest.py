import pytest
import sys
from unittest import mock
from pathlib import Path

# Add the parent directory to Python's path so it can find the camera package
camera_package_dir = Path(__file__).parent.parent.parent / "src"
sys.path.insert(0, str(camera_package_dir))

# Mock rclpy
@pytest.fixture(autouse=True)
def mock_rclpy():

    # Mock Node class
    class FakeNode:
        def __init__(self, name):
            self.name = name
            self.parameters = {}

        def get_logger(self):
            return mock.Mock(
                debug=mock.Mock(),
                info=mock.Mock(),
                warn =mock.Mock(),
                error=mock.Mock(),
                fatal=mock.Mock(),
            )
        
        def create_publisher(self, msg_type, topic, qos_profile):
            return mock.Mock()
        
        def create_timer(self, timer_period_sec, callback):
            return mock.Mock()
        
        def get_clock(self):
            fake_clock = mock.Mock()
            fake_clock.now = mock.Mock(return_value=mock.Mock(to_msg=mock.Mock()))
            return fake_clock
        
        def declare_parameter(self, name, value):
            self.parameters[name] = value

        def get_parameter(self, name):
            return mock.Mock(value=self.parameters.get(name, None))
        
        def destroy_node(self):
            pass
        
    # Mock cv bridge
    class FakeCvBridge:
        def cv2_to_imgmsg(self, frame, encoding="bgr8"):
            fake_msg = mock.Mock()
            fake_msg.header = mock.Mock()
            fake_msg.header.stamp = None
            return fake_msg
        
    

    # Add mocks to sys.modules
    fake_rclpy = mock.Mock()
    sys.modules['rclpy'] = fake_rclpy

    fake_rclpy_node = mock.Mock(Node=FakeNode)
    sys.modules['rclpy.node'] = fake_rclpy_node

    
    fake_qos = mock.Mock(
            QoSProfile=mock.Mock(),
            ReliabilityPolicy=mock.Mock(),
            HistoryPolicy=mock.Mock(),
        )
    sys.modules['rclpy.qos'] = fake_qos
    

    fake_sensor_msgs = mock.Mock()
    fake_sensor_msgs.msg = mock.Mock(Image=mock.Mock())
    sys.modules['sensor_msgs'] = fake_sensor_msgs
    sys.modules['sensor_msgs.msg'] = fake_sensor_msgs.msg

    fake_cv_bridge = mock.Mock(CvBridge=FakeCvBridge)
    sys.modules['cv_bridge'] = fake_cv_bridge




# Mock cv2
@pytest.fixture()
def mock_cv2():
    class FakeVideoCapture:
        def __init__(self, index, backend=None):
            self.index = index
            self.backend = backend
            self.opened = True

        def isOpened(self):
            return self.opened

        def read(self):
            # Return a fake frame (a simple black image)
            import numpy as np
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            return True, frame
        
        def set(self, prop_id, value):
            pass

        def get(self, prop_id):
            return 640.0

        def release(self):
            self.opened = False

    fake_cv2 = mock.Mock(
        VideoCapture=FakeVideoCapture,
        CAP_V4L2=200,
        CAP_PROP_FRAME_WIDTH=3,
        CAP_PROP_FRAME_HEIGHT=4,
        CAP_PROP_FPS=5,
    )
    sys.modules['cv2'] = fake_cv2

    yield

    # Cleanup sys.modules
    if 'camera.CameraNode' in sys.modules:
        del sys.modules['camera.CameraNode']
    if 'cv2' in sys.modules:
        del sys.modules['cv2']

# Mock cv2 that simulates camera won't open 
@pytest.fixture()
def mock_cv2_broken():
    class BrokenVideoCapture:
        def __init__(self, index, backend=None):
            self.index = index
            self.backend = backend

        def isOpened(self):
            return False

        def read(self):
            return False, None
        
        def set(self, prop_id, value):
            pass

        def get(self, prop_id):
            return 0.0

        def release(self):
            pass

    fake_cv2 = mock.Mock(
        VideoCapture=BrokenVideoCapture,
        CAP_V4L2=200,
        CAP_PROP_FRAME_WIDTH=3,
        CAP_PROP_FRAME_HEIGHT=4,
        CAP_PROP_FPS=5,
    )
    sys.modules['cv2'] = fake_cv2

    yield

    # Cleanup sys.modules
    if 'camera.CameraNode' in sys.modules:
        del sys.modules['camera.CameraNode']
    if 'cv2' in sys.modules:
        del sys.modules['cv2']

    