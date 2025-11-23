# tests/unit_tests/test_network_node.py
import os, sys
# Make project src importable for different test/run environments
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'src')))

import types
import time
import numpy as np
import pytest

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data

# Provide a small CvBridge shim so the test doesn't require the system package
class _FakeCvBridge:
    def imgmsg_to_cv2(self, msg, encoding="bgr8"):
        return np.zeros((64, 64, 3), dtype=np.uint8)
    def cv2_to_imgmsg(self, img, encoding="bgr8"):
        return Image()

# Inject lightweight fakes for modules the node imports at top-level
mod_status = types.ModuleType("networking.status")
class DummyStatusPusher:
    def __init__(self, node, peer_getter):
        self.node = node
    def battery_callback(self, msg):
        pass
    def push_status(self):
        pass
mod_status.StatusPusher = DummyStatusPusher
sys.modules["networking.status"] = mod_status

mod_peer = types.ModuleType("networking.webrtcpeer")
class DummySinglePeerSession:
    def __init__(self, node, sid, socketio):
        self.sid = sid
    async def start(self): pass
    async def process_answer(self, a): pass
    async def close(self): pass
mod_peer.SinglePeerSession = DummySinglePeerSession
sys.modules["networking.webrtcpeer"] = mod_peer

# Minimal Flask/CORS/SocketIO stubs to avoid side effects at import time
mod_flask = types.ModuleType("flask")
class _FakeFlask:
    def __init__(self, name): self.name = name
mod_flask.Flask = _FakeFlask
mod_flask.request = types.SimpleNamespace(sid=None)
sys.modules["flask"] = mod_flask

mod_flask_cors = types.ModuleType("flask_cors")
def _CORS(app): return None
mod_flask_cors.CORS = _CORS
sys.modules["flask_cors"] = mod_flask_cors

mod_flask_socketio = types.ModuleType("flask_socketio")
class _FakeSocketIO:
    def __init__(self, app, **kwargs): pass
    def on_event(self, *args, **kwargs): pass
mod_flask_socketio.SocketIO = _FakeSocketIO
sys.modules["flask_socketio"] = mod_flask_socketio

# Use the fake CvBridge unless a real one is already available
if "cv_bridge" not in sys.modules:
    sys.modules["cv_bridge"] = types.SimpleNamespace(CvBridge=_FakeCvBridge)

# Import the node under test; try both likely package layouts
try:
    from networking.networking.NetworkNode import NetworkNode
except Exception:
    from networking.NetworkNode import NetworkNode

@pytest.fixture(scope="session", autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()

def _spin_for(condition_fn, nodes, timeout=2.0, step=0.02):
    start = time.time()
    while time.time() - start < timeout:
        for n in nodes:
            try:
                rclpy.spin_once(n, timeout_sec=step)
            except Exception:
                pass
        if condition_fn():
            return True
        time.sleep(step)
    return False

def test_networknode_camera_and_cmdvel():
    # Instantiate node under test and a helper node
    network_node = NetworkNode()
    helper = Node("test_helper_node")

    # Publish a dummy Image to trigger camera callback
    pub = helper.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
    img_msg = Image()
    img_msg.header.stamp = helper.get_clock().now().to_msg()
    pub.publish(img_msg)

    # Wait for NetworkNode to process image and set latest_frame
    ok = _spin_for(lambda: getattr(network_node, "latest_frame", None) is not None, [network_node, helper], timeout=3.0)
    assert ok, "NetworkNode did not process camera image and set latest_frame"

    # Verify execute_command publishes Twist on cmd_vel
    received = {}
    def twist_cb(msg):
        received['msg'] = msg

    sub = helper.create_subscription(Twist, 'cmd_vel', twist_cb, 10)
    network_node.execute_command("forward")

    ok2 = _spin_for(lambda: 'msg' in received, [network_node, helper], timeout=1.5)
    assert ok2, "No Twist published to cmd_vel by execute_command"
    assert received['msg'].linear.x > 0.0

    # Cleanup
    helper.destroy_node()
    network_node.destroy_node()