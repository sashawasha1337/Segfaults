# tests/unit_tests/test_network_node_ros.py
import os
import sys
import time
import types
import numpy as np
import pytest

# Ensure project src is importable
ROOT_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'src'))
sys.path.insert(0, ROOT_SRC)

# Inject lightweight fakes for modules that may not be available in test environment
# networking.status and networking.webrtcpeer are used by NetworkNode
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

# Minimal flask + cors + socketio stubs to avoid side effects at import time
mod_flask = types.ModuleType("flask")
class FakeFlask:
    def __init__(self, name): self.name = name
mod_flask.Flask = FakeFlask
mod_flask.request = types.SimpleNamespace(sid=None)
sys.modules["flask"] = mod_flask

mod_flask_cors = types.ModuleType("flask_cors")
def CORS(app): return None
mod_flask_cors.CORS = CORS
sys.modules["flask_cors"] = mod_flask_cors

mod_flask_socketio = types.ModuleType("flask_socketio")
class FakeSocketIO:
    def __init__(self, app, **kwargs): pass
    def on_event(self, *args, **kwargs): pass
mod_flask_socketio.SocketIO = FakeSocketIO
sys.modules["flask_socketio"] = mod_flask_socketio

# Minimal cv_bridge stub to convert Image -> numpy array
mod_cv_bridge = types.ModuleType("cv_bridge")
class FakeCvBridge:
    def imgmsg_to_cv2(self, msg, encoding="bgr8"):
        # return a simple numpy image regardless of msg contents
        return np.zeros((64, 64, 3), dtype=np.uint8)
    def cv2_to_imgmsg(self, img, encoding="bgr8"):
        from sensor_msgs.msg import Image
        return Image()
mod_cv_bridge.CvBridge = FakeCvBridge
sys.modules["cv_bridge"] = mod_cv_bridge

# Now import ROS and the node under test
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge  # our fake

# Import the NetworkNode class (module path matches repo layout)
from networking.networking.NetworkNode import NetworkNode

@pytest.fixture(scope="session", autouse=True)
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()

def spin_for(condition_fn, nodes, timeout=2.0, step=0.02):
    """Spin nodes until condition_fn() is True or timeout expires."""
    start = time.time()
    while time.time() - start < timeout:
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=step)
        if condition_fn():
            return True
        time.sleep(step)
    return False

def test_networknode_camera_and_cmdvel():
    # Create instance of NetworkNode
    network_node = NetworkNode()

    # Helper node to publish image and subscribe to cmd_vel
    helper = Node("test_helper_node")

    # Publish a dummy Image
    pub = helper.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
    img_msg = Image()
    img_msg.header.stamp = helper.get_clock().now().to_msg()
    pub.publish(img_msg)

    # Spin both nodes until NetworkNode.latest_frame is set
    ok = spin_for(lambda: getattr(network_node, "latest_frame", None) is not None, [network_node, helper], timeout=3.0)
    assert ok, "NetworkNode did not process camera image and set latest_frame"

    # Now verify execute_command publishes Twist on cmd_vel
    received = {}
    def twist_cb(msg):
        received['msg'] = msg

    sub = helper.create_subscription(Twist, 'cmd_vel', twist_cb, 10)

    # Call execute_command
    network_node.execute_command("forward")

    ok2 = spin_for(lambda: 'msg' in received, [network_node, helper], timeout=1.5)
    assert ok2, "No Twist published to cmd_vel by execute_command"
    assert received['msg'].linear.x > 0.0

    # cleanup
    helper.destroy_node()
    network_node.destroy_node()
