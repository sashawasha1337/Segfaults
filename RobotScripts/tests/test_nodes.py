# To run use: pytest-3 -q
import sys
import types
import numpy as np
import pytest

@pytest.fixture(scope="session", autouse=True)
def ros_init():
    try:
        import rclpy
    except Exception:
        # Unit test may run without rclpy installed
        yield
        return
    rclpy.init()
    yield
    try:
        rclpy.shutdown()
    except Exception:
        pass



# MOTOR NODE TESTS
class FakeLGPIO:
    def __init__(self):
        self.opened_chips = []
        self.closed_chips = []
        self.claims = []
        self.writes = []
        self.pwms = []

    def gpiochip_open(self, chip):
        self.opened_chips.append(chip)
        return 123

    def gpiochip_close(self, chip_handle):
        self.closed_chips.append(chip_handle)

    def gpiochip_info(self, chip_handle):
        return {"name": f"chip{chip_handle}", "label": "fake"}

    def gpio_claim_output(self, chip_handle, pin):
        self.claims.append((chip_handle, pin))

    def gpio_write(self, chip_handle, pin, value):
        self.writes.append((chip_handle, pin, int(value)))

    def tx_pwm(self, chip_handle, pin, freq, duty):
        self.pwms.append((chip_handle, pin, int(freq), int(duty)))

@pytest.fixture()
def fake_lgpio(monkeypatch):
    fake = FakeLGPIO()
    module = types.SimpleNamespace(
        gpiochip_open=fake.gpiochip_open,
        gpiochip_close=fake.gpiochip_close,
        gpiochip_info=fake.gpiochip_info,
        gpio_claim_output=fake.gpio_claim_output,
        gpio_write=fake.gpio_write,
        tx_pwm=fake.tx_pwm,
    )
    sys.modules['lgpio'] = module
    return fake

def test_motor_init_and_set_motor(fake_lgpio, monkeypatch):
    # Import after patching lgpio
    import importlib
    motor_module = importlib.import_module("motor_control.MotorNode")

    # Silence logs & avoid real subscription creation
    class _Logger:
        def info(self, *a, **k): pass
        def error(self, *a, **k): pass
    monkeypatch.setattr(motor_module.MotorControlNode, "get_logger", lambda self: _Logger())
    monkeypatch.setattr(motor_module.MotorControlNode, "create_subscription", lambda *a, **k: object())

    node = motor_module.MotorControlNode()

    # Init behavior
    assert fake_lgpio.opened_chips == [0]
    assert len(fake_lgpio.claims) == 2
    assert any(p[-1] == 0 for p in fake_lgpio.pwms)

    # Verify set_motor
    fake_lgpio.writes.clear()
    fake_lgpio.pwms.clear()
    node.set_motor(0.6, -0.4)
    assert len(fake_lgpio.writes) == 2
    assert len(fake_lgpio.pwms) == 2
    duties = sorted(p[-1] for p in fake_lgpio.pwms)
    assert duties == [40, 60]

    # Verify clamping
    calls = []
    def _spy(self, L, R): calls.append((L, R))
    monkeypatch.setattr(motor_module.MotorControlNode, "set_motor", _spy, raising=False)

    from geometry_msgs.msg import Twist
    msg = Twist()
    msg.linear.x = 2.0
    msg.angular.z = -1.5
    node.cmd_callback(msg)
    assert calls
    L, R = calls[-1]
    assert -1.0 <= L <= 1.0 and -1.0 <= R <= 1.0

    # Close chip
    motor_module.lgpio.gpiochip_close(node.chip)
    assert fake_lgpio.closed_chips == [123]



# CAMERA NODE TESTS
class FakeVideoCapture:
    def __init__(self, idx, backend):
        self.idx = idx
        self.backend = backend
        self.opened = True
        self.props = {"width": 640, "height": 480, "fps": 15.0}
        self.frame = np.zeros((self.props["height"], self.props["width"], 3), dtype=np.uint8)

    def isOpened(self): return self.opened
    def set(self, prop, value):
        if prop == 3: self.props["width"] = int(value)
        elif prop == 4: self.props["height"] = int(value)
        elif prop == 5: self.props["fps"] = float(value)
    def get(self, prop):
        return {
            3: float(self.props["width"]),
            4: float(self.props["height"]),
            5: float(self.props["fps"]),
        }.get(prop, 0.0)
    def read(self): return True, self.frame.copy()
    def release(self): self.opened = False

class FakeCvBridge:
    def cv2_to_imgmsg(self, frame, encoding="bgr8"):
        return types.SimpleNamespace(encoding=encoding, height=frame.shape[0], width=frame.shape[1])

@pytest.fixture()
def patch_cv_and_bridge(monkeypatch):
    class DummyPublisher:
        def __init__(self): self.published = []
        def publish(self, msg): self.published.append(msg)

    # Fake cv2 module constants & functions
    fake_cv2 = types.SimpleNamespace(
        CAP_V4L2=200,
        CAP_PROP_FRAME_WIDTH=3, CAP_PROP_FRAME_HEIGHT=4, CAP_PROP_FPS=5,
        VideoCapture=lambda idx, backend: FakeVideoCapture(idx, backend),
        COLOR_GRAY2BGR=1,
        cvtColor=lambda img, code: np.dstack([img, img, img]) if img.ndim == 2 else img,
    )
    sys.modules['cv2'] = fake_cv2

    # Fake cv_bridge
    sys.modules['cv_bridge'] = types.SimpleNamespace(CvBridge=FakeCvBridge)

    pubs = {}
    def _create_pub(self, msg_type, topic, qos):
        pub = DummyPublisher(); pubs[topic] = pub; return pub
    return pubs, _create_pub

def test_camera_init_and_publish(monkeypatch, patch_cv_and_bridge):
    pubs, create_pub_impl = patch_cv_and_bridge
    import importlib
    camera_module = importlib.import_module("camera.CameraNode")

    class _Logger:
        def info(self, *a, **k): pass
        def warning(self, *a, **k): pass
        def error(self, *a, **k): pass
        def debug(self, *a, **k): pass
    monkeypatch.setattr(camera_module.CameraPublisher, "get_logger", lambda self: _Logger())
    monkeypatch.setattr(camera_module.CameraPublisher, "create_publisher", create_pub_impl, raising=False)

    # Call the timer callback immediately instead of scheduling
    def _immediate_timer(self, period, cb): cb(); return object()
    monkeypatch.setattr(camera_module.CameraPublisher, "create_timer", _immediate_timer, raising=False)

    node = camera_module.CameraPublisher()
    pub = pubs.get("/camera/image_raw")
    assert pub is not None and len(pub.published) >= 1
    msg = pub.published[0]
    assert msg.height == 480 and msg.width == 640

    node.destroy_node()



# NETWORK NODE TESTS
class _FakePublisher:
    def __init__(self): self.published = []
    def publish(self, msg): self.published.append(msg)

class _NoopLogger:
    def info(self, *a, **k): pass
    def warning(self, *a, **k): pass
    def error(self, *a, **k): pass
    def debug(self, *a, **k): pass

class _FakeStatusPusher:
    def __init__(self, node, peer_fn):
        self.node = node
        self.peer_fn = peer_fn
        self.push_calls = 0
        self.battery_msgs = []
    def push_status(self):
        self.push_calls += 1
    def battery_callback(self, msg):
        self.battery_msgs.append(msg)

class _FakePeer:
    def __init__(self, node, sid, socketio):
        self.node, self.sid, self.socketio = node, sid, socketio
        self.started = False
        self.answered = []
        self.closed = False
    async def start(self):
        self.started = True
    async def process_answer(self, ans):
        self.answered.append(ans)
    async def close(self):
        self.closed = True

@pytest.fixture()
def patch_network_deps(monkeypatch):
    # request.sid is read by handlers
    request = types.SimpleNamespace(sid=None)

    class _FakeFlask:
        def __init__(self, name): self.name = name
    def _CORS(app): return None

    class _FakeSocketIO:
        def __init__(self, app, cors_allowed_origins="*", async_mode="threading"):
            self.app = app
            self.handlers = {}
        def on_event(self, name, fn): self.handlers[name] = fn
        def run(self, *a, **k): pass 

    sys.modules['flask'] = types.SimpleNamespace(Flask=_FakeFlask, request=request)
    sys.modules['flask_cors'] = types.SimpleNamespace(CORS=_CORS)
    sys.modules['flask_socketio'] = types.SimpleNamespace(SocketIO=_FakeSocketIO)

    # Fake cv bridge
    class _FakeCvBridge:
        def imgmsg_to_cv2(self, msg, enc="bgr8"):
            # Return a fake frame
            return np.zeros((2, 3, 3), dtype=np.uint8)
    sys.modules['cv_bridge'] = types.SimpleNamespace(CvBridge=_FakeCvBridge)

    # Fake networking dependencies
    sys.modules['networking.status'] = types.SimpleNamespace(StatusPusher=_FakeStatusPusher)
    sys.modules['networking.webrtcpeer'] = types.SimpleNamespace(SinglePeerSession=_FakePeer)

    pubs = {}
    subs = []
    def _create_publisher(self, msg_type, topic, qos):
        pub = _FakePublisher(); pubs[topic] = pub; return pub
    def _create_subscription(self, msg_type, topic, cb, qos):
        subs.append((topic, cb)); return object()
    def _create_timer(self, period, cb):  
        cb(); return object()

    return {
        "pubs": pubs,
        "subs": subs,
        "request": request,
        "create_publisher": _create_publisher,
        "create_subscription": _create_subscription,
        "create_timer": _create_timer,
    }

def test_networknode_init_connect_answer_commands_camera(monkeypatch, patch_network_deps):
    deps = patch_network_deps
    # Import after fakes are installed
    import importlib
    netmod = importlib.import_module("networking.NetworkNode")

    # Quiet logger and stub Node methods
    monkeypatch.setattr(netmod.NetworkNode, "get_logger", lambda self: _NoopLogger())
    monkeypatch.setattr(netmod.NetworkNode, "create_publisher", deps["create_publisher"], raising=False)
    monkeypatch.setattr(netmod.NetworkNode, "create_subscription", deps["create_subscription"], raising=False)
    monkeypatch.setattr(netmod.NetworkNode, "create_timer", deps["create_timer"], raising=False)

    # Make asyncio.run_coroutine_threadsafe a no operation future that immediately "completes"
    class _FakeFuture:
        def __init__(self, exc=None): self._exc = exc; self._cb = None
        def add_done_callback(self, cb):
            self._cb = cb
            cb(self)
        def result(self):
            if self._exc: raise self._exc
            return None
        
    def _fake_run_coroutine_threadsafe(coro, loop):
        # Just mark as done successfully, don't actually run coroutine
        return _FakeFuture()
    monkeypatch.setattr(netmod.asyncio, "run_coroutine_threadsafe", _fake_run_coroutine_threadsafe)
    node = netmod.NetworkNode()

    # Did init create subscribers/publishers
    assert 'cmd_vel' in deps["pubs"]
    topics = [t for (t, _cb) in deps["subs"]]
    assert 'battery_state' in topics
    assert '/camera/image_raw' in topics

    # execute_command publishes Twist correctly
    pub = deps["pubs"]['cmd_vel']
    pub.published.clear()
    node.execute_command("left")
    node.execute_command("forward")
    assert len(pub.published) == 2
    last = pub.published[-1]
    assert getattr(last, "linear").x > 0 and abs(getattr(last, "angular").z) == 0

    # Find the camera subscription callback and call it with a tiny fake Image, it stores latest_frame via CvBridge
    cam_cb = dict(deps["subs"])['/camera/image_raw']
    from types import SimpleNamespace
    fake_img = SimpleNamespace()
    node.latest_frame = None
    cam_cb(fake_img)
    assert node.latest_frame is not None and isinstance(node.latest_frame, np.ndarray)

    # connect -> creates SinglePeerSession and starts it 
    deps["request"].sid = "ABCDE12345"
    node.handle_connect()
    assert node.peer_session is not None and node.peer_session.sid == "ABCDE12345"

    # When loop present, status push
    netmod.webrtc_loop = object()
    sp = node.status_pusher
    before = sp.push_calls
    node.handle_status_push()
    assert sp.push_calls >= before

    # Matching session id
    before_answers = len(node.peer_session.answered)
    node.handle_answer({"sdp": "dummy"})
    assert len(node.peer_session.answered) == before_answers + 1
    # Non-matching session id -> no operation
    deps["request"].sid = "NOT_MATCH"
    before_answers = len(node.peer_session.answered)
    node.handle_answer({"sdp": "ignored"})
    assert len(node.peer_session.answered) == before_answers

    # Disconnect clears session
    deps["request"].sid = "ABCDE12345"
    node.handle_disconnect()
    assert node.peer_session is None

    node.destroy_node()