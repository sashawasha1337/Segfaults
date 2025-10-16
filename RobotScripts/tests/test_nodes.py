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


# Motor node tests
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


# Camera node tests
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