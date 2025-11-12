#!/usr/bin/env python3
import sys
import numpy as np
#fix for model version mismatches?
sys.modules.setdefault('numpy._core', np.core)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import importlib.util

import cv2
import json
import base64
import os
import torch
from pathlib import Path
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "sort"))
from sort import Sort 



FIREBASE_STORAGE_BUCKET = os.environ.get("FIREBASE_STORAGE_BUCKET", "segfaults-database.firebasestorage.app")
SERVICE_ACCOUNT_FILE = "robot-service-account.json"
ROBOT_ID = os.environ.get("ROBOT_ID") or os.uname().nodename
ROBOT_UID = os.environ.get("ROBOT_UID") or ROBOT_ID
COLLECTION_PATH = "events"
PUBLISH_DEBUG_TOPIC = True


def bgr_to_jpeg_bytes(img_bgr, quality=90):
    ok, buf = cv2.imencode(".jpg", img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError("cv2.imencode failed")
    return base64.b64encode(buf.tobytes()).decode("ascii")


def ros_time_to_seconds(ros_time):
    return ros_time.sec + ros_time.nanosec * 1e-9

#intersection over union math
#dont ask me this is some stuff also pulled offline
def iou_xyxy(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    iw = max(0.0, inter_x2 - inter_x1)
    ih = max(0.0, inter_y2 - inter_y1)
    inter = iw * ih
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter + 1e-9
    return inter / union


class YoloV7Detector:
    def __init__(self, weights_path: str, device: str = "cuda:0",
                 img_size: int = 640, conf_thres: float = 0.3, logger=None):
        self.log = logger or (lambda *a, **k: None)
        self.img_size = int(img_size)
        self.conf_thres = float(conf_thres)

        # Device
        self.device = torch.device(device if (torch.cuda.is_available() and device.startswith("cuda")) else "cpu")
        self.log.info(f"YOLO: torch={torch.__version__} cuda_avail={torch.cuda.is_available()} device={self.device}")

        # Weight path must exist
        wp = str(Path(weights_path).expanduser())
        if not os.path.isfile(wp):
            raise FileNotFoundError(f"weights not found: {wp}")

        # Put yolov7 repo on path
        yolov7_root = "/ros2_ws/yolov7"
        if yolov7_root not in sys.path:
            sys.path.insert(0, yolov7_root)

        # Monkey-patch attempt_download to avoid calling `git tag`
        from utils import google_utils as ggu
        ggu.attempt_download = lambda f, *a, **k: f

        try:
            foundLib=importlib.util.find_spec("models.common")
            if foundLib is not None:
                import models.common
                models.common.Conv.fuse = lambda self, *a, **k: self
            foundLib= importlib.util.find_spec("models.yolo")
            if foundLib is not None:
                import models.yolo
                models.yolo.Detect.fuse = lambda self, *a, **k: self
        except Exception as e:
            self.log.info(f"model fuse dodge failed:{e}")

      

        # Load model
        from models.experimental import attempt_load
        self.log.info(f"YOLO: attempt_load({wp}) …")
        model = attempt_load(wp, map_location=self.device) 
        model.eval()

        

        self.fp16 = False
        if self.device.type == "cuda":
            try:
                model.half()
                self.fp16 = True
            except Exception:
                pass

        self.names = getattr(model, "names", {}) or getattr(getattr(model, "module", None), "names", {}) or {}
        self.model = model
        self.log.info("YOLO: loaded and ready.")

    @torch.no_grad()
    def infer(self, bgr_img: np.ndarray) -> np.ndarray:
        #stuff from offline that helps with image tracking
        h, w = bgr_img.shape[:2]
        scale = min(self.img_size / h, self.img_size / w)
        nh, nw = int(round(h * scale)), int(round(w * scale))
        img = cv2.resize(bgr_img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.zeros((self.img_size, self.img_size, 3), dtype=img.dtype)
        top = (self.img_size - nh) // 2
        left = (self.img_size - nw) // 2
        canvas[top:top + nh, left:left + nw] = img

        x = torch.from_numpy(canvas[:, :, ::-1].copy()).to(self.device)
        x = x.permute(2, 0, 1).contiguous()
        x = (x.half() if self.fp16 else x.float()) / 255.0
        x = x.unsqueeze(0)

        pred = self.model(x, augment=False)[0]

        from utils.general import non_max_suppression
        dets = non_max_suppression(pred, self.conf_thres, iou_thres=0.45,
                                   classes=None, agnostic=False)[0]
        if dets is None or len(dets) == 0:
            return np.zeros((0, 6), dtype=np.float32)

        # dets in [x1,y1,x2,y2,conf,cls] format
        dets = dets.detach().to("cpu").numpy().astype(np.float32)
        return dets


class YoloTrackNode(Node):
    def __init__(self):
        super().__init__('yolo_track_node')

        # Parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('weights', '/Model/yolov7-tiny.pt') 
        self.declare_parameter('conf', 0.3)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('img_size', 640)

        self.image_topic = self.get_parameter('image_topic').value
        self.weights = self.get_parameter('weights').value
        self.conf = self.get_parameter('conf').value
        self.device = self.get_parameter('device').value
        self.img_size = int(self.get_parameter('img_size').value)

        self.get_logger().info("YoloTrackNode starting up…")
        self.get_logger().info(f"Using weights={self.weights} device={self.device} img_size={self.img_size}")

        # YOLO model + bridge
        self.bridge = CvBridge()

        self.get_logger().info("Loading YOLOv7 model...")
        try:
            self.detector = YoloV7Detector(
                self.weights, device=self.device, img_size=self.img_size, conf_thres=self.conf,
                logger=self.get_logger()
            )
            self.get_logger().info("YOLOv7 loaded OK")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLOv7: {e}")
            raise

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(Image, self.image_topic, self.image_cb, qos)
        self.pub_events = self.create_publisher(String, 'compiled_events', 10)
        self.pub_img = self.create_publisher(Image, '/tracking/image', 10)

     
        self.tracker = Sort(max_age=30, min_hits=3, iou_threshold=0.3)
        self.get_logger().info("SORT tracker initialized")

        self.uploaded_ids = set()

    def _match_track_to_det(self, track_box, det_np):
        #returns cls_id and conf of a detection
        if det_np is None or len(det_np) == 0:
            return None, None
        ious = [iou_xyxy(track_box, det[:4]) for det in det_np]
        j = int(np.argmax(ious))
        if ious[j] <= 0.0:
            return None, None
        best = det_np[j]
        conf = float(best[4])
        cls_id = int(best[5])
        return cls_id, conf

    def image_cb(self, msg: Image):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        det_np = self.detector.infer(frame)  
        dets_for_sort = det_np[:, :5] if len(det_np) else np.empty((0, 5), dtype=np.float32)
        tracks = self.tracker.update(dets_for_sort)  
        annotated = frame.copy()
        ts_ms = int(ros_time_to_seconds(msg.header.stamp) * 1000)


        for x1, y1, x2, y2, track_id in tracks.astype(int):
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, f"ID {track_id}", (x1, max(0, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            if track_id not in self.uploaded_ids:
                self.uploaded_ids.add(track_id)
                cls_id, conf = self._match_track_to_det([x1, y1, x2, y2], det_np)
                label = self.detector.names.get(cls_id, str(cls_id)) if cls_id is not None else None
                jpeg_b64 = None

                try:
                    #maybe need this?
                    jpeg_b64 = bgr_to_jpeg_bytes(annotated, quality=90)
                except Exception as e:
                    self.get_logger().warn(f"JPEG encode failed: {e}")

                event = {
                    "timestamp_ms": ts_ms,
                    "robot_id": ROBOT_ID,
                    "users": [ROBOT_UID],
                    "bucket": FIREBASE_STORAGE_BUCKET,
                    "collection": COLLECTION_PATH,
                    "track_id": int(track_id),
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    "class_id": (int(cls_id) if cls_id is not None else None),
                    "label": label,
                    "conf": (float(conf) if conf is not None else None),
                    "image": jpeg_b64
                }
                self.pub_events.publish(String(data=json.dumps(event)))
                self.log.info(event)

        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        img_msg.header = msg.header
        self.pub_img.publish(img_msg)

        #keep set bounded
        if len(self.uploaded_ids) > 5000:
            self.uploaded_ids.clear()


def main(args=None):
    rclpy.init(args=args)
    node = YoloTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
