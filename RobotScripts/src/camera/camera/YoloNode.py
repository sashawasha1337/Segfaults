#!/usr/bin/env python3
import numpy as np
import os
import sys

#fix for model version mismatches?
#sys.modules.setdefault('numpy._core', np.core)


sys.path.insert(0, "/deep_sort_pytorch")
from deep_sort.deep_sort import DeepSort

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
import torch
from pathlib import Path
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

SIDE_CAR_SCRIPT = os.environ.get("SIDE_CAR_SCRIPT")
SIDE_CAR_PYTHON = os.environ.get("SIDE_CAR_PYTHON", "/opt/py39-env/bin/python3.9")
PUBLISH_DEBUG_TOPIC = True

def bgr_to_jpeg_bytes(img_bgr, quality=90):
    ok, buf = cv2.imencode(".jpg", img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError("cv2.imencode failed")
    return base64.b64encode(buf.tobytes()).decode("ascii")

def ros_time_to_seconds(ros_time):
    return ros_time.sec + ros_time.nanosec * 1e-9

#intersection over union math
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
    def __init__(self, weights_path: str, device: str = "cuda:0", img_size: int = 640, conf_thres: float = 0.3, logger=None):
        self.log = logger or (lambda *a, **k: None)
        self.img_size = int(img_size)
        self.conf_thres = float(conf_thres)

        # Device
        if isinstance(device, torch.device):
            self.device = device
        else:
            dev_str = str(device)
            if torch.cuda.is_available() and dev_str.startswith("cuda"):
                self.device = torch.device(dev_str)
            else:
                self.device = torch.device("cpu")

        self.log.info(f"YOLO: torch={torch.__version__} cuda_avail={torch.cuda.is_available()} device={self.device}")

        # Weight path must exist
        wp = str(Path(weights_path).expanduser())
        if not os.path.isfile(wp):
            raise FileNotFoundError(f"weights not found: {wp}")

        # Put yolov7 repo on path
        yolov7_root = "/ros2_ws/yolov7"
        if yolov7_root not in sys.path:
            sys.path.insert(0, yolov7_root)

        # patch attempt_download to avoid calling git tag
        from utils import google_utils as ggu
        ggu.attempt_download = lambda f, *a, **k: f

        try:
            foundLib = importlib.util.find_spec("models.common")
            if foundLib is not None:
                import models.common
                models.common.Conv.fuse = lambda self, *a, **k: self
            foundLib = importlib.util.find_spec("models.yolo")
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
    #YOLO inference
    @torch.no_grad()
    def infer(self, bgr_img: np.ndarray) -> np.ndarray:
        from utils.datasets import letterbox

        img = letterbox(
            bgr_img,
            new_shape=self.img_size,
            stride=32,
            auto=True
        )[0]
        img = img[:, :, ::-1]
        img = img.transpose(2, 0, 1)
        img = np.ascontiguousarray(img, dtype=np.float32)
        #normalizing
        img /= 255.0

        x = torch.from_numpy(img).to(self.device).unsqueeze(0)
        if self.fp16:
            x = x.half()

        pred = self.model(x, augment=False)[0]

        from utils.general import non_max_suppression
        dets = non_max_suppression(
            pred,
            self.conf_thres,
            iou_thres=0.45,
            classes=None,
            agnostic=False
        )[0]

        if dets is None or len(dets) == 0:
            return np.zeros((0, 6), dtype=np.float32)
        return dets.cpu().numpy().astype(np.float32)


class YoloTrackNode(Node):
    def __init__(self):
        super().__init__('yolo_track_node')

        # Parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('weights', '/Model/newbest.pt')
        self.declare_parameter('conf', 0.55)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('img_size', 640)

        self.image_topic = self.get_parameter('image_topic').value
        self.weights = self.get_parameter('weights').value
        self.conf = self.get_parameter('conf').value
        self.device = torch.device(self.get_parameter('device').value)
        self.img_size = int(self.get_parameter('img_size').value)

        self.get_logger().info("YoloTrackNode starting up…")
        self.get_logger().info(f"Using weights={self.weights} device={self.device} img_size={self.img_size}")

        # get YOLO model + bridge
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

        self.deepsort = DeepSort(
            "/deep_sort_pytorch/deep_sort/deep/checkpoint/ckpt.t7",
            max_age=20,
            max_dist=1.5,
            n_init=5,
            nn_budget=100,
            use_cuda=self.device.type == "cuda"
        )
        self.get_logger().info("SORT tracker initialized")

        self.uploaded_ids = set()
        self.sidecar_proc = None

      

    def _match_track_to_det(self, track_box, det_np):
        if det_np is None or len(det_np) == 0:
            return None, None

        # Compute IoU between tracked box and detections
        ious = [iou_xyxy(track_box, det[:4]) for det in det_np]
        best_idx = int(np.argmax(ious))
        best_iou = ious[best_idx]

        # require some overlap
        if best_iou < 0.2:
            return None, None

        best_det = det_np[best_idx]
        det_conf = float(best_det[4])
        cls_id = int(best_det[5])

        # YOLO confidence threshold
        if det_conf < self.conf:
            return None, None

        return cls_id, det_conf


    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        det_np = self.detector.infer(frame)

        # Filter low-confidence detections(does not mean no detections will show under threshold)
        det_np = det_np[det_np[:, 4] >= self.conf]

        if det_np.shape[0] > 0:
            bbox_xyxy = det_np[:, 0:4]
            confidences = det_np[:, 4]
            xywhs = np.zeros_like(bbox_xyxy)
            xywhs[:, 0] = (bbox_xyxy[:, 0] + bbox_xyxy[:, 2]) / 2.0
            xywhs[:, 1] = (bbox_xyxy[:, 1] + bbox_xyxy[:, 3]) / 2.0
            xywhs[:, 2] = bbox_xyxy[:, 2] - bbox_xyxy[:, 0]
            xywhs[:, 3] = bbox_xyxy[:, 3] - bbox_xyxy[:, 1]
            classes = det_np[:, 5]
        else:
            #if no detections pass in empty np arrays
            xywhs = np.empty((0, 4))
            confidences = np.empty((0,))
            classes = np.empty((0,))

        #Always call update once only 
        result = self.deepsort.update(xywhs, confidences, classes, frame, None)
        #fixing unexpected formatting
        if isinstance(result, (tuple, list)) and len(result) == 2:
            outputs, _ = result
        else:
            outputs = result


       
        if outputs is None or len(outputs) == 0:
            outputs = np.empty((0,6),dtype=float)
        else:
            outputs = np.asarray(outputs,dtype=float)
            if outputs.ndim ==1:
                outputs=outputs.reshape(1,-1)


        annotated=frame.copy()
        ts_ms = int(ros_time_to_seconds(msg.header.stamp) * 1000)


          
            

        for row in outputs:
                # Make sure each row is a flat vector/legible
                
                

                if row.size == 5:
                    x1, y1, x2, y2, track_id = row
                elif row.size >= 6:
                    x1, y1, x2, y2, _, track_id = row
                else:
                    self.get_logger().warn("Skipping malformed Deep SORT row")
                    self.get_logger().warn(f"RAW Deep SORT row: {row}")
                    continue

                # Draw tracking boxes
                cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(annotated, f"ID {int(track_id)}",
                            (int(x1), int(max(0, y1 - 5))),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Match YOLO confidence + class to this track
                cls_id, det_conf = self._match_track_to_det([x1, y1, x2, y2], det_np)

                if cls_id is None or det_conf is None or det_conf < self.conf:
                    continue

                # Only publish first time
                if track_id in self.uploaded_ids:
                    continue

                self.uploaded_ids.add(track_id)

                # Resolve label
                names = self.detector.names
                if isinstance(names, list) and cls_id < len(names):
                    label = names[cls_id]
                elif isinstance(names, dict):
                    label = names.get(cls_id, str(cls_id))
                else:
                    label = str(cls_id)

                # Encode image
                try:
                    jpeg_b64 = bgr_to_jpeg_bytes(annotated, quality=90)
                except Exception as e:
                    self.get_logger().warn(f"JPEG encode failed: {e}")
                    jpeg_b64 = None

                # Publish event
                event = {
                    "timestamp_ms": ts_ms,
                    "track_id": int(track_id),
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    "class_id": int(cls_id),
                    "label": label,
                    "conf": float(det_conf),
                    "image": jpeg_b64
                }

                self.pub_events.publish(String(data=json.dumps(event)))
                self.get_logger().info(str({key : value for key ,value in event.items() if key != "image"}))

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