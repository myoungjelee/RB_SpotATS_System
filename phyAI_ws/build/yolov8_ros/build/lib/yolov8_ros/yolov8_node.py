#!/usr/bin/env python3
from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

from cv_bridge import CvBridge
import cv2

from ultralytics import YOLO
from ultralytics.engine.results import Results, Boxes, Masks, Keypoints

# --- PyTorch 2.6 / Ultralytics 호환: 안전 글로벌 + weights_only 패치 ---
import torch.serialization
from ultralytics.nn.tasks import (
    DetectionModel, SegmentationModel, PoseModel, ClassificationModel
)
torch.serialization.add_safe_globals([
    DetectionModel, SegmentationModel, PoseModel, ClassificationModel
])
try:
    from ultralytics.nn.tasks import RTDETRDetectionModel
    torch.serialization.add_safe_globals([RTDETRDetectionModel])
except Exception:
    pass

import inspect, torch as _torch
if "weights_only" in inspect.signature(_torch.load).parameters:
    _orig = _torch.load
    def _patched(*args, **kwargs):
        # 신뢰 가능한 가중치라는 전제에서 False
        kwargs.setdefault("weights_only", False)
        return _orig(*args, **kwargs)
    _torch.load = _patched
# ---------------------------------------------------------------------

from sensor_msgs.msg import Image
from yolov8_msgs.msg import (
    Point2D,
    BoundingBox2D,
    Mask as MaskMsg,
    KeyPoint2D,
    KeyPoint2DArray,
    Detection,
    DetectionArray,
)
from std_srvs.srv import SetBool


class Yolov8Node(Node):
    def __init__(self) -> None:
        super().__init__("yolov8_node")

        # params (원본 기본값 유지)
        self.declare_parameter("model", "yolov8m.pt")
        model = self.get_parameter("model").get_parameter_value().string_value

        self.declare_parameter("device", "cuda:0")  # 필요시 런치에서 cpu로 덮어쓰기
        self.device = self.get_parameter("device").get_parameter_value().string_value

        self.declare_parameter("threshold", 0.5)
        self.threshold = self.get_parameter("threshold").get_parameter_value().double_value

        self.declare_parameter("enable", True)
        self.enable = self.get_parameter("enable").get_parameter_value().bool_value

        self.declare_parameter("image_reliability", QoSReliabilityPolicy.BEST_EFFORT)
        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy(
                self.get_parameter("image_reliability").get_parameter_value().integer_value
            ),
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.cv_bridge = CvBridge()
        self.yolo = YOLO(model)
        try:
            self.yolo.fuse()
        except Exception:
            pass

        # pubs
        self._pub = self.create_publisher(DetectionArray, "detections", 10)

        # subs
        self._sub = self.create_subscription(
            Image, "image_raw", self.image_cb, image_qos_profile
        )

        # services
        self._srv = self.create_service(SetBool, "enable", self.enable_cb)

        self.get_logger().info(f"YOLO node started (model={model}, device={self.device}, thr={self.threshold})")

    def enable_cb(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self.enable = req.data
        res.success = True
        return res

    def parse_hypothesis(self, results: Results) -> List[Dict]:
        hypothesis_list: List[Dict] = []
        box_data: Boxes
        for box_data in results.boxes:
            hypothesis = {
                "class_id": int(box_data.cls),
                "class_name": self.yolo.names[int(box_data.cls)],
                "score": float(box_data.conf),
            }
            hypothesis_list.append(hypothesis)
        return hypothesis_list

    def parse_boxes(self, results: Results) -> List[BoundingBox2D]:
        boxes_list: List[BoundingBox2D] = []
        box_data: Boxes
        for box_data in results.boxes:
            msg = BoundingBox2D()
            box = box_data.xywh[0]
            msg.center.position.x = float(box[0])
            msg.center.position.y = float(box[1])
            msg.size.x = float(box[2])
            msg.size.y = float(box[3])
            boxes_list.append(msg)
        return boxes_list

    def parse_masks(self, results: Results) -> List[MaskMsg]:
        masks_list: List[MaskMsg] = []

        def create_point2d(x: float, y: float) -> Point2D:
            p = Point2D()
            p.x = x
            p.y = y
            return p

        m: Masks
        for m in results.masks:
            msg = MaskMsg()
            msg.data = [create_point2d(float(ele[0]), float(ele[1])) for ele in m.xy[0].tolist()]
            msg.height = results.orig_img.shape[0]
            msg.width  = results.orig_img.shape[1]
            masks_list.append(msg)
        return masks_list

    def parse_keypoints(self, results: Results) -> List[KeyPoint2DArray]:
        keypoints_list: List[KeyPoint2DArray] = []
        pts: Keypoints
        for pts in results.keypoints:
            msg_array = KeyPoint2DArray()
            if pts.conf is None:
                keypoints_list.append(msg_array)
                continue
            for kp_id, (p, conf) in enumerate(zip(pts.xy[0], pts.conf[0])):
                if conf >= self.threshold:
                    msg = KeyPoint2D()
                    msg.id = kp_id + 1
                    msg.point.x = float(p[0])
                    msg.point.y = float(p[1])
                    msg.score = float(conf)
                    msg_array.data.append(msg)
            keypoints_list.append(msg_array)
        return keypoints_list
    # -------------------------------------------------------------

    def image_cb(self, msg: Image) -> None:
        if not self.enable:
            return

        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        try:
            results_list = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=self.threshold,
                device=self.device
            )
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

        if not results_list:
            return

        results: Results = results_list[0].cpu()

        detections_msg = DetectionArray()

        hypothesis, boxes, masks, keypoints = [], [], [], []
        if results.boxes:
            hypothesis = self.parse_hypothesis(results)
            boxes = self.parse_boxes(results)
        if results.masks:
            masks = self.parse_masks(results)
        if results.keypoints:
            keypoints = self.parse_keypoints(results)

        for i in range(len(hypothesis) if results.boxes else 0):
            aux_msg = Detection()
            aux_msg.class_id = hypothesis[i]["class_id"]
            aux_msg.class_name = hypothesis[i]["class_name"]
            aux_msg.score = hypothesis[i]["score"]
            aux_msg.bbox = boxes[i]
            if results.masks and i < len(masks):
                aux_msg.mask = masks[i]
            if results.keypoints and i < len(keypoints):
                aux_msg.keypoints = keypoints[i]
            detections_msg.detections.append(aux_msg)

        detections_msg.header = msg.header
        self._pub.publish(detections_msg)


def main():
    rclpy.init()
    node = Yolov8Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
