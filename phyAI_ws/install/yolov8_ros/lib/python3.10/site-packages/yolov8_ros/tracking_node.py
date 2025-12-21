#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

import message_filters
import numpy as np
from cv_bridge import CvBridge

from ultralytics.trackers import BOTSORT, BYTETracker
from ultralytics.utils import IterableSimpleNamespace, yaml_load
from ultralytics.utils.checks import check_requirements, check_yaml
from ultralytics.engine.results import Boxes

from sensor_msgs.msg import Image
from yolov8_msgs.msg import DetectionArray

class TrackingNode(Node):
    def __init__(self) -> None:
        super().__init__("tracking_node")

        # -----------------------
        # Parameters
        # -----------------------
        self.declare_parameter("tracker", "bytetrack.yaml")
        tracker_yaml = self.get_parameter("tracker").get_parameter_value().string_value

        # (선택) 이미지 QoS 신뢰도 파라미터화
        self.declare_parameter("image_reliability", QoSReliabilityPolicy.BEST_EFFORT)
        rel_val = self.get_parameter("image_reliability").get_parameter_value().integer_value
        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy(rel_val),
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # -----------------------
        # Runtime
        # -----------------------
        self.cv_bridge = CvBridge()
        self.tracker = self.create_tracker(tracker_yaml)

        # Publisher: 추적 결과
        self.pub_tracking = self.create_publisher(DetectionArray, "tracking", 10)

        # Subscribers (동기화)
        image_sub = message_filters.Subscriber(self, Image, "image_raw", qos_profile=image_qos_profile)
        dets_sub = message_filters.Subscriber(self, DetectionArray, "detections", qos_profile=10)

        self._sync = message_filters.ApproximateTimeSynchronizer(
            (image_sub, dets_sub), queue_size=10, slop=0.5
        )
        self._sync.registerCallback(self.detections_cb)

        self.get_logger().info("TrackingNode initialized (vanilla)")

    def create_tracker(self, tracker_yaml: str):
        """Ultralytics BYTETrack/BOTSORT 트래커 생성"""
        TRACKER_MAP = {"bytetrack": BYTETracker, "botsort": BOTSORT}
        check_requirements("lap")
        tracker_file = check_yaml(tracker_yaml)
        cfg = IterableSimpleNamespace(**yaml_load(tracker_file))
        assert cfg.tracker_type in TRACKER_MAP, f"Unsupported tracker_type: {cfg.tracker_type}"
        # frame_rate는 필요 시 yaml에서 설정되며, 기본 1로 두어도 동작에는 무방
        return TRACKER_MAP[cfg.tracker_type](args=cfg, frame_rate=1)

    def detections_cb(self, img_msg: Image, detections_msg: DetectionArray) -> None:
        """이미지+디텍션 입력 → 트래킹 → ID/바운딩박스 갱신 → tracking 퍼블리시"""
        tracked_msg = DetectionArray()
        tracked_msg.header = img_msg.header

        # OpenCV 이미지 변환 (BGR 보장)
        frame = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

        # DetectionArray → Ultralytics Boxes 입력 형태로 변환 (xyxy, conf, cls)
        det_list = []
        for det in detections_msg.detections:
            x1 = det.bbox.center.position.x - det.bbox.size.x / 2.0
            y1 = det.bbox.center.position.y - det.bbox.size.y / 2.0
            x2 = det.bbox.center.position.x + det.bbox.size.x / 2.0
            y2 = det.bbox.center.position.y + det.bbox.size.y / 2.0
            det_list.append([x1, y1, x2, y2, det.score, det.class_id])

        if det_list:
            # Boxes: (N,6) with xyxy + conf + cls
            det_boxes = Boxes(np.array(det_list, dtype=np.float32), (img_msg.height, img_msg.width))

            # 트래커 업데이트 (프레임과 함께)
            tracks = self.tracker.update(det_boxes, frame)

            # tracks를 기반으로 각 detection의 bbox/id 갱신
            if len(tracks) > 0:
                for t in tracks:
                    # t가 [x1,y1,x2,y2,score,cls,det_index] 형태라는 가정(사용하던 패턴 유지)
                    # 마지막 원소가 원본 detection 인덱스라고 가정
                    det_index = int(t[-1])
                    if det_index < 0 or det_index >= len(detections_msg.detections):
                        continue

                    d = detections_msg.detections[det_index]

                    # 갱신된 박스를 xywh로 반영
                    tracked_box = Boxes(np.array([t[:-1]], dtype=np.float32), (img_msg.height, img_msg.width))
                    box_xywh = tracked_box.xywh[0]
                    d.bbox.center.position.x = float(box_xywh[0])
                    d.bbox.center.position.y = float(box_xywh[1])
                    d.bbox.size.x = float(box_xywh[2])
                    d.bbox.size.y = float(box_xywh[3])

                    # 트랙 ID가 제공되면 부여
                    if hasattr(tracked_box, "is_track") and tracked_box.is_track and hasattr(tracked_box, "id"):
                        try:
                            d.id = str(int(tracked_box.id))
                        except Exception:
                            pass

                    tracked_msg.detections.append(d)
            else:
                # 트랙이 없으면 원본 디텍션을 그대로 전달
                tracked_msg.detections.extend(detections_msg.detections)
        else:
            # 디텍션이 없으면 빈 메시지 퍼블리시
            pass

        self.pub_tracking.publish(tracked_msg)


def main():
    rclpy.init()
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
