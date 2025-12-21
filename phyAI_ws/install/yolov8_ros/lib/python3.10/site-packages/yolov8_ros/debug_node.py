#!/usr/bin/env python3
# Copyright (C) 2023  Miguel Ángel González Santamarta
# GPLv3

import cv2
import random
import numpy as np
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

import message_filters
from cv_bridge import CvBridge
from ultralytics.utils.plotting import Annotator, colors

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from yolov8_msgs.msg import BoundingBox2D, KeyPoint2D, KeyPoint3D, Detection, DetectionArray


class DebugNode(Node):
    def __init__(self) -> None:
        super().__init__("debug_node")

        self._class_to_color = {}
        self.cv_bridge = CvBridge()

        # QoS 파라미터 (센서 이미지 기본 QoS)
        self.declare_parameter("image_reliability", QoSReliabilityPolicy.BEST_EFFORT)
        rel_val = self.get_parameter("image_reliability").get_parameter_value().integer_value
        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy(rel_val),
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # Publishers
        self.pub_dbg_image = self.create_publisher(Image, "dbg_image", 10)
        self.pub_bb_markers = self.create_publisher(MarkerArray, "dbg_bb_markers", 10)
        self.pub_kp_markers = self.create_publisher(MarkerArray, "dbg_kp_markers", 10)

        # Subscribers (동기화)
        image_sub = message_filters.Subscriber(self, Image, "image_raw", qos_profile=image_qos_profile)
        dets_sub = message_filters.Subscriber(self, DetectionArray, "detections", qos_profile=10)
        self.sync = message_filters.ApproximateTimeSynchronizer((image_sub, dets_sub), 10, 0.5)
        self.sync.registerCallback(self.detections_cb)

        self.get_logger().info("DebugNode initialized (vanilla)")

    # --------------------------
    # Drawing helpers
    # --------------------------
    def draw_box(self, cv_image: np.ndarray, detection: Detection, color: Tuple[int]) -> np.ndarray:
        label = detection.class_name
        score = detection.score
        track_id = detection.id
        box_msg: BoundingBox2D = detection.bbox

        x1 = round(box_msg.center.position.x - box_msg.size.x / 2.0)
        y1 = round(box_msg.center.position.y - box_msg.size.y / 2.0)
        x2 = round(box_msg.center.position.x + box_msg.size.x / 2.0)
        y2 = round(box_msg.center.position.y + box_msg.size.y / 2.0)

        cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)

        text = f"{label} ({track_id}) ({score:.3f})"
        cv2.putText(cv_image, text, (x1 + 5, y1 + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1, cv2.LINE_AA)
        return cv_image

    def draw_mask(self, cv_image: np.ndarray, detection: Detection, color: Tuple[int]) -> np.ndarray:
        if not detection.mask.data:
            return cv_image

        mask_array = np.array([[int(p.x), int(p.y)] for p in detection.mask.data])
        layer = cv_image.copy()
        layer = cv2.fillPoly(layer, pts=[mask_array], color=color)
        cv2.addWeighted(cv_image, 0.4, layer, 0.6, 0, cv_image)
        cv_image = cv2.polylines(cv_image, [mask_array], isClosed=True,
                                 color=color, thickness=2, lineType=cv2.LINE_AA)
        return cv_image

    def draw_keypoints(self, cv_image: np.ndarray, detection: Detection) -> np.ndarray:
        if not detection.keypoints.data:
            return cv_image

        ann = Annotator(cv_image)

        # 점 찍기
        for kp in detection.keypoints.data:  # type: KeyPoint2D
            color_k = [int(x) for x in ann.kpt_color[kp.id - 1]] \
                      if len(detection.keypoints.data) == 17 else colors(kp.id - 1)
            cv2.circle(cv_image, (int(kp.point.x), int(kp.point.y)),
                       5, color_k, -1, lineType=cv2.LINE_AA)

        # 스켈레톤 연결
        def get_pk_pose(kp_id: int):
            for kp in detection.keypoints.data:
                if kp.id == kp_id:
                    return (int(kp.point.x), int(kp.point.y))
            return None

        for i, sk in enumerate(ann.skeleton):
            p1 = get_pk_pose(sk[0])
            p2 = get_pk_pose(sk[1])
            if p1 is not None and p2 is not None:
                cv2.line(cv_image, p1, p2, [int(x) for x in ann.limb_color[i]],
                         thickness=2, lineType=cv2.LINE_AA)
        return cv_image

    # --------------------------
    # Marker helpers (3D 옵션)
    # --------------------------
    def create_bb_marker(self, detection: Detection) -> Marker:
        bbox3d = detection.bbox3d
        marker = Marker()
        marker.header.frame_id = bbox3d.frame_id
        marker.ns = "yolov8_3d"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = False

        marker.pose.position.x = bbox3d.center.position.x
        marker.pose.position.y = bbox3d.center.position.y
        marker.pose.position.z = bbox3d.center.position.z
        marker.pose.orientation.w = 1.0

        marker.scale.x = bbox3d.size.x
        marker.scale.y = bbox3d.size.y
        marker.scale.z = bbox3d.size.z

        marker.color.b = 0.0
        marker.color.g = detection.score * 255.0
        marker.color.r = (1.0 - detection.score) * 255.0
        marker.color.a = 0.4

        marker.lifetime = Duration(seconds=0.5).to_msg()
        marker.text = detection.class_name
        return marker

    def create_kp_marker(self, kp: KeyPoint3D, frame_id: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = "yolov8_3d"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.frame_locked = False

        marker.pose.position.x = kp.point.x
        marker.pose.position.y = kp.point.y
        marker.pose.position.z = kp.point.z
        marker.pose.orientation.w = 1.0

        marker.scale.x = marker.scale.y = marker.scale.z = 0.05
        marker.color.b = kp.score * 255.0
        marker.color.g = 0.0
        marker.color.r = (1.0 - kp.score) * 255.0
        marker.color.a = 0.4

        marker.lifetime = Duration(seconds=0.5).to_msg()
        marker.text = str(kp.id)
        return marker

    # --------------------------
    # Callback
    # --------------------------
    def detections_cb(self, img_msg: Image, det_msg: DetectionArray) -> None:
        # 이미지 변환 (BGR 보장)
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        bb_markers = MarkerArray()
        kp_markers = MarkerArray()

        # 디텍션 그리기
        for det in det_msg.detections:
            # 클래스별 고정 랜덤 색상
            label = det.class_name or "unknown"
            if label not in self._class_to_color:
                r = random.randint(0, 255)
                g = random.randint(0, 255)
                b = random.randint(0, 255)
                self._class_to_color[label] = (b, g, r)  # OpenCV는 BGR
            color = self._class_to_color[label]

            cv_image = self.draw_box(cv_image, det, color)
            cv_image = self.draw_mask(cv_image, det, color)
            cv_image = self.draw_keypoints(cv_image, det)

            # 3D 마커 옵션 (있을 때만)
            if det.bbox3d.frame_id:
                m = self.create_bb_marker(det)
                m.header.stamp = img_msg.header.stamp
                m.id = len(bb_markers.markers)
                bb_markers.markers.append(m)

            if det.keypoints3d.frame_id:
                for kp in det.keypoints3d.data:
                    m = self.create_kp_marker(kp, det.keypoints3d.frame_id)
                    m.header.stamp = img_msg.header.stamp
                    m.id = len(kp_markers.markers)
                    kp_markers.markers.append(m)

        # 퍼블리시
        self.pub_dbg_image.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, encoding=img_msg.encoding))
        self.pub_bb_markers.publish(bb_markers)
        self.pub_kp_markers.publish(kp_markers)

        cv2.imshow('results', cv_image)
        cv2.waitKey(10)


def main():
    rclpy.init()
    node = DebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
