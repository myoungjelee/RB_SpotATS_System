import cv2
import json
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

class SnapshotVisualizer:
    def __init__(self, logger):
        self.logger = logger
        self.latest_cv_image = None
        self.window_name = "System2 Target Popup"

    def update_image(self, msg: CompressedImage):
        """압축 이미지를 수신하여 OpenCV 포맷으로 캐싱 (디버그 강화)"""
        try:
            # 1. 들어온 데이터 크기 확인
            data_len = len(msg.data)
            self.logger.info(f"[Visualizer] Rx Image Msg: {data_len} bytes, Format: {msg.format}")

            if data_len == 0:
                self.logger.error("[Visualizer] Received EMPTY data buffer!")
                return

            # 2. 데이터가 전부 0인지(전송된 패킷 자체가 비었는지) 확인
            np_arr = np.frombuffer(msg.data, np.uint8)
            if np.max(np_arr) == 0:
                self.logger.warn("[Visualizer] WARNING: Raw compressed data is ALL ZEROS (Black).")

            # 3. 디코딩 시도
            decoded_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # 4. 디코딩 결과 확인
            if decoded_img is None:
                self.logger.error("[Visualizer] cv2.imdecode returned None! (Corrupted data?)")
                return
            
            # 5. 디코딩 된 픽셀값 확인
            max_val = np.max(decoded_img)
            avg_val = np.mean(decoded_img)
            
            if max_val == 0:
                 self.logger.warn(f"[Visualizer] Decoded Image is PURE BLACK (Max pixel: 0). Input was valid JPEG but content is black.")
            else:
                 self.logger.info(f"[Visualizer] Image Decoded OK. Shape: {decoded_img.shape}, Avg Brightness: {avg_val:.1f}")

            self.latest_cv_image = decoded_img

        except Exception as e:
            self.logger.warn(f"[Visualizer] Image decode failed: {e}")

    def show_popup(self, msg: String):
        """타겟 정보를 받아 저장된 이미지 위에 그림을 그리고 팝업 노출"""
        if self.latest_cv_image is None:
            self.logger.warn("[Visualizer] Info received but no image to draw on.")
            return

        try:
            info = json.loads(msg.data)
            # 원본 이미지 보존을 위해 복사
            display_img = self.latest_cv_image.copy()

            # 정보 파싱
            event = info.get("event", "UNKNOWN")
            cls_name = info.get("class", "object")
            obj_id = info.get("id", "?")
            
            # 좌표 계산을 위한 정보 추출
            center = info.get("center", {})
            bbox = info.get("bbox")  # [x, y, w, h]
            range_m = info.get("range_m")

            # 1. 빨간색 Bounding Box 그리기 (BGR: 0, 0, 255)
            # bbox의 x,y가 0이어도 center 좌표를 기준으로 재계산하여 그립니다.
            if bbox and len(bbox) == 4 and center:
                # 너비와 높이 추출
                w = int(bbox[2])
                h = int(bbox[3])
                
                # 중심점 추출
                cx = int(center.get("x", 0))
                cy = int(center.get("y", 0))

                # 중심점 기준 좌측 상단(x1, y1) 및 우측 하단(x2, y2) 계산
                x1 = int(cx - w / 2)
                y1 = int(cy - h / 2)
                x2 = x1 + w
                y2 = y1 + h
                
                # 박스 그리기
                cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 0, 255), 3)

                # 라벨 텍스트 생성
                label = f"[{obj_id}] {cls_name}"
                if range_m:
                    label += f" ({float(range_m):.1f}m)"

                # 텍스트 사이즈 계산
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                
                # 텍스트가 화면 위로 잘리지 않도록 위치 조정
                text_y = y1 - 10
                if text_y < 25:
                    text_y = y2 + 25

                # 라벨 배경 (검정 박스)
                cv2.rectangle(display_img, (x1, text_y - th - 5), (x1 + tw, text_y + 5), (0, 0, 0), -1)
                
                # 라벨 텍스트 (흰색)
                cv2.putText(display_img, label, (x1, text_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # 2. 이벤트 상태 표시 (좌측 상단)
            cv2.putText(display_img, f"EVENT: {event}", (20, 40),
                        cv2.FONT_HERSHEY_DUPLEX, 1.0, (0, 255, 255), 2)

            # 3. 팝업 출력
            cv2.imshow(self.window_name, display_img)
            cv2.waitKey(1)

            self.logger.info(f"[Visualizer] Popup displayed for target {obj_id}")

        except Exception as e:
            self.logger.error(f"[Visualizer] Failed to draw popup: {e}")

    def close(self):
        try:
            cv2.destroyWindow(self.window_name)
        except Exception:
            pass