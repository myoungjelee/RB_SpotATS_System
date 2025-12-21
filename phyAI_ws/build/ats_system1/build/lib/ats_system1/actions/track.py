#!/usr/bin/env python3
import math, time, atexit
import threading
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer
from rclpy.node import Node

# Depth
from sensor_msgs.msg import Image
try:
    import numpy as np
except Exception:
    np = None

# ----------- Math/Utils -----------
def _quat_to_rotmat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    # base <- camera (row-major)
    return [
        [1 - 2*(yy+zz),     2*(xy - wz),     2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx+zz),     2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx+yy)],
    ]

def _axis_vec(name: str):
    n = (name or "").strip().lower()
    if n == 'x':   return (1.0, 0.0, 0.0)
    if n == '-x':  return (-1.0, 0.0, 0.0)
    if n == 'y':   return (0.0, 1.0, 0.0)
    if n == '-y':  return (0.0, -1.0, 0.0)
    if n == 'z':   return (0.0, 0.0, 1.0)
    if n == '-z':  return (0.0, 0.0, -1.0)
    return (0.0, 0.0, 1.0)  # default: optical Z

def _clamp(x, lo, hi):
    return hi if x > hi else (lo if x < lo else x)

def _as_str(v):
    try: return str(int(v))
    except Exception: return str(v) if v is not None else None

def _fmt(v, fmt="{:.3f}"):
    try:
        if v is None: return "None"
        if isinstance(v, (int, float)): return fmt.format(v)
        return str(v)
    except Exception:
        return str(v)

# ----------- Depth Buffer -----------
class DepthBuffer:
    """
    최신 depth 이미지를 보관하고, bbox ROI의 대표 depth(m)를 계산한다.
    - 인코딩 자동 처리(32FC1=미터, 16UC1=mm→m)
    - 프레임 크기 매핑
    - 중앙부 크롭/백분위수/최소 유효비율
    - 오래된 프레임 무시(max_age_sec)
    """
    def __init__(self, node: Node, topic: str = "/depth"):
        self.node = node
        self.topic = topic
        self._img = None   # (H, W) float32 in meters
        self._w = None
        self._h = None
        self._stamp = 0.0
        self.max_age_sec = 0.25
        self._sub = node.create_subscription(Image, topic, self._on_msg, 10)

    def _on_msg(self, msg: Image):
        if np is None:
            return
        try:
            w, h = int(msg.width), int(msg.height)
            enc = getattr(msg, "encoding", "").upper()
            if enc in ("32FC1", "32FC", "TYPE_32FC1"):
                arr = np.frombuffer(msg.data, dtype=np.float32)
                if arr.size != w*h: return
                img_m = arr.reshape(h, w)
            elif enc in ("16UC1", "16U", "TYPE_16UC1", "MONO16"):
                arr = np.frombuffer(msg.data, dtype=np.uint16).astype(np.float32)
                if arr.size != w*h: return
                img_m = (arr.reshape(h, w)) / 1000.0
            else:
                arr = np.frombuffer(msg.data, dtype=np.float32)
                if arr.size != w*h:
                    arr2 = np.frombuffer(msg.data, dtype=np.uint16).astype(np.float32)
                    if arr2.size != w*h:
                        self.node.get_logger().warn(f"[depth] unsupported encoding '{enc}', size mismatch")
                        return
                    img_m = (arr2.reshape(h, w)) / 1000.0
                else:
                    img_m = arr.reshape(h, w)
            self._img = img_m
            self._w, self._h = w, h
            self._stamp = time.time()
        except Exception as e:
            self.node.get_logger().warn(f"[depth] parse failed: {e}")

    def has_image(self) -> bool:
        if self._img is None: return False
        if self._stamp and (time.time() - self._stamp) > self.max_age_sec:
            return False
        return True

    def roi_mean_depth(
        self,
        bbox_xywh: List[float],
        vision_w: int, vision_h: int,
        depth_valid_min: float = 0.2,
        depth_valid_max: float = 20.0,
        *,
        center_crop: float = 0.5,
        use_percentile: float = 0.5,
        min_valid_ratio: float = 0.02
    ) -> Optional[float]:
        if self._img is None or np is None:
            return None
        if not bbox_xywh or len(bbox_xywh) < 4:
            return None

        x, y, w, h = bbox_xywh
        if w <= 0 or h <= 0:
            return None

        sx = self._w / max(1.0, float(vision_w))
        sy = self._h / max(1.0, float(vision_h))

        dx1 = int(max(0, math.floor(x * sx)))
        dy1 = int(max(0, math.floor(y * sy)))
        dx2 = int(min(self._w, math.ceil((x + w) * sx)))
        dy2 = int(min(self._h, math.ceil((y + h) * sy)))
        if dx2 <= dx1 or dy2 <= dy1:
            return None

        roi = self._img[dy1:dy2, dx1:dx2]
        if roi.size == 0:
            return None

        if 0.0 < center_crop < 1.0:
            rw, rh = roi.shape[1], roi.shape[0]
            cw = int(max(1, rw * center_crop))
            ch = int(max(1, rh * center_crop))
            cx = rw // 2
            cy = rh // 2
            roi = roi[max(0, cy - ch//2): min(rh, cy + (ch - ch//2)),
                      max(0, cx - cw//2): min(rw, cx + (cw - cw//2))]
            if roi.size == 0:
                return None

        mask = np.isfinite(roi) & (roi > 0.0) & (roi >= depth_valid_min) & (roi <= depth_valid_max)
        valid = roi[mask]
        if valid.size < max(1, int(roi.size * min_valid_ratio)):
            return None

        p = float(use_percentile)
        if p <= 0.0:   return float(np.min(valid))
        if p >= 1.0:   return float(np.max(valid))
        if abs(p - 0.5) < 1e-6: return float(np.median(valid))
        return float(np.quantile(valid, p))

# ----------- Tracker -----------
class Tracker:
    """
    목적:
      - 카메라는 '픽셀 오차'로 대상 락(ATS yaw/pitch)
      - 바디는 TF yaw_rel PD정렬
      - Depth ROI로 거리 유지

    비동기 운용:
      - start(params, rules): 백그라운드 스레드 시작/갱신
      - wait_initial_success(timeout): 초기 성공(정렬+거리) 대기(1회 트리거)
      - stop(flush=True): 중단 및 안전 플러시
      - status(): 최근 상태 스냅샷 반환
      - run(): 구버전 블로킹 모드(하위호환)
    """
    def __init__(
        self,
        node: Node,
        cmd_pub,
        gimbal_pub,
        vision_cache,
        depth: DepthBuffer,
        ats_twist_topic: str = "/ats_twist",
        publish_legacy_array: bool = True,
        tf_buffer: Optional[Buffer] = None,
        tf_base_frame: str = "body",
        tf_camera_frame: str = "Camera",
        use_tf_align: bool = True,
        camera_forward_axis: str = "z",
    ):
        self.node = node
        self.cmd_pub = cmd_pub
        self.gimbal_pub = gimbal_pub
        self.vision = vision_cache
        self.depth = depth

        # pubs
        self.ats_twist_pub = self.node.create_publisher(Twist, ats_twist_topic, 10)
        self.publish_legacy_array = bool(publish_legacy_array)

        # TF/프레임
        self.tf_buffer = tf_buffer
        self.tf_base_frame = tf_base_frame
        self.tf_camera_frame = tf_camera_frame
        self.use_tf_align = bool(use_tf_align)
        self.camera_forward_axis = camera_forward_axis

        # 내부 상태
        self._prev_yaw = 0.0
        self._prev_t = time.time()
        self._dyaw_lpf = 0.0
        self._last_log = 0.0

        # 안전 플러시
        self._flush_times = 6
        self._flush_sleep = 0.02
        atexit.register(self._atexit_flush)

        # sticky target
        self._lock: Dict[str, Any] = {"id": None, "bbox": None, "center": None, "class": None, "last_t": 0.0}
        self._sticky_id_last = None

        # 짐벌 PID 상태
        self._yaw_i = 0.0
        self._yaw_prev_err = 0.0
        self._yaw_d_lpf = 0.0
        self._yaw_cmd_prev = 0.0
        self._prev_cam_t = time.time()
        self._last_target_id_for_i = None

        # ----------- 비동기 운용 상태 -----------
        self._cancel_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._cfg_params: Dict[str, Any] = {}
        self._cfg_rules: Dict[str, Any] = {}

        self._evt_initial_success = threading.Event()
        self._initial_success_fired = False

        self._status_lock = threading.Lock()
        self._status: Dict[str, Any] = {
            "aligned": False,
            "in_dist": False,
            "yaw_err_deg": None,
            "range_m": None,
            "dist_err": None,
            "target": {"id": None, "class": None},
            "vx": 0.0, "wz": 0.0,
            "timestamp": time.time(),
        }

        self._align_success_logged = False
        self._align_success_cooldown_until = 0.0

        # ----------- (추가) 거리 스틱 캐시 -----------
        self._last_range_m: Optional[float] = None
        self._last_range_t: float = 0.0

    # ----------- 안전 정지 -----------
    def _publish_zero_once(self):
        t = Twist(); t.linear.x = 0.0; t.angular.z = 0.0
        try: self.cmd_pub.publish(t)
        except Exception: pass

        if self.gimbal_pub is not None and self.publish_legacy_array:
            m = Float64MultiArray(); m.data = [0.0, 0.0]
            try: self.gimbal_pub.publish(m)
            except Exception: pass

        tw = Twist(); tw.angular.z = 0.0; tw.angular.y = 0.0
        try: self.ats_twist_pub.publish(tw)
        except Exception: pass

    def _flush_stop(self, times: Optional[int] = None, sleep_s: Optional[float] = None):
        n = self._flush_times if times is None else int(times)
        dt = self._flush_sleep if sleep_s is None else float(sleep_s)
        for _ in range(max(1, n)):
            self._publish_zero_once()
            time.sleep(max(0.0, dt))

    def _atexit_flush(self):
        try: self._flush_stop()
        except Exception: pass

    # ----------- 제어 스레드 관리 -----------
    def start(self, params: Dict[str, Any], rules: Dict[str, Any]) -> None:
        """백그라운드 추적 시작(이미 실행 중이면 파라미터 갱신만)."""
        with self._status_lock:
            self._cfg_params = dict(params or {})
            self._cfg_rules = dict(rules or {})

        # 초기 성공 이벤트 리셋
        self._evt_initial_success.clear()
        self._initial_success_fired = False

        if self._thread and self._thread.is_alive():
            # 이미 실행 중 → cancel 없이 파라미터 갱신만
            self.node.get_logger().info("[track] already running → update params/rules only")
            return

        # 새로 시작
        self._cancel_event.clear()
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()
        self.node.get_logger().info("[track] started (async thread)")

    def wait_initial_success(self, timeout: Optional[float] = None) -> bool:
        """정렬+거리 오차 허용을 success_hold_sec 동안 만족하면 True(1회)."""
        return self._evt_initial_success.wait(timeout=timeout)

    def stop(self, flush: bool = True) -> None:
        """비동기 추적 중단."""
        self._cancel_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if flush:
            self._flush_stop()
        self.node.get_logger().info("[track] stopped")

    def status(self) -> Dict[str, Any]:
        with self._status_lock:
            return dict(self._status)

    def request_cancel(self):
        """(하위호환) 외부 cancel 신호."""
        self._cancel_event.set()

    # ----------- TF: yaw_rel (왼−/오+) -----------
    def _lookup_yaw_rel(self, log_period: float) -> Tuple[bool, float, Tuple[float,float]]:
        if self.tf_buffer is None:
            now = time.time()
            if now - self._last_log > log_period:
                self.node.get_logger().warn("[align] TF buffer is None")
                self._last_log = now
            return False, 0.0, (0.0, 0.0)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_base_frame, self.tf_camera_frame, rclpy.time.Time()
            )
            R = _quat_to_rotmat(tf.transform.rotation)  # base<-camera
            fx, fy, fz = _axis_vec(self.camera_forward_axis)
            vx = R[0][0]*fx + R[0][1]*fy + R[0][2]*fz
            vy = R[1][0]*fx + R[1][1]*fy + R[1][2]*fz
            yaw_rel = -math.atan2(vy, vx)  # 왼쪽(−), 오른쪽(+)
            return True, float(yaw_rel), (vx, vy)
        except Exception as e:
            now = time.time()
            if now - self._last_log > log_period:
                self.node.get_logger().warn(
                    f"[align] TF lookup failed ({self.tf_base_frame}->{self.tf_camera_frame}): {e}"
                )
                self._last_log = now
            return False, 0.0, (0.0, 0.0)

    # ----------- Target 선택(간단) -----------
    def _choose_target(self, targets: List[Dict[str, Any]], chosen_id: Any, fw: int, fh: int):
        chosen_s = _as_str(chosen_id)
        if chosen_s is not None:
            for t in targets:
                if _as_str(t.get("id")) == chosen_s:
                    return t, {"method": "id"}
        if targets:
            def area(t):
                b = t.get("bbox")
                return (b[2]*b[3]) if (isinstance(b, list) and len(b) >= 4) else 0.0
            best = max(targets, key=area)
            return best, {"method": "largest"}
        return None, {"method": "none"}

    def _update_lock(self, t: Dict[str, Any]):
        self._lock.update({
            "id": t.get("id"), "bbox": t.get("bbox"),
            "center": t.get("center"), "class": t.get("class"),
            "last_t": time.time()
        })

    # ----------- 픽셀→각속도 유틸 -----------
    @staticmethod
    def _px_to_yaw_rad(err_px_x: float, frame_w: int, cam_hfov_deg: float) -> float:
        if frame_w <= 1: return 0.0
        hfov_rad = math.radians(cam_hfov_deg)
        return (err_px_x / (frame_w * 0.5)) * (hfov_rad * 0.5)

    # ----------- (로그 헬퍼) -----------
    def _log_status_block(
        self,
        tid: Any, tclass: Any,
        yaw_rel_deg: float, wz: float, vx: float,
        follow_dist: float, range_m: Optional[float], err: Optional[float],
        aligned: bool, yaw_deadband_enter_deg: float,
        yaw_rate: float, pitch_rate: float
    ):
        lines = [
            "[TRACK STATUS]",
            f"- Target    : class={tclass}, id={_as_str(tid)}",
            f"- Range     : {_fmt(range_m,'{:.2f}')} m  (follow={_fmt(follow_dist,'{:.2f}')} m, err={_fmt(err,'{:+.2f}')} m)",
            f"- Align     : yaw_rel={_fmt(yaw_rel_deg,'{:.2f}')} deg, aligned={aligned} (≤{_fmt(yaw_deadband_enter_deg,'{:.1f}')}° hold)",
            f"- Command   : wz={_fmt(wz,'{:+.3f}')}, vx={_fmt(vx,'{:+.3f}')}  |  gimbal(y,p)=({_fmt(yaw_rate,'{:+.4f}')}, {_fmt(pitch_rate,'{:+.4f}')})",
        ]
        self.node.get_logger().info("\n" + "\n".join(lines))

    def _log_success_window(self, in_yaw: bool, in_dist: bool,
                            yaw_hold: float, dist_hold: float, both_hold: float,
                            success_yaw_deg: float, dist_tol_m: float, success_hold_sec: float):
        ok = lambda b: "✔" if b else "✗"
        lines = [
            "[SUCCESS WINDOW]",
            f"- Gimbal  : {ok(in_yaw)}  (hold {yaw_hold:.2f}/{success_hold_sec:.2f}s, thr ≤{success_yaw_deg:.1f}°)",
            f"- Distance: {ok(in_dist)} (hold {dist_hold:.2f}/{success_hold_sec:.2f}s, thr ≤{dist_tol_m:.2f} m)",
            f"- BOTH    : {ok(in_yaw and in_dist)} (hold {both_hold:.2f}/{success_hold_sec:.2f}s)",
        ]
        self.node.get_logger().info("\n" + "\n".join(lines))

    def _log_result(self, status: str, reason: str):
        text = "\n" + "\n".join([
            "[RESULT]",
            f"- Status : {status}",
            f"- Reason : {reason}",
        ])
        status_u = (status or "").upper()
        if status_u in ("SUCCESS", "OK", "DONE"):
            self.node.get_logger().info(text)     # ★ 성공은 INFO
        elif status_u in ("TIMEOUT", "CANCELED", "CANCELLED", "FAIL", "FAILED", "ERROR"):
            self.node.get_logger().warn(text)     # ★ 실패/타임아웃은 WARN
        else:
            self.node.get_logger().info(text)

    # ----------- 내부 공통 루프(블로킹/비동기 공용) -----------
    def _control_step(self, cfg: Dict[str, Any]) -> Tuple[bool, bool, float, float, float, float, Optional[float], Optional[float]]:
        """
        한 사이클 제어 실행.
        반환: (aligned, moved, yaw_rel_deg, wz, vx, yaw_rate, range_m, dist_err)
        """
        # ----------- 파라미터(필요 최소치) -----------
        k_yaw        = float(cfg.get("k_yaw_px",   0.001)) #0.00045
        k_yaw_d      = float(cfg.get("k_yaw_d_px", 0.0005)) #0.00025
        yaw_d_lpf_a  = float(cfg.get("yaw_d_lpf_alpha", 0.2))
        k_yaw_i      = float(cfg.get("k_yaw_i_px", 0.0005)) #.00001
        i_cap        = float(cfg.get("yaw_i_cap",  0.08))
        slew_cap     = float(cfg.get("yaw_slew_cap", 4.0))
        cmd_lpf_a    = float(cfg.get("yaw_cmd_lpf_alpha", 0.25))

        k_pitch = float(cfg.get("k_pitch_px", 0.0004))
        gimbal_rate_cap = float(cfg.get("gimbal_rate_cap", 1.2))
        cam_hfov_deg = float(cfg.get("cam_hfov_deg", 70.0))
        px_deadband  = float(cfg.get("px_deadband", 6.0)) #12.0

        kp_align         = float(cfg.get("kp_align", 3.0))
        kd_align         = float(cfg.get("kd_align", 2.0))
        kd_lpf_alpha     = float(cfg.get("kd_lpf_alpha", 0.25))
        wz_cap           = float(cfg.get("wz_cap", 3.0))
        yaw_align_sign   = float(cfg.get("yaw_align_sign", -1.0))

        yaw_deadband_enter_deg = float(cfg.get("yaw_deadband_enter_deg", 4.0))
        yaw_deadband_exit_deg  = float(cfg.get("yaw_deadband_exit_deg",  6.5))
        wz_stop_eps            = float(cfg.get("wz_stop_eps",            0.5))

        follow_dist      = float(cfg.get("follow_dist", 4.5))
        k_follow         = float(cfg.get("k_follow",     0.35))
        vx_cap           = float(cfg.get("vx_cap",       3.0)) #2.0
        vx_min_abs       = float(cfg.get("vx_min_abs",   0.0))
        move_when_yaw_deg= float(cfg.get("move_when_yaw_deg", 10.0)) #8.0

        depth_valid_min  = float(cfg.get("depth_valid_min", 0.2))
        depth_valid_max  = float(cfg.get("depth_valid_max", 20.0))
        depth_center_crop   = float(cfg.get("depth_center_crop", 0.5))
        depth_use_percentile= float(cfg.get("depth_use_percentile", 0.5))
        depth_min_valid_ratio = float(cfg.get("depth_min_valid_ratio", 0.02))

        log_period = float(cfg.get("log_period", 0.5))

        # 1) Vision & 타깃
        v = self.vision.snapshot()
        targets = v.get("targets", []) or []
        chosen  = v.get("primary_id")
        fw      = int(v.get("frame_w", 1280))
        fh      = int(v.get("frame_h", 720))
        cx, cy  = fw * 0.5, fh * 0.5

        cand, meta = self._choose_target(targets, chosen, fw, fh)
        if not cand:
            self._publish_zero_once()
            return False, False, 0.0, 0.0, 0.0, 0.0, None, None

        # sticky
        old_id = self._lock.get("id")
        self._update_lock(cand)
        new_id = self._lock.get("id")
        if _as_str(new_id) != _as_str(old_id):
            self.node.get_logger().info(
                f"[sticky] target id changed: {old_id} -> {new_id} (reason={meta.get('method')})"
            )
        if _as_str(new_id) != _as_str(self._last_target_id_for_i):
            self._last_target_id_for_i = _as_str(new_id)
            self._yaw_i = 0.0

        # 2) 짐벌 제어
        bbox, center = cand.get("bbox"), cand.get("center")
        if bbox and len(bbox) >= 4:
            x, y, w, h = bbox
            err_px_x = (x + w*0.5) - cx
            err_px_y = (y + h*0.5) - cy
        elif isinstance(center, dict) and "x" in center and "y" in center:
            err_px_x = float(center["x"]) - cx
            err_px_y = float(center["y"]) - cy
        else:
            self._publish_zero_once()
            return False, False, 0.0, 0.0, 0.0, 0.0, None, None

        if abs(err_px_x) < px_deadband: err_px_x = 0.0
        if abs(err_px_y) < px_deadband: err_px_y = 0.0

        now_cam = time.time()
        dt2 = max(1e-3, now_cam - self._prev_cam_t)

        derr = (err_px_x - self._yaw_prev_err) / dt2
        self._yaw_d_lpf = (1.0 - yaw_d_lpf_a) * self._yaw_d_lpf + yaw_d_lpf_a * derr

        allow_i = (abs(err_px_x) <= (px_deadband * 3.0))
        yaw_i_next = self._yaw_i
        if k_yaw_i > 0.0 and allow_i:
            yaw_i_next = self._yaw_i + (k_yaw_i * err_px_x * dt2)
            yaw_i_next = _clamp(yaw_i_next, -i_cap, +i_cap)

        yaw_rate_cmd = -(k_yaw * err_px_x + k_yaw_d * self._yaw_d_lpf + yaw_i_next)
        yaw_rate_cmd = (1.0 - cmd_lpf_a) * self._yaw_cmd_prev + cmd_lpf_a * yaw_rate_cmd

        max_step = slew_cap * dt2
        yaw_step = _clamp(yaw_rate_cmd - self._yaw_cmd_prev, -max_step, +max_step)
        yaw_rate_cmd = self._yaw_cmd_prev + yaw_step

        yaw_rate_sat = _clamp(yaw_rate_cmd, -gimbal_rate_cap, gimbal_rate_cap)
        if abs(yaw_rate_cmd) < gimbal_rate_cap * 0.999:
            self._yaw_i = yaw_i_next

        self._yaw_prev_err = err_px_x
        self._yaw_cmd_prev = yaw_rate_sat
        self._prev_cam_t = now_cam

        yaw_rate   = float(yaw_rate_sat)
        pitch_rate = _clamp( k_pitch * err_px_y, -gimbal_rate_cap, gimbal_rate_cap)

        if self.publish_legacy_array and (self.gimbal_pub is not None):
            m = Float64MultiArray(); m.data = [float(yaw_rate), float(pitch_rate)]
            self.gimbal_pub.publish(m)
        tw_ats = Twist()
        tw_ats.angular.z = float(yaw_rate)
        tw_ats.angular.y = float(pitch_rate)
        self.ats_twist_pub.publish(tw_ats)

        # 3) TF yaw_rel → 회전(PD)
        ok_tf, yaw_rel, vxy = self._lookup_yaw_rel(log_period)
        if not ok_tf:
            t0 = Twist(); t0.linear.x = 0.0; t0.angular.z = 0.0
            self.cmd_pub.publish(t0)
            return False, False, 999.0, 0.0, 0.0, yaw_rate, None, None

        now = time.time()
        dt = max(1e-3, now - self._prev_t)
        dyaw = yaw_rel - self._prev_yaw
        dyaw_dt = dyaw / dt
        self._dyaw_lpf = (1.0 - kd_lpf_alpha) * self._dyaw_lpf + kd_lpf_alpha * dyaw_dt

        yaw_err_deg = abs(math.degrees(yaw_rel))
        aligned = False
        if yaw_err_deg <= float(cfg.get("yaw_deadband_enter_deg", 4.0)):
            aligned = True
            wz = 0.0
        else:
            if yaw_err_deg < float(cfg.get("yaw_deadband_exit_deg", 6.5)):
                aligned = True
            wz = float(cfg.get("yaw_align_sign", -1.0)) * (
                float(cfg.get("kp_align", 3.0)) * yaw_rel + float(cfg.get("kd_align", 2.0)) * self._dyaw_lpf
            )

        wz = _clamp(wz, -float(cfg.get("wz_cap", 3.0)), +float(cfg.get("wz_cap", 3.0)))
        if abs(wz) < float(cfg.get("wz_stop_eps", 0.5)) and yaw_err_deg <= (float(cfg.get("yaw_deadband_enter_deg", 4.0)) + 0.5):
            wz = 0.0

        self._prev_yaw = yaw_rel
        self._prev_t   = now

        # 4) Depth ROI → 거리 유지(vx)
        follow_dist = float(cfg.get("follow_dist", 4.0))
        k_follow    = float(cfg.get("k_follow", 0.7)) #0.35
        vx_cap      = float(cfg.get("vx_cap", 2.0))
        vx_min_abs  = float(cfg.get("vx_min_abs", 0.0))
        move_when_yaw_deg = float(cfg.get("move_when_yaw_deg", 8.0))

        depth_valid_min  = float(cfg.get("depth_valid_min", 0.2))
        depth_valid_max  = float(cfg.get("depth_valid_max", 20.0))
        depth_center_crop   = float(cfg.get("depth_center_crop", 0.5))
        depth_use_percentile= float(cfg.get("depth_use_percentile", 0.5))
        depth_min_valid_ratio = float(cfg.get("depth_min_valid_ratio", 0.02))

        bbox = self._lock.get("bbox")
        fw   = int(v.get("frame_w", 1280))
        fh   = int(v.get("frame_h", 720))

        range_m = None
        if bbox and self.depth is not None and self.depth.has_image():
            range_m = self.depth.roi_mean_depth(
                bbox, fw, fh,
                depth_valid_min=depth_valid_min, depth_valid_max=depth_valid_max,
                center_crop=depth_center_crop,
                use_percentile=depth_use_percentile,
                min_valid_ratio=depth_min_valid_ratio
            )

        # ----------- (추가) 거리값 캐싱/재사용 -----------
        now_ts = time.time()
        if (range_m is not None) and (math.isfinite(range_m)):
            self._last_range_m = float(range_m)
            self._last_range_t = now_ts

        use_last_when_none = bool(cfg.get("use_last_range_on_none", True))
        dist_grace_sec     = float(cfg.get("dist_grace_sec", 0.6))
        if (range_m is None) and use_last_when_none:
            if (self._last_range_m is not None) and ((now_ts - self._last_range_t) <= dist_grace_sec):
                range_m = self._last_range_m  # 최근 유효값 재사용

        if (range_m is not None) and (yaw_err_deg <= move_when_yaw_deg):
            dist_err = (range_m - follow_dist)
            vx = _clamp(k_follow * dist_err, -vx_cap, +vx_cap)
            if abs(vx) < vx_min_abs:
                vx = 0.0
        else:
            dist_err = None
            vx = 0.0

        # 명령 송신
        tw = Twist()
        tw.linear.x = float(vx)
        tw.angular.z = float(wz)
        self.cmd_pub.publish(tw)

        return aligned, (abs(vx) > 0.0), yaw_err_deg, wz, vx, yaw_rate, range_m, dist_err

    # ----------- 스레드 메인 -----------
    def _thread_main(self):
        # 초기화
        self._flush_stop()
        self._evt_initial_success.clear()
        self._initial_success_fired = False

        # 내부 타임스텝
        with self._status_lock:
            cfg = dict(self._cfg_params)
            rules = dict(self._cfg_rules)
        hz = float(cfg.get("hz", 60.0))
        dt_loop = 1.0 / max(1.0, hz)
        log_period = float(cfg.get("log_period", 0.5))
        dwell_sec  = float(cfg.get("dwell_sec", 0.8))
        max_time   = float(cfg.get("max_time", 0.0))  # 비동기 모드: 0이면 무제한

        # 초기성공 판단 파라미터 (로깅용 임계값 포함)
        success_yaw_deg   = float(cfg.get("success_yaw_deg", cfg.get("yaw_deadband_enter_deg", 4.0)))
        dist_tol_m        = float(cfg.get("dist_tol_m", 0.5))
        success_hold_sec  = float(cfg.get("success_hold_sec", 1.0))

        # (추가) hold 프리즈 옵션
        freeze_dist_hold_on_none = bool(cfg.get("freeze_dist_hold_on_none", True))

        start_t = time.time()
        yaw_hold  = 0.0   # 짐벌(각도) 조건 유지
        dist_hold = 0.0   # 거리 조건 유지
        both_hold = 0.0   # 동시 조건 유지
        last_log_t = 0.0

        # 내부 PID 상태 리셋
        self._prev_t = time.time()
        self._prev_yaw = 0.0
        self._dyaw_lpf = 0.0
        self._last_log = 0.0

        self._yaw_i = 0.0
        self._yaw_prev_err = 0.0
        self._yaw_d_lpf = 0.0
        self._yaw_cmd_prev = 0.0
        self._prev_cam_t = time.time()
        self._last_target_id_for_i = None

        try:
            while not self._cancel_event.wait(dt_loop):
                with self._status_lock:
                    cfg = dict(self._cfg_params)
                    rules = dict(self._cfg_rules)

                aligned, moved, yaw_err_deg, wz, vx, yaw_rate, range_m, dist_err = self._control_step(cfg)

                # in-yaw / in-dist 판정
                in_yaw  = (yaw_err_deg <= success_yaw_deg)
                in_dist = (range_m is not None) and (abs((range_m - float(cfg.get("follow_dist", 4.0)))) <= dist_tol_m)

                # ── hold 타이머(프리즈 로직 반영)
                # yaw hold
                yaw_hold_next = (yaw_hold + dt_loop) if in_yaw else 0.0

                # dist hold: 결측(None)인데 프리즈면 유지
                if in_dist:
                    dist_hold_next = dist_hold + dt_loop
                else:
                    if (range_m is None) and freeze_dist_hold_on_none:
                        dist_hold_next = dist_hold  # 유지
                    else:
                        dist_hold_next = 0.0

                # both hold: in_yaw AND in_dist 이면 증가
                # 결측(None)이고 프리즈이며, 이전 dist_hold가 양수면 '유지'로 해석해 both_hold는 리셋하지 않음
                if in_yaw and in_dist:
                    both_hold_next = both_hold + dt_loop
                else:
                    if (in_yaw and (range_m is None) and freeze_dist_hold_on_none and (dist_hold > 0.0)):
                        both_hold_next = both_hold  # 유지(증가X, 리셋X)
                    else:
                        both_hold_next = 0.0

                yaw_hold, dist_hold, both_hold = yaw_hold_next, dist_hold_next, both_hold_next

                # 상태 업데이트
                with self._status_lock:
                    self._status.update({
                        "aligned": (yaw_err_deg <= float(cfg.get("yaw_deadband_enter_deg", 4.0))),
                        "in_dist": in_dist,
                        "yaw_err_deg": yaw_err_deg,
                        "range_m": range_m,
                        "dist_err": (None if range_m is None else (range_m - float(cfg.get("follow_dist", 4.0)))),
                        "target": {"id": self._lock.get("id"), "class": self._lock.get("class")},
                        "vx": vx, "wz": wz,
                        "timestamp": time.time(),
                    })

                # 로그
                now = time.time()
                if now - last_log_t > log_period:
                    self._log_status_block(
                        tid=self._lock.get("id"), tclass=self._lock.get("class"),
                        yaw_rel_deg=yaw_err_deg, wz=wz, vx=vx,
                        follow_dist=float(cfg.get("follow_dist", 4.0)),
                        range_m=range_m, err=(None if range_m is None else range_m - float(cfg.get("follow_dist", 4.0))),
                        aligned=(yaw_err_deg <= float(cfg.get("yaw_deadband_enter_deg", 4.0))),
                        yaw_deadband_enter_deg=float(cfg.get("yaw_deadband_enter_deg", 4.0)),
                        yaw_rate=yaw_rate, pitch_rate=0.0  # pitch 로그 필요시 추가
                    )
                    self._log_success_window(
                        in_yaw=in_yaw, in_dist=in_dist,
                        yaw_hold=yaw_hold, dist_hold=dist_hold, both_hold=both_hold,
                        success_yaw_deg=success_yaw_deg, dist_tol_m=dist_tol_m, success_hold_sec=success_hold_sec
                    )
                    last_log_t = now

                # (옵션) 정렬만 성공 이벤트 로그
                if yaw_hold >= dwell_sec:
                    # 원샷(한 번만)로 하고 싶으면:
                    if not self._align_success_logged and bool(cfg.get("log_align_success_once", True)):
                        self._log_result("SUCCESS",
                                        f"Gimbal aligned ≤{float(cfg.get('yaw_deadband_enter_deg',4.0)):.1f}° for {dwell_sec:.2f}s")
                        self._align_success_logged = True

                    # 혹은 쿨다운 방식(예: 3초마다 한 번만 찍기)
                    cd = float(cfg.get("align_success_log_cooldown_sec", 0.0))  # 기본 0=미사용
                    if cd > 0.0 and now >= self._align_success_cooldown_until:
                        self._log_result("SUCCESS",
                                        f"Gimbal aligned ≤{float(cfg.get('yaw_deadband_enter_deg',4.0)):.1f}° for {dwell_sec:.2f}s")
                        self._align_success_cooldown_until = now + cd

                # 초기 성공(정렬+거리) 1회 트리거
                if (not self._initial_success_fired) and (both_hold >= success_hold_sec):
                    self._initial_success_fired = True
                    self._evt_initial_success.set()
                    self.node.get_logger().info(
                        f"[track] INITIAL SUCCESS (yaw≤{success_yaw_deg:.1f}°, |dist_err|≤{dist_tol_m:.2f}m for {success_hold_sec:.2f}s)"
                    )
                    # 이후에도 계속 추적 유지

                # (옵션) 최대시간
                if max_time > 0.0 and (now - start_t) > max_time:
                    self._log_result("TIMEOUT", "Exceeded max_time (async)")
                    break

        finally:
            self._flush_stop()

    # ----------- 블로킹 run (하위호환) -----------
    def run(self, params: Dict[str, Any], rules: Dict[str, Any]) -> bool:
        """구버전: 블로킹 실행. 내부 루프는 기존과 동일하며, ALIGN 성공 로그를 남기고 True/False 반환."""
        # 비동기와 동일 파라미터 사용
        cfg = dict(params or {})
        hz = float(cfg.get("hz", 60.0))
        dt_loop = 1.0 / max(1.0, hz)
        log_period = float(cfg.get("log_period", 0.5))
        dwell_sec  = float(cfg.get("dwell_sec", 0.8))
        max_time   = float(cfg.get("max_time", 90.0)) #타임아웃 없애고 싶은 경우 0넣으면 됨

        start_t = time.time()
        yaw_hold = 0.0
        last_log_t = 0.0

        # 초기화
        self._flush_stop()
        self._cancel_event.clear()

        # 내부 PID 상태 리셋
        self._prev_t = time.time()
        self._prev_yaw = 0.0
        self._dyaw_lpf = 0.0
        self._last_log = 0.0

        self._yaw_i = 0.0
        self._yaw_prev_err = 0.0
        self._yaw_d_lpf = 0.0
        self._yaw_cmd_prev = 0.0
        self._prev_cam_t = time.time()
        self._last_target_id_for_i = None

        try:
            while True:
                if self._cancel_event.wait(dt_loop):
                    self.node.get_logger().info("[track] CANCEL 수신 → 즉시 중단")
                    self._flush_stop()
                    self._log_result("CANCELED", "User requested cancel")
                    return False

                aligned, moved, yaw_err_deg, wz, vx, yaw_rate, range_m, dist_err = self._control_step(cfg)

                now = time.time()
                if now - last_log_t > log_period:
                    self._log_status_block(
                        tid=self._lock.get("id"), tclass=self._lock.get("class"),
                        yaw_rel_deg=yaw_err_deg, wz=wz, vx=vx,
                        follow_dist=float(cfg.get("follow_dist", 4.0)),
                        range_m=range_m, err=dist_err,
                        aligned=(yaw_err_deg <= float(cfg.get("yaw_deadband_enter_deg", 4.0))),
                        yaw_deadband_enter_deg=float(cfg.get("yaw_deadband_enter_deg", 4.0)),
                        yaw_rate=yaw_rate, pitch_rate=0.0
                    )
                    last_log_t = now

                if yaw_err_deg <= float(cfg.get("yaw_deadband_enter_deg", 4.0)):
                    yaw_hold += dt_loop
                    if yaw_hold >= dwell_sec:
                        self._log_result("SUCCESS", f"Aligned ≤{float(cfg.get('yaw_deadband_enter_deg',4.0)):.1f}° for {dwell_sec:.2f}s")
                        return True
                else:
                    yaw_hold = 0.0

                if (now - start_t) > max_time:
                    self._log_result("TIMEOUT", "Exceeded max_time")
                    return False

        finally:
            self._flush_stop()
