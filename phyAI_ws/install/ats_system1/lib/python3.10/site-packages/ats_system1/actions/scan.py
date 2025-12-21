#!/usr/bin/env python3
import math, time, json
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Twist
from tf2_ros import Buffer

# ----------- TF 유틸 -----------
def _quat_to_rotmat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return [
        [1 - 2*(yy+zz),     2*(xy - wz),     2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx+zz),     2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx+yy)],
    ]

def _axis_vec(name: str):
    n = (name or "").strip().lower()
    if   n == 'x':  return (1.0, 0.0, 0.0)
    elif n == '-x': return (-1.0, 0.0, 0.0)
    elif n == 'y':  return (0.0, 1.0, 0.0)
    elif n == '-y': return (0.0, -1.0, 0.0)
    elif n == 'z':  return (0.0, 0.0, 1.0)
    elif n == '-z': return (0.0, 0.0, -1.0)
    return (0.0, 0.0, 1.0)

def _lookup_yaw_rel(tf_buffer: Buffer, base: str, cam: str, camera_forward_axis: str, node: Node) -> Tuple[bool, float]:
    try:
        tf = tf_buffer.lookup_transform(base, cam, rclpy.time.Time())
        R = _quat_to_rotmat(tf.transform.rotation)  # base<-camera
        fx, fy, fz = _axis_vec(camera_forward_axis)
        vx = R[0][0]*fx + R[0][1]*fy + R[0][2]*fz
        vy = R[1][0]*fx + R[1][1]*fy + R[1][2]*fz
        yaw_rel = -math.atan2(vy, vx)   # 좌:-, 우:+
        return True, float(yaw_rel)
    except Exception as e:
        node.get_logger().warn(f"[scan] TF lookup failed ({base}->{cam}): {e}")
        return False, 0.0

def _clamp(x, lo, hi):
    return hi if x > hi else (lo if x < lo else x)

# ----------- /vision_context_raw 구독 -----------
class VisionCache:
    def __init__(self, node: Node, topic: str = "/vision_context_raw"):
        self._node = node
        self._latest: Dict[str, Any] = {"objects": []}
        node.create_subscription(String, topic, self._on_msg, 50)
        node.get_logger().info(f"[vision] subscribing {topic}")

    def _on_msg(self, msg: String):
        try:
            raw = json.loads(msg.data)
        except Exception as e:
            self._node.get_logger().warn(f"[vision] JSON parse failed: {e}")
            return
        objs_in = raw.get("objects") or raw.get("targets") or raw.get("detections") or []
        objs: List[Dict[str, Any]] = []
        for o in objs_in:
            cls_ = o.get("class") or o.get("label") or o.get("name")
            tid  = o.get("id") or o.get("track_id")
            center = o.get("center")
            bbox   = o.get("bbox") or o.get("xywh")
            if isinstance(bbox, dict):
                bbox = [float(bbox.get("x", 0.0)), float(bbox.get("y", 0.0)),
                        float(bbox.get("w", 0.0)), float(bbox.get("h", 0.0))]
                if center is None:
                    center = {"x": bbox[0] + bbox[2]*0.5, "y": bbox[1] + bbox[3]*0.5}
            if cls_ is None:
                continue
            cls_norm = cls_.lower() if isinstance(cls_, str) else str(cls_)
            objs.append({
                "class": cls_norm,
                "id": (str(tid) if tid is not None else None),
                "center": center,
                "bbox": bbox,
            })
        self._latest = {
            "frame_w": raw.get("frame_w") or raw.get("width"),
            "frame_h": raw.get("frame_h") or raw.get("height"),
            "objects": objs,
        }

    def snapshot(self) -> Dict[str, Any]:
        return dict(self._latest)

# ----------- 핵심 실행 -----------
def exec_scan(
    node: Node,
    cmd_pub,
    sweep_deg: float,
    yaw_rate_dps: float,
    *,
    pitch_deg_up: float = 0.0,
    pitch_deg_down: float = 0.0,
    duration_sec: float = 20.0,
    watch_classes: Optional[List[Any]] = None,
    watch_ids: Optional[List[Any]] = None,
    report_on_found: bool = True,
    tf_base_frame: str = "body",
    tf_camera_frame: str = "Camera",
    camera_forward_axis: str = "z",
    gimbal_yaw_sign: float = -1.0,
    vision_topic: str = "/vision_context_raw",
    report_topic: str = "/scan_report",

    # ALIGN PID
    align_deadband_deg: float = 2.0,
    align_hold_sec: float = 0.4,
    align_timeout_sec: float = 4.0,
    k_align_p: float = 1.2,
    k_align_i: float = 0.15,
    k_align_d: float = 0.35,
    align_i_cap: float = 0.12,
    yaw_cmd_lpf_alpha: float = 0.25,
    yaw_slew_dps2: float = 120.0,
    align_rate_dps_max: Optional[float] = None,
) -> bool:
    node.get_logger().info("[scan] exec_scan ENTERED (debug marker A)")


    # 파라미터
    sweep_rad = math.radians(max(0.0, float(sweep_deg)))
    half_span = 0.5 * sweep_rad
    if half_span <= 1e-4:
        node.get_logger().warn("[scan] sweep_deg too small; nothing to do")
        return True
    yaw_rate_max = math.radians(abs(float(yaw_rate_dps)))
    align_rate_max = math.radians(abs(float(align_rate_dps_max if align_rate_dps_max is not None else yaw_rate_dps)))

    tf_buffer = getattr(node, "tf_buffer", None)
    if tf_buffer is None:
        node.get_logger().error("[scan] tf_buffer is None")
        return False

    # 퍼블리셔/구독자
    gimbal_pub = getattr(node, "pub_gimbal", None)
    ats_twist_pub = getattr(node, "create_publisher")(Twist, "/ats_twist", 10)
    report_pub = getattr(node, "create_publisher")(String, report_topic, 10)
    vision = VisionCache(node, vision_topic)
    snapshot_req_pub = node.create_publisher(String, '/vision/snapshot_req', 10)

    # 유틸
    def publish_gimbal(yaw_rate: float, pitch_rate: float):
        yaw_rate = float(gimbal_yaw_sign) * float(yaw_rate)
        if gimbal_pub is not None:
            m = Float64MultiArray(); m.data = [float(yaw_rate), float(pitch_rate)]
            gimbal_pub.publish(m)
        tw = Twist()
        tw.angular.z = float(yaw_rate)
        tw.angular.y = float(pitch_rate)
        ats_twist_pub.publish(tw)

    def stop_all():
        try:
            publish_gimbal(0.0, 0.0)
            t = Twist(); t.linear.x = 0.0; t.angular.z = 0.0
            cmd_pub.publish(t)
        except Exception:
            pass

    # 루프 타이밍
    hz = 60.0
    dt = 1.0 / hz
    t0 = time.time()
    last_log = 0.0
    last_vision_log = 0.0
    found_once = False

    # 발견 처리(로그+토픽+_do_report)
    def handle_found(obj: Dict[str, Any]):
        nonlocal found_once
        if found_once:
            return
        found_once = True

        msg = {
            "event": "FOUND",
            "class": obj.get("class"),
            "id": obj.get("id"),
            "center": obj.get("center"),
            "bbox": obj.get("bbox"),
            "range_m": obj.get("range_m"), # 추가
            "time": time.time(),
        }
        node.get_logger().info(f"[scan] FOUND → {msg}")
        try:
            report_pub.publish(String(data=json.dumps(msg)))
        except Exception as e:
            node.get_logger().warn(f"[scan] report publish failed: {e}")

        # VisionContextBuilder에게 스냅샷 요청 전송
        # 이 메시지를 VisionContextBuilder가 받아서 이미지와 함께 System2로 쏨
        try:
            snapshot_req_pub.publish(String(data=json.dumps(msg)))
            node.get_logger().info("[scan] Requested SNAPSHOT to VisionContextBuilder")
        except Exception as e:
            node.get_logger().warn(f"[scan] snapshot req failed: {e}")

        if report_on_found and hasattr(node, "_do_report"):
            node.get_logger().info("[scan] report_on_found → _do_report() 호출")
            try:
                node._do_report({"delay_sec": 0.2})
            except Exception as e:
                node.get_logger().warn(f"[scan] _do_report failed: {e}")

    try:
        # ----------- ALIGN: yaw_rel → 0° -----------
        deadband = math.radians(float(align_deadband_deg))
        i_term = 0.0
        d_lpf  = 0.0
        prev_err = None
        prev_cmd = 0.0
        last_ok_t: Optional[float] = None
        align_t0 = time.time()

        while rclpy.ok():
            if (time.time() - t0) > float(duration_sec):
                node.get_logger().info("[scan] duration elapsed during ALIGN → done")
                stop_all(); return True
            if (time.time() - align_t0) > float(align_timeout_sec):
                node.get_logger().warn("[scan] ALIGN timeout → continue to SWEEP anyway")
                break

            ok, yaw_rel = _lookup_yaw_rel(tf_buffer, tf_base_frame, tf_camera_frame, camera_forward_axis, node)
            if not ok:
                stop_all(); time.sleep(dt); continue

            err = -yaw_rel
            err_eff = 0.0 if abs(err) <= deadband else err

            # I (deadband 근처에서만 적분)
            if abs(err) <= (3.0 * deadband):
                i_term += k_align_i * err_eff * dt
                i_term = _clamp(i_term, -align_i_cap, +align_i_cap)
            else:
                i_term = 0.0

            # D (LPF)
            derr = 0.0 if prev_err is None else (err - prev_err) / dt
            d_lpf = (1.0 - 0.2) * d_lpf + 0.2 * derr

            yaw_rate_cmd = (k_align_p * err_eff) + (k_align_d * d_lpf) + i_term
            yaw_rate_cmd = (1.0 - yaw_cmd_lpf_alpha) * prev_cmd + yaw_cmd_lpf_alpha * yaw_rate_cmd

            max_step = math.radians(yaw_slew_dps2) * dt
            yaw_rate_cmd = _clamp(yaw_rate_cmd, prev_cmd - max_step, prev_cmd + max_step)
            yaw_rate_cmd = _clamp(yaw_rate_cmd, -align_rate_max, +align_rate_max)

            publish_gimbal(yaw_rate_cmd, 0.0)

            # hold 판정
            if abs(err) <= deadband:
                if last_ok_t is None:
                    last_ok_t = time.time()
                elif (time.time() - last_ok_t) >= float(align_hold_sec):
                    node.get_logger().info("[scan] ALIGN done (yaw≈0°)")
                    break
            else:
                last_ok_t = None

            if (time.time() - last_log) > 0.8:
                node.get_logger().info(
                    f"[scan/ALIGN] yaw_rel={math.degrees(yaw_rel):+.1f}°, "
                    f"err={math.degrees(err):+.1f}°, cmd={math.degrees(yaw_rate_cmd):+.1f}°/s"
                )
                last_log = time.time()

            prev_err = err
            prev_cmd = yaw_rate_cmd
            time.sleep(dt)

        publish_gimbal(0.0, 0.0)
        time.sleep(0.05)

        # ───────── SWEEP: ±sweep/2 왕복 ─────────
        k_yaw    = 1.5
        eps_edge = math.radians(1.5)
        target_yaw = -half_span
        pitch_dir  = +1.0

        while rclpy.ok():
            now = time.time()
            if (now - t0) > float(duration_sec):
                node.get_logger().info("[scan] duration elapsed → done")
                stop_all(); return True

            ok, yaw_rel = _lookup_yaw_rel(tf_buffer, tf_base_frame, tf_camera_frame, camera_forward_axis, node)
            if not ok:
                stop_all(); time.sleep(dt); continue

            # 경계 반전
            if abs((-half_span) - yaw_rel) <= eps_edge:
                target_yaw = +half_span; pitch_dir *= -1.0
            elif abs((+half_span) - yaw_rel) <= eps_edge:
                target_yaw = -half_span; pitch_dir *= -1.0

            yaw_err = target_yaw - yaw_rel
            yaw_rate_cmd = _clamp(k_yaw * yaw_err, -yaw_rate_max, +yaw_rate_max)

            pitch_speed = yaw_rate_max * 0.2
            pitch_rate_cmd = pitch_speed * pitch_dir

            publish_gimbal(yaw_rate_cmd, pitch_rate_cmd)

            # 비전 확인
            snap = vision.snapshot()
            objs = snap.get("objects") or []
            if objs and (watch_classes or watch_ids):
                cls_set = {(c.lower() if isinstance(c, str) else str(c)) for c in (watch_classes or [])}
                id_set  = {str(i) for i in (watch_ids or [])}
                for o in objs:
                    oc = o.get("class")
                    oid = o.get("id")
                    if (cls_set and oc in cls_set) or (id_set and oid in id_set):
                        handle_found(o)
                        stop_all()
                        return True

            if now - last_vision_log > 0.6:
                node.get_logger().info(f"[scan] vision objects={len(objs)}")
                if objs:
                    # 첫 객체의 요약(확인용)
                    o0 = objs[0]
                    node.get_logger().info(f"[scan] vision sample → class={o0.get('class')}, id={o0.get('id')}, center={o0.get('center')}")
                last_vision_log = now

            if now - last_log > 0.8:
                node.get_logger().info(
                    f"[scan/SWEEP] yaw_rel={math.degrees(yaw_rel):+.1f}°, "
                    f"target={math.degrees(target_yaw):+.1f}°, "
                    f"cmd_yaw_rate={math.degrees(yaw_rate_cmd):+.1f}°/s"
                )
                last_log = now

            time.sleep(dt)

    except KeyboardInterrupt:
        stop_all(); return False
    except Exception as e:
        node.get_logger().warn(f"[scan] exception: {e}")
        stop_all(); return False
