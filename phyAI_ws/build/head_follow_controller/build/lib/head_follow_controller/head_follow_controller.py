#!/usr/bin/env python3
import math, time, signal, atexit
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
import rclpy.time

try:
    from ats_system1.utils.geometry import quat_to_yaw
except Exception:
    def quat_to_yaw(q) -> float:
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

def _quat_to_rotmat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    # 3x3 회전행렬 (row-major, base<-camera)
    return [
        [1 - 2*(yy+zz),     2*(xy - wz),     2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx+zz),     2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx+yy)],
    ]

def _axis_vec(name: str):
    name = (name or "").strip().lower()
    if name == 'x':   return (1.0, 0.0, 0.0)
    if name == '-x':  return (-1.0, 0.0, 0.0)
    if name == 'y':   return (0.0, 1.0, 0.0)
    if name == '-y':  return (0.0, -1.0, 0.0)
    if name == 'z':   return (0.0, 0.0, 1.0)
    if name == '-z':  return (0.0, 0.0, -1.0)
    # 기본: 카메라 Z forward (ROS optical frame)
    return (0.0, 0.0, 1.0)

class HeadFollow(Node):
    """
    카메라(ATS 짐벌) 헤딩에 맞춰 바디가 회전/전진.
    yaw_rel = (카메라 광학축을 base로 변환한 벡터의 XY 평면 방위각)
      - 왼쪽(−), 오른쪽(+) 부호 일관성 유지
      - 빠른 추종을 위해 회전에 D항 포함 (Kd * d(yaw)/dt)
    종료/신호/예외에도 반드시 정지 명령을 여러 번 송신.
    """
    def __init__(self):
        super().__init__('head_follow_controller')

        # ---------- 파라미터 (속도 상향 기본값) ----------
        self.declare_parameter('cmd_topic',           '/cmd_vel')
        self.declare_parameter('tf_base_frame',       'body')
        self.declare_parameter('tf_camera_frame',     'Camera')
        self.declare_parameter('camera_forward_axis', 'z')   # 'z', '-z', 'x', '-x', ...

        # 회전 응답 (더 빠르게)
        self.declare_parameter('kp_align',            3.5)   # ↑ Kp
        self.declare_parameter('kd_align',            0.5)   # ↑ D항
        self.declare_parameter('kd_lpf_alpha',        0.35)  # D항 LPF
        self.declare_parameter('wz_cap',              2.5)   # ↑ 최대 회전속도
        self.declare_parameter('yaw_deadband_deg',    0.3)   # ↓ 데드밴드
        self.declare_parameter('yaw_align_sign',      1.0)   # 부호 필요한 경우 ±1.0

        # 전진 응답 (더 빠르게)
        self.declare_parameter('vx_max',              2.5)   # ↑ 최대 전진속도
        self.declare_parameter('vx_min',              0.15)  # 최소 전진 보장
        self.declare_parameter('vx_curve',            'linear')  # 보수성 완화('cos'보다 덜 깎임)

        # 주기 & 로그
        self.declare_parameter('hz',                  60.0)  # ↑ 제어주기
        self.declare_parameter('log_period',          0.5)

        # 읽기
        self.cmd_topic         = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.tf_base_frame     = self.get_parameter('tf_base_frame').get_parameter_value().string_value
        self.tf_camera_frame   = self.get_parameter('tf_camera_frame').get_parameter_value().string_value
        self.camera_forward_ax = self.get_parameter('camera_forward_axis').get_parameter_value().string_value

        self.kp_align          = float(self.get_parameter('kp_align').value)
        self.kd_align          = float(self.get_parameter('kd_align').value)
        self.kd_lpf_a          = float(self.get_parameter('kd_lpf_alpha').value)
        self.wz_cap            = float(self.get_parameter('wz_cap').value)
        self.yaw_deadband_deg  = float(self.get_parameter('yaw_deadband_deg').value)
        self.yaw_align_sign    = float(self.get_parameter('yaw_align_sign').value)

        self.vx_max            = float(self.get_parameter('vx_max').value)
        self.vx_min            = float(self.get_parameter('vx_min').value)
        self.vx_curve          = str(self.get_parameter('vx_curve').value).lower()

        self.hz                = float(self.get_parameter('hz').value)
        self.log_period        = float(self.get_parameter('log_period').value)

        # ---------- Pub / TF ----------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 상태
        self._last_log    = 0.0
        self._prev_yaw    = 0.0
        self._prev_t      = time.time()
        self._dyaw_lpf    = 0.0
        self._running     = True  # deadman (노드 살아있을 때만 움직이게)

        # 시작 시 안전 정지 신호 여러 번
        self._flush_stop(prefix="[INIT]")

        # ---------- 타이머 ----------
        dt = 1.0 / max(1.0, self.hz)
        self.timer = self.create_timer(dt, self._on_timer)

        # 종료 훅 (Ctrl+C, kill, atexit)
        atexit.register(self._flush_stop, "[ATEXIT]")
        signal.signal(signal.SIGINT,  self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

        self.get_logger().info(
            f"[INIT] cmd_topic={self.cmd_topic}, base={self.tf_base_frame}, cam={self.tf_camera_frame}, "
            f"Kp={self.kp_align}, Kd={self.kd_align}, wz_cap={self.wz_cap}, yaw_sign={self.yaw_align_sign}, "
            f"vx_max={self.vx_max}, vx_min={self.vx_min}, curve={self.vx_curve}, Hz={self.hz}, cam_fwd={self.camera_forward_ax}"
        )

    # ----------- 안전 정지 관련 -----------
    def _publish_zero_once(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.cmd_pub.publish(t)

    def _flush_stop(self, prefix="[STOP]"):
        # 컨트롤러가 마지막 명령을 유지하는 경우 대비: 여러 번 0 전송
        for _ in range(5):
            self._publish_zero_once()
            time.sleep(0.02)
        try:
            self.get_logger().info(f"{prefix} cmd_vel ZERO flushed")
        except Exception:
            pass

    def _on_signal(self, signum, frame):
        # 신호 들어오면 즉시 정지
        try:
            self.get_logger().warn(f"[SIGNAL] {signum} received → STOP")
        except Exception:
            pass
        self._running = False
        self._flush_stop(prefix="[SIGNAL]")
        # 노드가 이 안에서 바로 shutdown하면 콜스택에 따라 rclpy가 이미 내려가 있을 수 있음
        # 여기서는 정지만 확실히 보장

    # ============== TF에서 yaw_rel 계산 ==============
    def _lookup_yaw_rel(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_base_frame, self.tf_camera_frame, rclpy.time.Time()
            )
            R = _quat_to_rotmat(tf.transform.rotation)  # base<-camera
            fx, fy, fz = _axis_vec(self.camera_forward_ax)
            vx = R[0][0]*fx + R[0][1]*fy + R[0][2]*fz
            vy = R[1][0]*fx + R[1][1]*fy + R[1][2]*fz
            # atan2는 CCW 양수 → 본 요구사항(왼쪽 −, 오른쪽 +)을 위해 부호 반전
            yaw_rel = -math.atan2(vy, vx)
            return True, float(yaw_rel), (vx, vy)
        except Exception as e:
            now = time.time()
            if now - self._last_log > self.log_period:
                self.get_logger().warn(f"[TF] lookup failed ({self.tf_base_frame}->{self.tf_camera_frame}): {e}")
                self._last_log = now
            return False, 0.0, (0.0, 0.0)

    # ----------- 주기 제어 -----------
    def _on_timer(self):
        if not self._running:
            # 안전: 러닝 플래그가 내려갔으면 계속 0 출력
            self._publish_zero_once()
            return

        ok, yaw_rel, vxy = self._lookup_yaw_rel()
        if not ok:
            self._publish_zero_once()
            return

        # ----------- 회전: PD (Kp*yaw + Kd*dyaw/dt) -----------
        now = time.time()
        dt = max(1e-3, now - self._prev_t)
        dyaw = yaw_rel - self._prev_yaw
        dyaw_dt = dyaw / dt
        # D항 LPF로 노이즈 완화
        self._dyaw_lpf = (1.0 - self.kd_lpf_a) * self._dyaw_lpf + self.kd_lpf_a * dyaw_dt

        yaw_err_deg = abs(math.degrees(yaw_rel))
        if yaw_err_deg <= self.yaw_deadband_deg:
            wz = 0.0
        else:
            wz = self.yaw_align_sign * (self.kp_align * yaw_rel + self.kd_align * self._dyaw_lpf)
            wz = max(-self.wz_cap, min(self.wz_cap, wz))

        self._prev_yaw = yaw_rel
        self._prev_t   = now

        # ----------- 전진 속도: 덜 보수적으로 게이팅 -----------
        if self.vx_curve == 'cos':
            gate = max(0.0, math.cos(abs(yaw_rel)))
        else:
            # 선형 게이트 + 감쇠완화(지수)로 큰 각도에서도 속도 유지
            base = max(0.0, 1.0 - abs(yaw_rel) / math.radians(70.0))
            gate = base ** 0.7  # 0.6~0.9 권장

        vx = self.vx_min + (self.vx_max - self.vx_min) * gate

        # 송신
        self._publish(vx, wz)

        # 로그
        if now - self._last_log > self.log_period:
            turn = "LEFT(CCW)" if wz > 0 else ("RIGHT(CW)" if wz < 0 else "STOP")
            self.get_logger().info(
                f"[CTRL] yaw_rel={math.degrees(yaw_rel):5.1f}deg | wz={wz:+.3f} rad/s ({turn}) | "
                f"vx={vx:.3f} m/s | vxy=({vxy[0]:+.2f},{vxy[1]:+.2f}) | cam_fwd={self.camera_forward_ax} | sign={self.yaw_align_sign:+.1f}"
            )
            self._last_log = now

    def _publish(self, vx: float, wz: float):
        t = Twist()
        t.linear.x  = float(vx)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)

    # 노드 파괴 시에도 정지 보장
    def destroy_node(self):
        try:
            self._running = False
            self._flush_stop(prefix="[DESTROY]")
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = HeadFollow()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
