#!/usr/bin/env python3
import json
import threading
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from jsonschema import Draft202012Validator

from ats_system1.core.plan_schema import HIGH_LEVEL_PLAN_SCHEMA
from ats_system1.core.guards import eval_guard
from ats_system1.core.replan import emit_replan
from ats_system1.core.vision_context import VisionCache
from ats_system1.core.state_publisher import make_state
from ats_system1.io.topics import create_publishers, create_subscriptions
from ats_system1.utils.geometry import quat_to_yaw

# 단위 액션들
from ats_system1.actions.move_to import Nav2Navigator, exec_move_to
from ats_system1.actions.scan import exec_scan
from ats_system1.actions.track import Tracker, DepthBuffer
from ats_system1.actions.return_to_home import exec_return_to_home
from ats_system1.actions.report_and_wait import exec_report_and_wait


class System1ExecutorNode(Node):
    """
    System-1 Executor (6 Unit Actions 전용)

    지원 Task:
      - move_to         : 지정 위치로 이동 (Nav2 기반)
      - scan            : ATS로 주변 스캔
      - report_and_wait : 현재 상황 보고 + System2 응답 대기
      - track           : 타겟 추적 (Tracker)
      - return_to_home  : Home 위치로 복귀(move_to 재사용)
    """

    def __init__(self):
        super().__init__("system1_executor_node")

        # ---------- TF ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._last_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0, "ok": False}
        # 주기적으로 포즈는 업데이트하지만, 로그는 debug로만 남김
        self.create_timer(1.0, self._log_tf_pose)

        # ---------- 상태 ----------
        self.validator = Draft202012Validator(HIGH_LEVEL_PLAN_SCHEMA)
        self.current_plan: Optional[Dict[str, Any]] = None
        self.current_index = -1
        self.queue_status = "idle"
        self.mission_id = ""
        self.execution_generation = 0

        # 필요시 guard에서 사용하는 심볼
        self.ROE_OK = False
        self.SAFE_BACKSTOP = True
        self.BATTERY_SOC = 0.5
        self.MAX_SPEED = 0.6

        # ---------- Vision Frame 기본값 ----------
        self.declare_parameter("frame_w_default", 1280)
        self.declare_parameter("frame_h_default", 720)
        self._frame_w_default = int(self.get_parameter("frame_w_default").value)
        self._frame_h_default = int(self.get_parameter("frame_h_default").value)

        # ---------- Home Pose (return_to_home용) ----------
        self.declare_parameter("home_x", 8.0)
        self.declare_parameter("home_y", -10.0)
        self.declare_parameter("home_yaw", 0.0)
        self._home_pose = {
            "x": float(self.get_parameter("home_x").value),
            "y": float(self.get_parameter("home_y").value),
            "yaw": float(self.get_parameter("home_yaw").value),
        }

        # ---------- Pub/Sub ----------
        pubs = create_publishers(self)
        self.pub_ats = pubs["ats"]
        self.pub_replan = pubs["replan"]
        self.pub_cmd = pubs["cmd_vel"]
        self.pub_gimbal = pubs["gimbal"]

        create_subscriptions(self, self.on_plan_cmd, self.on_vision)
        self.create_subscription(String, "/vision_context_raw", self.on_vision_raw, 10)

        # ---------- System2로 report 컨텍스트 보내는 토픽 ----------
        self.pub_report_context = self.create_publisher(
            String,
            "/system2/report_context",
            10,
        )

        # ---------- Vision 캐시 ----------
        self.vision = VisionCache(self._frame_w_default, self._frame_h_default)

        # ---------- DepthBuffer / Tracker ----------
        self.depth = DepthBuffer(self, topic="/depth")
        self.tracker = Tracker(
            self,
            self.pub_cmd,
            self.pub_gimbal,
            self.vision,
            self.depth,
            ats_twist_topic="/ats_twist",
            publish_legacy_array=True,
            tf_buffer=self.tf_buffer,
            tf_base_frame="body",
            tf_camera_frame="Camera",
            use_tf_align=True,
            camera_forward_axis="z",
        )

        # ---------- Nav2 (move_to용) ----------
        self._nav_feedback = {"distance_remaining": None, "stamp": time.time()}
        self.navigator = Nav2Navigator(self, self._nav_feedback.update)

        # ---------- 상태 퍼블리시 ----------
        self.lock = threading.Lock()
        self.create_timer(0.5, self.publish_state)

        self.get_logger().info("System-1 Executor up (6 unit actions).")

    # ----------- 유틸: 하드 스톱 -----------
    def _hard_stop(self):
        """모든 구동 정지(바디, 짐벌, 네비). 큐는 유지."""
        # 1) Tracker 정지
        try:
            if hasattr(self.tracker, "stop"):
                self.tracker.stop(flush=True)
            elif hasattr(self.tracker, "request_cancel"):
                self.tracker.request_cancel()
        except Exception as e:
            self.get_logger().warn(f"[cancel] tracker stop failed: {e}")

        # 2) Tracker 내부 flush 사용 (가능시)
        try:
            if hasattr(self.tracker, "_flush_stop"):
                self.tracker._flush_stop(times=6, sleep_s=0.02)
        except Exception:
            pass

        # 3) Nav2 goal 취소
        try:
            if hasattr(self.navigator, "cancel_all"):
                self.navigator.cancel_all()
        except Exception as e:
            self.get_logger().warn(f"[cancel] navigator.cancel_all 실패: {e}")

    # ----------- Plan 콜백 -----------
    def on_plan_cmd(self, msg):
        # msg.plan_json 기반 (ATS 상위에서 JSON string 전달한다고 가정)
        try:
            plan = json.loads(msg.plan_json)
        except Exception as e:
            self.get_logger().error(f"[Plan] JSON 파싱 실패: {e}")
            return

        # intent == cancel → 즉시 정지
        if isinstance(plan, dict) and plan.get("intent") == "cancel":
            self.get_logger().info("[Plan] CANCEL intent 수신 → 실행 중 작업 즉시 중단")
            self._hard_stop()
            with self.lock:

                self.execution_generation += 1
                self.current_plan = None
                self.current_index = -1
                self.mission_id = plan.get("mission_id", "")
                self.queue_status = "idle"
            self.publish_state()
            return
        
        self.get_logger().info("[Plan] 새 플랜 수신 → 현재 액션들(hard stop) 후 재시작")
        self._hard_stop()

        # 스키마 검증 (HIGH_LEVEL_PLAN_SCHEMA는 6개 task enum 기준)
        errs = sorted(self.validator.iter_errors(plan), key=lambda e: e.path)
        if errs:
            for er in errs:
                self.get_logger().error(
                    f"[Plan] 스키마 오류: {list(er.path)} -> {er.message}"
                )
            emit_replan(
                self,
                self.pub_replan,
                self.mission_id,
                "SCHEMA_INVALID",
                {"errors": [er.message for er in errs]},
            )
            return

        # 새 플랜 요약 로그
        steps = plan.get("steps", [])
        self.get_logger().info(
            f"[Plan] 새 플랜 수신: mission_id={plan.get('mission_id','')}, "
            f"intent='{plan.get('intent','')}', steps={len(steps)}"
        )

        current_gen = 0
        with self.lock:
            self.execution_generation += 1  # 번호표 증가
            current_gen = self.execution_generation # 로컬 변수에 담기

            prev_status = self.queue_status
            self.current_plan = plan
            self.mission_id = plan.get("mission_id", "")
            self.current_index = 0
            self.queue_status = "running"

        # 큐 상태 전환 로그
        self.get_logger().info(
            f"[Plan] queue_status: {prev_status} -> {self.queue_status}, start_idx=0"
        )

        threading.Thread(target=self._run_loop, args=(current_gen,), daemon=True).start()
        self.get_logger().info("[Plan] 실행 스레드 시작 (_run_loop)")

    # ----------- Vision 콜백 -----------
    def on_vision(self, msg):
        data = msg.data.strip() if hasattr(msg, "data") and msg.data else ""
        if not data or data[0] not in ("{", "["):
            return
        try:
            raw = json.loads(data)
        except Exception as e:
            self.get_logger().warn(f"[vision] non-JSON payload ignored: {e}")
            return

        try:
            norm = self._normalize_raw_vision(raw) if isinstance(raw, (dict, list)) else raw
            self.vision.update_from_msg(json.dumps(norm), self.get_logger())
        except Exception as e:
            self.get_logger().warn(f"[vision] normalize/update failed: {e}")

    def on_vision_raw(self, msg: String):
        try:
            raw = json.loads(msg.data) if msg and msg.data else {}
            norm = self._normalize_raw_vision(raw)
            self.vision.update_from_msg(json.dumps(norm), self.get_logger())
        except Exception as e:
            self.get_logger().warn(f"[vision/raw] parse failed: {e}")

    # ----------- 실행 루프 -----------
    def _run_loop(self, my_generation):
        while rclpy.ok():
            with self.lock:
                if self.execution_generation != my_generation:
                    self.get_logger().warn(f"[Thread {my_generation}] Expired! New gen is {self.execution_generation}. Exiting.")
                    return
                plan = self.current_plan
                idx = self.current_index
                status = self.queue_status

            if plan is None or status != "running":
                # 실행 중이 아니면 루프 종료
                self.get_logger().debug(
                    f"[_run_loop] 종료 조건: plan={plan is None}, status={status}"
                )
                break

            steps = plan.get("steps", [])
            if idx >= len(steps):
                with self.lock:
                    if self.execution_generation != my_generation:
                        self.get_logger().warn(
                            f"[Thread {my_generation}] generation 변경 감지 (all done) → 종료"
                        )
                        return
                    self.queue_status = "done"
                self.get_logger().info("[Plan] 모든 스텝 완료 → queue_status=done")
                
                break

            step = steps[idx]
            task = step.get("task")
            params = step.get("params", {}) or {}
            guard = step.get("guard", "")
            retry_left = int(step.get("retry", 0))

            # 현재 step 시작 로그
            self.get_logger().info(
                f"[Step] START idx={idx}, task={task}, retry_left={retry_left}"
            )

            # ----------- Guard 체크 -----------
            if guard:
                sym = {
                    "ROE_OK": self.ROE_OK,
                    "SAFE_BACKSTOP": self.SAFE_BACKSTOP,
                    "BATTERY_SOC": float(self.BATTERY_SOC),
                    "MAX_SPEED": float(self.MAX_SPEED),
                }
                try:
                    guard_ok = bool(eval_guard(guard, sym))
                except Exception as e:
                    # 자연어 guard 등으로 eval 실패하는 경우 → 그냥 통과 처리
                    self.get_logger().warn(
                        f"[Guard] eval 실패, guard='{guard}' → 무시하고 True로 처리: {e}"
                    )
                    guard_ok = True

                if not guard_ok:
                    self.get_logger().warn(f"[Guard] 불만족 → pause idx={idx}")
                    with self.lock:
                        if self.execution_generation != my_generation:
                            self.get_logger().warn(
                                f"[Thread {my_generation}] generation 변경 감지 (guard) → 종료"
                            )
                            return
                        self.queue_status = "paused"
                    break
            ok = False

            # ----------- 단위 액션 분기 -----------
            if task == "move_to":
                goal = params.get("goal", {})
                replan_rules = plan.get("replan_rules", {})
                self.get_logger().info(
                    f"[move_to] goal={goal}, replan_rules={replan_rules}"
                )
                ok = exec_move_to(
                    self, self.navigator, self._nav_feedback, goal, replan_rules
                )
                self.get_logger().info(f"[move_to] result ok={ok}")

            elif task == "scan":
                params = step.get("params", {})

                sweep_deg      = float(params.get("sweep_deg", 90.0))
                yaw_rate_dps   = float(params.get("yaw_rate_dps", params.get("yaw_rate", 20.0)))
                pitch_deg_up   = float(params.get("pitch_deg_up", 0.0))
                pitch_deg_down = float(params.get("pitch_deg_down", 0.0))
                duration_sec   = float(params.get("duration_sec", 20.0))

                watch_classes  = params.get("watch_classes")
                watch_ids      = params.get("watch_ids")
                report_on_found = bool(params.get("report_on_found", True))

                self.get_logger().info(
                    f"[scan] params: sweep_deg={sweep_deg}, yaw_rate_dps={yaw_rate_dps}, "
                    f"watch_classes={watch_classes}, watch_ids={watch_ids}, "
                    f"duration_sec={duration_sec}, report_on_found={report_on_found}"
                )

                ok = exec_scan(
                    node=self,
                    cmd_pub=self.pub_cmd,
                    sweep_deg=sweep_deg,
                    yaw_rate_dps=yaw_rate_dps,
                    pitch_deg_up=pitch_deg_up,
                    pitch_deg_down=pitch_deg_down,
                    duration_sec=duration_sec,
                    watch_classes=watch_classes,
                    watch_ids=watch_ids,
                    report_on_found=report_on_found,
                )
                self.get_logger().info(f"[scan] result ok={ok}")

            elif task == "report_and_wait":
                self.get_logger().info("[report_and_wait] 실행 시작")
                ok = exec_report_and_wait(
                    node=self,
                    vision=self.vision,
                    last_pose=self._last_pose,
                    mission_id=self.mission_id,
                    params=params,
                )
                self.get_logger().info(f"[report_and_wait] result ok={ok}")

            elif task == "track":
                self.get_logger().info(f"[track] params={params}")
                ok = self._do_track(params, plan)
                self.get_logger().info(f"[track] result ok={ok}")

            elif task == "return_to_home":
                replan_rules = plan.get("replan_rules", {})
                self.get_logger().info(
                    f"[return_to_home] home={self._home_pose}, params={params}, replan_rules={replan_rules}"
                )
                ok = exec_return_to_home(
                    node=self,
                    navigator=self.navigator,
                    nav_feedback=self._nav_feedback,
                    params=params,
                    replan_rules=replan_rules,
                )
                self.get_logger().info(f"[return_to_home] result ok={ok}")

            else:
                self.get_logger().error(f"[Step] unknown task={task}")
                ok = False
            
            with self.lock:
                if self.execution_generation != my_generation:
                    self.get_logger().warn(
                        f"[Thread {my_generation}] generation 변경 감지 (post-step) "
                        f"→ 결과 적용하지 않고 종료"
                    )
                    return

            # ----------- 결과 처리 -----------
            if not ok:
                if retry_left > 0:
                    step["retry"] = retry_left - 1
                    self.get_logger().warn(
                        f"[Step] FAILED task={task}, idx={idx} → 재시도 남음 {step['retry']}"
                    )
                    # 같은 index에서 재시도 (current_index 그대로)
                else:
                    self.get_logger().error(
                        f"[Step] FAILED task={task}, idx={idx} → 재시도 없음, queue_status=error"
                    )
                    emit_replan(
                        self,
                        self.pub_replan,
                        self.mission_id,
                        "STEP_FAILED",
                        {"task": task, "idx": idx},
                    )
                    with self.lock:
                        if self.execution_generation != my_generation:
                            self.get_logger().warn(
                                f"[Thread {my_generation}] generation 변경 감지 (step fail) → 종료"
                            )
                            return
                        self.queue_status = "error"
                    break
            else:
                with self.lock:

                    prev_idx = self.current_index
                    self.current_index += 1
                    next_idx = self.current_index

                self.get_logger().info(
                    f"[Step] DONE idx={idx}, task={task} → next_idx={next_idx}"
                )
                time.sleep(1.0)

        # 루프 종료 시점에서 상태 한 번 더 publish
        self.publish_state()

    # ----------- 단위 액션 헬퍼 -----------
    def _do_report(self, params: dict) -> bool:
        vision_snapshot = self.vision.snapshot()
        msg = (
            f"[report] mission={self.mission_id}, "
            f"pose={self._last_pose}, "
            f"primary_id={vision_snapshot.get('primary_id')}, "
            f"num_targets={len(vision_snapshot.get('targets', []))}"
        )
        self.get_logger().info(msg)
        self.publish_state()
        time.sleep(float(params.get("delay_sec", 0.5)))
        return True

    def _do_wait_for_command(self, params: dict) -> bool:
        timeout = float(params.get("timeout_sec", 3.0))
        self.get_logger().info(f"[wait_for_command] {timeout:.1f}s 대기")
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout:
            time.sleep(0.2)
        return True

    def _do_track(self, params: dict, plan: Optional[dict]) -> bool:
        """Tracker 기반 타겟 추적: 초기 정렬+거리 성공 여부까지만 판정."""
        rules = (plan or {}).get("replan_rules", {})
        succ_wait = float(
            params.get(
                "succ_wait_sec",
                rules.get("track_initial_success_wait_sec", 3.0),
            )
        )

        if hasattr(self.tracker, "start") and hasattr(
            self.tracker, "wait_initial_success"
        ):
            self.tracker.start(params, rules)
            got = self.tracker.wait_initial_success(timeout=succ_wait)
            ok = bool(got)
            if ok:
                self.get_logger().info(
                    f"[track] INITIAL SUCCESS (≤{succ_wait:.1f}s) → 다음 스텝 진행"
                )
            else:
                self.get_logger().warn(
                    f"[track] initial success TIMEOUT (>{succ_wait:.1f}s)"
                )
            return ok
        else:
            self.get_logger().warn("[track] legacy run() fallback 사용")
            return self.tracker.run(params, rules)

    # ----------- 상태 퍼블리시 / TF -----------
    def state_string(self) -> str:
        if (
            self.queue_status == "running"
            and self.current_plan
            and 0 <= self.current_index < len(self.current_plan.get("steps", []))
        ):
            return self.current_plan["steps"][self.current_index].get("task", "")
        return self.queue_status

    def publish_state(self):
        vision_snapshot = self.vision.snapshot()
        msg = make_state(
            self,
            self._last_pose,
            self.mission_id,
            self.state_string(),
            self.queue_status,
            self.current_plan,
            self.current_index,
            self.ROE_OK,
            self.SAFE_BACKSTOP,
            self.BATTERY_SOC,
            self.MAX_SPEED,
            vision_snapshot,
        )
        self.pub_ats.publish(msg)

    def _log_tf_pose(self):
        """TF 기반 현재 포즈 업데이트 (로그는 debug로만)."""
        try:
            tf = self.tf_buffer.lookup_transform("map", "body", rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = quat_to_yaw(q)
            self._last_pose.update(
                {"x": t.x, "y": t.y, "yaw": yaw, "ok": True}
            )
            # 주기 로그 → debug로 다운
            self.get_logger().debug(
                f"[pose/tf] map: x={t.x:.2f}, y={t.y:.2f}, yaw={yaw:.2f}"
            )
        except Exception as e:
            self._last_pose["ok"] = False
            # TF 실패도 debug로 (원하면 warn으로 바꿀 수 있음)
            self.get_logger().debug(f"[pose/tf] lookup failed: {e}")

    # ----------- Vision 정규화 -----------
    def _normalize_raw_vision(self, raw):
        """
        VisionContextBuilder 포맷을 캐시 공용 포맷으로 변환.
        - 새 포맷(dict): {"frame_w","frame_h","objects":[...]}
        - 구 포맷(list): [{...}, {...}]
        """
        frame_w = self._frame_w_default
        frame_h = self._frame_h_default

        if isinstance(raw, dict) and "objects" in raw:
            try:
                frame_w = int(raw.get("frame_w", frame_w))
                frame_h = int(raw.get("frame_h", frame_h))
            except Exception:
                pass
            src_list = raw.get("objects", [])
        else:
            src_list = raw if isinstance(raw, list) else []

        targets = []
        primary_id = None

        for obj in (src_list or []):
            try:
                tid = obj.get("id")
                cls = obj.get("class") or obj.get("class_name") or "object"

                cx = cy = None
                if isinstance(obj.get("center"), dict):
                    cx = obj["center"].get("x")
                    cy = obj["center"].get("y")

                rng = None
                if obj.get("range_m") is not None:
                    try:
                        rng = float(obj.get("range_m"))
                    except Exception:
                        rng = None

                bbox = None
                if isinstance(obj.get("bbox"), dict):
                    w = obj["bbox"].get("w")
                    h = obj["bbox"].get("h")
                    if (
                        w is not None
                        and h is not None
                        and cx is not None
                        and cy is not None
                    ):
                        x = cx - w / 2.0
                        y = cy - h / 2.0
                        bbox = [x, y, w, h]

                tgt = {
                    "id": tid,
                    "class": cls,
                    "bbox": bbox,
                    "range_m": rng,
                    "center": {"x": cx, "y": cy}
                    if (cx is not None and cy is not None)
                    else None,
                }
                targets.append(tgt)
                if primary_id is None and tid is not None:
                    primary_id = tid
            except Exception as e:
                self.get_logger().warn(f"[vision/raw] skip obj: {e}")

        lost_sec = 0.0 if targets else 999.0
        return {
            "targets": targets,
            "primary_id": primary_id,
            "lost_sec": lost_sec,
            "frame_w": frame_w,
            "frame_h": frame_h,
        }


def main():
    rclpy.init()
    node = System1ExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
