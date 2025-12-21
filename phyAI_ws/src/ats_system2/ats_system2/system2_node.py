#!/usr/bin/env python3
import json
import threading
import time
from typing import Optional, Any, Dict, Tuple

import cv2  # ★ GUI 갱신용 추가
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ats_msgs.msg import PlanCommand, ReplanRequest, AtsState
from sensor_msgs.msg import CompressedImage

from .models import System1State
from .llm_planner import build_plan_dict
from .utils.visualizer import SnapshotVisualizer


def _should_treat_as_no(user_cmd: str) -> bool:
    """운용자 입력이 '추가 명령 없음' 계열인지 판정."""
    if not user_cmd:
        return True
    s = user_cmd.strip().lower()
    if s in (
        "no", "n", "none", "skip", "pass",
        "없어", "그냥 둬", "그냥", "끝", "종료"
    ):
        return True
    return False


class System2Node(Node):
    def __init__(self):
        super().__init__('system2_node')

        # 최근 상태 캐시 (/ats_state 기반)
        self.latest_state: Optional[System1State] = None
        
        # 시각화 도구 초기화
        self.visualizer = SnapshotVisualizer(self.get_logger())
        
        # [핵심 1] GUI가 멈추지 않도록 주기적으로 waitKey를 호출하는 타이머 추가
        self.create_timer(0.1, self.gui_timer_callback)

        # [핵심 2] report_and_wait 응답 처리를 위한 상태 변수
        self.pending_mission_id = None
        self.pending_context = None
        self.waiting_for_decision = False

        # ----------- 구독 -----------
        self.snap_img_sub = self.create_subscription(
            CompressedImage, '/system2/snapshot/image', self.on_snapshot_image, 10
        )
        self.snap_info_sub = self.create_subscription(
            String, '/system2/snapshot/info', self.on_snapshot_info, 10
        )
        self.state_sub = self.create_subscription(
            String, '/ats_state', self.state_callback, 10
        )
        self.user_cmd_sub = self.create_subscription(
            String, '/system2/user_command', self.user_command_callback, 10
        )
        self.report_ctx_sub = self.create_subscription(
            String, '/system2/report_context', self.report_context_callback, 10
        )

        # ----------- 퍼블리셔 -----------
        self.plan_cmd_pub = self.create_publisher(PlanCommand, '/system2/plan_cmd', 10)
        self.plan_log_pub = self.create_publisher(String, '/high_level_plan', 10)
        self.decision_pub = self.create_publisher(String, '/system1/report_decision', 10)

        self.get_logger().info("System2Node started (Non-blocking GUI & Input).")

        # 상시 운영자 콘솔 입력 루프를 별도 스레드로 실행
        self._keyboard_thread = threading.Thread(
            target=self._operator_input_loop,
            daemon=True,
        )
        self._keyboard_thread.start()

    # [추가] GUI 갱신용 타이머 콜백
    def gui_timer_callback(self):
        # OpenCV 창 이벤트 처리를 위해 주기적으로 호출
        cv2.waitKey(1)

    # ----------- /snapshot 콜백 -----------
    def on_snapshot_image(self, msg: CompressedImage):
        self.get_logger().info(f"[System2] Snapshot Image Received! ({len(msg.data)} bytes)")
        try:
            self.visualizer.update_image(msg)
        except Exception as e:
            self.get_logger().error(f"[System2] visualizer.update_image failed: {e}")

    def on_snapshot_info(self, msg: String):
        self.get_logger().info(f"[System2] Snapshot Info Received: {msg.data}")
        try:
            self.visualizer.show_popup(msg)
        except Exception as e:
            self.get_logger().error(f"[System2] visualizer.show_popup failed: {e}")

    # ----------- /ats_state 콜백 -----------
    def state_callback(self, msg: String):
        try:
            state_dict = json.loads(msg.data)
            self.latest_state = System1State(**state_dict)
        except Exception:
            pass

    # ----------- /system2/report_context 콜백 (Non-blocking 수정) -----------
    def report_context_callback(self, msg: String):
        """
        System1에서 보고가 들어오면, 블로킹 input()을 쓰지 않고 
        '대기 상태(Flag)'로 전환.
        """
        try:
            payload = json.loads(msg.data)
            mission_id = payload.get("mission_id")
            context = payload.get("context", {}) or {}
            vision = context.get("vision", {})
            
            # 로그 출력
            targets = vision.get("targets") or []
            t_desc = f"{len(targets)} targets detected"
            if targets:
                t0 = targets[0]
                t_desc += f" (Main: {t0.get('class')} id={vision.get('primary_id')})"

            self.get_logger().info(f"\n>>> [REPORT RECEIVED] Mission: {mission_id}")
            self.get_logger().info(f">>> Situation: {t_desc}")
            self.get_logger().info(">>> 운용자 명령을 입력하세요. (명령이 없으면 엔터)")

            # 상태 저장 (입력 스레드가 이걸 보고 처리함)
            self.pending_mission_id = mission_id
            self.pending_context = context
            self.waiting_for_decision = True
            
        except Exception as e:
            self.get_logger().error(f"Report parsing failed: {e}")

    # ----------- 공통 명령 처리 (Decision 연동 로직 추가) -----------
    def _handle_user_command_raw(self, raw: str, source: str = "user_command_topic"):
        """
        입력된 명령을 처리합니다.
        - 대기 중인 리포트가 있다면 그에 대한 응답(Decision)으로 처리
        - 없다면 일반 독립 명령으로 처리
        """
        user_command, extra_context, mission_from_payload = self._parse_user_command_payload(raw)

        # 1. 현재 리포트 응답 대기 중인지 확인
        is_report_response = self.waiting_for_decision
        current_mission_id = self.pending_mission_id

        # 2. 명령이 없는 경우 ("그냥 둬", 엔터 등)
        if _should_treat_as_no(user_command):
            if is_report_response:
                # 대기 중이었으면 "명령 없음" 결정 전송 후 대기 해제
                self._publish_decision(
                    current_mission_id,
                    "no_command",
                    user_command,
                    "Operator skipped",
                )
                self.waiting_for_decision = False
                self.pending_mission_id = None
                self.get_logger().info("[System2] Report handled: No command (Resumed).")
            else:
                if user_command:  # 대기중 아닌데 "그냥" 이라고 치면 무시
                    self.get_logger().info("[System2] Ignored empty command.")
            return

        # 3. 명령이 있는 경우 -> LLM 플랜 생성
        self.get_logger().info(f"[System2] Generating plan for: '{user_command}'")

        # 컨텍스트 병합 (리포트 대기 중이면 리포트 컨텍스트 우선 사용)
        ctx_to_use = self.pending_context if is_report_response else extra_context

        try:
            try:
                plan_dict = build_plan_dict(
                    user_command=user_command,
                    system1_state=self.latest_state,
                    extra_context=ctx_to_use,
                )
            except TypeError:
                plan_dict = build_plan_dict(user_command, self.latest_state)
        except Exception as e:
            self.get_logger().error(f"[System2] LLM plan generation failed: {e}")
            # 실패 시에도 대기 중이었다면 decision을 보내줘야 System1이 안 멈춤
            if is_report_response:
                self._publish_decision(
                    current_mission_id,
                    "no_command",
                    user_command,
                    f"Error: {e}",
                )
                self.waiting_for_decision = False
            return

        # mission_id 매핑
        if is_report_response and current_mission_id:
            plan_dict["mission_id"] = current_mission_id
        elif mission_from_payload:
            plan_dict["mission_id"] = mission_from_payload

        
        # 4. 리포트 응답인 경우: 먼저 decision, 그 다음 플랜 전송
        if is_report_response:
            # (1) report_and_wait 종료시키기 위한 decision 먼저 전송
            self._publish_decision(
                current_mission_id,
                "new_plan",
                user_command,
                "New plan generated",
            )
            self.waiting_for_decision = False
            self.pending_mission_id = None
            self.get_logger().info("[System2] Report handled: decision(new_plan) sent.")

            # (2) 그 다음에 실제 플랜 전송
            self._send_plan(plan_dict, source=source)
            self.get_logger().info("[System2] Plan sent (Report response).")

        else:
            # 리포트 응답이 아니라면 기존처럼 플랜만 전송
            self._send_plan(plan_dict, source=source)
            self.get_logger().info("[System2] Plan sent (Independent command).")




        # 헬퍼: 플랜 전송 + 로그 공통 처리
    def _send_plan(self, plan_dict: Dict[str, Any], source: str = "unknown"):
        # 정규화 및 전송
        plan_dict = self._normalize_plan_for_schema(plan_dict)
        plan_str = json.dumps(plan_dict, ensure_ascii=False)

        # ------- 전체 플랜 JSON 로그 출력 -------
        try:
            pretty = json.dumps(plan_dict, ensure_ascii=False, indent=2)
            self.get_logger().info(f"[System2] ({source}) generated HighLevelPlan:\n{pretty}")
        except Exception as e:
            self.get_logger().warn(
                f"[System2] ({source}) Failed to dump plan_dict for logging: {e}"
            )

        # Plan 전송
        self.plan_cmd_pub.publish(PlanCommand(plan_json=plan_str))
        self.get_logger().info(
            f"[System2] Published /system2/plan_cmd (steps={len(plan_dict.get('steps', []))})"
        )
        self.plan_log_pub.publish(String(data=plan_str))



    # 헬퍼: 결정 메시지 전송
    def _publish_decision(self, mission_id, decision, cmd, note):
        payload = {
            "mission_id": mission_id,
            "decision": decision,
            "user_command": cmd,
            "note": note
        }
        self.decision_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    # ----------- 기존 헬퍼 함수들 -----------

    def _parse_user_command_payload(self, raw: str) -> Tuple[str, Optional[Dict[str, Any]], Optional[str]]:
        raw_strip = raw.strip()
        if not raw_strip:
            return "", None, None
        try:
            obj = json.loads(raw_strip)
            if isinstance(obj, dict) and "user_command" in obj:
                return str(obj.get("user_command", "")).strip(), obj.get("context"), obj.get("mission_id")
        except Exception:
            pass
        return raw_strip, None, None

    def _normalize_plan_for_schema(self, plan: Dict[str, Any]) -> Dict[str, Any]:
        if not isinstance(plan, dict): return plan
        allowed_top = {"version", "mission_id", "intent", "constraints", "steps", "replan_rules"}
        for k in list(plan.keys()):
            if k not in allowed_top: plan.pop(k, None)
        if not isinstance(plan.get("version"), str): plan["version"] = "1.0.0"
        constraints = plan.get("constraints")
        if constraints is None: plan["constraints"] = []
        elif not isinstance(constraints, list): plan["constraints"] = [str(constraints)]
        replan_rules = plan.get("replan_rules")
        if replan_rules is None or not isinstance(replan_rules, dict): plan["replan_rules"] = {}
        else: plan["replan_rules"] = replan_rules
        steps = plan.get("steps")
        if not isinstance(steps, list): plan["steps"] = []; return plan
        allowed_step_keys = {"task", "params", "guard", "retry"}
        for i, step in enumerate(steps):
            if not isinstance(step, dict): continue
            for k in list(step.keys()):
                if k not in allowed_step_keys: step.pop(k, None)
            if step.get("guard") is None: step.pop("guard", None)
            try: step["retry"] = max(0, int(step.get("retry", 0)))
            except: step["retry"] = 0
            params = step.get("params")
            if not isinstance(params, dict): step["params"] = {}
            if step.get("task") == "move_to":
                goal = step["params"].get("goal")
                if not isinstance(goal, dict):
                    x = step["params"].get("x")
                    y = step["params"].get("y")
                    yaw = step["params"].get("yaw", 0.0)
                    if x is not None and y is not None:
                        step["params"]["goal"] = {"x": float(x), "y": float(y), "yaw": float(yaw)}
                        for k in ("x", "y", "z"):
                            if k in step["params"]: step["params"].pop(k)
        return plan

    def user_command_callback(self, msg: String):
        self._handle_user_command_raw(msg.data, source="user_command_topic")

    # 상시 콘솔 입력 루프 (스레드)
    def _operator_input_loop(self):
        while rclpy.ok():
            try:
                # 여기서 블로킹되어도 메인 스레드(GUI 타이머)는 돕니다.
                cmd = input("\n[System2/Command] > ")
                cmd = (cmd or "").strip()
                if not cmd:
                    continue
                # 메인 스레드의 로직 호출 (Thread-safe 가정)
                self._handle_user_command_raw(cmd, source="operator_console")
            except (EOFError, KeyboardInterrupt):
                break
            except Exception as e:
                print(f"[System2] Console Input Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = System2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if hasattr(node, 'visualizer'):
        node.visualizer.close()
        
    node.destroy_node()
    rclpy.shutdown()