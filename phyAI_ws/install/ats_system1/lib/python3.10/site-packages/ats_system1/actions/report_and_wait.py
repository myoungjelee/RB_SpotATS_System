# ats_system1/actions/report_and_wait.py

import json
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _summarize_context(
    mission_id: str,
    pose: Dict[str, Any],
    vision_snapshot: Dict[str, Any],
) -> str:
    """사람이 읽기 쉬운 한 줄 요약 문자열 생성."""
    x = pose.get("x", 0.0)
    y = pose.get("y", 0.0)
    yaw = pose.get("yaw", 0.0)
    pose_ok = pose.get("ok", False)

    primary_id = vision_snapshot.get("primary_id")
    targets = vision_snapshot.get("targets", []) or []
    num_targets = len(targets)

    if targets:
        t0 = targets[0]
        cls = t0.get("class", "object")
        rng = t0.get("range_m")
        desc = f"{cls}"
        if rng is not None:
            desc += f" (거리 약 {rng:.1f}m)"
        target_str = f"{num_targets}개 타겟 감지, 주요 타겟: id={primary_id}, {desc}"
    else:
        target_str = "감지된 타겟 없음"

    if pose_ok:
        pose_str = f"map 기준 위치=({x:.2f}, {y:.2f}), yaw={yaw:.2f}rad"
    else:
        pose_str = "map 기준 위치=Unknown(TF 실패)"

    return (
        f"[REPORT] mission={mission_id}, "
        f"{pose_str}, "
        f"{target_str}"
    )


def exec_report_and_wait(
    node: Node,
    vision,
    last_pose: Dict[str, Any],
    mission_id: str,
    params: Optional[Dict[str, Any]] = None,
) -> bool:
    """
    System-1 단위 액션: report_and_wait

    1) 현재 pose + vision_snapshot를 요약해서 로그로 출력
    2) publish_state() 호출해서 /ats_state 최신화
    3) 동일한 컨텍스트를 /system2/report_context 로 JSON 형태로 publish
    4) /system1/report_decision 을 기다리면서 블로킹
       - decision='continue'  → True 반환 (다음 task로 진행)
       - decision='new_plan' → True 반환 (곧 /high_level_plan으로 플랜 교체)
       - decision='end'      → True/False는 정책에 따라, 일단 True

    운용자 입력 + LLM 호출 + 새 플랜 생성은 전부 System-2에서 처리.
    """
    params = params or {}

    # 1) 컨텍스트 스냅샷 + 요약 로그
    vision_snapshot = vision.snapshot()
    summary = _summarize_context(mission_id, last_pose, vision_snapshot)
    node.get_logger().info(summary)

    # 2) 최신 상태를 한 번 더 publish해서 System-2가 /ats_state 기준으로 읽을 수 있게
    try:
        if hasattr(node, "publish_state"):
            node.publish_state()
    except Exception as e:
        node.get_logger().warn(f"[report_and_wait] publish_state 예외: {e}")

    # 3) System-2로 컨텍스트 payload 전송
    payload = {
        "mission_id": mission_id,
        "context": {
            "pose": last_pose,
            "vision": vision_snapshot,
            "state_string": getattr(node, "state_string", lambda: "")(),
        },
        "source": "system1_report_and_wait",
    }

    pub_ctx = getattr(node, "pub_report_context", None)
    if pub_ctx is None:
        node.get_logger().warn(
            "[report_and_wait] pub_report_context가 없어 System-2로 컨텍스트를 전송하지 못했습니다. "
            "System2 연동 전까지는 로그 확인용으로만 동작합니다."
        )
        time.sleep(float(params.get("fallback_delay_sec", 0.5)))
        return True

    msg = String()
    msg.data = json.dumps(payload, ensure_ascii=False)
    pub_ctx.publish(msg)
    node.get_logger().info("[report_and_wait] System-2로 report_context 전송 완료")

    # 4) /system1/report_decision 을 기다리면서 블로킹
    decision_holder = {"value": None}

    def decision_cb(dec_msg: String):
        try:
            data = json.loads(dec_msg.data)
        except Exception as e:
            node.get_logger().warn(
                f"[report_and_wait] /system1/report_decision JSON parse 실패: {e}"
            )
            return

        mid = data.get("mission_id")
        if mid and mid != mission_id:
            # 다른 미션이면 무시
            return

        if decision_holder["value"] is not None:
            return

        decision_holder["value"] = data.get("decision", "continue")
        node.get_logger().info(
            f"[report_and_wait] decision 수신: {decision_holder['value']}"
        )

    sub = node.create_subscription(
        String,
        "/system1/report_decision",
        decision_cb,
        10,
    )

    timeout_sec = float(params.get("wait_timeout_sec", 600.0))
    t0 = time.time()

    try:
        while rclpy.ok() and decision_holder["value"] is None:
            if timeout_sec > 0.0 and (time.time() - t0) > timeout_sec:
                node.get_logger().warn(
                    "[report_and_wait] decision timeout → 'continue' 로 처리"
                )
                break
            time.sleep(0.1)
    finally:
        # 구독자 정리
        try:
            node.destroy_subscription(sub)
        except Exception:
            pass

    decision = decision_holder["value"] or "continue"
    node.get_logger().info(f"[report_and_wait] 최종 decision={decision}")

    # 정책:
    # - 'continue'  → 현재 플랜의 다음 task 실행
    # - 'new_plan' → 이미 /high_level_plan 이 publish 되었을 것이므로, 여기서는 True만 반환
    # - 'end'      → 일단 True 반환 (필요하면 이후 정책 바꿔도 됨)
    if decision == "continue":
        return True
    elif decision == "new_plan":
        return True
    elif decision == "end":
        return True
    else:
        # 알 수 없는 decision → 안전하게 True
        return True
