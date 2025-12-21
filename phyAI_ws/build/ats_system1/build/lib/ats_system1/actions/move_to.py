#!/usr/bin/env python3
import math
import time
from typing import Any, Dict, Optional, Callable

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose


class Nav2Navigator:
    """
    Nav2 NavigateToPose 액션에 대한 래퍼.

    - System1ExecutorNode 메인스레드는 rclpy.spin()으로 돌고 있고,
      _run_loop는 별도 스레드이므로, 여기서는 spin_until_future_complete를 쓰면 안 된다.
    - 대신 send_goal_async / cancel_goal_async 결과를 polling으로 기다린다.
    """

    def __init__(
        self,
        node: Node,
        feedback_cb: Optional[Callable[[Dict[str, Any]], None]] = None,
        action_name: str = "navigate_to_pose",
        frame_id: str = "map",
    ):
        self._node = node
        self._feedback_cb = feedback_cb
        self._action_name = action_name
        self._frame_id = frame_id

        self._client = ActionClient(node, NavigateToPose, self._action_name)
        self._last_goal_handle = None

        self._node.get_logger().info(
            f"[Nav2Navigator] created for action '{self._action_name}' in frame '{self._frame_id}'"
        )

    # yaw(rad) → Quaternion
    @staticmethod
    def _yaw_to_quat(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    # 내부 feedback 핸들러 (Humble: feedback_callback(feedback_msg) 시그니처)
    def _on_feedback(self, feedback_msg):
        try:
            fb = feedback_msg.feedback
            data = {
                "distance_remaining": getattr(fb, "distance_remaining", None),
                "stamp": time.time(),
            }
            if self._feedback_cb:
                self._feedback_cb(data)
        except Exception as e:
            self._node.get_logger().warn(f"[Nav2Navigator] feedback_cb 예외: {e}")

    def start(self, goal: Dict[str, Any]):
        """
        goal: {"x": float, "y": float, "yaw": float}
        반환: (goal_handle, result_future) 또는 (None, None) on failure
        """
        if not isinstance(goal, dict):
            self._node.get_logger().error("[Nav2Navigator] goal 형식 오류 (dict 아님)")
            return None, None

        try:
            x = float(goal.get("x"))
            y = float(goal.get("y"))
            yaw = float(goal.get("yaw", 0.0))
        except Exception as e:
            self._node.get_logger().error(f"[Nav2Navigator] goal 파싱 실패: {e}")
            return None, None

        # 액션 서버 준비 대기 (polling)
        t0 = time.time()
        while not self._client.wait_for_server(timeout_sec=0.2):
            if not rclpy.ok():
                self._node.get_logger().error(
                    "[Nav2Navigator] rclpy 종료 상태에서 server wait"
                )
                return None, None
            if time.time() - t0 > 5.0:
                self._node.get_logger().error(
                    f"[Nav2Navigator] action server '{self._action_name}' 연결 실패"
                )
                return None, None

        # Goal 메시지 생성
        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.header.frame_id = self._frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = self._yaw_to_quat(yaw)

        self._node.get_logger().info(
            f"[Nav2Navigator] send goal: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )

        # 비동기 goal 전송
        send_goal_future = self._client.send_goal_async(
            msg,
            feedback_callback=self._on_feedback,
        )

        # executor는 메인 스레드에서 돌고 있으므로 여기서는 polling만 한다
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.01)

        if not rclpy.ok():
            self._node.get_logger().warn("[Nav2Navigator] rclpy.ok() == False during send_goal")
            return None, None

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self._node.get_logger().warn("[Nav2Navigator] goal rejected")
            return None, None

        self._last_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        self._node.get_logger().info("[Nav2Navigator] goal accepted")
        return goal_handle, result_future

    def cancel(self, goal_handle):
        if goal_handle is None:
            self._node.get_logger().warn("[Nav2Navigator] cancel 요청 but goal_handle is None")
            return
        try:
            self._node.get_logger().info("[Nav2Navigator] cancel goal 요청")
            fut = goal_handle.cancel_goal_async()
            # 마찬가지로 polling
            t0 = time.time()
            while rclpy.ok() and not fut.done() and (time.time() - t0) < 2.0:
                time.sleep(0.01)
        except Exception as e:
            self._node.get_logger().warn(f"[Nav2Navigator] cancel 실패: {e}")

    def cancel_all(self):
        if self._last_goal_handle:
            self.cancel(self._last_goal_handle)


# ----------- exec_move_to (단위 액션) -----------

def exec_move_to(
    node: Node,
    navigator: Nav2Navigator,
    nav_feedback: Dict[str, Any],
    goal: Dict[str, Any],
    replan_rules: Optional[Dict[str, Any]] = None,
) -> bool:
    """
    System-1 단위 액션: move_to

    - node: rclpy Node
    - navigator: Nav2Navigator 인스턴스
    - nav_feedback: feedback_cb에서 업데이트하는 dict
    - goal: {"x","y","yaw"}
    - replan_rules: 하드스턱/그레이스 파라미터
    """
    if not isinstance(goal, dict):
        node.get_logger().error("[move_to] goal 파라미터 없음/형식 오류")
        return False

    goal_handle, result_future = navigator.start(goal)
    if not goal_handle or not result_future:
        node.get_logger().warn("[move_to] navigator.start 실패")
        return False

    # ----------- 진행 정체 / 근접 판단 파라미터 (기본값 완화) -----------
    rules = dict(replan_rules or {})
    hard_stuck = float(rules.get("hard_stuck_timeout_sec",60.0))      # 20 → 40s
    grace_sec = float(rules.get("progress_grace_sec", 50.0))            # 5 → 8s
    eps_m = float(rules.get("progress_epsilon_m", 0.05))               # 0.03 → 0.05m
    near_goal_eps = float(rules.get("near_goal_epsilon_m", 1.0))      # 목표 근처는 stuck 판정 제외

    node.get_logger().info(
        f"[move_to] start, rules: hard_stuck={hard_stuck}s, "
        f"grace={grace_sec}s, eps={eps_m}m, near_goal={near_goal_eps}m"
    )

    accept_t = time.time()
    last_prog = accept_t
    best_dist = float("inf")
    last_print = 0.0

    while rclpy.ok():
        time.sleep(0.2)
        now = time.time()

        # ----------- 디버그 출력 (주기 3초로 완화) -----------
        if now - last_print > 3.0:
            dist_dbg = nav_feedback.get("distance_remaining", None)
            last_pose = getattr(node, "_last_pose", {})
            if last_pose.get("ok", False):
                node.get_logger().info(
                    "[move_to] pose(map)=({:.2f}, {:.2f}, yaw={:.2f}), dist_remain={}".format(
                        last_pose.get("x", 0.0),
                        last_pose.get("y", 0.0),
                        last_pose.get("yaw", 0.0),
                        dist_dbg if dist_dbg is not None else "None",
                    )
                )
            else:
                node.get_logger().info(
                    f"[move_to] pose=Unknown(TF fail), dist_remain={dist_dbg if dist_dbg is not None else 'None'}"
                )
            last_print = now

        # ----------- 진행 상황 확인 (stuck 감지) -----------
        dist = nav_feedback.get("distance_remaining", None)
        if dist is not None:
            # 현재까지의 최고 기록(best_dist) 갱신
            if dist < (best_dist - eps_m):
                best_dist = dist
                last_prog = now

            # 목표 근처(near_goal_eps 미터 이내)에서는 stuck 판단을 하지 않고
            # Nav2 결과만 기다린다.
            if dist < near_goal_eps:
                # 여기서는 단순히 Nav2의 결과만 보도록 하고,
                # stuck 타이머는 더 이상 사용하지 않음.
                pass
            else:
                # grace_sec 이후, hard_stuck 동안 진전 없으면 stuck으로 판단
                if (now - accept_t) > grace_sec and (now - last_prog) > hard_stuck:
                    node.get_logger().warn(
                        "[move_to] 진행 정체 감지 → Nav2 goal cancel & 실패 반환 "
                        f"(dist≈{dist:.3f}m, best≈{best_dist:.3f}m)"
                    )
                    try:
                        navigator.cancel(goal_handle)
                    except Exception as e:
                        node.get_logger().warn(f"[move_to] navigator.cancel 실패: {e}")
                    return False

        # ----------- Nav2 결과 확인 (result_future polling) -----------
        if result_future.done():
            try:
                res = result_future.result()
            except Exception as e:
                node.get_logger().warn(f"[move_to] result_future 예외: {e}")
                return False

            status = getattr(res, "status", None)
            ok = bool(status == 4)  # SUCCEEDED
            if ok:
                node.get_logger().info("[move_to] Nav2 SUCCEEDED")
            else:
                node.get_logger().warn(f"[move_to] Nav2 FAILED, status={status}")
            return ok

    node.get_logger().warn("[move_to] rclpy.ok() == False → 실패")
    return False