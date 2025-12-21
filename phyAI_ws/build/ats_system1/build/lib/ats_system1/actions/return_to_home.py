#!/usr/bin/env python3
from typing import Any, Dict, Optional

from rclpy.node import Node

from ats_system1.actions.move_to import Nav2Navigator, exec_move_to


def _resolve_home_goal(node: Node, params: Optional[Dict[str, Any]] = None) -> Dict[str, float]:
    """
    return_to_home에서 사용할 goal을 결정한다.

    우선순위:
      1) params 안에 goal이 dict로 들어온 경우 → 그대로 사용
      2) node._home_pose가 정의되어 있으면 → 그 값 사용
      3) 마지막 fallback: node 파라미터(home_x, home_y, home_yaw)를 읽어서 사용
    """
    params = params or {}

    # 1) 플랜에서 goal이 직접 들어온 경우 우선
    goal = params.get("goal")
    if isinstance(goal, dict):
        try:
            x = float(goal.get("x"))
            y = float(goal.get("y"))
            yaw = float(goal.get("yaw", 0.0))
            return {"x": x, "y": y, "yaw": yaw}
        except Exception:
            # 형식 이상하면 무시하고 아래 fallback 사용
            pass

    # 2) System1ExecutorNode에서 미리 세팅해둔 _home_pose가 있으면 활용
    if hasattr(node, "_home_pose"):
        hp = getattr(node, "_home_pose", {})
        try:
            return {
                "x": float(hp.get("x", 0.0)),
                "y": float(hp.get("y", 0.0)),
                "yaw": float(hp.get("yaw", 0.0)),
            }
        except Exception:
            pass

    # 3) 마지막 fallback: 파라미터에서 직접 읽기
    #    (이미 declare_parameter 되어 있다고 가정)
    try:
        home_x = float(node.get_parameter("home_x").value)
        home_y = float(node.get_parameter("home_y").value)
        home_yaw = float(node.get_parameter("home_yaw").value)
    except Exception:
        home_x, home_y, home_yaw = 0.0, 0.0, 0.0

    return {"x": home_x, "y": home_y, "yaw": home_yaw}


def exec_return_to_home(
    node: Node,
    navigator: Nav2Navigator,
    nav_feedback: Dict[str, Any],
    params: Optional[Dict[str, Any]] = None,
    replan_rules: Optional[Dict[str, Any]] = None,
) -> bool:
    """
    System-1 단위 액션: return_to_home

    - home 위치는 _resolve_home_goal()에서 결정
      (params.goal → node._home_pose → 파라미터(home_x/home_y/home_yaw) 순)
    - 실제 이동은 exec_move_to 를 그대로 재사용
    """
    goal = _resolve_home_goal(node, params)
    node.get_logger().info(
        "[return_to_home] goal → x={:.3f}, y={:.3f}, yaw={:.3f}".format(
            goal["x"], goal["y"], goal["yaw"]
        )
    )

    return exec_move_to(
        node=node,
        navigator=navigator,
        nav_feedback=nav_feedback,
        goal=goal,
        replan_rules=replan_rules,
    )
