#!/usr/bin/env python3
import sys, json, threading, time
from typing import Optional

import rclpy
from rclpy.node import Node
from ats_msgs.msg import PlanCommand

HELP = """
[ats_keyboard_control]
- 숫자 + Enter : 해당 ID 대상으로 track 플랜 publish
- 'c'  + Enter : 취소 플랜 publish(현재 동작 즉시 중단)
- 'h'  + Enter : 도움말
- 'q'  + Enter : 종료
"""

def make_track_plan_json(
    mission_id: str,
    cls: str,
    target_id: str,
    follow_dist: float,
    dwell_sec: float,
    max_time: float
) -> str:
    plan = {
        "version": "1.0.0",
        "mission_id": mission_id,
        "intent": cls,
        "steps": [{
            "task": "track",
            "params": {
                "class": cls,
                "id": str(target_id),
                "follow_dist": float(follow_dist),
                "dwell_sec": float(dwell_sec),
                "max_time": float(max_time)
            }
        }],
        "replan_rules": {"lost_target_sec": 5.0}
    }
    return json.dumps(plan, separators=(",", ":"))

def make_cancel_plan_json(mission_id: str) -> str:
    # 스키마 오류 방지를 위해 steps를 보내지 않고 intent="cancel"만 보냄
    #    (executor에서 intent=='cancel'을 빠르게 처리하도록 구현 필요)
    return json.dumps({
        "version": "1.0.0",
        "mission_id": mission_id,
        "intent": "cancel"
    }, separators=(",", ":"))

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('ats_keyboard_control')

        # 기본값 선언(런타임 기본 설정) — 사용자는 --ros-args 로만 덮어씀
        self.declare_parameter('topic', '/system2/plan_cmd')
        self.declare_parameter('mission_prefix', 'demo')
        self.declare_parameter('class', 'person')
        self.declare_parameter('follow_dist', 4.5)
        self.declare_parameter('dwell_sec', 0.8)
        self.declare_parameter('max_time', 30.0)

        topic = self.get_parameter('topic').value
        self.pub = self.create_publisher(PlanCommand, topic, 10)

        self.cls = self.get_parameter('class').value
        self.follow_dist = float(self.get_parameter('follow_dist').value)
        self.dwell_sec = float(self.get_parameter('dwell_sec').value)
        self.max_time = float(self.get_parameter('max_time').value)
        self.mission_prefix = self.get_parameter('mission_prefix').value

        self._running = True
        self._input_thread = threading.Thread(target=self._stdin_loop, daemon=True)
        self._input_thread.start()

        self.get_logger().info(HELP.strip())
        self.get_logger().info(
            f"[defaults] topic={topic}, class={self.cls}, follow_dist={self.follow_dist}, "
            f"dwell_sec={self.dwell_sec}, max_time={self.max_time}, mission_prefix={self.mission_prefix}"
        )

    def destroy_node(self):
        self._running = False
        return super().destroy_node()

    def _stdin_loop(self):
        while self._running:
            try:
                line = sys.stdin.readline()
                if not line:
                    time.sleep(0.05)
                    continue
                cmd = line.strip()
                if not cmd:
                    continue

                lo = cmd.lower()
                if lo == 'q':
                    self.get_logger().info("quit requested")
                    rclpy.shutdown()
                    break
                if lo == 'h':
                    self.get_logger().info("\n" + HELP.strip())
                    continue
                if lo == 'c':
                    self.publish_cancel()
                    continue

                if cmd.isdigit():
                    self.publish_track(cmd)
                else:
                    self.get_logger().warn(f"입력 인식 실패: '{cmd}' (숫자 / c / h / q 만 지원)")
            except Exception as e:
                self.get_logger().error(f"stdin loop error: {e}")
                time.sleep(0.2)

    def publish_track(self, target_id: str):
        mission_id = f"{self.mission_prefix}-track-{target_id}"
        js = make_track_plan_json(
            mission_id=mission_id,
            cls=self.cls,
            target_id=target_id,
            follow_dist=self.follow_dist,
            dwell_sec=self.dwell_sec,
            max_time=self.max_time
        )
        msg = PlanCommand()
        msg.mission_id = mission_id
        msg.plan_json = js
        self.pub.publish(msg)
        self.get_logger().info(
            f"[PUBLISH track] mission_id={mission_id}, class={self.cls}, id={target_id}, "
            f"follow_dist={self.follow_dist}, dwell_sec={self.dwell_sec}, max_time={self.max_time}"
        )

    def publish_cancel(self):
        mission_id = f"{self.mission_prefix}-cancel-{int(time.time())}"
        js = make_cancel_plan_json(mission_id=mission_id)
        msg = PlanCommand()
        msg.mission_id = mission_id
        msg.plan_json = js
        self.pub.publish(msg)
        self.get_logger().info(f"[PUBLISH cancel] mission_id={mission_id} (intent=cancel, no steps)")

def main(args: Optional[list] = None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
