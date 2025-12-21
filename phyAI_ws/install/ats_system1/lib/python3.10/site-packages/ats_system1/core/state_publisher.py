import json
from ats_msgs.msg import AtsState

def make_state(node, cache_pose, mission,
               system_state: str, queue_status: str,
               plan: dict | None, current_index: int,
               roe_ok: bool, safe_backstop: bool,
               battery_soc: float, max_speed: float,
               vision_snapshot: dict) -> AtsState:
    msg = AtsState()
    msg.stamp = node.get_clock().now().to_msg()
    msg.system_state = system_state

    if cache_pose.get("ok", False):
        msg.pose_frame = "map"
        msg.pose_x = float(cache_pose["x"])
        msg.pose_y = float(cache_pose["y"])
        msg.pose_yaw = float(cache_pose["yaw"])
    else:
        msg.pose_frame, msg.pose_x, msg.pose_y, msg.pose_yaw = "map", 0.0, 0.0, 0.0

    msg.vel_vx = msg.vel_vy = msg.vel_wz = 0.0
    msg.battery_soc = float(battery_soc); msg.battery_voltage = 48.0
    msg.roe_ok = roe_ok; msg.safe_backstop = safe_backstop; msg.max_speed = float(max_speed)
    msg.plan_json = json.dumps(plan) if plan else ""
    msg.current_index = int(current_index); msg.queue_status = queue_status
    msg.primary_id = -1; msg.lost_sec = 0.0
    msg.vision_json = json.dumps(vision_snapshot)
    msg.history_json = "[]"; msg.events_json = "[]"
    return msg
