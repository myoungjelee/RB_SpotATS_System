import json
from ats_msgs.msg import ReplanRequest

def emit_replan(node, pub, mission_id: str, reason: str, ctx: dict):
    req = ReplanRequest()
    req.stamp = node.get_clock().now().to_msg()
    req.mission_id = mission_id
    req.reason = reason
    req.context_json = json.dumps(ctx)
    pub.publish(req)
