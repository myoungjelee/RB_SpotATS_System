from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from ats_msgs.msg import PlanCommand, ReplanRequest, AtsState

def create_publishers(node):
    return {
        "ats": node.create_publisher(AtsState, "/ats_state", 10),
        "replan": node.create_publisher(ReplanRequest, "/system1/replan_request", 10),
        "cmd_vel": node.create_publisher(Twist, "/cmd_vel", 10),
        "gimbal": node.create_publisher(Float64MultiArray, "/lock_control", 10),
    }

def create_subscriptions(node, on_plan, on_vision):
    node.create_subscription(PlanCommand, "/system2/plan_cmd", on_plan, 10)
    node.create_subscription(String, "/vision_context", on_vision, 10)
