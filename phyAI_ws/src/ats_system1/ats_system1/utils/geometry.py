import math
from geometry_msgs.msg import Quaternion

def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    half = yaw * 0.5
    q.x = 0.0; q.y = 0.0
    q.z = math.sin(half); q.w = math.cos(half)
    return q

def quat_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)
