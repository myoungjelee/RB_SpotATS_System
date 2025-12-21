#!/usr/bin/env python3
import os
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid


class MapInfoLogger(Node):
    def __init__(self):
        super().__init__('map_info_logger')

        # /map 은 nav2 map_server 기준으로 transient_local + reliable 로 나오므로
        # 여기서도 동일하게 맞춰줘야, 과거에 한 번만 발행된 맵도 바로 받을 수 있음.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.cb,
            qos
        )

        self.log_path = os.path.expanduser("~/map_info_log.txt")
        self.got = False
        self.get_logger().info("[map_info_logger] Waiting for /map message (latched)...")

    def cb(self, msg: OccupancyGrid):
        if self.got:
            return
        self.got = True

        res = msg.info.resolution
        w = msg.info.width
        h = msg.info.height
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        o = msg.info.origin.orientation

        # yaw 계산 (보통 0)
        yaw = math.atan2(2.0 * (o.w * o.z), 1.0 - 2.0 * (o.z * o.z))

        lines = [
            "================ Map Info Logger ================",
            f"Timestamp : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            f"Resolution: {res:.6f} m/cell",
            f"Map Size  : {w} x {h} cells",
            f"Origin(x,y): ({ox:.3f}, {oy:.3f})",
            f"Origin yaw: {yaw:.6f} rad",
        ]

        if abs(yaw) < 1e-3:
            x_min = ox
            y_min = oy
            x_max = ox + w * res
            y_max = oy + h * res
            lines += [
                "------ Corners (no rotation) ------",
                f"Bottom-left : ({x_min:.3f}, {y_min:.3f})",
                f"Top-right   : ({x_max:.3f}, {y_max:.3f})",
                f"X range: {x_min:.3f} → {x_max:.3f}",
                f"Y range: {y_min:.3f} → {y_max:.3f}",
            ]
        else:
            # 맵이 회전된 경우: 여기선 알림만, 필요하면 이후에 회전 포함한 꼭짓점 계산 추가
            lines.append("※ Map origin yaw ≠ 0 (rotated). Corner computation with rotation needed.")

        log_text = "\n".join(lines)

        # 콘솔 출력
        self.get_logger().info("\n" + log_text)

        # 파일 저장 (append)
        try:
            with open(self.log_path, "a") as f:
                f.write(log_text + "\n\n")
            self.get_logger().info(f"[map_info_logger] Saved to {self.log_path}")
        except Exception as e:
            self.get_logger().error(f"[map_info_logger] Failed to write file: {e}")

        # 한 번 기록하고 바로 종료
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MapInfoLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
