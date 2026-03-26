from __future__ import annotations

import rclpy
from rclpy.node import Node


class RobotNode(Node):
    def __init__(self) -> None:
        super().__init__("robot")
        self.get_logger().info("robot package scaffold ready")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
