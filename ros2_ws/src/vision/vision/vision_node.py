from __future__ import annotations

import rclpy
from rclpy.node import Node


class VisionNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_node")
        self.get_logger().info("vision package scaffold ready")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
