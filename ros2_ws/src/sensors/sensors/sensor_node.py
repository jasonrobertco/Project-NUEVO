from __future__ import annotations

import rclpy
from rclpy.node import Node


class SensorNode(Node):
    def __init__(self) -> None:
        super().__init__("sensor_node")
        self.get_logger().info("sensors package scaffold ready")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
