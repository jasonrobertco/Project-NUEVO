from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node

from robot.robot import Robot


class RobotNode(Node):
    def __init__(self) -> None:
        super().__init__("robot")
        self.robot = Robot(self)
        self.get_logger().info("robot node ready")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotNode()

    # ROS spin runs in a background thread so main.run() can block freely.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        from robot.main import run
        run(node.robot)
    finally:
        try:
            node.robot.shutdown()
        except Exception as exc:
            node.get_logger().error(f"robot shutdown failed: {exc}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
