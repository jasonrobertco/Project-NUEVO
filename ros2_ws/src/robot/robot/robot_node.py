from __future__ import annotations

import signal
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from robot.robot import Robot


class RobotNode(Node):
    def __init__(self) -> None:
        super().__init__("robot")
        self.robot = Robot(self)
        self.get_logger().info("robot node ready")


def _safe_log(node: Node, level: str, message: str) -> None:
    try:
        getattr(node.get_logger(), level)(message)
    except Exception:
        pass


def _raise_keyboard_interrupt(signum, frame) -> None:
    raise KeyboardInterrupt()


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RobotNode()

    # ROS spin runs in a background thread so main.run() can block freely.
    def _spin() -> None:
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    old_sigint = signal.getsignal(signal.SIGINT)
    old_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    try:
        # from robot.main import run
        from robot.main import run
        run(node.robot)
    except KeyboardInterrupt:
        _safe_log(node, "info", "robot node interrupted; shutting down")
    finally:
        try:
            node.robot.shutdown()
        except Exception as exc:
            _safe_log(node, "error", f"robot shutdown failed: {exc}")
        try:
            signal.signal(signal.SIGINT, old_sigint)
            signal.signal(signal.SIGTERM, old_sigterm)
        except Exception:
            pass
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            spin_thread.join(timeout=2.0)
