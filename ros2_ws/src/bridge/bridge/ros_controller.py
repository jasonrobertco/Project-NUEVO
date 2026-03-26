from __future__ import annotations

import threading

import rclpy

from .bridge_node import BridgeNode


class RosBridgeController:
    """Owns ROS startup and shutdown around the shared bridge runtime."""

    def __init__(self, runtime):
        self._runtime = runtime
        self._node = None
        self._thread = None

    def start(self) -> None:
        if not rclpy.ok():
            rclpy.init()
        self._node = BridgeNode(self._runtime)
        self._thread = self._node.spin_in_thread()

    def publish_decoded(self, msg_dict: dict) -> None:
        if self._node is not None:
            self._node.publish_decoded(msg_dict)

    def stop(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if rclpy.ok():
            rclpy.shutdown()
        if isinstance(self._thread, threading.Thread):
            self._thread.join(timeout=1.0)
            self._thread = None
