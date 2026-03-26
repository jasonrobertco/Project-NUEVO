"""Shared bridge runtime for plain Python mode and ROS-integrated mode."""
from __future__ import annotations

import asyncio
from typing import Callable, Optional

from .config import MOCK_MODE
from .message_router import MessageRouter
from .serial_manager import MockSerialManager, SerialManager
from .ws_manager import WSManager


RosControllerFactory = Callable[["BridgeRuntime"], object]
SerialManagerFactory = Callable[[MessageRouter, WSManager], object]


class BridgeRuntime:
    """Owns the shared bridge state for the web UI and optional ROS wrapper."""

    def __init__(
        self,
        ros_controller_factory: Optional[RosControllerFactory] = None,
        serial_manager_factory: Optional[SerialManagerFactory] = None,
    ):
        self.ws_manager = WSManager()
        self.message_router = MessageRouter(self.ws_manager)
        if serial_manager_factory is not None:
            self.serial_manager = serial_manager_factory(self.message_router, self.ws_manager)
        elif MOCK_MODE:
            self.serial_manager = MockSerialManager(self.message_router, self.ws_manager)
        else:
            self.serial_manager = SerialManager(self.message_router, self.ws_manager)
        self.message_router.attach_transport_sender(self.serial_manager.send)

        self._ros_controller_factory = ros_controller_factory
        self._ros_controller = None
        self._serial_task: Optional[asyncio.Task] = None
        self._query_task: Optional[asyncio.Task] = None
        self._started = False

    @property
    def ros_enabled(self) -> bool:
        return self._ros_controller_factory is not None

    async def start(self) -> None:
        if self._started:
            return

        print(f"[Runtime] Starting bridge runtime (mock={MOCK_MODE}, ros={self.ros_enabled})...")

        if self._ros_controller_factory is not None:
            self._ros_controller = self._ros_controller_factory(self)
            self.serial_manager.set_decoded_message_sink(self._ros_controller)
            self._ros_controller.start()

        self._serial_task = asyncio.create_task(self.serial_manager.run())

        async def periodic_router_queries() -> None:
            while True:
                await asyncio.sleep(1.0)
                if self.serial_manager.stats.get("connected", False):
                    self.message_router.poll_runtime_queries()

        self._query_task = asyncio.create_task(periodic_router_queries())
        self._started = True

    async def stop(self) -> None:
        if not self._started:
            return

        print("[Runtime] Shutting down bridge runtime...")
        self.serial_manager.stop()

        if self._query_task is not None:
            self._query_task.cancel()
            try:
                await self._query_task
            except asyncio.CancelledError:
                pass
            self._query_task = None

        if self._serial_task is not None:
            await self._serial_task
            self._serial_task = None

        if self._ros_controller is not None:
            self._ros_controller.stop()
            self._ros_controller = None

        self._started = False

    def handle_command(self, cmd: str, data: dict) -> bool:
        result = self.message_router.handle_outgoing(cmd, data)
        if result is None:
            return False
        tlv_type, payload = result
        self.serial_manager.send(tlv_type, payload)
        return True

    def handle_ws_command(self, cmd: str, data: dict) -> bool:
        return self.handle_command(cmd, data)

    def health_dict(self) -> dict:
        return {
            "status": "ok",
            "mock_mode": MOCK_MODE,
            "ros2_mode": self.ros_enabled,
            "serial_connected": self.serial_manager.stats.get("connected", False),
            "ws_connections": self.ws_manager.get_connection_count(),
        }
