import unittest

from nuevo_bridge.runtime import BridgeRuntime
from nuevo_bridge.webapp import create_app


class _FakeSerialManager:
    def __init__(self, message_router, ws_manager):
        self.message_router = message_router
        self.ws_manager = ws_manager
        self.sent = []
        self.stats = {"connected": False}
        self._stopped = False
        self.decoded_sink = None

    def set_decoded_message_sink(self, sink) -> None:
        self.decoded_sink = sink

    def send(self, tlv_type, payload) -> None:
        self.sent.append((tlv_type, payload))

    async def run(self) -> None:
        import asyncio

        while not self._stopped:
            await asyncio.sleep(0.01)

    def stop(self) -> None:
        self._stopped = True


class _FakeRosController:
    def __init__(self, runtime):
        self.runtime = runtime
        self.started = False
        self.stopped = False
        self.messages = []

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def publish_decoded(self, msg_dict: dict) -> None:
        self.messages.append(msg_dict)


class BridgeRuntimeTests(unittest.IsolatedAsyncioTestCase):
    async def test_plain_runtime_reuses_shared_command_path(self):
        runtime = BridgeRuntime(serial_manager_factory=_FakeSerialManager)
        app = create_app(runtime)
        self.assertIs(app.state.bridge_runtime, runtime)

        await runtime.start()
        try:
            sent = runtime.serial_manager.sent
            self.assertEqual(sent, [])
            ok = runtime.handle_command("sys_cmd", {"command": 2})
            self.assertTrue(ok)
            self.assertEqual(len(sent), 1)
        finally:
            await runtime.stop()

    async def test_ros_mode_attaches_external_message_sink(self):
        controllers = []

        def _factory(runtime):
            controller = _FakeRosController(runtime)
            controllers.append(controller)
            return controller

        runtime = BridgeRuntime(
            ros_controller_factory=_factory,
            serial_manager_factory=_FakeSerialManager,
        )

        await runtime.start()
        try:
            self.assertEqual(len(controllers), 1)
            controller = controllers[0]
            self.assertTrue(controller.started)
            self.assertIs(runtime.serial_manager.decoded_sink, controller)
        finally:
            await runtime.stop()

        self.assertTrue(controllers[0].stopped)
