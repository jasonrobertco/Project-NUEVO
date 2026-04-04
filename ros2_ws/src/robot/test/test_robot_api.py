from __future__ import annotations

import importlib
import sys
import types
import unittest
from unittest import mock
from pathlib import Path


class FakePublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.published: list[object] = []

    def publish(self, msg: object) -> None:
        self.published.append(msg)


class FakeLogger:
    def error(self, _msg: str) -> None:
        pass


class FakeFuture:
    def __init__(self) -> None:
        self._done = False
        self._callbacks: list = []

    def add_done_callback(self, callback) -> None:
        self._callbacks.append(callback)

    def done(self) -> bool:
        return self._done

    def result(self):
        return None


class FakeClient:
    def wait_for_service(self, timeout_sec=None) -> bool:
        return True

    def call_async(self, _request) -> FakeFuture:
        return FakeFuture()


class FakeNode:
    def __init__(self) -> None:
        self.publishers: dict[str, FakePublisher] = {}
        self.logger = FakeLogger()

    def create_publisher(self, _msg_type, topic: str, _qos: int) -> FakePublisher:
        publisher = FakePublisher(topic)
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, *_args, **_kwargs):
        return None

    def create_client(self, *_args, **_kwargs) -> FakeClient:
        return FakeClient()

    def get_logger(self) -> FakeLogger:
        return self.logger


def _install_fake_robot_dependencies() -> None:
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    rclpy.node = rclpy_node
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node

    bridge_interfaces = types.ModuleType("bridge_interfaces")
    bridge_interfaces_msg = types.ModuleType("bridge_interfaces.msg")
    bridge_interfaces_srv = types.ModuleType("bridge_interfaces.srv")

    def make_message(name: str):
        return type(name, (), {})

    for name in [
        "DCEnable",
        "DCHome",
        "DCPidReq",
        "DCPidSet",
        "DCResetPosition",
        "DCSetPosition",
        "DCSetPwm",
        "DCSetVelocity",
        "DCStateAll",
        "IOInputState",
        "IOSetLed",
        "IOSetNeopixel",
        "SensorImu",
        "SensorKinematics",
        "ServoEnable",
        "ServoSet",
        "ServoStateAll",
        "StepConfigSet",
        "StepEnable",
        "StepHome",
        "StepMove",
        "StepStateAll",
        "SysOdomParamSet",
        "SysOdomReset",
        "SystemPower",
        "SystemState",
    ]:
        setattr(bridge_interfaces_msg, name, make_message(name))

    class SetFirmwareState:
        class Request:
            def __init__(self) -> None:
                self.target_state = 0
                self.timeout_sec = 0.0

    bridge_interfaces_srv.SetFirmwareState = SetFirmwareState
    bridge_interfaces.msg = bridge_interfaces_msg
    bridge_interfaces.srv = bridge_interfaces_srv
    sys.modules["bridge_interfaces"] = bridge_interfaces
    sys.modules["bridge_interfaces.msg"] = bridge_interfaces_msg
    sys.modules["bridge_interfaces.srv"] = bridge_interfaces_srv


class RobotApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        _install_fake_robot_dependencies()
        package_root = Path(__file__).resolve().parents[1]
        if str(package_root) not in sys.path:
            sys.path.insert(0, str(package_root))
        cls.robot_module = importlib.import_module("robot.robot")
        cls.hardware_map = importlib.import_module("robot.hardware_map")

    def setUp(self) -> None:
        self.node = FakeNode()
        self.robot = self.robot_module.Robot(self.node)

    def test_diff_drive_uses_1_based_motor_numbers(self) -> None:
        self.robot.set_velocity(100.0, 0.0)

        published = self.node.publishers["/dc_set_velocity"].published
        self.assertEqual(len(published), 2)
        self.assertEqual([msg.motor_number for msg in published], [1, 2])

    def test_body_velocity_auto_enables_drive_motors(self) -> None:
        self.robot.set_velocity(100.0, 0.0)

        published = self.node.publishers["/dc_enable"].published
        self.assertEqual(len(published), 2)
        self.assertEqual([msg.motor_number for msg in published], [1, 2])
        self.assertTrue(all(msg.mode == int(self.hardware_map.DCMotorMode.VELOCITY) for msg in published))

    def test_body_velocity_skips_reenable_when_drive_motors_are_already_enabled(self) -> None:
        enabled = int(self.hardware_map.DCMotorMode.VELOCITY)
        self.robot._dc_state = types.SimpleNamespace(
            motors=[
                types.SimpleNamespace(mode=enabled),
                types.SimpleNamespace(mode=enabled),
                types.SimpleNamespace(mode=0),
                types.SimpleNamespace(mode=0),
            ]
        )

        self.robot.set_velocity(100.0, 0.0)

        self.assertEqual(self.node.publishers["/dc_enable"].published, [])

    def test_diff_drive_mapping_is_configurable(self) -> None:
        self.robot.set_drive_wheels(3, 4)
        self.robot.set_velocity(100.0, 0.0)

        published = self.node.publishers["/dc_set_velocity"].published
        self.assertEqual([msg.motor_number for msg in published], [3, 4])

    def test_high_level_velocity_respects_runtime_wheel_direction_flags(self) -> None:
        self.robot.set_odometry_parameters(
            left_motor_dir_inverted=False,
            right_motor_dir_inverted=False,
        )
        self.node.publishers["/dc_set_velocity"].published.clear()
        self.robot.set_velocity(100.0, 0.0)
        both_forward = self.node.publishers["/dc_set_velocity"].published
        self.assertGreater(both_forward[0].target_ticks, 0)
        self.assertGreater(both_forward[1].target_ticks, 0)

        self.robot.set_odometry_parameters(
            left_motor_dir_inverted=True,
            right_motor_dir_inverted=False,
        )
        self.node.publishers["/dc_set_velocity"].published.clear()
        self.robot.set_velocity(100.0, 0.0)
        mixed = self.node.publishers["/dc_set_velocity"].published
        self.assertLess(mixed[0].target_ticks, 0)
        self.assertGreater(mixed[1].target_ticks, 0)

    def test_set_wheel_diameter_publishes_full_odom_param_snapshot(self) -> None:
        self.robot.set_wheel_diameter_mm(80.0)

        msg = self.node.publishers["/sys_odom_param_set"].published[-1]
        self.assertEqual(msg.wheel_diameter_mm, 80.0)
        self.assertEqual(msg.wheel_base_mm, 333.0)
        self.assertEqual(msg.initial_theta_deg, 90.0)
        self.assertEqual(msg.left_motor_number, 1)
        self.assertFalse(msg.left_motor_dir_inverted)
        self.assertEqual(msg.right_motor_number, 2)
        self.assertTrue(msg.right_motor_dir_inverted)

    def test_duplicate_odom_motor_pair_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "must be different"):
            self.robot.set_odometry_parameters(left_motor_id=2, right_motor_id=2)

    def test_zero_brightness_defaults_to_led_off_mode(self) -> None:
        self.robot.set_led(1, 0)
        msg = self.node.publishers["/io_set_led"].published[-1]
        self.assertEqual(msg.led_id, 1)
        self.assertEqual(msg.brightness, 0)
        self.assertEqual(msg.mode, 0)

    def test_nonzero_brightness_defaults_to_steady_mode(self) -> None:
        self.robot.set_led(1, 255)
        msg = self.node.publishers["/io_set_led"].published[-1]
        self.assertEqual(msg.mode, 1)
        self.assertEqual(msg.brightness, 255)

    def test_led_mode_enum_matches_firmware_contract(self) -> None:
        self.robot.set_led(1, 128, mode=self.hardware_map.LEDMode.BREATHE)
        msg = self.node.publishers["/io_set_led"].published[-1]
        self.assertEqual(msg.mode, 3)
        self.assertEqual(msg.period_ms, 1000)
        self.assertEqual(msg.duty_cycle, 500)

    def test_blink_defaults_to_symmetric_duty_cycle(self) -> None:
        self.robot.set_led(1, 255, mode=self.hardware_map.LEDMode.BLINK)
        msg = self.node.publishers["/io_set_led"].published[-1]
        self.assertEqual(msg.mode, 2)
        self.assertEqual(msg.period_ms, 1000)
        self.assertEqual(msg.duty_cycle, 500)

    def test_invalid_motor_number_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "motor_id must be between 1 and 4"):
            self.robot.set_motor_velocity(0, 100.0)

    def test_invalid_pid_loop_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "loop_type must be one of"):
            self.robot.request_pid(1, 2)

    def test_shutdown_stops_and_disables_drive_motors(self) -> None:
        self.robot.shutdown()

        velocity_msgs = self.node.publishers["/dc_set_velocity"].published
        disable_msgs = self.node.publishers["/dc_enable"].published
        self.assertEqual([msg.motor_number for msg in velocity_msgs], [1, 2])
        self.assertTrue(all(msg.target_ticks == 0 for msg in velocity_msgs))
        self.assertEqual([msg.motor_number for msg in disable_msgs], [1, 2])
        self.assertTrue(all(msg.mode == int(self.hardware_map.DCMotorMode.DISABLED) for msg in disable_msgs))

    def test_button_edges_are_latched_in_subscription_callback(self) -> None:
        first = self.robot_module.IOInputState()
        first.button_mask = 0
        first.limit_mask = 0
        self.robot._on_io_input(first)

        pressed = self.robot_module.IOInputState()
        pressed.button_mask = 0b1
        pressed.limit_mask = 0
        self.robot._on_io_input(pressed)

        self.assertTrue(self.robot.get_button(1))
        self.assertTrue(self.robot.was_button_pressed(1))
        self.assertFalse(self.robot.was_button_pressed(1))

    def test_wait_helpers_translate_public_ids_to_state_array_indices(self) -> None:
        dc_state = types.SimpleNamespace(
            motors=[
                types.SimpleNamespace(position=0, mode=0),
                types.SimpleNamespace(position=0, mode=0),
                types.SimpleNamespace(position=0, mode=0),
                types.SimpleNamespace(position=123, mode=0),
            ]
        )
        step_state = types.SimpleNamespace(
            steppers=[
                types.SimpleNamespace(motion_state=1),
                types.SimpleNamespace(motion_state=1),
                types.SimpleNamespace(motion_state=1),
                types.SimpleNamespace(motion_state=0),
            ]
        )
        self.robot._dc_state = dc_state
        self.robot._step_state = step_state

        self.assertTrue(self.robot._wait_dc_position(4, 123, 0, 0.01))
        self.assertTrue(self.robot._wait_dc_not_homing(4, 0.01))
        with mock.patch.object(self.robot_module.time, "sleep", lambda *_args, **_kwargs: None):
            self.assertTrue(self.robot._wait_stepper_idle(4, 0.01))


if __name__ == "__main__":
    unittest.main()
