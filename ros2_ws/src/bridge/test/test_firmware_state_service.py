import threading
import time
import unittest

from bridge.firmware_state_service import (
    RESULT_REJECTED,
    RESULT_SUCCESS,
    RESULT_TIMEOUT,
    STATE_ESTOP,
    STATE_ERROR,
    STATE_IDLE,
    STATE_RUNNING,
    SYS_CMD_RESET,
    SYS_CMD_START,
    FirmwareStateTransitionCoordinator,
)


class FirmwareStateTransitionCoordinatorTests(unittest.TestCase):
    def test_successful_running_transition_sends_start_and_waits_for_streamed_state(self):
        coordinator = FirmwareStateTransitionCoordinator(default_timeout_sec=0.2)
        coordinator.observe_system_state({"state": STATE_IDLE, "warningFlags": 0, "errorFlags": 0})

        sent_commands = []

        def send_command(command: int) -> bool:
            sent_commands.append(command)

            def complete_transition():
                time.sleep(0.02)
                coordinator.observe_system_state({"state": STATE_RUNNING, "warningFlags": 0, "errorFlags": 0})

            threading.Thread(target=complete_transition, daemon=True).start()
            return True

        result = coordinator.request_transition(STATE_RUNNING, 0.1, send_command)

        self.assertTrue(result.success)
        self.assertEqual(result.result_code, RESULT_SUCCESS)
        self.assertEqual(result.final_state, STATE_RUNNING)
        self.assertEqual(sent_commands, [SYS_CMD_START])

    def test_rejects_running_request_from_error_without_sending(self):
        coordinator = FirmwareStateTransitionCoordinator(default_timeout_sec=0.2)
        coordinator.observe_system_state({"state": STATE_ERROR, "warningFlags": 0, "errorFlags": 4})

        sent_commands = []
        result = coordinator.request_transition(
            STATE_RUNNING,
            0.1,
            lambda command: sent_commands.append(command) or True,
        )

        self.assertFalse(result.success)
        self.assertEqual(result.result_code, RESULT_REJECTED)
        self.assertEqual(result.final_state, STATE_ERROR)
        self.assertEqual(sent_commands, [])

    def test_times_out_when_target_state_is_not_observed(self):
        coordinator = FirmwareStateTransitionCoordinator(default_timeout_sec=0.2)
        coordinator.observe_system_state({"state": STATE_IDLE, "warningFlags": 0, "errorFlags": 0})

        sent_commands = []
        result = coordinator.request_transition(
            STATE_RUNNING,
            0.05,
            lambda command: sent_commands.append(command) or True,
        )

        self.assertFalse(result.success)
        self.assertEqual(result.result_code, RESULT_TIMEOUT)
        self.assertEqual(result.final_state, STATE_IDLE)
        self.assertEqual(sent_commands, [SYS_CMD_START])

    def test_idle_request_from_estop_uses_reset(self):
        coordinator = FirmwareStateTransitionCoordinator(default_timeout_sec=0.2)
        coordinator.observe_system_state({"state": STATE_ESTOP, "warningFlags": 0, "errorFlags": 0})

        sent_commands = []

        def send_command(command: int) -> bool:
            sent_commands.append(command)
            coordinator.observe_system_state({"state": STATE_IDLE, "warningFlags": 0, "errorFlags": 0})
            return True

        result = coordinator.request_transition(STATE_IDLE, 0.1, send_command)

        self.assertTrue(result.success)
        self.assertEqual(result.result_code, RESULT_SUCCESS)
        self.assertEqual(result.final_state, STATE_IDLE)
        self.assertEqual(sent_commands, [SYS_CMD_RESET])


if __name__ == "__main__":
    unittest.main()
