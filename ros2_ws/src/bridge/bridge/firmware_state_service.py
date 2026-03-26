from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional


STATE_INIT = 0
STATE_IDLE = 1
STATE_RUNNING = 2
STATE_ERROR = 3
STATE_ESTOP = 4

RESULT_SUCCESS = 0
RESULT_REJECTED = 1
RESULT_TIMEOUT = 2
RESULT_BRIDGE_UNAVAILABLE = 3

SYS_CMD_START = 1
SYS_CMD_STOP = 2
SYS_CMD_RESET = 3
SYS_CMD_ESTOP = 4

DEFAULT_TIMEOUT_SEC = 2.0


@dataclass(frozen=True)
class TransitionPlan:
    command: Optional[int]
    target_state: int
    wait_states: tuple[int, ...]
    description: str


@dataclass(frozen=True)
class TransitionResult:
    success: bool
    result_code: int
    final_state: int
    warning_flags: int
    error_flags: int
    message: str


class FirmwareStateTransitionCoordinator:
    """Tracks streamed system state and executes bounded state transitions."""

    def __init__(self, default_timeout_sec: float = DEFAULT_TIMEOUT_SEC):
        self._default_timeout_sec = default_timeout_sec
        self._condition = threading.Condition()
        self._latest_state: Optional[dict] = None

    def observe_system_state(self, data: dict) -> None:
        snapshot = {
            "state": int(data.get("state", STATE_INIT)),
            "warningFlags": int(data.get("warningFlags", 0)),
            "errorFlags": int(data.get("errorFlags", 0)),
        }
        with self._condition:
            self._latest_state = snapshot
            self._condition.notify_all()

    def request_transition(
        self,
        target_state: int,
        timeout_sec: float,
        send_command: Callable[[int], bool],
    ) -> TransitionResult:
        effective_timeout = timeout_sec if timeout_sec > 0.0 else self._default_timeout_sec

        with self._condition:
            current = self._snapshot_unlocked()
            if current is None:
                return self._result(
                    False,
                    RESULT_BRIDGE_UNAVAILABLE,
                    None,
                    "No /sys_state telemetry available yet.",
                )
            planned = self._plan_transition(int(target_state), current)

        if isinstance(planned, TransitionResult):
            return planned

        if planned.command is not None and not send_command(planned.command):
            return self._result(
                False,
                RESULT_BRIDGE_UNAVAILABLE,
                current,
                f"Failed to send {self._command_name(planned.command)}.",
            )

        deadline = time.monotonic() + effective_timeout
        with self._condition:
            while True:
                current = self._snapshot_unlocked() or current
                if int(current["state"]) == planned.target_state:
                    message = "Already in target state." if planned.command is None else f"Reached {self._state_name(planned.target_state)}."
                    return self._result(True, RESULT_SUCCESS, current, message)

                if int(current["state"]) not in planned.wait_states:
                    return self._result(
                        False,
                        RESULT_REJECTED,
                        current,
                        f"Transition to {self._state_name(planned.target_state)} rejected by streamed state {self._state_name(int(current['state']))}.",
                    )

                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return self._result(
                        False,
                        RESULT_TIMEOUT,
                        current,
                        f"Timed out waiting for {self._state_name(planned.target_state)} after {effective_timeout:.2f}s.",
                    )
                self._condition.wait(timeout=remaining)

    def _snapshot_unlocked(self) -> Optional[dict]:
        return dict(self._latest_state) if self._latest_state is not None else None

    def _plan_transition(self, target_state: int, current: dict) -> TransitionPlan | TransitionResult:
        current_state = int(current["state"])

        if target_state == current_state:
            return self._result(True, RESULT_SUCCESS, current, "Already in target state.")

        if target_state == STATE_RUNNING:
            if current_state == STATE_IDLE:
                return TransitionPlan(
                    command=SYS_CMD_START,
                    target_state=STATE_RUNNING,
                    wait_states=(STATE_IDLE, STATE_RUNNING),
                    description="IDLE -> RUNNING",
                )
            return self._result(False, RESULT_REJECTED, current, "RUNNING can only be requested from IDLE.")

        if target_state == STATE_IDLE:
            if current_state == STATE_RUNNING:
                return TransitionPlan(
                    command=SYS_CMD_STOP,
                    target_state=STATE_IDLE,
                    wait_states=(STATE_RUNNING, STATE_IDLE),
                    description="RUNNING -> IDLE",
                )
            if current_state in (STATE_ERROR, STATE_ESTOP):
                return TransitionPlan(
                    command=SYS_CMD_RESET,
                    target_state=STATE_IDLE,
                    wait_states=(STATE_ERROR, STATE_ESTOP, STATE_IDLE),
                    description="ERROR/ESTOP -> IDLE",
                )
            return self._result(False, RESULT_REJECTED, current, "IDLE can only be requested from RUNNING, ERROR, or ESTOP.")

        if target_state == STATE_ESTOP:
            return TransitionPlan(
                command=SYS_CMD_ESTOP,
                target_state=STATE_ESTOP,
                wait_states=(STATE_INIT, STATE_IDLE, STATE_RUNNING, STATE_ERROR, STATE_ESTOP),
                description="any -> ESTOP",
            )

        return self._result(False, RESULT_REJECTED, current, f"Unsupported target state {target_state}.")

    def _result(self, success: bool, result_code: int, state: Optional[dict], message: str) -> TransitionResult:
        state = state or {}
        return TransitionResult(
            success=success,
            result_code=result_code,
            final_state=int(state.get("state", STATE_INIT)),
            warning_flags=int(state.get("warningFlags", 0)),
            error_flags=int(state.get("errorFlags", 0)),
            message=message,
        )

    @staticmethod
    def _state_name(state: int) -> str:
        names = {
            STATE_INIT: "INIT",
            STATE_IDLE: "IDLE",
            STATE_RUNNING: "RUNNING",
            STATE_ERROR: "ERROR",
            STATE_ESTOP: "ESTOP",
        }
        return names.get(state, f"UNKNOWN({state})")

    @staticmethod
    def _command_name(command: int) -> str:
        names = {
            SYS_CMD_START: "START",
            SYS_CMD_STOP: "STOP",
            SYS_CMD_RESET: "RESET",
            SYS_CMD_ESTOP: "ESTOP",
        }
        return names.get(command, f"UNKNOWN({command})")
