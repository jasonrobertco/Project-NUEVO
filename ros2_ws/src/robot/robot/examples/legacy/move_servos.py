"""
move_servos.py — button-triggered servo sequence example
========================================================
This example uses the same plain state-machine style as main.py.

To try it as the active robot program, copy this file over main.py:

    cp ros2_ws/src/robot/robot/examples/move_servos.py \
       ros2_ws/src/robot/robot/main.py

Behavior:
- robot enters RUNNING once at startup
- BTN_1 starts a custom servo sequence
- BTN_2 cancels the active sequence
- when the sequence finishes, the program returns to IDLE
"""

from __future__ import annotations

import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED
from robot.robot import FirmwareState, Robot, Unit
from robot.util import TaskHandle, run_task


# ---------------------------------------------------------------------------
# Robot build configuration
# ---------------------------------------------------------------------------

POSITION_UNIT = Unit.MM


# ---------------------------------------------------------------------------
# Example custom sequence configuration
# ---------------------------------------------------------------------------

SEQUENCE_SERVO_1 = 1
SEQUENCE_SERVO_2 = 2


def configure_robot(robot: Robot) -> None:
    """Apply the user unit. This example does not need path or odometry setup."""
    robot.set_unit(POSITION_UNIT)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.RED, 255)


def show_sequence_leds(robot: Robot) -> None:
    robot.set_led(LED.RED, 0)
    robot.set_led(LED.GREEN, 255)


def start_robot(robot: Robot) -> None:
    """Put the firmware in RUNNING before the example starts listening for buttons."""
    robot.set_state(FirmwareState.RUNNING)


def my_sequence_1(robot: Robot, blocking: bool = True, timeout: float = None):
    """
    Example custom servo sequence.

    Students can edit this function or copy it to create more sequences.

    Example usage:
        handle = my_sequence_1(robot, blocking=False)
        if handle.is_finished():
            print("sequence finished")
    """

    def worker(task: TaskHandle) -> None:
        robot.enable_servo(SEQUENCE_SERVO_1)
        robot.enable_servo(SEQUENCE_SERVO_2)

        robot.set_servo(SEQUENCE_SERVO_1, 30)
        if not task.sleep(0.5):
            return

        robot.set_servo(SEQUENCE_SERVO_2, 120)
        if not task.sleep(0.5):
            return

        robot.set_servo(SEQUENCE_SERVO_1, 90)
        if not task.sleep(0.5):
            return

        robot.set_servo(SEQUENCE_SERVO_2, 90)
        if not task.sleep(0.5):
            return

        robot.disable_servo(SEQUENCE_SERVO_1)
        robot.disable_servo(SEQUENCE_SERVO_2)

    return run_task(worker, blocking=blocking, timeout=timeout)


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    sequence_handle = None

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            print("[FSM] IDLE")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)

            if robot.get_button(Button.BTN_1):
                sequence_handle = my_sequence_1(robot, blocking=False)
                print("[FSM] RUN_SEQUENCE")
                state = "RUN_SEQUENCE"

        elif state == "RUN_SEQUENCE":
            show_sequence_leds(robot)

            if robot.get_button(Button.BTN_2):
                if sequence_handle is not None:
                    sequence_handle.cancel()
                    sequence_handle.wait(timeout=1.0)
                    sequence_handle = None
                print("[FSM] IDLE")
                state = "IDLE"

            elif sequence_handle is not None and sequence_handle.is_finished():
                sequence_handle = None
                print("[FSM] IDLE")
                state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
