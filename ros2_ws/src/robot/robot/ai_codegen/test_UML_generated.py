from __future__ import annotations
import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LED,
    LEDMode,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        # ── FSM states go here ────────────────────────────────────────────
        if state == "INIT":
            current = robot.get_state()
            if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            robot.set_led(LED.ORANGE, 0)
            robot.set_led(LED.PURPLE, 0)
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                robot.set_led(LED.GREEN, 200, mode=LEDMode.BLINK, period_ms=500)
                robot.set_led(LED.ORANGE, 200)
                state = "ORANGE"

        elif state == "ORANGE":
            if robot.was_button_pressed(Button.BTN_1):
                robot.set_led(LED.GREEN, 0)
                robot.set_led(LED.PURPLE, 200)
                robot.set_led(LED.ORANGE, 0)
                state = "PURPLE"

        elif state == "PURPLE":
            if robot.was_button_pressed(Button.BTN_1):
                robot.set_led(LED.ORANGE, 0)
                robot.set_led(LED.PURPLE, 0)
                state = "IDLE"

        # ── Tick-rate control (do not modify) ─────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
