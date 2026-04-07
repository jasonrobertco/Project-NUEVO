from __future__ import annotations
import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, LEDMode, Motor
from robot.robot import FirmwareState, Robot, Unit


POSITION_UNIT   = Unit.MM
WHEEL_DIAMETER  = 74.0   # mm — update if different
WHEEL_BASE      = 333.0  # mm — update if different
INITIAL_THETA   = 90.0   # degrees


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA,
        left_motor_id=Motor.DC_M1,
        left_motor_dir_inverted=False,
        right_motor_id=Motor.DC_M2,
        right_motor_dir_inverted=True,
    )


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    # No motion handles needed — this is a pure I/O program.

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        # ── FSM states ────────────────────────────────────────────────────

        if state == "INIT":
            # 1. Clear any fault before requesting RUNNING.
            current = robot.get_state()
            if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()

            # 2. Transition to RUNNING.
            robot.set_state(FirmwareState.RUNNING)

            # Steps 3 & 4 (odometry reset / wait) omitted — pure I/O program.

            # Entry action for IDLE: dim both orange and purple LEDs.
            robot.set_led(LED.ORANGE, 0)
            robot.set_led(LED.PURPLE, 0)
            state = "IDLE"

        elif state == "IDLE":
            # Guard: button_pressed(1)
            if robot.was_button_pressed(Button.BTN_1):
                # Transition action: blink GREEN LED.
                robot.set_led(LED.GREEN, 200, mode=LEDMode.BLINK, period_ms=500)
                # Entry action for ORANGE: lights up orange LED.
                robot.set_led(LED.ORANGE, 200)
                state = "ORANGE"

        elif state == "ORANGE":
            # Guard: button_pressed(1)
            if robot.was_button_pressed(Button.BTN_1):
                # Transition action: stop blink GREEN LED.
                robot.set_led(LED.GREEN, 0)
                # Entry action for PURPLE: lights up purple LED; dim the orange LED.
                robot.set_led(LED.PURPLE, 200)
                robot.set_led(LED.ORANGE, 0)
                state = "PURPLE"

        elif state == "PURPLE":
            # Guard: button_pressed(1)
            if robot.was_button_pressed(Button.BTN_1):
                # Entry action for IDLE: dim both orange and purple LEDs.
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
