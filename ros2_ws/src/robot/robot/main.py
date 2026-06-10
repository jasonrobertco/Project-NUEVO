"""
main_manipulator.py — Nerf ball shooter demo
============================================

Demonstrates the Shooter manipulator API on a 3-servo + DC_M3 flywheel build:
  - Servo 1: yaw (left/right)
  - Servo 2: pitch (up/down)
  - Servo 3: loader arm
  - DC_M3: flywheel (open-loop PWM)

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp main_manipulator.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
Press BTN_1 to aim at the demo angles and shoot at the demo distance.
Press BTN_2 to run a single load cycle (loader test, flywheel off).
Press BTN_3 to calibrate the loader (rest, then step toward push).
Press BTN_6 to move the loader to rest position.
Press BTN_7 to move the loader to push position.

Tune DEMO_YAW_DEG, DEMO_PITCH_DEG, and DEMO_DISTANCE_M in shooter.py or
below once you've measured the physical build.
"""

from __future__ import annotations

import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, POSITION_UNIT
from robot.robot import FirmwareState, Robot
from robot.shooter import (
    Shooter,
    LOADER_PUSH_DEG,
    LOADER_REST_DEG,
    PITCH_CENTER_DEG,
    YAW_CENTER_DEG,
)


# ---------------------------------------------------------------------------
# Demo shot parameters — edit to match your field / target layout
# ---------------------------------------------------------------------------

DEMO_YAW_DEG = YAW_CENTER_DEG
DEMO_PITCH_DEG = PITCH_CENTER_DEG
DEMO_DISTANCE_M = 1.5


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def run_demo_shot(shooter: Shooter) -> None:
    """Aim and fire one ball at the demo angles and distance."""
    print(
        f"[SHOOT] yaw={DEMO_YAW_DEG:.0f}° pitch={DEMO_PITCH_DEG:.0f}° "
        f"distance={DEMO_DISTANCE_M:.1f} m"
    )
    shooter.aim_and_shoot(DEMO_YAW_DEG, DEMO_PITCH_DEG, DEMO_DISTANCE_M)
    print("[SHOOT] complete")


def run_load_test(shooter: Shooter) -> None:
    """Cycle the loader without spinning the flywheel."""
    print("[LOAD] single load cycle")
    shooter.load()
    print("[LOAD] complete")


def run_loader_calibration(shooter: Shooter) -> None:
    """Move loader to rest, then step toward push for position tuning."""
    target = shooter.calibrate_loader()
    print(f"[CALIB] loader stepped to {target:.0f}° — update LOADER_REST_DEG if this looks right")


def run(robot: Robot) -> None:
    configure_robot(robot)

    shooter: Shooter | None = None
    state = "INIT"

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            shooter = Shooter(robot)
            shooter.enable()
            show_idle_leds(robot)
            print(
                "[FSM] IDLE — BTN_1: shoot, BTN_2: load, BTN_3: calib, "
                "BTN_6: loader rest, BTN_7: loader push"
            )
            print(
                f"[CFG] demo yaw={DEMO_YAW_DEG:.0f}° pitch={DEMO_PITCH_DEG:.0f}° "
                f"distance={DEMO_DISTANCE_M:.1f} m"
            )
            state = "IDLE"

        elif state == "IDLE":
            if shooter is None:
                state = "INIT"
                continue

            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)
                print("[FSM] RUN_SHOT")
                state = "RUN_SHOT"

            elif robot.was_button_pressed(Button.BTN_2):
                show_running_leds(robot)
                print("[FSM] RUN_LOAD")
                state = "RUN_LOAD"

            elif robot.was_button_pressed(Button.BTN_3):
                show_running_leds(robot)
                print("[FSM] RUN_LOADER_CALIB")
                state = "RUN_LOADER_CALIB"

            elif robot.was_button_pressed(Button.BTN_6):
                shooter.loader_rest()
                print(f"[LOADER] rest ({LOADER_REST_DEG:.0f}°)")

            elif robot.was_button_pressed(Button.BTN_7):
                shooter.loader_push()
                print(f"[LOADER] push ({LOADER_PUSH_DEG:.0f}°)")

        elif state == "RUN_SHOT":
            if shooter is not None:
                try:
                    run_demo_shot(shooter)
                finally:
                    shooter.spin_down()
            show_idle_leds(robot)
            print("[FSM] IDLE")
            state = "IDLE"

        elif state == "RUN_LOAD":
            if shooter is not None:
                run_load_test(shooter)
            show_idle_leds(robot)
            print("[FSM] IDLE")
            state = "IDLE"

        elif state == "RUN_LOADER_CALIB":
            if shooter is not None:
                run_loader_calibration(shooter)
            show_idle_leds(robot)
            print("[FSM] IDLE")
            state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
