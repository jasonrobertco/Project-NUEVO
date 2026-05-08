"""
mechanism_test_tilt_servo.py — isolated tilt servo test
=======================================================
Safely exercises the turret tilt/range servo only.

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/mechanism_test_tilt_servo.py main.py
    ros2 run robot robot

IMPORTANT
---------
The Robot API uses 1-based servo channels (1-16), while PCA9685 wiring is
often labeled starting at 0. This script assumes the tilt servo is wired to
physical PCA9685 channel 1, which maps to Robot API channel 2.
Update the channel constant below to match your actual wiring.
"""

from __future__ import annotations

import time

from robot.hardware_map import POSITION_UNIT
from robot.robot import FirmwareState, Robot


# Provisional wiring:
#   physical PCA9685 channel 1 -> Robot API channel 2
TILT_SERVO_CHANNEL = 2

MIN_PULSE_US = 1000
CENTER_PULSE_US = 1500
MAX_PULSE_US = 2000

LOW_PULSE_US = 1300
HIGH_PULSE_US = 1700
STEP_WAIT_S = 1.0


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def set_pulse(robot: Robot, channel: int, pulse_us: int) -> None:
    safe_pulse = max(MIN_PULSE_US, min(MAX_PULSE_US, int(pulse_us)))
    robot.set_servo_pulse(channel, safe_pulse)


def sleep_step(seconds: float) -> None:
    time.sleep(seconds)


def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    print("[MECH] Tilt servo test starting")
    print(
        "[MECH] Using Robot API channel "
        f"{TILT_SERVO_CHANNEL} for provisional PCA9685 physical channel 1"
    )

    robot.enable_servo(TILT_SERVO_CHANNEL)

    try:
        print(f"[MECH] Move tilt servo to center: {CENTER_PULSE_US} us")
        set_pulse(robot, TILT_SERVO_CHANNEL, CENTER_PULSE_US)
        sleep_step(STEP_WAIT_S)

        print(f"[MECH] Move tilt servo down: {LOW_PULSE_US} us")
        set_pulse(robot, TILT_SERVO_CHANNEL, LOW_PULSE_US)
        sleep_step(STEP_WAIT_S)

        print(f"[MECH] Return tilt servo to center: {CENTER_PULSE_US} us")
        set_pulse(robot, TILT_SERVO_CHANNEL, CENTER_PULSE_US)
        sleep_step(STEP_WAIT_S)

        print(f"[MECH] Move tilt servo up: {HIGH_PULSE_US} us")
        set_pulse(robot, TILT_SERVO_CHANNEL, HIGH_PULSE_US)
        sleep_step(STEP_WAIT_S)

        print(f"[MECH] Return tilt servo to center: {CENTER_PULSE_US} us")
        set_pulse(robot, TILT_SERVO_CHANNEL, CENTER_PULSE_US)
        sleep_step(STEP_WAIT_S)

        print("[MECH] Tilt servo test complete")
    finally:
        print("[MECH] Disabling tilt servo")
        robot.disable_servo(TILT_SERVO_CHANNEL)
