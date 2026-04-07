"""
buttons_and_leds.py — button-triggered LED control
===================================================
Start here. This example has no motion. It teaches the FSM pattern and the
I/O API before anything moves.

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/buttons_and_leds.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
Each press of BTN_1 cycles the green LED through four modes:

    OFF  →  STEADY  →  BLINK  →  BREATHE  →  OFF  →  ...

BTN_2 resets to OFF at any time.
The orange LED stays on while in IDLE to show the program is running.

WHAT THIS TEACHES
-----------------
1. The FSM structure — a string state variable + if/elif inside while True
2. was_button_pressed() — fires exactly once per physical press (edge latch)
3. get_button()         — True as long as the button is held (level read)
4. set_led() modes      — OFF, steady ON, BLINK, BREATHE
5. The tick-rate loop   — DEFAULT_FSM_HZ keeps loop timing predictable
"""

from __future__ import annotations
import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, LEDMode
from robot.robot import FirmwareState, Robot, Unit


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

POSITION_UNIT = Unit.MM

# The green LED cycles through these modes in order.
_CYCLE = [
    ("OFF",     LEDMode.OFF,     0,   0),    # (label, mode, brightness, period_ms)
    ("STEADY",  LEDMode.ON,    200,   0),
    ("BLINK",   LEDMode.BLINK, 200, 600),
    ("BREATHE", LEDMode.BREATHE, 200, 1500),
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def apply_green_led(robot: Robot, index: int) -> None:
    """Apply the LED setting at _CYCLE[index] to the green LED."""
    label, mode, brightness, period_ms = _CYCLE[index]
    robot.set_led(LED.GREEN, brightness, mode=mode, period_ms=period_ms)
    print(f"[FSM] green LED → {label}")


# ---------------------------------------------------------------------------
# run() — entry point called by the robot node
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    cycle_index = 0  # current position in _CYCLE

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        # ── INIT ──────────────────────────────────────────────────────────
        if state == "INIT":
            start_robot(robot)
            robot.set_led(LED.ORANGE, 200)
            robot.set_led(LED.GREEN, 0)
            print("[FSM] IDLE — press BTN_1 to cycle LED, BTN_2 to reset")
            state = "IDLE"

        # ── IDLE ──────────────────────────────────────────────────────────
        elif state == "IDLE":

            # was_button_pressed() latches the rising edge in the ROS
            # callback, so short taps are never missed even if the loop
            # runs slower than the button release. Use it for one-shot
            # actions like toggling or advancing a sequence.
            if robot.was_button_pressed(Button.BTN_1):
                cycle_index = (cycle_index + 1) % len(_CYCLE)
                apply_green_led(robot, cycle_index)

            # get_button() reads the current level — True as long as the
            # button is physically held. Use it for continuous actions
            # like "keep moving while held".
            if robot.get_button(Button.BTN_2):
                cycle_index = 0
                apply_green_led(robot, cycle_index)
                robot.set_led(LED.ORANGE, 200)  # restore idle indicator

        # ── Tick-rate control ─────────────────────────────────────────────
        # Keeps the loop running at exactly DEFAULT_FSM_HZ (= IO_INPUT_HZ,
        # 50 Hz). Without this, the loop would spin as fast as the CPU
        # allows, wasting resources and making timing unpredictable.
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
