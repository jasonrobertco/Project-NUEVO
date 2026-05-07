"""
user_io.py — buttons, LEDs, and NeoPixel demo
==============================================
Demonstrates the three button-reading patterns and onboard LED control.
BTN_1 interacts within each phase. BTN_2 advances to the next phase.
A NeoPixel rainbow animates throughout to show background state updates.

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/user_io.py main.py
    ros2 run robot robot

THE THREE PHASES
----------------

  Phase 1 — EDGE LATCH  (was_button_pressed)
    BTN_1 cycles GREEN through four modes:
      OFF → steady ON → BLINK (500 ms) → BREATHE (1 000 ms) → …
    Each press fires exactly once — short presses are never missed.
    Press BTN_2 to advance to Phase 2.

  Phase 2 — LEVEL READ  (get_button)
    Hold BTN_1 to light ORANGE. Release to turn it off.
    This pattern is useful for "hold to run" or joystick-style inputs.
    Press BTN_2 to advance to Phase 3.

  Phase 3 — BLOCKING WAIT  (wait_for_button)
    The FSM pauses entirely — tick loop stops, NeoPixel freezes — until
    BTN_1 is pressed. Useful for "confirm before starting" patterns.
    On press, RED flashes three times, then the program exits cleanly.

WHAT THIS TEACHES
-----------------
1. was_button_pressed() — edge latch: one event per physical press
2. get_button()         — level read: True while held, False when released
3. wait_for_button()    — blocking: pauses everything until pressed
4. set_led()            — OFF / steady ON / BLINK / BREATHE modes
5. set_neopixel()       — per-pixel RGB color for NeoPixel strips
"""

from __future__ import annotations

import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, LEDMode, POSITION_UNIT
from robot.robot import FirmwareState, Robot


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# How bright the onboard LEDs glow (0–255)
LED_BRIGHTNESS = 220

# NeoPixel settings
NEOPIXEL_INDEX  = 0      # index of the pixel to animate (0-based)
NEO_SPEED_DEG_S = 90.0   # hue rotation speed in degrees/second

# RED flash count when Phase 3 completes
DONE_FLASH_COUNT = 3
DONE_FLASH_ON_S  = 0.2
DONE_FLASH_OFF_S = 0.2


# ---------------------------------------------------------------------------
# LED cycle modes for Phase 1
# ---------------------------------------------------------------------------

# Ordered list of (mode, period_ms) pairs the GREEN LED cycles through.
_LED_CYCLE: list[tuple[LEDMode, int]] = [
    (LEDMode.OFF,     0),
    (LEDMode.ON,      0),
    (LEDMode.BLINK,   500),
    (LEDMode.BREATHE, 1000),
]


def _apply_led_cycle(robot: Robot, index: int) -> None:
    mode, period_ms = _LED_CYCLE[index]
    brightness = 0 if mode == LEDMode.OFF else LED_BRIGHTNESS
    robot.set_led(LED.GREEN, brightness, mode=mode,
                  period_ms=period_ms if period_ms else None)
    names = {LEDMode.OFF: "OFF", LEDMode.ON: "steady ON",
             LEDMode.BLINK: f"BLINK {period_ms} ms",
             LEDMode.BREATHE: f"BREATHE {period_ms} ms"}
    print(f"  [LED] GREEN → {names[mode]}")


# ---------------------------------------------------------------------------
# NeoPixel rainbow helper
# ---------------------------------------------------------------------------

def _hsv_to_rgb(h_deg: float, s: float = 1.0, v: float = 0.4) -> tuple[int, int, int]:
    """Convert HSV (h in degrees) to (r, g, b) in 0–255."""
    h = h_deg % 360.0
    c = v * s
    x = c * (1.0 - abs((h / 60.0) % 2.0 - 1.0))
    m = v - c
    if h < 60:
        r, g, b = c, x, 0.0
    elif h < 120:
        r, g, b = x, c, 0.0
    elif h < 180:
        r, g, b = 0.0, c, x
    elif h < 240:
        r, g, b = 0.0, x, c
    elif h < 300:
        r, g, b = x, 0.0, c
    else:
        r, g, b = c, 0.0, x
    return int((r + m) * 255), int((g + m) * 255), int((b + m) * 255)


def _update_neopixel(robot: Robot, hue: float) -> None:
    r, g, b = _hsv_to_rgb(hue)
    robot.set_neopixel(NEOPIXEL_INDEX, r, g, b)


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


def all_leds_off(robot: Robot) -> None:
    for led in (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE):
        robot.set_led(led, 0)


def flash_red_and_done(robot: Robot) -> None:
    for _ in range(DONE_FLASH_COUNT):
        robot.set_led(LED.RED, LED_BRIGHTNESS)
        time.sleep(DONE_FLASH_ON_S)
        robot.set_led(LED.RED, 0)
        time.sleep(DONE_FLASH_OFF_S)


# ---------------------------------------------------------------------------
# run() — entry point called by the robot node
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)
    all_leds_off(robot)

    state         = "PHASE1_EDGE"
    led_cycle_idx = 0      # current position in _LED_CYCLE
    neo_hue       = 0.0    # current NeoPixel hue (degrees)
    last_neo_time = time.monotonic()

    print("=" * 60)
    print("PHASE 1 — EDGE LATCH  (was_button_pressed)")
    print("  BTN_1: cycle GREEN  OFF → ON → BLINK → BREATHE → ...")
    print("  BTN_2: advance to Phase 2")
    print("=" * 60)
    _apply_led_cycle(robot, led_cycle_idx)

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        # ── NeoPixel rainbow (runs every tick except during blocking wait) ──
        elapsed_neo   = now - last_neo_time
        neo_hue       = (neo_hue + NEO_SPEED_DEG_S * elapsed_neo) % 360.0
        last_neo_time = now
        _update_neopixel(robot, neo_hue)

        # ── Phase 1: edge latch ──────────────────────────────────────────────
        if state == "PHASE1_EDGE":
            if robot.was_button_pressed(Button.BTN_1):
                led_cycle_idx = (led_cycle_idx + 1) % len(_LED_CYCLE)
                _apply_led_cycle(robot, led_cycle_idx)

            elif robot.was_button_pressed(Button.BTN_2):
                all_leds_off(robot)
                print()
                print("=" * 60)
                print("PHASE 2 — LEVEL READ  (get_button)")
                print("  BTN_1: hold to light ORANGE, release to turn off")
                print("  BTN_2: advance to Phase 3")
                print("=" * 60)
                state = "PHASE2_LEVEL"

        # ── Phase 2: level read ──────────────────────────────────────────────
        elif state == "PHASE2_LEVEL":
            held = robot.get_button(Button.BTN_1)
            robot.set_led(LED.ORANGE, LED_BRIGHTNESS if held else 0)

            if robot.was_button_pressed(Button.BTN_2):
                robot.set_led(LED.ORANGE, 0)
                print()
                print("=" * 60)
                print("PHASE 3 — BLOCKING WAIT  (wait_for_button)")
                print("  BTN_1: confirm (blocks the tick loop — NeoPixel will freeze)")
                print("=" * 60)
                state = "PHASE3_BLOCKING"

        # ── Phase 3: blocking wait ───────────────────────────────────────────
        # wait_for_button() blocks the entire thread — the tick loop, the
        # NeoPixel update, and any other work all pause here.  Use this
        # pattern when nothing else needs to happen until the user acts.
        elif state == "PHASE3_BLOCKING":
            pressed = robot.wait_for_button(Button.BTN_1, timeout=60.0)
            if pressed:
                print("  BTN_1 detected — done!")
                flash_red_and_done(robot)
                all_leds_off(robot)
                robot.set_neopixel(NEOPIXEL_INDEX, 0, 0, 0)
                print("Demo complete. Shutting down.")
                robot.shutdown()
                return
            else:
                print("  [timeout] waiting again...")

        # ── Tick-rate control ────────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
