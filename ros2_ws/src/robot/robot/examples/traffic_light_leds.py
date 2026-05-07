"""
traffic_light_leds.py - vision-triggered LED control
====================================================
This example reads traffic-light detections from the vision node and uses the
Robot API to light the matching onboard LED.

HOW TO RUN
----------
Start the vision node in another terminal:

    ros2 run vision vision_node

Then copy this file over main.py and restart the robot node:

    cp examples/traffic_light_leds.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
If a red traffic light is detected, the red LED turns on.
If a green traffic light is detected, the green LED turns on.
If no new red/green traffic light is seen for 2 seconds, all LEDs turn off.

WHAT THIS TEACHES
-----------------
1. Reading vision results through the Robot API
2. Finding a specific detected class and reading its attributes
3. Holding an output for a short time without blocking the FSM loop
"""

from __future__ import annotations

import time

from robot.hardware_map import DEFAULT_FSM_HZ, LED, POSITION_UNIT
from robot.robot import FirmwareState, Robot


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

LED_BRIGHTNESS = 255
LIGHT_HOLD_SEC = 2.0
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.50


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision()


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def dim_all_leds(robot: Robot) -> None:
    for led in (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE):
        robot.set_led(led, 0)


def show_traffic_light_color(robot: Robot, color: str) -> None:
    if color == "red":
        robot.set_led(LED.RED, LED_BRIGHTNESS)
        robot.set_led(LED.GREEN, 0)
    elif color == "green":
        robot.set_led(LED.RED, 0)
        robot.set_led(LED.GREEN, LED_BRIGHTNESS)


def find_traffic_light_color(robot: Robot) -> str | None:
    """Return the best recent red/green traffic-light result, or None."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None

    best_color = None
    best_confidence = -1.0

    for detection in robot.get_detections("traffic light"):
        confidence = float(detection["confidence"])
        if confidence < MIN_TRAFFIC_LIGHT_CONFIDENCE:
            continue

        attributes = detection.get("attributes", {})
        color_attribute = attributes.get("color", {})
        color = color_attribute.get("value")
        if color not in ("red", "green"):
            continue

        if confidence > best_confidence:
            best_confidence = confidence
            best_color = str(color)

    return best_color


# ---------------------------------------------------------------------------
# run() - entry point called by the robot node
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    lights_off_at = 0.0
    last_shown_color = None

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        # -- INIT -----------------------------------------------------------
        if state == "INIT":
            start_robot(robot)
            dim_all_leds(robot)
            print("[FSM] WATCHING - show a red or green traffic light")
            state = "WATCHING"

        # -- WATCHING -------------------------------------------------------
        elif state == "WATCHING":
            now = time.monotonic()

            traffic_light_color = find_traffic_light_color(robot)

            if traffic_light_color in ("red", "green"):
                show_traffic_light_color(robot, traffic_light_color)
                lights_off_at = now + LIGHT_HOLD_SEC

                if traffic_light_color != last_shown_color:
                    print(f"[VISION] traffic light: {traffic_light_color}")
                last_shown_color = traffic_light_color

            elif lights_off_at > 0.0 and now >= lights_off_at:
                dim_all_leds(robot)
                lights_off_at = 0.0
                if last_shown_color is not None:
                    print("[VISION] no recent red/green light - LEDs off")
                last_shown_color = None

            # Generic API version for custom objects:
            # detections = robot.get_detections("my_object")
            # value = robot.get_detection_attribute("my_object", "my_attribute")

        # -- Tick-rate control ---------------------------------------------
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
