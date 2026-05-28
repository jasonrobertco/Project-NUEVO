"""
main.py — Integrated Traffic Light Control & Obstacle Avoidance Pure Pursuit
============================================================================
This script implements vision-triggered start/stop logic alongside the
Pure Pursuit path planner with dynamic lane-switching obstacle avoidance.
"""

from __future__ import annotations

import math
import time

from robot.hardware_map import DEFAULT_FSM_HZ, LED, Motor
from robot.robot import FirmwareState, Robot, Unit
from robot.util import densify_polyline

# ---------------------------------------------------------------------------
# Hardware & Odometry Configuration
# ---------------------------------------------------------------------------
POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M2
LEFT_WHEEL_DIR_INVERTED = True
RIGHT_WHEEL_MOTOR = Motor.DC_M1
RIGHT_WHEEL_DIR_INVERTED = False

# ---------------------------------------------------------------------------
# System Configuration
# ---------------------------------------------------------------------------
LED_BRIGHTNESS = 255
LIGHT_HOLD_SEC = 2.0
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.50
TAG_ID = 11

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
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
    
    # Enable all necessary sensor pipelines
    robot.enable_vision()
    robot.enable_lidar()
    robot.enable_gps()
    robot.set_tracked_tag_id(TAG_ID)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)


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
# Main FSM Loop
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
            
            # 1. Define and densify the obstacle avoidance path
            path_control_points = [
                (0.0, 3352.8),
                (609.6, 3352.8),
                (609.6, 609.6),
            ]
            path = densify_polyline(path_control_points, spacing=400.0)

            # 2. Configure the pure pursuit lane-switching controller
            robot._nav_follow_pp_path(
                lookahead_distance=100.0,
                max_linear_speed=140.0,
                max_angular_speed=1.5,
                goal_tolerance=20.0,
                obstacles_range=450.0,
                view_angle=math.radians(70.0),
                safe_dist=250.0,
                avoidance_delay=150,
                alpha_Ld=0.7,
                offset=270.0,
                lane_width=500.0,
                obstacle_avoidance=True,
                x_L=300.0,
            )
            robot._set_obstacle_avoidance_path(path)
            
            print("[FSM] INIT Complete. WATCHING - show a green traffic light")
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
                
                if traffic_light_color == "green":
                    print("[FSM] Green light seen. Entering MOVING state.")
                    state = "MOVING"

            elif lights_off_at > 0.0 and now >= lights_off_at:
                dim_all_leds(robot)
                lights_off_at = 0.0
                if last_shown_color is not None:
                    print("[VISION] no recent red/green light - LEDs off")
                last_shown_color = None

        # -- MOVING ---------------------------------------------------------
        elif state == "MOVING":
            now = time.monotonic()
            traffic_light_color = find_traffic_light_color(robot)

            # Interrupt condition 1: Red light detected
            # Interrupt condition 2: Vision system is stale/lost
            if traffic_light_color == "red" or not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
                robot.set_velocity(0, 0)
                
                if traffic_light_color == "red":
                    show_traffic_light_color(robot, "red")
                    print("[FSM] Red light detected! Stopping evasion path.")
                    lights_off_at = now + LIGHT_HOLD_SEC
                    last_shown_color = "red"
                else:
                    dim_all_leds(robot)
                    print("[FSM] Vision stale. Stopping for safety.")
                    lights_off_at = 0.0
                    last_shown_color = None

                # Return to watching for the next green light
                state = "WATCHING"
            
            else:
                # No interrupts triggered; execute next control frame of APF
                next_state = robot._nav_follow_pp_path_loop()
                
                # The path loop returns "IDLE" when the final waypoint is reached
                if next_state == "IDLE":
                    robot.set_velocity(0, 0)
                    dim_all_leds(robot)
                    print("[FSM] Reached the end of the path. Resetting to WATCHING.")
                    state = "WATCHING"

        # -- Tick-rate control ---------------------------------------------
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()