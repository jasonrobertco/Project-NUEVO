"""
motion_basics.py — drive, turn, read pose, navigate to a point
==============================================================
This example introduces all the foundational motion APIs in one mission.
Read it top to bottom — the states follow the natural robot startup order.

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/motion_basics.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
Press BTN_1 to start. The robot:

  1. Moves forward 400 mm
  2. Turns left (CCW) 90°
  3. Prints the current odometry pose to the console
  4. Navigates back to the origin (0, 0)

When done, press BTN_1 again to repeat. BTN_2 cancels at any time.

WHAT THIS TEACHES
-----------------
1. configure_robot()      — set unit, wheel geometry, motor mapping
2. reset_odometry()       — zero the pose before the mission starts
3. move_forward()         — blocking motion (waits for completion)
4. turn_by()              — relative heading change, also blocking
5. get_pose()             — read current (x, y, theta_deg) from odometry
6. move_to()              — non-blocking with MotionHandle
7. handle.is_finished()   — poll completion from the FSM loop
8. handle.cancel()        — stop a running motion
"""

from __future__ import annotations
import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.robot import FirmwareState, Robot, Unit


# ---------------------------------------------------------------------------
# Robot hardware configuration — edit these to match your robot
# ---------------------------------------------------------------------------

POSITION_UNIT    = Unit.MM
WHEEL_DIAMETER   = 74.0   # mm
WHEEL_BASE       = 333.0  # mm
INITIAL_THETA    = 90.0   # degrees — heading at odometry reset

LEFT_WHEEL_MOTOR          = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED   = False
RIGHT_WHEEL_MOTOR         = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED  = True


# ---------------------------------------------------------------------------
# Mission parameters
# ---------------------------------------------------------------------------

FORWARD_DISTANCE  = 400.0  # mm
TURN_DEGREES      = 90.0   # degrees CCW (positive = left turn)
VELOCITY          = 100.0  # mm/s
TOLERANCE         = 20.0   # mm — how close is "arrived"
TOLERANCE_DEG     = 3.0    # degrees — turn completion threshold


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    """Apply unit and wheel geometry. Called once at startup."""
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )


def start_robot(robot: Robot) -> None:
    """Put firmware in RUNNING and zero the odometry pose."""
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    # Wait for the first kinematics message so get_pose() is valid.
    robot.wait_for_pose_update(timeout=0.5)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


# ---------------------------------------------------------------------------
# run() — entry point called by the robot node
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    move_handle = None  # holds the MotionHandle for non-blocking calls

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        # ── INIT ──────────────────────────────────────────────────────────
        if state == "INIT":
            start_robot(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to start, BTN_2 to cancel")
            state = "IDLE"

        # ── IDLE ──────────────────────────────────────────────────────────
        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                show_moving_leds(robot)
                print("[FSM] LEG_1 — moving forward")

                # blocking=True makes the call wait here until done.
                # The FSM loop does NOT advance until the motion finishes.
                # Simple and readable; use it when nothing else needs to
                # happen while the robot moves.
                robot.move_forward(
                    distance=FORWARD_DISTANCE,
                    velocity=VELOCITY,
                    tolerance=TOLERANCE,
                    blocking=True,
                )
                state = "TURNING"

        # ── TURNING ───────────────────────────────────────────────────────
        elif state == "TURNING":
            print("[FSM] TURNING — 90° left")

            # turn_by also runs blocking here. The robot turns, then the
            # FSM advances to PRINT_POSE.
            robot.turn_by(
                delta_deg=TURN_DEGREES,
                blocking=True,
                tolerance_deg=TOLERANCE_DEG,
            )
            state = "PRINT_POSE"

        # ── PRINT_POSE ────────────────────────────────────────────────────
        elif state == "PRINT_POSE":
            # get_pose() returns (x, y, theta_deg) in the current unit.
            x, y, theta_deg = robot.get_pose()
            print(f"[FSM] pose  x={x:.1f} mm  y={y:.1f} mm  θ={theta_deg:.1f}°")

            print("[FSM] RETURNING — navigating to origin")

            # blocking=False returns a MotionHandle immediately.
            # The robot starts moving in a background thread.
            # The FSM continues to the next loop iteration so it can
            # check for cancellation while the robot is moving.
            move_handle = robot.move_to(
                x=0.0,
                y=0.0,
                velocity=VELOCITY,
                tolerance=TOLERANCE,
                blocking=False,
            )
            state = "RETURNING"

        # ── RETURNING ─────────────────────────────────────────────────────
        elif state == "RETURNING":
            # BTN_2 cancels the active motion at any time.
            if robot.get_button(Button.BTN_2):
                if move_handle is not None:
                    move_handle.cancel()
                    move_handle.wait(timeout=1.0)
                    move_handle = None
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] IDLE — cancelled")
                state = "IDLE"

            # is_finished() is a non-blocking poll — True once the background
            # thread reaches the goal or times out.
            elif move_handle is not None and move_handle.is_finished():
                move_handle = None
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] DONE — press BTN_1 to run again")
                state = "DONE"

        # ── DONE ──────────────────────────────────────────────────────────
        elif state == "DONE":
            if robot.was_button_pressed(Button.BTN_1):
                # Reset odometry so the next run starts from (0, 0) again.
                robot.reset_odometry()
                robot.wait_for_pose_update(timeout=0.5)
                show_moving_leds(robot)
                print("[FSM] LEG_1 — moving forward")
                robot.move_forward(
                    distance=FORWARD_DISTANCE,
                    velocity=VELOCITY,
                    tolerance=TOLERANCE,
                    blocking=True,
                )
                state = "TURNING"

        # ── Tick-rate control ─────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
