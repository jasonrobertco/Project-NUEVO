"""
pure_pursuit.py — waypoint path following with pure pursuit
============================================================
This example uses purepursuit_follow_path() to drive a closed triangular
path. It shows how to tune the key parameters and how to use a non-blocking
MotionHandle to keep the FSM responsive during a long path.

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/pure_pursuit.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
BTN_1 starts the path. The robot drives a triangle:

    origin (0, 0)  →  (0, 600)  →  (500, 300)  →  back to (0, 0)

BTN_2 cancels the path at any time and returns to IDLE.
The green LED pulses while moving; the orange LED is on at rest.

WHAT THIS TEACHES
-----------------
1. purepursuit_follow_path()  — multi-waypoint path with tunable parameters
2. Key parameters explained    — velocity, lookahead, tolerance, advance_radius
3. densify_polyline()          — optional: insert intermediate points for
                                 smoother tracking on curves or long segments
4. Non-blocking MotionHandle   — keep the FSM loop alive during a long path

PARAMETER GUIDE
---------------
velocity        Forward speed in user units/s. Higher = faster but harder to
                stay on path at turns.

lookahead       The "carrot" distance ahead on the path that the robot steers
                toward. Larger = smoother but cuts corners more. A good
                starting value is 1–2× the robot width.

tolerance       Arrival threshold for the FINAL waypoint in user units. The
                robot stops when it gets this close to the last point.

advance_radius  Radius in user units for dropping INTERMEDIATE waypoints.
                When the robot gets within advance_radius of a waypoint, it
                stops targeting that waypoint and moves to the next one.
                Defaults to tolerance if not set. Increase it to prevent the
                robot from decelerating toward every intermediate point.

max_angular_rad_s  Angular rate clamp in rad/s. Lower = gentler turns but
                   may cause the robot to cut corners on tight paths.
"""

from __future__ import annotations
import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.robot import FirmwareState, Robot, Unit
from robot.util import densify_polyline  # noqa: F401 — used in the optional line below


# ---------------------------------------------------------------------------
# Robot hardware configuration — edit these to match your robot
# ---------------------------------------------------------------------------

POSITION_UNIT   = Unit.MM
WHEEL_DIAMETER  = 74.0   # mm
WHEEL_BASE      = 333.0  # mm
INITIAL_THETA   = 90.0   # degrees

LEFT_WHEEL_MOTOR         = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED  = False
RIGHT_WHEEL_MOTOR        = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True


# ---------------------------------------------------------------------------
# Path definition — (x, y) in mm, starting from the origin
# ---------------------------------------------------------------------------

PATH_CONTROL_POINTS = [
    (0.0,   0.0),   # start (matches odometry origin after reset)
    (0.0, 600.0),   # leg 1 — straight forward
    (500.0, 300.0), # leg 2 — diagonal
    (0.0,   0.0),   # leg 3 — back to origin
]

# Optional: insert densely spaced intermediate points between control points.
# Useful when individual segments are long and the robot drifts off the line,
# or when a curved segment needs finer tracking.
# Uncomment the next line to enable:
# PATH_CONTROL_POINTS = densify_polyline(PATH_CONTROL_POINTS, spacing=50.0)


# ---------------------------------------------------------------------------
# Motion parameters — tune these for your robot and floor surface
# ---------------------------------------------------------------------------

VELOCITY         = 150.0  # mm/s
LOOKAHEAD        = 120.0  # mm  — steer toward a point this far ahead
TOLERANCE        = 25.0   # mm  — stop threshold at the final waypoint
ADVANCE_RADIUS   = 80.0   # mm  — drop intermediate waypoints at this radius
MAX_ANGULAR_RAD_S = 1.5   # rad/s — angular rate clamp


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
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
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
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
    drive_handle = None

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        # ── INIT ──────────────────────────────────────────────────────────
        if state == "INIT":
            start_robot(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to start path, BTN_2 to cancel")
            state = "IDLE"

        # ── IDLE ──────────────────────────────────────────────────────────
        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                show_moving_leds(robot)
                print(f"[FSM] MOVING — {len(PATH_CONTROL_POINTS)} waypoints")

                drive_handle = robot.purepursuit_follow_path(
                    waypoints=PATH_CONTROL_POINTS,
                    velocity=VELOCITY,
                    lookahead=LOOKAHEAD,
                    tolerance=TOLERANCE,
                    advance_radius=ADVANCE_RADIUS,
                    max_angular_rad_s=MAX_ANGULAR_RAD_S,
                    blocking=False,
                )
                state = "MOVING"

        # ── MOVING ────────────────────────────────────────────────────────
        elif state == "MOVING":

            if robot.get_button(Button.BTN_2):
                if drive_handle is not None:
                    drive_handle.cancel()
                    drive_handle.wait(timeout=1.0)
                    drive_handle = None
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] IDLE — path cancelled")
                state = "IDLE"

            elif drive_handle is not None and drive_handle.is_finished():
                x, y, theta_deg = robot.get_pose()
                print(f"[FSM] path complete  x={x:.1f}  y={y:.1f}  θ={theta_deg:.1f}°")
                drive_handle = None
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] IDLE — press BTN_1 to run again")
                state = "IDLE"

        # ── Tick-rate control ─────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
