"""
course_route_rough.py — rough multi-section competition route
==============================================================
This is a pragmatic first-pass route script for the course shown in the sketch:

1. start -> checkpoint 1: straight climb
2. checkpoint 1 -> checkpoint 2: straight ramp descent with extra speed
3. checkpoint 2 -> checkpoint 3: preplanned cone slalom
4. checkpoint 3 -> checkpoint 4 / finish: slower descent over speed bumps

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/course_route_rough.py main.py
    ros2 run robot robot

BTN_1 starts the route.
BTN_2 cancels the current motion and returns to IDLE.

This route is intentionally rough. The waypoint values below are meant to be
edited quickly on the field after a test run.
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LED,
    POSITION_UNIT,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot


ENABLE_GPS = False
ENABLE_LIDAR = False
TAG_ID = -1

# Keep the same wheel mapping currently used by the team's working main.py.
LEFT_WHEEL_MOTOR = 2
LEFT_WHEEL_DIR_INVERTED = True
RIGHT_WHEEL_MOTOR = 1
RIGHT_WHEEL_DIR_INVERTED = False


# ---------------------------------------------------------------------------
# Rough course geometry in the odometry-reset frame.
# Start pose is assumed to be near the green box facing +Y.
# Units are millimeters.
# ---------------------------------------------------------------------------

CHECKPOINT_1_Y_MM = 2350.0
CHECKPOINT_2_Y_MM = 350.0
RIGHT_LANE_X_MM = 560.0
CHECKPOINT_3_Y_MM = 2350.0
FINISH_Y_MM = 380.0


# ---------------------------------------------------------------------------
# Section tuning
# ---------------------------------------------------------------------------

SECTION_1_SPEED_MM_S = 180.0
SECTION_1_TOL_MM = 40.0

RAMP_SPEED_MM_S = 260.0
RAMP_TOL_MM = 45.0

CONE_SPEED_MM_S = 170.0
CONE_LOOKAHEAD_MM = 180.0
CONE_TOL_MM = 60.0
CONE_ADVANCE_RADIUS_MM = 140.0
CONE_MAX_ANGULAR_RAD_S = 1.8

SPEED_BUMP_SPEED_MM_S = 135.0
SPEED_BUMP_TOL_MM = 50.0

OPTIONAL_PULL_INTO_FINISH_BOX = False
FINAL_BOX_X_MM = 900.0
FINAL_BOX_Y_MM = 260.0
FINAL_BOX_SPEED_MM_S = 120.0
FINAL_BOX_TOL_MM = 50.0

STATUS_PRINT_INTERVAL_S = 0.5


# Preplanned slalom through the cone section. Edit these after each test run.
CONE_PATH_WAYPOINTS = [
    (0.0, CHECKPOINT_2_Y_MM),
    (180.0, 520.0),
    (RIGHT_LANE_X_MM, 760.0),
    (380.0, 1060.0),
    (RIGHT_LANE_X_MM, 1380.0),
    (400.0, 1720.0),
    (RIGHT_LANE_X_MM, 2050.0),
    (RIGHT_LANE_X_MM, CHECKPOINT_3_Y_MM),
]


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    odom_confirmed = robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
        timeout=3.0,
    )
    if not odom_confirmed:
        raise RuntimeError(
            "Odometry parameter confirmation failed; refusing to start route. "
            f"Expected left={LEFT_WHEEL_MOTOR} inverted={LEFT_WHEEL_DIR_INVERTED}, "
            f"right={RIGHT_WHEEL_MOTOR} inverted={RIGHT_WHEEL_DIR_INVERTED}."
        )

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        print(f"[sensor] GPS enabled — tracking tag {TAG_ID}")

    if ENABLE_LIDAR:
        robot.enable_lidar()
        print("[sensor] lidar enabled")


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def reset_mission_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2.0s; continuing anyway")
        robot.wait_for_pose_update(timeout=0.5)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 180)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 180)


def cancel_motion(handle) -> None:
    if handle is None:
        return
    handle.cancel()
    handle.wait(timeout=1.0)


def print_status(robot: Robot, label: str) -> None:
    x_mm, y_mm, theta_deg = robot.get_pose()
    print(f"  [{label}] pose=({x_mm:6.0f}, {y_mm:6.0f}) mm  θ={theta_deg:5.1f}°")


def start_section_1(robot: Robot):
    print("[route] section 1: straight to checkpoint 1")
    return robot.move_forward(
        distance=CHECKPOINT_1_Y_MM,
        velocity=SECTION_1_SPEED_MM_S,
        tolerance=SECTION_1_TOL_MM,
        blocking=False,
    )


def start_section_2_ramp(robot: Robot):
    ramp_distance_mm = CHECKPOINT_1_Y_MM - CHECKPOINT_2_Y_MM
    print("[route] section 2: fast straight ramp descent")
    return robot.move_backward(
        distance=ramp_distance_mm,
        velocity=RAMP_SPEED_MM_S,
        tolerance=RAMP_TOL_MM,
        blocking=False,
    )


def start_section_3_cones(robot: Robot):
    print("[route] section 3: cone slalom path")
    return robot.purepursuit_follow_path(
        waypoints=CONE_PATH_WAYPOINTS,
        velocity=CONE_SPEED_MM_S,
        lookahead=CONE_LOOKAHEAD_MM,
        tolerance=CONE_TOL_MM,
        advance_radius=CONE_ADVANCE_RADIUS_MM,
        max_angular_rad_s=CONE_MAX_ANGULAR_RAD_S,
        blocking=False,
    )


def start_section_4_speed_bumps(robot: Robot):
    speed_bump_distance_mm = CHECKPOINT_3_Y_MM - FINISH_Y_MM
    print("[route] section 4: slower descent over speed bumps")
    return robot.move_backward(
        distance=speed_bump_distance_mm,
        velocity=SPEED_BUMP_SPEED_MM_S,
        tolerance=SPEED_BUMP_TOL_MM,
        blocking=False,
    )


def start_final_pull_in(robot: Robot):
    print("[route] optional final pull into manipulation box")
    return robot.move_to(
        x=FINAL_BOX_X_MM,
        y=FINAL_BOX_Y_MM,
        velocity=FINAL_BOX_SPEED_MM_S,
        tolerance=FINAL_BOX_TOL_MM,
        blocking=False,
    )


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    motion_handle = None
    last_status_print_at = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        if state in {
            "RUN_SECTION_1",
            "RUN_SECTION_2",
            "RUN_SECTION_3",
            "RUN_SECTION_4",
            "RUN_FINAL_BOX",
        } and robot.was_button_pressed(Button.BTN_2):
            cancel_motion(motion_handle)
            motion_handle = None
            robot.stop()
            show_idle_leds(robot)
            print("[FSM] IDLE — route cancelled")
            state = "IDLE"

        elif state == "INIT":
            start_robot(robot)
            reset_mission_pose(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to start rough course route")
            print(
                "[CFG] sec1 straight="
                f"{CHECKPOINT_1_Y_MM:.0f} mm @ {SECTION_1_SPEED_MM_S:.0f} mm/s | "
                f"ramp={CHECKPOINT_1_Y_MM - CHECKPOINT_2_Y_MM:.0f} mm @ {RAMP_SPEED_MM_S:.0f} mm/s | "
                f"cones={len(CONE_PATH_WAYPOINTS)} waypoints | "
                f"speed bumps={CHECKPOINT_3_Y_MM - FINISH_Y_MM:.0f} mm @ {SPEED_BUMP_SPEED_MM_S:.0f} mm/s"
            )
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_running_leds(robot)
                motion_handle = start_section_1(robot)
                last_status_print_at = now
                state = "RUN_SECTION_1"

        elif state == "RUN_SECTION_1":
            if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                print_status(robot, "section 1")
                last_status_print_at = now
            if motion_handle is not None and motion_handle.is_finished():
                print_status(robot, "checkpoint 1 reached")
                motion_handle = start_section_2_ramp(robot)
                state = "RUN_SECTION_2"

        elif state == "RUN_SECTION_2":
            if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                print_status(robot, "ramp")
                last_status_print_at = now
            if motion_handle is not None and motion_handle.is_finished():
                print_status(robot, "checkpoint 2 reached")
                motion_handle = start_section_3_cones(robot)
                state = "RUN_SECTION_3"

        elif state == "RUN_SECTION_3":
            if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                print_status(robot, "cones")
                last_status_print_at = now
            if motion_handle is not None and motion_handle.is_finished():
                print_status(robot, "checkpoint 3 reached")
                motion_handle = start_section_4_speed_bumps(robot)
                state = "RUN_SECTION_4"

        elif state == "RUN_SECTION_4":
            if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                print_status(robot, "speed bumps")
                last_status_print_at = now
            if motion_handle is not None and motion_handle.is_finished():
                print_status(robot, "finish line reached")
                motion_handle = None
                robot.stop()
                if OPTIONAL_PULL_INTO_FINISH_BOX:
                    motion_handle = start_final_pull_in(robot)
                    state = "RUN_FINAL_BOX"
                else:
                    show_idle_leds(robot)
                    print("[FSM] DONE — route complete")
                    state = "IDLE"

        elif state == "RUN_FINAL_BOX":
            if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                print_status(robot, "final box")
                last_status_print_at = now
            if motion_handle is not None and motion_handle.is_finished():
                print_status(robot, "final position reached")
                motion_handle = None
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] DONE — route complete")
                state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
