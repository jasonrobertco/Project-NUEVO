"""
motion_basics.py — FSM-based motion basics
==========================================
Teaches the core drive-motion Robot API calls using the same tick-loop pattern
as the student-facing `main.py`.

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/motion_basics.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
Press BTN_1 to start. The robot runs the same basic motion sequence twice:

  1. Non-blocking version:
     - turn left 90°
     - turn right 90°
     - move forward 500 mm
     - move backward 500 mm
  2. Blocking version:
     - turn left 90°
     - turn right 90°
     - move forward 500 mm
     - move backward 500 mm

BTN_2 cancels the non-blocking pass and returns to IDLE. During the blocking
pass, the robot stays inside one FSM state until that whole pass finishes.

WHAT THIS TEACHES
-----------------
1. `set_odometry_parameters()` and `reset_odometry()` setup flow
2. State-by-state non-blocking motion with `MotionHandle`
3. One-state blocking motion with the same high-level motion methods
4. Optional lidar and GPS enable flow during a blocking example
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    INITIAL_THETA_DEG,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot


# Shared drive-base and lidar/GPS hardware calibration lives in
# robot/hardware_map.py. If you need to change wheel geometry, wheel motors,
# lidar mount, lidar self-filtering, or GPS tag body offset values, edit
# ros2_ws/src/robot/robot/hardware_map.py. You can also just set those values
# here locally if you want.
ENABLE_LIDAR = False

ENABLE_GPS = False

# IMPORTANT: update TAG_ID to match the tag assigned to this robot.
TAG_ID = -1


# ---------------------------------------------------------------------------
# Configuration — edit these to match your robot
# ---------------------------------------------------------------------------

TURN_DEGREES = 90.0
FORWARD_DISTANCE_MM = 500.0
DRIVE_VELOCITY_MM_S = 200.0
DRIVE_TOLERANCE_MM = 20.0
TURN_TOLERANCE_DEG = 3.0


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

    if ENABLE_LIDAR:
        robot.enable_lidar()
        robot.set_lidar_mount(
            x_mm=LIDAR_MOUNT_X_MM,
            y_mm=LIDAR_MOUNT_Y_MM,
            theta_deg=LIDAR_MOUNT_THETA_DEG,
        )
        robot.set_lidar_filter(
            range_min_mm=LIDAR_RANGE_MIN_MM,
            range_max_mm=LIDAR_RANGE_MAX_MM,
            fov_deg=LIDAR_FOV_DEG,
        )
        robot.start_lidar_world_publisher()
        print("[sensor] lidar enabled — subscribing to /scan")

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
        print(f"[sensor] GPS enabled — tracking ArUco tag {TAG_ID}")


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


def cancel_motion(handle) -> None:
    if handle is None:
        return
    handle.cancel()
    handle.wait(timeout=1.0)


def print_status(robot: Robot, label: str) -> None:
    ox, oy, otheta = robot.get_odometry_pose()
    if ENABLE_GPS and robot.has_fused_pose():
        fused_pose = robot.get_fused_pose()
        if fused_pose is not None:
            fx, fy, ftheta = fused_pose
            print(
                f"  [{label}] odom=({ox:6.0f}, {oy:6.0f}) mm  θ={otheta:5.1f}°  |  "
                f"fused=({fx:6.0f}, {fy:6.0f}) mm  θ={ftheta:5.1f}°  "
                f"(gps_fresh={robot.is_gps_active()})"
            )
            return
    print(f"  [{label}] odom=({ox:6.0f}, {oy:6.0f}) mm  θ={otheta:5.1f}°")


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    motion_handle = None

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state.startswith("NON_BLOCKING_") and robot.was_button_pressed(Button.BTN_2):
            cancel_motion(motion_handle)
            motion_handle = None
            robot.stop()
            show_idle_leds(robot)
            print("[FSM] IDLE — non-blocking pass cancelled")
            state = "IDLE"

        elif state == "INIT":
            start_robot(robot)
            robot.reset_odometry()
            if not robot.wait_for_odometry_reset(timeout=2.0):
                print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
                robot.wait_for_pose_update(timeout=0.5)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to start")
            print("[FSM] BTN_2 cancels only the non-blocking pass")
            print(
                f"[CFG] turn={TURN_DEGREES:.0f}° forward={FORWARD_DISTANCE_MM:.0f} mm "
                f"velocity={DRIVE_VELOCITY_MM_S:.0f} mm/s"
            )
            if ENABLE_LIDAR:
                print(
                    f"[CFG] lidar mount=({LIDAR_MOUNT_X_MM:.0f}, {LIDAR_MOUNT_Y_MM:.0f}) mm "
                    f"theta={LIDAR_MOUNT_THETA_DEG:.1f}° filter={LIDAR_RANGE_MIN_MM:.0f}-"
                    f"{LIDAR_RANGE_MAX_MM:.0f} mm fov={LIDAR_FOV_DEG}"
                )
            if ENABLE_GPS:
                print(
                    f"[CFG] gps tag_id={TAG_ID} tag_body=({TAG_BODY_OFFSET_X_MM:.0f}, "
                    f"{TAG_BODY_OFFSET_Y_MM:.0f}) mm"
                )
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                robot.reset_odometry()
                if not robot.wait_for_odometry_reset(timeout=2.0):
                    print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
                    robot.wait_for_pose_update(timeout=0.5)
                show_running_leds(robot)
                print("[FSM] NON_BLOCKING_TURN_LEFT")
                motion_handle = robot.turn_by(
                    delta_deg=TURN_DEGREES,
                    blocking=False,
                    tolerance_deg=TURN_TOLERANCE_DEG,
                )
                state = "NON_BLOCKING_WAIT_TURN_LEFT"

        elif state == "NON_BLOCKING_WAIT_TURN_LEFT":
            if motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                print_status(robot, "non-blocking turn left done")
                print("[FSM] NON_BLOCKING_TURN_RIGHT")
                motion_handle = robot.turn_by(
                    delta_deg=-TURN_DEGREES,
                    blocking=False,
                    tolerance_deg=TURN_TOLERANCE_DEG,
                )
                state = "NON_BLOCKING_WAIT_TURN_RIGHT"

        elif state == "NON_BLOCKING_WAIT_TURN_RIGHT":
            if motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                print_status(robot, "non-blocking turn right done")
                print("[FSM] NON_BLOCKING_MOVE_FORWARD")
                motion_handle = robot.move_forward(
                    distance=FORWARD_DISTANCE_MM,
                    velocity=DRIVE_VELOCITY_MM_S,
                    tolerance=DRIVE_TOLERANCE_MM,
                    blocking=False,
                )
                state = "NON_BLOCKING_WAIT_MOVE_FORWARD"

        elif state == "NON_BLOCKING_WAIT_MOVE_FORWARD":
            if motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                print_status(robot, "non-blocking move forward done")
                print("[FSM] NON_BLOCKING_MOVE_BACKWARD")
                motion_handle = robot.move_backward(
                    distance=FORWARD_DISTANCE_MM,
                    velocity=DRIVE_VELOCITY_MM_S,
                    tolerance=DRIVE_TOLERANCE_MM,
                    blocking=False,
                )
                state = "NON_BLOCKING_WAIT_MOVE_BACKWARD"

        elif state == "NON_BLOCKING_WAIT_MOVE_BACKWARD":
            if motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                print_status(robot, "non-blocking move backward done")
                print("[FSM] NON_BLOCKING_DONE — starting blocking pass")
                state = "START_BLOCKING_PASS"

        elif state == "START_BLOCKING_PASS":
            robot.reset_odometry()
            if not robot.wait_for_odometry_reset(timeout=2.0):
                print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
                robot.wait_for_pose_update(timeout=0.5)
            print("[FSM] BLOCKING_PASS")
            state = "BLOCKING_PASS"

        elif state == "BLOCKING_PASS":
            print("[FSM] BLOCKING_TURN_LEFT")
            robot.turn_by(
                delta_deg=TURN_DEGREES,
                blocking=True,
                tolerance_deg=TURN_TOLERANCE_DEG,
            )
            print_status(robot, "blocking turn left done; sleeping for 1 second")
            time.sleep(1.0)
            
            print("[FSM] BLOCKING_TURN_RIGHT")
            robot.turn_by(
                delta_deg=-TURN_DEGREES,
                blocking=True,
                tolerance_deg=TURN_TOLERANCE_DEG,
            )
            print_status(robot, "blocking turn right done; sleeping for 1 second")
            time.sleep(1.0)
            
            print("[FSM] BLOCKING_MOVE_FORWARD")
            robot.move_forward(
                distance=FORWARD_DISTANCE_MM,
                velocity=DRIVE_VELOCITY_MM_S,
                tolerance=DRIVE_TOLERANCE_MM,
                blocking=True,
            )
            print_status(robot, "blocking move forward done; sleeping for 1 second")
            time.sleep(1.0)
            
            print("[FSM] BLOCKING_MOVE_BACKWARD")
            robot.move_backward(
                distance=FORWARD_DISTANCE_MM,
                velocity=DRIVE_VELOCITY_MM_S,
                tolerance=DRIVE_TOLERANCE_MM,
                blocking=True,
            )
            print_status(robot, "blocking move backward done; sleeping for 1 second")
            time.sleep(1.0)
            
            robot.stop()
            show_idle_leds(robot)
            print("[FSM] DONE — press BTN_1 to run again")
            motion_handle = None
            state = "DONE"

        elif state == "DONE":
            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)
                robot.reset_odometry()
                if not robot.wait_for_odometry_reset(timeout=2.0):
                    print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
                    robot.wait_for_pose_update(timeout=0.5)
                print("[FSM] NON_BLOCKING_TURN_LEFT")
                motion_handle = robot.turn_by(
                    delta_deg=TURN_DEGREES,
                    blocking=False,
                    tolerance_deg=TURN_TOLERANCE_DEG,
                )
                state = "NON_BLOCKING_WAIT_TURN_LEFT"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
