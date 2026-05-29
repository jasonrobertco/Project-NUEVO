"""
main.py — short maze mission using LAPF + lidar
===============================================
Press BTN_1 to start a short maze run. BTN_2 cancels the active goal.

The mission uses `lapf_to_goal()` as the primary planner, so the robot chases
LiDAR-shaped virtual targets instead of running pure pursuit as the top-level
mission controller.
"""

from __future__ import annotations

import math
import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LED,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    POSITION_UNIT,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot


ENABLE_LIDAR = True
ENABLE_GPS = False

# Preserved from the earlier team-specific maze entrypoint.
TAG_ID = 11

# Preserved from working28/main28backup behavior.
LEFT_WHEEL_MOTOR = 2
LEFT_WHEEL_DIR_INVERTED = True
RIGHT_WHEEL_MOTOR = 1
RIGHT_WHEEL_DIR_INVERTED = False

MAZE_GOALS_MM = (
    (0.0, 700.0),
)

VELOCITY_MM_S = 140.0
TOLERANCE_MM = 60.0
MAX_ANGULAR_RAD_S = 1.0
STATUS_PRINT_INTERVAL_S = 0.5

LEASH_LENGTH_MM = 400.0
REPULSION_RANGE_MM = 300.0
TARGET_SPEED_MM_S = 200.0
REPULSION_GAIN = 550.0
ATTRACTION_GAIN = 1.0
FORCE_EMA_ALPHA = 0.35
INFLATION_MARGIN_MM = 150.0
LEASH_HALF_ANGLE_DEG = 25.0


def resolve_lapf_config() -> dict[str, float]:
    return {
        "leash_length_mm": float(LEASH_LENGTH_MM),
        "repulsion_range_mm": float(REPULSION_RANGE_MM),
        "target_speed_mm_s": float(TARGET_SPEED_MM_S),
        "repulsion_gain": float(REPULSION_GAIN),
        "attraction_gain": float(ATTRACTION_GAIN),
        "force_ema_alpha": float(FORCE_EMA_ALPHA),
        "inflation_margin_mm": float(INFLATION_MARGIN_MM),
        "leash_half_angle_deg": float(LEASH_HALF_ANGLE_DEG),
    }


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
    )
    if not odom_confirmed:
        raise RuntimeError(
            "Odometry parameter confirmation failed; refusing to start mission. "
            f"Expected left={LEFT_WHEEL_MOTOR} inverted={LEFT_WHEEL_DIR_INVERTED}, "
            f"right={RIGHT_WHEEL_MOTOR} inverted={RIGHT_WHEEL_DIR_INVERTED}."
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


def reset_mission_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
        robot.wait_for_pose_update(timeout=0.5)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def cancel_motion(robot: Robot, handle) -> None:
    if handle is not None:
        handle.cancel()
        handle.wait(timeout=1.0)
    robot.stop()


def print_status(robot: Robot, goal_index: int) -> None:
    x, y, theta = robot.get_odometry_pose()
    virtual_target = robot.get_virtual_target()
    obstacle_tracks = robot.get_obstacle_tracks()

    if virtual_target is None:
        vt_summary = "vt=(none)"
    else:
        vt_summary = f"vt=({virtual_target[0]:6.0f}, {virtual_target[1]:6.0f}) mm"

    print(
        f"  goal={goal_index + 1}/{len(MAZE_GOALS_MM)} "
        f"odom=({x:6.0f}, {y:6.0f}) mm "
        f"theta={theta:5.1f} deg "
        f"{vt_summary} tracked={len(obstacle_tracks)}"
    )


def start_goal(robot: Robot, goal_index: int):
    cfg = resolve_lapf_config()
    goal_x, goal_y = MAZE_GOALS_MM[goal_index]
    return robot.lapf_to_goal(
        goal_x,
        goal_y,
        velocity=VELOCITY_MM_S,
        tolerance=TOLERANCE_MM,
        leash_length_mm=cfg["leash_length_mm"],
        repulsion_range_mm=cfg["repulsion_range_mm"],
        target_speed_mm_s=cfg["target_speed_mm_s"],
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
        repulsion_gain=cfg["repulsion_gain"],
        attraction_gain=cfg["attraction_gain"],
        force_ema_alpha=cfg["force_ema_alpha"],
        inflation_margin_mm=cfg["inflation_margin_mm"],
        leash_half_angle_deg=cfg["leash_half_angle_deg"],
        blocking=False,
    )


def distance_to_goal_mm(robot: Robot, goal_index: int) -> float:
    x, y, _theta = robot.get_odometry_pose()
    goal_x, goal_y = MAZE_GOALS_MM[goal_index]
    return math.hypot(goal_x - x, goal_y - y)


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    motion_handle = None
    goal_index = 0
    last_status_print_at = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        if state == "INIT":
            start_robot(robot)
            reset_mission_pose(robot)
            show_idle_leds(robot)
            goal_index = 0
            print("[FSM] IDLE — press BTN_1 to start the LAPF maze mission, BTN_2 to cancel")
            print(f"[CFG] goals={MAZE_GOALS_MM}")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_running_leds(robot)
                goal_index = 0
                motion_handle = start_goal(robot, goal_index)
                last_status_print_at = now
                print(f"[FSM] MOVING — started LAPF goal 1/{len(MAZE_GOALS_MM)} -> {MAZE_GOALS_MM[0]}")
                state = "MOVING"

        elif state == "MOVING":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE — maze mission cancelled")
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot, goal_index)
                    last_status_print_at = now
                if motion_handle is not None and motion_handle.is_finished():
                    remaining_mm = distance_to_goal_mm(robot, goal_index)
                    if remaining_mm > TOLERANCE_MM:
                        print(
                            f"[warn] motion finished {remaining_mm:.0f} mm from goal "
                            f"{goal_index + 1}; stopping instead of advancing"
                        )
                        robot.stop()
                        motion_handle = None
                        show_idle_leds(robot)
                        print("[FSM] IDLE — press BTN_1 to run again")
                        state = "IDLE"
                        continue
                    goal_index += 1
                    if goal_index >= len(MAZE_GOALS_MM):
                        print("[FSM] DONE — maze mission complete")
                        print_status(robot, len(MAZE_GOALS_MM) - 1)
                        robot.stop()
                        motion_handle = None
                        show_idle_leds(robot)
                        print("[FSM] IDLE — press BTN_1 to run again")
                        state = "IDLE"
                    else:
                        motion_handle = start_goal(robot, goal_index)
                        print(
                            f"[FSM] MOVING — started LAPF goal {goal_index + 1}/{len(MAZE_GOALS_MM)} "
                            f"-> {MAZE_GOALS_MM[goal_index]}"
                        )

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
