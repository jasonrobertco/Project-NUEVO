"""
apf_obstacle_avoidance.py — APF path following with lidar
=========================================================
Artificial potential fields (APF) path following using the current `main.py`
style FSM loop.

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/apf_obstacle_avoidance.py main.py
    ros2 run robot robot

Lidar must already be publishing `/scan`. One standard bring-up is:

    ros2 launch rplidar_ros rplidar_c1.launch.py

Press BTN_1 to start the path. BTN_2 cancels the active run.

WHAT THIS TEACHES
-----------------
1. `enable_lidar()` and optional `enable_gps()` setup
2. Non-blocking `apf_follow_path()` inside an FSM loop
3. Periodic status printing while motion is active
4. Clean cancel / rerun handling with a `MotionHandle`
"""

from __future__ import annotations

import time
import math

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    Motor,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
)
from robot.robot import FirmwareState, Robot, Unit


# Shared lidar/GPS hardware calibration lives in robot/hardware_map.py.
# If you need to change lidar mount, lidar self-filtering, or GPS tag body
# offset values, edit ros2_ws/src/robot/robot/hardware_map.py.
# You can also just set those values here locally if you want.
ENABLE_LIDAR = True
ENABLE_GPS = False

# IMPORTANT: update TAG_ID to match your robot when GPS is enabled.
TAG_ID = -1

POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

WAYPOINTS_MM = [
    (0.0, 1000.0),
    (0.0, 2000.0),
]

VELOCITY_MM_S = 150.0
_APF_API_LOOKAHEAD_MM = 50.0  # current APF wrapper still accepts this, but the planner does not use it
TOLERANCE_MM = 50.0
MAX_ANGULAR_RAD_S = 1.0
# Clearance buffer: mm of air gap between robot surface and obstacle surface
# at which repulsion begins. This is a physical gap, not an axle-to-obstacle
# distance.
REPULSION_RANGE_MM = 300.0
REPULSION_GAIN = 1200.0
# Robot rectangle in robot frame (axle = rotation centre = origin).
# Rear-drive:  front = axle→front bumper, rear = axle→rear bumper.
# Front-drive: swap front and rear values.
ROBOT_FRONT_MM      = 400.0   # axle to front face
ROBOT_REAR_MM       = 100.0   # axle to rear face
ROBOT_HALF_WIDTH_MM = 200.0   # half-width of body

STATUS_PRINT_INTERVAL_S = 0.5


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


def print_status(robot: Robot) -> None:
    if ENABLE_GPS and robot.has_fused_pose():
        x, y, theta = robot.get_fused_pose()
        label = "fused"
    else:
        x, y, theta = robot.get_odometry_pose()
        label = "odom "
    raw_obstacle_count = len(robot.get_obstacles())
    obstacle_tracks = robot.get_obstacle_tracks()
    tracked_count = len(obstacle_tracks)
    if obstacle_tracks:
        nearest_boundary_mm = min(
            max(
                0.0,
                math.hypot(float(track["x"]) - x, float(track["y"]) - y) - float(track["radius"]),
            )
            for track in obstacle_tracks
        )
        track_summary = f" tracked={tracked_count} nearest_track={nearest_boundary_mm:.0f} mm"
    else:
        track_summary = f" tracked={tracked_count}"
    freshness = f" gps_fresh={robot.is_gps_active()}" if ENABLE_GPS and label == "fused" else ""
    print(
        f"  {label}=({x:6.0f}, {y:6.0f}) mm  θ={theta:5.1f}°"
        f"  raw_obstacles={raw_obstacle_count}{track_summary}{freshness}"
    )


def start_path(robot: Robot):
    return robot.apf_follow_path(
        WAYPOINTS_MM,
        velocity=VELOCITY_MM_S,
        lookahead=_APF_API_LOOKAHEAD_MM,
        tolerance=TOLERANCE_MM,
        repulsion_range=REPULSION_RANGE_MM,
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
        repulsion_gain=REPULSION_GAIN,
        robot_front_mm=ROBOT_FRONT_MM,
        robot_rear_mm=ROBOT_REAR_MM,
        robot_half_width_mm=ROBOT_HALF_WIDTH_MM,
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

        if state == "INIT":
            start_robot(robot)
            reset_mission_pose(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to start APF path, BTN_2 to cancel")
            print(f"[CFG] waypoints={WAYPOINTS_MM}")
            print(
                f"[CFG] velocity={VELOCITY_MM_S:.0f} mm/s tolerance={TOLERANCE_MM:.0f} mm "
                f"repulsion_range={REPULSION_RANGE_MM:.0f} mm gain={REPULSION_GAIN:.0f} "
                f"max_angular={MAX_ANGULAR_RAD_S:.1f} rad/s"
            )
            print(
                f"[CFG] robot rect: front={ROBOT_FRONT_MM:.0f} rear={ROBOT_REAR_MM:.0f} "
                f"half_width={ROBOT_HALF_WIDTH_MM:.0f} mm"
            )
            if ENABLE_LIDAR:
                print(
                    f"[CFG] lidar mount=({LIDAR_MOUNT_X_MM:.0f}, {LIDAR_MOUNT_Y_MM:.0f}) mm "
                    f"theta={LIDAR_MOUNT_THETA_DEG:.1f}° filter={LIDAR_RANGE_MIN_MM:.0f}-"
                    f"{LIDAR_RANGE_MAX_MM:.0f} mm fov={LIDAR_FOV_DEG}"
                )
                print(
                    f"[CFG] tracker ttl={robot.OBSTACLE_TRACK_TTL_S:.1f}s max_tracks={robot.OBSTACLE_TRACK_MAX_TRACKS} "
                    f"planner_tracks={robot.APF_MAX_PLANNER_TRACKS} max_disk_radius={robot.OBSTACLE_TRACK_MAX_DISK_RADIUS_MM:.0f} mm"
                )
            if ENABLE_GPS:
                print(
                    f"[CFG] gps tag_id={TAG_ID} tag_body=({TAG_BODY_OFFSET_X_MM:.0f}, "
                    f"{TAG_BODY_OFFSET_Y_MM:.0f}) mm"
                )
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_running_leds(robot)
                motion_handle = start_path(robot)
                last_status_print_at = now
                print("[FSM] MOVING — APF path started")
                state = "MOVING"

        elif state == "MOVING":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE — cancelled")
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status_print_at = now
                if motion_handle is not None and motion_handle.is_finished():
                    robot.stop()
                    motion_handle = None
                    show_idle_leds(robot)
                    print("[FSM] DONE — path complete")
                    print_status(robot)
                    state = "DONE"

        elif state == "DONE":
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_running_leds(robot)
                motion_handle = start_path(robot)
                last_status_print_at = now
                print("[FSM] MOVING — APF path restarted")
                state = "MOVING"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
