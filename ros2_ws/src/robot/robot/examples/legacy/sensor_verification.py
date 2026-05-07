"""
sensor_verification.py — Lidar + GPS + pure-pursuit UI verification run.

Purpose
-------
Drive a 2-grid-cell square path while printing live sensor diagnostics so you
can simultaneously verify:
  1. Lidar config    — obstacle count printed each tick; cloud published to
                       /lidar_world_points and appears on the World Map canvas.
  2. GPS setup       — tag detection, arena-frame fix, and gps_active flag.
  3. UI plots        — fused trail (blue), odometry trail (green), GPS cross
                       (yellow), and lidar cloud all update during the run.

HOW TO USE
----------
1. Copy to main.py:
       cp examples/sensor_verification.py main.py

2. Tune the constants below (robot hardware + lidar installation).

3. Restart the robot node:
       ros2 run robot robot

4. Open the UI → RPi Sensors → World Map.

5. Press BTN_1 to start the path.  Watch:
     - Console: GPS/lidar diagnostics print every second.
     - World Map canvas: fused trail (blue) and odometry trail (green) diverge;
       GPS cross (yellow) appears when the tag is visible; lidar cloud updates.

6. Press BTN_2 at any time to cancel and return to IDLE.
   Press BTN_1 again from IDLE to run the path again.

"""

from __future__ import annotations
import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.robot import FirmwareState, Robot, Unit


# ── Robot hardware ─────────────────────────────────────────────────────────────
POSITION_UNIT            = Unit.MM
WHEEL_DIAMETER           = 74.0    # mm
WHEEL_BASE               = 333.0   # mm
INITIAL_THETA            = 90.0    # degrees — robot faces +Y at start

LEFT_WHEEL_MOTOR         = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED  = False
RIGHT_WHEEL_MOTOR        = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

# ── Lidar mount & filter ───────────────────────────────────────────────────────
# Body frame origin = midpoint of the two drive wheels. +x = robot forward, +y = robot left.
LIDAR_MOUNT_X_MM      = 0.0    # lidar x position in robot body frame (mm, +x = forward)
LIDAR_MOUNT_Y_MM      = 0.0    # lidar y position in robot body frame (mm, +y = left)
LIDAR_MOUNT_THETA_DEG = 0.0    # lidar heading offset relative to robot forward (deg, CCW+)
                                # use 180.0 if lidar is mounted facing backward

LIDAR_RANGE_MIN_MM    = 150.0          # discard returns closer than this from lidar origin (mm)
LIDAR_RANGE_MAX_MM    = 6000.0         # discard returns farther than this from lidar origin (mm)
# FOV window in robot-aligned frame centred on lidar origin (0° = robot +x forward, CCW positive).
# (-180, 180) = full scan; (-90, 90) = front hemisphere only.
LIDAR_FOV_DEG         = (-180.0, 180.0)

# ── GPS / ArUco tag ────────────────────────────────────────────────────────────
GPS_TAG_ID = -1   # -1 = accept any tag; set to your actual tag ID

# ── Pure-pursuit path ──────────────────────────────────────────────────────────
# A 2-grid-cell square, centred on the odometry origin (wherever you place the robot).
# The robot starts at (0, 0) after reset and returns there at the end.
GRID_MM = 610
_S = GRID_MM * 2   # 1220 mm side length
PATH = [
    (0.0,  0.0),
    (_S,   0.0),
    (_S,   _S),
    (0.0,  _S),
    (0.0,  0.0),
]

VELOCITY           = 150.0   # mm/s
LOOKAHEAD          = 120.0   # mm
TOLERANCE          = 30.0    # mm  — final waypoint stop threshold
ADVANCE_RADIUS     = 80.0    # mm  — intermediate waypoint drop radius
MAX_ANGULAR_RAD_S  = 1.5     # rad/s

# ── Diagnostics ────────────────────────────────────────────────────────────────
DIAG_INTERVAL_S = 1.0   # how often to print the sensor status line


# ──────────────────────────────────────────────────────────────────────────────

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
    robot.enable_gps()
    robot.set_tracked_tag_id(GPS_TAG_ID)
    # Publish lidar world points continuously on the ROS spin thread — works
    # even during blocking navigation calls.
    robot.start_lidar_world_publisher(topic='/lidar_world_points', hz=10.0)


def print_diagnostics(robot: Robot, label: str) -> None:
    fx, fy, ftheta = robot.get_fused_pose()
    rx, ry, rtheta = robot.get_pose()
    gps_ok  = robot.is_gps_active()
    n_obs   = len(robot.get_obstacles())

    print(
        f"[{label}] "
        f"fused=({fx:.0f}, {fy:.0f}, {ftheta:.1f}°)  "
        f"odom=({rx:.0f}, {ry:.0f})  "
        f"GPS={'OK' if gps_ok else '--'}  "
        f"lidar_pts={n_obs}"
    )


def run(robot: Robot) -> None:
    configure_robot(robot)

    state        = "INIT"
    drive_handle = None
    last_diag_t  = 0.0

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        now = time.monotonic()

        # ── INIT ──────────────────────────────────────────────────────────────
        if state == "INIT":
            current = robot.get_state()
            if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            robot.reset_odometry()
            robot.wait_for_pose_update(timeout=0.5)
            robot.set_led(LED.ORANGE, 200)
            print("[INIT] Ready — press BTN_1 to start path, BTN_2 to cancel")
            print(f"       GPS tag: {'any' if GPS_TAG_ID == -1 else GPS_TAG_ID}")
            print(f"       Path: {len(PATH)} waypoints, side={_S:.0f} mm")
            state = "IDLE"

        # ── IDLE ──────────────────────────────────────────────────────────────
        elif state == "IDLE":
            if now - last_diag_t >= DIAG_INTERVAL_S:
                print_diagnostics(robot, "IDLE")
                last_diag_t = now

            if robot.was_button_pressed(Button.BTN_1):
                robot.set_led(LED.GREEN, 200)
                robot.set_led(LED.ORANGE, 0)
                print(f"[IDLE→MOVING] Starting {len(PATH)}-waypoint path")
                drive_handle = robot.purepursuit_follow_path(
                    waypoints=PATH,
                    velocity=VELOCITY,
                    lookahead=LOOKAHEAD,
                    tolerance=TOLERANCE,
                    advance_radius=ADVANCE_RADIUS,
                    max_angular_rad_s=MAX_ANGULAR_RAD_S,
                    blocking=False,
                )
                state = "MOVING"

        # ── MOVING ────────────────────────────────────────────────────────────
        elif state == "MOVING":
            if now - last_diag_t >= DIAG_INTERVAL_S:
                print_diagnostics(robot, "MOVING")
                last_diag_t = now

            if robot.get_button(Button.BTN_2):
                drive_handle.cancel()
                drive_handle.wait(timeout=1.0)
                drive_handle = None
                robot.stop()
                robot.set_led(LED.ORANGE, 200)
                robot.set_led(LED.GREEN, 0)
                print("[MOVING→IDLE] Cancelled — press BTN_1 to restart")
                state = "IDLE"

            elif drive_handle is not None and drive_handle.is_finished():
                drive_handle = None
                robot.stop()
                robot.set_led(LED.ORANGE, 200)
                robot.set_led(LED.GREEN, 0)
                print_diagnostics(robot, "DONE")
                print("[MOVING→IDLE] Path complete — press BTN_1 to run again")
                state = "IDLE"

        # ── Tick-rate control (do not modify) ─────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
