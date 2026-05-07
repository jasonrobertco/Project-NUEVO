"""
lidar_obstacle_test.py — incremental step test with obstacle detection and lidar diagnostics
=============================================================================================
Purpose
-------
Move the robot forward in fixed increments.  Before each step, check whether
a lidar obstacle is present in the forward cone.  If one is detected, stop and
stay stopped.  Every tick writes a log row so the session can be replayed for
analysis.

HOW TO RUN
----------
    cp examples/lidar_obstacle_test.py main.py
    ros2 run robot robot

Log file: /tmp/lidar_obstacle_test.csv
    Columns: wall_time, scan_age_ms, n_raw_pts, n_fwd_pts, min_fwd_dist_mm,
             obstacle_detected, pose_x, pose_y, theta_deg, fsm_state, note

CURRENT QoS SETUP
-----------------
robot.py subscribes to /scan with BEST_EFFORT + depth=1 so obstacle processing
always sees the newest lidar frame instead of a backlog:

    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    _LIDAR_QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    node.create_subscription(LaserScan, '/scan', self._on_lidar, _LIDAR_QOS)

RELIABLE publisher (rplidar) + BEST_EFFORT subscriber is a valid QoS pairing in
ROS2. With depth=1 the DDS layer keeps only the single most-recent scan, so the
callback sees fresh data and avoids stale-scan queueing.

This test script installs its own /scan subscription with depth=1 alongside the
one inside Robot so the log can show the scan age directly.  Compare
`scan_age_ms` in the log: values consistently > one scan period (~100 ms at
10 Hz) confirm buffer lag.

OBSTACLE CHECK GEOMETRY
-----------------------
`robot._obstacles_mm` is in the raw lidar frame.  The lidar is mounted 180°
rotated relative to the robot forward axis, so:

    raw lidar +x  →  robot backward
    raw lidar -x  →  robot forward

After applying the 180° correction (same rotation used by the current
obstacle-avoidance planner):
    x_fwd = -x_raw,  y_fwd = -y_raw

Obstacles with x_fwd > 0 are in front of the robot.  This test filters to a
forward cone of ±FORWARD_CONE_DEG and reports the minimum distance.
"""

from __future__ import annotations

import csv
import math
import os
import time
import threading
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.robot import FirmwareState, Robot, Unit


# ---------------------------------------------------------------------------
# Robot hardware configuration
# ---------------------------------------------------------------------------

POSITION_UNIT          = Unit.MM
WHEEL_DIAMETER         = 74.0    # mm
WHEEL_BASE             = 333.0   # mm
INITIAL_THETA_DEG      = 90.0

LEFT_WHEEL_MOTOR       = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED  = False
RIGHT_WHEEL_MOTOR      = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True


# ---------------------------------------------------------------------------
# Test parameters
# ---------------------------------------------------------------------------

STEP_DISTANCE_MM   = 150.0   # how far to drive per increment
STEP_VELOCITY_MM_S = 80.0    # forward speed during each step
STEP_TOLERANCE_MM  = 20.0    # arrival window for each step

STOP_DISTANCE_MM   = 300.0   # stop if any obstacle is closer than this (in robot forward frame)
FORWARD_CONE_DEG   = 45.0    # half-angle of forward detection cone

LOG_FILE = "/tmp/lidar_obstacle_test.csv"
LOG_COLUMNS = [
    "wall_time_s",
    "scan_age_ms",      # ms since the lidar scan was stamped — high value means buffer lag
    "n_raw_pts",        # total valid lidar points this tick
    "n_fwd_pts",        # points inside the forward cone after rotation correction
    "min_fwd_dist_mm",  # closest obstacle in the forward cone (inf = none)
    "obstacle_detected",
    "pose_x_mm",
    "pose_y_mm",
    "theta_deg",
    "fsm_state",
    "note",
]


# ---------------------------------------------------------------------------
# Diagnostic /scan subscriber
# ---------------------------------------------------------------------------

class LidarDiagnostics:
    """
    A separate /scan subscriber installed with depth=1 + BEST_EFFORT.

    This runs alongside robot._on_lidar (which uses depth=10 RELIABLE) so we
    can compare their timestamps and confirm whether the Robot subscription is
    delivering stale data.

    The last_stamp_s attribute holds the ROS header stamp of the most-recently
    received scan as a float (seconds).  Compare to time.time() to get lag.
    """

    def __init__(self, node) -> None:
        self.last_stamp_s: float = 0.0
        self._lock = threading.Lock()

        # depth=1 + BEST_EFFORT: always the freshest scan, no queuing.
        # This is what the /scan subscription in robot.py SHOULD use.
        _qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        node.create_subscription(LaserScan, '/scan', self._cb, _qos)

    def _cb(self, msg: LaserScan) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self.last_stamp_s = stamp

    def age_ms(self) -> float:
        """Wall-clock age of the last received scan in milliseconds."""
        with self._lock:
            stamp = self.last_stamp_s
        if stamp == 0.0:
            return float('inf')
        return (time.time() - stamp) * 1000.0


# ---------------------------------------------------------------------------
# Obstacle detection helper
# ---------------------------------------------------------------------------

_ROT_180 = np.array([[-1.0, 0.0], [0.0, -1.0]])  # 180° rotation matrix


def check_forward_obstacle(robot: Robot, stop_dist_mm: float, cone_deg: float):
    """
    Return (obstacle_detected, min_fwd_dist_mm, n_raw, n_fwd).

    Reads robot._obstacles_mm under the lock, applies the 180° lidar mount
    correction (same transform used by the current obstacle-avoidance planner),
    then checks the forward
    cone.  The robot frame after correction: +x = forward, +y = left.
    """
    with robot._lock:
        obs = robot._obstacles_mm.copy()  # shape (N, 2) or empty

    n_raw = len(obs)
    if n_raw == 0:
        return False, float('inf'), 0, 0

    # Apply 180° rotation to correct for reversed lidar mount.
    corrected = (obs @ _ROT_180.T)   # shape (N, 2), now +x is robot forward

    # Filter to forward cone: x > 0 and |atan2(y,x)| < cone_deg
    cone_rad = math.radians(cone_deg)
    fwd_mask = (corrected[:, 0] > 0) & (
        np.abs(np.arctan2(corrected[:, 1], corrected[:, 0])) < cone_rad
    )
    fwd_pts = corrected[fwd_mask]
    n_fwd = len(fwd_pts)

    if n_fwd == 0:
        return False, float('inf'), n_raw, 0

    # Distance from robot origin (0,0) to each forward point.
    dists = np.linalg.norm(fwd_pts, axis=1)
    min_dist = float(np.min(dists))
    detected = min_dist < stop_dist_mm
    return detected, min_dist, n_raw, n_fwd


# ---------------------------------------------------------------------------
# Setup helpers
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


def start_robot(robot: Robot) -> None:
    if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.5)


# ---------------------------------------------------------------------------
# run() — entry point
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    # Install diagnostic lidar subscriber on the same ROS node.
    diag = LidarDiagnostics(robot._node)

    # Open log file.
    log_path = Path(LOG_FILE)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = open(log_path, "w", newline="")
    writer = csv.DictWriter(log_file, fieldnames=LOG_COLUMNS)
    writer.writeheader()
    log_file.flush()
    print(f"[TEST] Logging to {log_path}")

    def log(fsm_state: str, note: str = "") -> None:
        x, y, theta = robot.get_pose()
        detected, min_fwd, n_raw, n_fwd = check_forward_obstacle(
            robot, STOP_DISTANCE_MM, FORWARD_CONE_DEG
        )
        writer.writerow({
            "wall_time_s":      f"{time.time():.3f}",
            "scan_age_ms":      f"{diag.age_ms():.1f}",
            "n_raw_pts":        n_raw,
            "n_fwd_pts":        n_fwd,
            "min_fwd_dist_mm":  f"{min_fwd:.1f}" if math.isfinite(min_fwd) else "inf",
            "obstacle_detected": int(detected),
            "pose_x_mm":        f"{x:.1f}",
            "pose_y_mm":        f"{y:.1f}",
            "theta_deg":        f"{theta:.1f}",
            "fsm_state":        fsm_state,
            "note":             note,
        })
        log_file.flush()

    # ── FSM ───────────────────────────────────────────────────────────────────

    state          = "INIT"
    step_origin_x  = 0.0   # x at the start of the current step
    step_origin_y  = 0.0
    step_count     = 0
    blocked_reason = ""

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    print("[TEST] Waiting for BTN_1 to start incremental step test.")
    print(f"       Step size: {STEP_DISTANCE_MM} mm | Stop distance: {STOP_DISTANCE_MM} mm | "
          f"Forward cone: ±{FORWARD_CONE_DEG}°")

    try:
        while True:

            # ── INIT ──────────────────────────────────────────────────────────
            if state == "INIT":
                start_robot(robot)
                robot.set_led(LED.ORANGE, 200)
                robot.set_led(LED.GREEN, 0)
                log("INIT", "startup")
                print("[FSM] IDLE — press BTN_1 to begin, BTN_2 to abort at any time")
                state = "IDLE"

            # ── IDLE ──────────────────────────────────────────────────────────
            elif state == "IDLE":
                if robot.was_button_pressed(Button.BTN_1):
                    x, y, _ = robot.get_pose()
                    step_origin_x, step_origin_y = x, y
                    step_count = 0
                    robot.set_led(LED.ORANGE, 0)
                    robot.set_led(LED.GREEN, 200)
                    log("IDLE", "BTN_1 pressed - starting")
                    print(f"[FSM] STEP_CHECK (step {step_count})")
                    state = "STEP_CHECK"

            # ── STEP_CHECK — scan before moving ───────────────────────────────
            elif state == "STEP_CHECK":
                detected, min_fwd, n_raw, n_fwd = check_forward_obstacle(
                    robot, STOP_DISTANCE_MM, FORWARD_CONE_DEG
                )
                scan_age = diag.age_ms()
                print(
                    f"[STEP_CHECK] step={step_count}  "
                    f"scan_age={scan_age:.0f} ms  "
                    f"raw_pts={n_raw}  fwd_pts={n_fwd}  "
                    f"min_fwd={min_fwd:.0f} mm  obstacle={detected}"
                )
                note = (
                    f"pre-step check | scan_age={scan_age:.0f}ms"
                    + (" | STALE SCAN" if scan_age > 200 else "")
                )
                log("STEP_CHECK", note)

                if detected:
                    robot.stop()
                    robot.set_led(LED.GREEN, 0)
                    robot.set_led(LED.RED if hasattr(LED, 'RED') else LED.ORANGE, 255)
                    blocked_reason = f"obstacle at {min_fwd:.0f} mm before step {step_count}"
                    print(f"[FSM] BLOCKED — {blocked_reason}")
                    state = "BLOCKED"
                else:
                    x, y, _ = robot.get_pose()
                    step_origin_x, step_origin_y = x, y
                    print(f"[FSM] STEPPING — step {step_count}, "
                          f"origin ({step_origin_x:.0f}, {step_origin_y:.0f})")
                    robot.set_velocity(STEP_VELOCITY_MM_S, 0.0)
                    state = "STEPPING"

            # ── STEPPING — drive until step distance reached ───────────────────
            elif state == "STEPPING":
                x, y, _ = robot.get_pose()
                dist_travelled = math.hypot(x - step_origin_x, y - step_origin_y)

                # Mid-step obstacle check every tick.
                detected, min_fwd, n_raw, n_fwd = check_forward_obstacle(
                    robot, STOP_DISTANCE_MM, FORWARD_CONE_DEG
                )
                scan_age = diag.age_ms()
                log("STEPPING", f"dist={dist_travelled:.0f}mm scan_age={scan_age:.0f}ms")

                if detected:
                    robot.stop()
                    robot.set_led(LED.GREEN, 0)
                    blocked_reason = (
                        f"obstacle at {min_fwd:.0f} mm during step {step_count} "
                        f"(after {dist_travelled:.0f} mm)"
                    )
                    print(f"[FSM] BLOCKED mid-step — {blocked_reason}")
                    state = "BLOCKED"

                elif dist_travelled >= STEP_DISTANCE_MM - STEP_TOLERANCE_MM:
                    robot.stop()
                    step_count += 1
                    print(f"[FSM] STEP_PAUSE — completed step {step_count - 1}")
                    state = "STEP_PAUSE"

            # ── STEP_PAUSE — brief stop between steps ─────────────────────────
            elif state == "STEP_PAUSE":
                # Wait one full tick (lidar delivers ~10 Hz, give it 3 cycles).
                # Log during pause so we can see scan age settle.
                _, min_fwd, n_raw, n_fwd = check_forward_obstacle(
                    robot, STOP_DISTANCE_MM, FORWARD_CONE_DEG
                )
                log("STEP_PAUSE", f"inter-step pause | step={step_count}")
                # After 3 ticks the FSM will have visited here 3 times; use a
                # simple counter rather than a separate timer.
                if not hasattr(robot, '_pause_ticks'):
                    robot._pause_ticks = 0
                robot._pause_ticks += 1
                if robot._pause_ticks >= 3:
                    robot._pause_ticks = 0
                    print(f"[FSM] STEP_CHECK (step {step_count})")
                    state = "STEP_CHECK"

            # ── BLOCKED — stopped due to obstacle ─────────────────────────────
            elif state == "BLOCKED":
                _, min_fwd, n_raw, n_fwd = check_forward_obstacle(
                    robot, STOP_DISTANCE_MM, FORWARD_CONE_DEG
                )
                scan_age = diag.age_ms()
                log("BLOCKED", f"{blocked_reason} | scan_age={scan_age:.0f}ms")
                print(
                    f"[BLOCKED] fwd_pts={n_fwd}  min_fwd={min_fwd:.0f} mm  "
                    f"scan_age={scan_age:.0f} ms  — press BTN_1 to retry, BTN_2 to finish"
                )
                if robot.was_button_pressed(Button.BTN_1):
                    print("[FSM] STEP_CHECK (retry)")
                    log("BLOCKED", "BTN_1 retry")
                    state = "STEP_CHECK"
                elif robot.was_button_pressed(Button.BTN_2):
                    print("[FSM] DONE — BTN_2 pressed")
                    state = "DONE"

            # ── DONE ──────────────────────────────────────────────────────────
            elif state == "DONE":
                log("DONE", f"finished after {step_count} steps")
                robot.stop()
                print(f"[TEST] Finished.  {step_count} steps completed.  Log: {log_path}")
                break

            # BTN_2 aborts from anywhere except DONE/BLOCKED.
            if state not in ("DONE", "BLOCKED", "IDLE", "INIT"):
                if robot.was_button_pressed(Button.BTN_2):
                    robot.stop()
                    log(state, "BTN_2 abort")
                    print("[FSM] DONE — aborted by BTN_2")
                    state = "DONE"

            # ── Tick-rate control ─────────────────────────────────────────────
            next_tick += period
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()

    finally:
        robot.stop()
        log_file.close()
        print(f"[TEST] Log closed: {log_path}")
