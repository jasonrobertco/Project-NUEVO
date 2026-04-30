"""
obstacle_avoidance.py — DWA-based obstacle avoidance example
=============================================================
Restored from commit 8894254 (added obstacle avoidance).

Known issues (do not fix here — see comments):
  - robot._nav_follow_dwa_path() passes mm values to a planner that expects SI units.
  - robot._nav_follow_path_loop() passes an extra `period` argument that DWAPlanner
    does not accept; will raise TypeError at runtime.
  - robot._draw_lidar_obstacles() requires matplotlib (not imported in robot.py)
    and treats self._obstacles_mm as a numpy array when it is a list.
"""

from __future__ import annotations
import time

from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.util import densify_polyline
from robot.path_planner import PurePursuitPlanner
import math
import numpy as np


# ---------------------------------------------------------------------------
# Robot build configuration
# ---------------------------------------------------------------------------

POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True


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


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.ORANGE, 255)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 255)


def start_robot(robot: Robot) -> None:
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    drive_handle = None
    period = 1.0 / float(DEFAULT_FSM_HZ)
    print(f"FSM period: {period:.3f} seconds")
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            print("[FSM] INIT (odometry reset)")
            path_control_points = [
                (0.0,   0.0),
                (0.0, 2500.0),
                (700.0, 2500.0),
            ]
            path = np.float64(densify_polyline(path_control_points, spacing=500.0))
            robot._nav_follow_dwa_path(
                max_vel_mm=200.0,
                max_acc_mm=300.0,
                max_angular_rad=1.5,
                max_angular_acc_rad=2.0,
                lookahead_mm=200.0,
                advance_radius_mm=150.0,
                tolerance_mm=100.0,
                gains_of_costs=[2.0, 0.02, 1.0, 0.3, 0.1], # [gain_goal, gain_heading, gain_obs_base, gain_speed, gain_path]
                period=period,
                predict_time=3.0,
                predict_velocity_samples_resolution=[10.0, 0.1],
                obstacles_range_mm=1000.0,
                ttc_weight=0.1,
            )
            print("Path is ready, Entering IDLE state.")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            robot._draw_lidar_obstacles()
            print("[FSM] IDLE - Press BTN_1 to enter MOVING state.")
            if robot.get_button(Button.BTN_1):
                print("Start Moving!")
                print("[FSM] MOVING")
                state = "MOVING"

        elif state == "MOVING":
            show_moving_leds(robot)
            # if next_tick % 0.5 < period: # print every half second
            #     robot._draw_lidar_obstacles()
            #     print("Obstacle figure updated.")
            state = robot._nav_follow_path_loop(path, period)

        # FSM refresh rate control
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
