"""
main.py - checkpoint 2 bridge mission
=====================================
Press BTN_1 to start the route. BTN_2 cancels the active motion.

This version intentionally does not use LiDAR, LAPF, or obstacle avoidance.
The robot follows a short odometry-only sequence:

1. drive to the checkpoint 2 approach point
2. turn right 90 degrees
3. drive forward a short amount
4. turn right 90 degrees
5. drive forward across the bridge/ramp
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal
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


# Preserved from working28/main28backup behavior.
LEFT_WHEEL_MOTOR = 2
LEFT_WHEEL_DIR_INVERTED = True
RIGHT_WHEEL_MOTOR = 1
RIGHT_WHEEL_DIR_INVERTED = False

DRIVE_VELOCITY_MM_S = 140.0
DRIVE_TOLERANCE_MM = 60.0
TURN_TOLERANCE_DEG = 3.0
STATUS_PRINT_INTERVAL_S = 0.5

CHECKPOINT_1_APPROACH_DISTANCE_MM = 3350.0
BRIDGE_ALIGN_DISTANCE_MM = 150.0
BRIDGE_CROSS_DISTANCE_MM = 2200.0
TILE_MM = 300.0

StepKind = Literal["move_to", "turn_by", "move_forward"]


@dataclass(frozen=True)
class MissionStep:
    label: str
    kind: StepKind
    value: tuple[float, float] | float


MISSION_STEPS: tuple[MissionStep, ...] = (
    MissionStep("drive to checkpoint 1 approach", "move_forward", CHECKPOINT_1_APPROACH_DISTANCE_MM),
    MissionStep("turn right toward bridge lane", "turn_by", -90.0),
    MissionStep("drive into bridge lane", "move_forward", BRIDGE_ALIGN_DISTANCE_MM),
    MissionStep("turn right to face bridge", "turn_by", -90.0),

    # Checkpoint 1 reached here.
    MissionStep("cross bridge", "move_forward", BRIDGE_CROSS_DISTANCE_MM),
    MissionStep("turn left after bridge", "turn_by", 90.0),
    MissionStep("drive 1.5 tiles toward obstacle section", "move_forward", TILE_MM * 1.5),
    MissionStep("turn left toward obstacle course", "turn_by", 90.0),

    # Checkpoint 2 reached here. Robot should be facing the cones/obstacle course.
    # TODO: Start obstacle avoidance algorithm here.
    # Future implementation should switch from odometry-only scripted motion
    # to LiDAR/LAPF-based obstacle navigation for the obstacle course section.
)


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
            "Odometry parameter confirmation failed; refusing to start mission. "
            f"Expected left={LEFT_WHEEL_MOTOR} inverted={LEFT_WHEEL_DIR_INVERTED}, "
            f"right={RIGHT_WHEEL_MOTOR} inverted={RIGHT_WHEEL_DIR_INVERTED}."
        )


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


def start_step(robot: Robot, step: MissionStep):
    if step.kind == "move_to":
        x_mm, y_mm = step.value
        return robot.move_to(
            x_mm,
            y_mm,
            velocity=DRIVE_VELOCITY_MM_S,
            tolerance=DRIVE_TOLERANCE_MM,
            blocking=False,
        )

    if step.kind == "move_forward":
        return robot.move_forward(
            step.value,
            velocity=DRIVE_VELOCITY_MM_S,
            tolerance=DRIVE_TOLERANCE_MM,
            blocking=False,
        )

    if step.kind == "turn_by":
        return robot.turn_by(
            step.value,
            tolerance_deg=TURN_TOLERANCE_DEG,
            blocking=False,
        )

    raise ValueError(f"Unknown mission step kind: {step.kind}")


def print_status(robot: Robot, step_index: int) -> None:
    x, y, theta = robot.get_odometry_pose()
    step = MISSION_STEPS[step_index]
    print(
        f"  step={step_index + 1}/{len(MISSION_STEPS)} "
        f"{step.label} "
        f"odom=({x:6.0f}, {y:6.0f}) mm "
        f"theta={theta:5.1f} deg"
    )


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    motion_handle = None
    step_index = 0
    last_status_print_at = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        if state == "INIT":
            start_robot(robot)
            reset_mission_pose(robot)
            show_idle_leds(robot)
            step_index = 0
            print("[FSM] IDLE - press BTN_1 to start checkpoint 2 bridge mission, BTN_2 to cancel")
            print(f"[CFG] steps={MISSION_STEPS}")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_running_leds(robot)
                step_index = 0
                motion_handle = start_step(robot, MISSION_STEPS[step_index])
                last_status_print_at = now
                print(f"[FSM] MOVING - started step 1/{len(MISSION_STEPS)}: {MISSION_STEPS[0].label}")
                state = "MOVING"

        elif state == "MOVING":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE - mission cancelled")
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot, step_index)
                    last_status_print_at = now

                if motion_handle is not None and motion_handle.is_finished():
                    step_index += 1
                    if step_index >= len(MISSION_STEPS):
                        print("[FSM] DONE - bridge crossing complete")
                        print_status(robot, len(MISSION_STEPS) - 1)
                        robot.stop()
                        motion_handle = None
                        show_idle_leds(robot)
                        print("[FSM] IDLE - press BTN_1 to run again")
                        state = "IDLE"
                    else:
                        motion_handle = start_step(robot, MISSION_STEPS[step_index])
                        print(
                            f"[FSM] MOVING - started step {step_index + 1}/{len(MISSION_STEPS)}: "
                            f"{MISSION_STEPS[step_index].label}"
                        )

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
