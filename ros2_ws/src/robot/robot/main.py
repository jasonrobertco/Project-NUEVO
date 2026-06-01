"""
main.py - checkpoint 2 bridge mission
=====================================
Press BTN_1 to start the route. BTN_2 cancels the active motion.

The robot follows a short odometry-only sequence to checkpoint 2, then switches
to LiDAR/LAPF obstacle avoidance for the later checkpoint sections.

1. drive forward to the checkpoint 1 approach point
2. turn right 90 degrees
3. drive forward a short amount
4. turn right 90 degrees
5. drive forward across the bridge/ramp
6. turn left 90 degrees
7. drive past the bridge exit toward the obstacle section
8. turn left 90 degrees toward the obstacle course
9. obstacle-avoid up the field (LAPF, watchdog + recovery), ending when the
   outer course wall is detected within standoff ahead
10. square up to the outer wall and approach it to a fixed standoff (re-zero
    against an absolute reference instead of a drifted odometry coordinate)
11. turn onto the checkpoint-3 finish straightaway
12. verify the straightaway is clear, then drive it -> checkpoint 3 reached
13. obstacle-avoid straight 5 course tiles to checkpoint 4, then stop (LAPF)
14. checkpoint 5 manipulator placeholder (scaffold only)

LAPF watchdog/recovery (steps 9 & 13):
    A LAPF segment can stall in an APF local minimum (attraction ~= repulsion)
    or get boxed in; the planner loop only ends on goal-or-cancel, so without a
    watchdog the FSM would hang forever. Each segment is monitored by a
    no-progress detector (authoritative) and a per-attempt wall-clock cap
    (last resort). On a stall the FSM runs a bounded, symmetry-breaking,
    rear-guarded back-up-and-retry; after the retry budget it safe-stops to
    IDLE. BTN_2 cleanly cancels at every point, including mid-recovery.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal
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

# Commanded magnitude for a physical 90-degree turn. The robot overshoots a
# raw 90.0 command by ~10 deg, so the calibrated command is reduced to land on
# a true right angle. Tune this on the venue: if the robot still turns too far,
# lower it; if it falls short, raise it. All scripted right-angle turns
# (cp1/2 sequence and the cp3 terminal turns) reference this so there is one
# knob to adjust.
RIGHT_ANGLE_TURN_DEG = 82.0

# Checkpoint 1/2 scripted distances. These are EMPIRICALLY HAND-TUNED on the
# physical bridge/ramp, not derived from the course grid. Do not change them
# without re-tuning on the venue — the cp1/2 sequence is meant to stay
# behaviorally fixed.
CHECKPOINT_1_APPROACH_DISTANCE_MM = 2600.0   # start -> checkpoint 1 approach point
BRIDGE_ALIGN_DISTANCE_MM = 500.0             # short nudge into the bridge lane
BRIDGE_CROSS_DISTANCE_MM = 2100.0            # length of the bridge/ramp crossing
# Post-bridge exit hop toward the obstacle section. This is NOT a course tile:
# it is a hand-tuned 450 mm exit distance, kept intentionally independent of the
# 610 mm course grid (COURSE_TILE_MM). Previously written as TILE_MM * 1.5 with
# TILE_MM = 300, which read like "1.5 tiles" but was always 450 mm; renamed to a
# literal so it can't be confused with the course grid.
BRIDGE_EXIT_DISTANCE_MM = 600.0

# --- Bridge-entry wall-square correction -----------------------------------
# The bridge is crossed open-loop on odometry over a long distance, so any
# heading error left by the two preceding 90-degree turns becomes a large
# lateral deviation (deviation ~= BRIDGE_CROSS_DISTANCE_MM * sin(theta_err)).
# Right after the 2nd turn the LiDAR sees a wall to the LEFT (parallel to the
# crossing) and a wall BEHIND (perpendicular). We fit a line to each and turn
# to null the heading error before committing to the crossing. This corrects
# HEADING only, not lateral offset.
#
# FIELD TUNING REQUIRED. The two wall standoffs below MUST be measured at the
# bridge mouth on the venue, and the correction SIGN is UNVERIFIED (same
# caveat as the scripted turns) — bench-test the direction before trusting it
# on the bridge. The correction is deliberately conservative: it does NOTHING
# (no turn) whenever the scan is ambiguous, so a noisy read cannot make the
# crossing worse.
BRIDGE_SQUARE_ENABLED = True
BRIDGE_LEFT_WALL_DIST_MM = 200.0          # expected perpendicular distance to the left wall
BRIDGE_REAR_WALL_DIST_MM = 500.0          # expected distance to the wall behind
BRIDGE_WALL_BAND_MM = 250.0               # keep points within +/- this of the expected standoff
BRIDGE_WALL_SAMPLE_DEPTH_MM = 800.0       # along-wall window of points to fit
BRIDGE_SQUARE_MIN_POINTS = 8              # minimum inliers to trust a line fit
BRIDGE_SQUARE_DEADBAND_DEG = 1.5          # below this error, don't bother turning
BRIDGE_SQUARE_MAX_CORRECTION_DEG = 15.0   # refuse larger corrections (guards a bad fit)
BRIDGE_SQUARE_CROSSCHECK_DEG = 6.0        # left vs rear estimates must agree within this

# Checkpoint 2 and later obstacle-avoidance course sections.
# The UI/course grid uses 610 mm cells (see WorldCanvas.tsx / vm_demo.py).
COURSE_TILE_MM = 610.0
CHECKPOINT_3_APPROACH_TILES = 5.0
CHECKPOINT_3_FINAL_STRAIGHT_TILES = 1.0
CHECKPOINT_4_STRAIGHT_TILES = 5.0

CHECKPOINT_3_APPROACH_DISTANCE_MM = COURSE_TILE_MM * CHECKPOINT_3_APPROACH_TILES
CHECKPOINT_3_FINAL_STRAIGHT_DISTANCE_MM = COURSE_TILE_MM * CHECKPOINT_3_FINAL_STRAIGHT_TILES
CHECKPOINT_4_DISTANCE_MM = COURSE_TILE_MM * CHECKPOINT_4_STRAIGHT_TILES

# LAPF tuning for the checkpoint 2+ obstacle-avoidance runs:
# - OBSTACLE_AVOIDANCE_SPEED_MM_S: lower if the robot reacts too late or slips.
# - OBSTACLE_AVOIDANCE_TOLERANCE_MM: larger accepts a looser checkpoint arrival.
# - LAPF_LEASH_LENGTH_MM: how far the virtual target can run ahead of the robot.
#   Smaller is more cautious; larger is smoother but can cut closer to cones.
# - LAPF_REPULSION_RANGE_MM: how early tracked obstacles affect the virtual target.
#   Increase if avoidance starts too late; decrease if it swerves too early.
# - LAPF_TARGET_SPEED_MM_S: virtual-target motion speed. Lower damps aggressive
#   target motion; higher lets the virtual target move around obstacles faster.
# - LAPF_REPULSION_GAIN: obstacle push strength. Increase if it clips obstacles;
#   decrease if it overreacts or oscillates.
# - LAPF_FORCE_EMA_ALPHA: force smoothing. Lower is smoother/slower to react;
#   higher follows the newest LiDAR obstacle estimate more aggressively.
# - LAPF_INFLATION_MARGIN_MM: extra radius added around each tracked obstacle.
#   Increase this for more clearance around cones.
# - LAPF_LEASH_HALF_ANGLE_DEG: forward cone for the virtual target. Smaller keeps
#   the robot driving straighter; larger allows wider avoidance maneuvers.
OBSTACLE_AVOIDANCE_SPEED_MM_S = 140.0
OBSTACLE_AVOIDANCE_TOLERANCE_MM = 60.0
OBSTACLE_AVOIDANCE_MAX_ANGULAR_RAD_S = 1.0
LAPF_LEASH_LENGTH_MM = 400.0
LAPF_REPULSION_RANGE_MM = 300.0
LAPF_TARGET_SPEED_MM_S = 200.0
LAPF_REPULSION_GAIN = 550.0
LAPF_ATTRACTION_GAIN = 1.0
LAPF_FORCE_EMA_ALPHA = 0.35
LAPF_INFLATION_MARGIN_MM = 150.0
LAPF_LEASH_HALF_ANGLE_DEG = 25.0

# ---------------------------------------------------------------------------
# LAPF stall watchdog + recovery (checkpoint 2+ obstacle-avoidance segments).
#
# Detection (per active LAPF attempt):
#   - No-progress (AUTHORITATIVE): if the best distance-to-goal has not improved
#     by >= NO_PROGRESS_EPS_MM within NO_PROGRESS_WINDOW_S, the robot is stuck.
#     -> recover + retry.
#   - Per-attempt wall-clock (LAST RESORT): catches a "slow orbit" that keeps
#     netting >EPS progress per window but never converges. Scales with the
#     remaining distance so retries (shorter remaining distance) get a tighter
#     cap. -> safe-stop, NOT retried (retrying an orbit doesn't help).
#   - Segment hard ceiling: a global backstop over the whole segment including
#     all recovery actions. -> safe-stop.
#
# Recovery (bounded, symmetry-breaking, rear-guarded):
#   Backing up and re-issuing the IDENTICAL goal would walk back into the same
#   symmetric local minimum. So each retry also (a) reverses to add clearance
#   (guarded — see below), (b) reorients the heading by an alternating offset,
#   and (c) re-issues toward a goal shifted laterally to the alternating side.
#   Perturbing heading + goal breaks the goal/obstacle collinearity that creates
#   the force null, so the attempt cannot reproduce the identical stall. The
#   lateral side alternates per retry so the second attempt tries the other side
#   if the first stayed blocked.
#
#   Blind reverse is unmodeled by the forward-facing planner, so before any
#   reverse we check the rear LiDAR sector (confirmed tracks) for clearance and
#   hard-cap the reverse distance. If the rear isn't clear we skip the reverse
#   and do reorientation only (or safe-stop).
NO_PROGRESS_WINDOW_S = 4.0
NO_PROGRESS_EPS_MM = 50.0
WALLCLOCK_CAP_FACTOR = 1.5
WALLCLOCK_CAP_FLOOR_S = 12.0
SEGMENT_HARD_CEILING_S = 35.0
MAX_AVOIDANCE_RETRIES = 2

RECOVERY_REVERSE_MM = 300.0              # nominal back-up distance
RECOVERY_MIN_REVERSE_MM = 50.0           # below this, skip the reverse entirely
RECOVERY_REVERSE_SPEED_MM_S = 100.0      # slow, so BTN_2 polling stays responsive
RECOVERY_REVERSE_TOLERANCE_MM = 40.0
RECOVERY_REVERSE_MARGIN_MM = 150.0       # keep this clearance behind the robot
RECOVERY_MIN_REAR_CLEARANCE_MM = 250.0   # min rear clearance to allow reversing
RECOVERY_REVERSE_TIMEOUT_S = 3.0
RECOVERY_HEADING_OFFSET_DEG = 25.0       # ~leash half-angle, to reaim the cone
RECOVERY_REORIENT_TIMEOUT_S = 2.0
RECOVERY_LATERAL_OFFSET_MM = 275.0       # ~inflation_margin + max disk radius
REAR_SECTOR_HALF_ANGLE_DEG = 60.0        # +/- around directly-behind

# Allow a little slack over the LAPF goal tolerance when confirming that a
# finished LAPF handle actually reached the goal (vs the motion thread ending
# early). Lets us tell success from failure on completion.
SUCCESS_MARGIN_MM = 20.0

# ---------------------------------------------------------------------------
# Checkpoint 3 wall-referenced terminal maneuver (Fix #2). Replaces the old
# blind turn / drive-1-tile / turn hop onto a drift-corrupted goal coordinate.
# The cp2->cp3 LAPF run now drives up the field until the outer course wall is
# within WALL_DETECT_STANDOFF_MM ahead, squares up to that wall and creeps to a
# fixed standoff (re-zeroing against an absolute physical reference), then turns
# onto the checkpoint-3 finish straightaway and only drives it once the front
# LiDAR confirms the path is clear. If it can't confirm, it safe-stops to IDLE
# rather than driving blind.
#
# All distances/turns below need FIELD TUNING and several depend on the
# self-footprint filter (Fix #1) keeping the forward cone phantom-free — verify
# the `nearest obstacles` dump is clean before trusting front_clearance_mm().
#
# IMPORTANT: the two turn SIGNS are UNVERIFIED. turn_by(+deg) = right,
# turn_by(-deg) = left in this codebase, but which way squares up to the wall
# and which way faces the finish straightaway must be confirmed PHYSICALLY.
# Keep BTN_2 ready on the first run.
WALL_DETECT_STANDOFF_MM = 500.0        # end the LAPF run when the wall is this close ahead
WALL_ARM_REMAINING_MM = 1200.0         # only arm wall-detection within this much of the goal
                                       # coordinate, so a CONE in the lower field can't
                                       # prematurely trigger the terminal maneuver (~2 tiles)
# Minimum forward advance before the cp3 terminal turn may fire. Advance is the
# projection of (current pose - cp2 start) onto the approach heading, so lateral
# drift / weaving doesn't inflate it. ANDed with the wall/standoff trigger: even
# once the wall is detected, the turn is HELD until advance reaches this floor,
# and the approach keeps running until then.
#
# Starting value derivation: field reports show the turn currently fires ~300 mm
# (~half a 610 mm tile) too early. Wall-detection can arm as early as
# remaining <= WALL_ARM_REMAINING_MM (1200 mm), i.e. advance >= 3050 - 1200 =
# ~1850 mm along the 5-tile approach, which lines up with that early turn. Adding
# the ~300 mm correction -> ~2150 mm. NEEDS FIELD TUNING: the fire-time log below
# prints the exact measured advance, so dial this in from one run rather than
# guessing (set it to the logged fire advance + ~300, or wherever the turn should
# actually start).
CHECKPOINT_3_MIN_ADVANCE_MM = 2150.0
WALL_APPROACH_TARGET_MM = 250.0        # stop this far from the wall (the re-zero)
WALL_APPROACH_SPEED_MM_S = 80.0        # slow closed-loop approach speed
WALL_APPROACH_TIMEOUT_S = 8.0          # watchdog for the approach drive (not under LAPF watchdog)
CLEAR_PATH_MIN_MM = 1000.0             # straightaway is "clear" if nearest front return is beyond this
FRONT_CONE_HALF_WIDTH_MM = 150.0       # half-width of the forward cone used to read the wall
FRONT_CLEARANCE_MAX_RANGE_MM = 2000.0  # ignore returns beyond this when reading the wall
# Both turns: SIGN *and* MAGNITUDE need physical verification. If the wall the
# robot re-zeroes against is the same one detected straight ahead, CP3_FACE_WALL
# may be ~0 (already squared up); if the re-zero wall is the perpendicular
# finish-side wall, it's ~90. CP3_FACE_STRAIGHTAWAY then turns from facing the
# wall onto the finish straightaway. Set both on the venue.
CP3_FACE_WALL_TURN_DEG = RIGHT_ANGLE_TURN_DEG          # turn to face the re-zero wall (SIGN UNVERIFIED; magnitude calibrated)
CP3_FACE_STRAIGHTAWAY_TURN_DEG = RIGHT_ANGLE_TURN_DEG  # turn onto the finish straightaway (SIGN UNVERIFIED; magnitude calibrated)

# ---------------------------------------------------------------------------
# Checkpoint 3 deterministic 3-cone slalom (replaces the reactive-LAPF cp2->cp3
# approach when SLALOM_ENABLED). The cp3 section has exactly 3 cones across the
# lane (right/left/right); the robot weaves through the open gap on each
# (pass left/right/left), then hands off to the EXISTING wall-square -> finish
# maneuver once past cone 3. Cone positions come from live LiDAR (field entry
# varies >1 m run-to-run, so hardcoded waypoints don't survive).
#
# Method (per the approved plan): narrow the LiDAR to a forward lane window so
# the tracker isn't saturated by walls/bridge (Layer A), then in the FSM accept
# only cone-sized, isolated, persistent tracks in a lane window ahead (Layer B);
# sequentially identify cone i, pure-pursuit a gate offset to the OPEN side
# (auto-chosen from the cone's measured lateral sign, R/L/R pattern as fallback),
# mark it passed by along-axis progress, repeat, then exit to the wall maneuver.
# On any cone-ID failure (count != 3, none found, ambiguous) -> safe-stop, and
# the LiDAR filter is restored. ALL values below NEED FIELD TUNING; the first
# run prints a per-candidate dump (S/D/ahead/r) so they can be dialed in.
#
# DECISIONS BAKED IN (correct after the test): pure-pursuit gates; sequential
# per-cone detection; Layer-A narrowing with the slalom carrying its own ceiling
# (no LAPF reverse during slalom; full filter restored at handoff); safe-stop on
# failure; open side auto-detected with R/L/R fallback. Set SLALOM_ENABLED=False
# to fall back to the reactive LAPF approach + wall finish.
SLALOM_ENABLED = True
SLALOM_SPEED_MM_S = 120.0              # pursuit/creep speed during the slalom (slower than 140 cruise)
# Layer A — source-level lane window applied to the LiDAR during the slalom only,
# restored to the full configured filter at handoff / on any exit. Narrowing the
# FOV forward blinds the rear sector, so the slalom does NOT use LAPF reverse
# recovery (it has its own ceiling); the cp3->cp4 LAPF leg runs with full FOV.
CONE_FOV_HALF_DEG = 40.0               # forward FOV half-window (+/- deg) during slalom
CONE_RANGE_MAX_MM = 2500.0             # max LiDAR range during slalom (drop far walls/bridge)
# Layer B — cone identification filter (on confirmed tracks, in the approach-axis
# frame: S = forward from cp2 start, D = lateral, +left).
CONE_LANE_HALF_WIDTH_MM = 900.0        # |D| window. GENEROUS on purpose: lateral entry varies ~1 m
                                       # run-to-run, so a tight window would miss a cone. Tune DOWN
                                       # from the first-run dump if walls intrude.
CONE_DETECT_MAX_AHEAD_MM = 1500.0      # only look this far ahead for the next cone (~2.5 tiles)
CONE_MIN_RADIUS_MM = 20.0              # cone-size gate low (reject specks)
CONE_MAX_RADIUS_MM = 90.0              # cone-size gate high (just above the 75 mm tracker clamp)
CONE_ISOLATION_MIN_MM = 300.0          # a cone has NO other track within this (rejects wall segments)
CONE_CONFIRM_FRAMES = 3                # consecutive frames a candidate must persist before accepted
CONE_ASSOC_MM = 200.0                  # frame-to-frame match radius for the persistence counter
CONE_SIDE_MIN_D_MM = 80.0              # |D| below this is too central to read a side -> use pattern
CONE_PASS_SIDES = (1, -1, 1)           # fallback open side per cone: R/L/R -> pass L/R/L (+1 = left)
# Weave / gate geometry.
CONE_PASS_CLEARANCE_MM = 375.0         # lateral gate offset to the open side (cone r + half robot + margin)
CONE_PASS_CLEARANCE_TOL_MM = 75.0      # lateral slack: count as clear at (clearance - tol)
CONE_GATE_FORWARD_MM = 250.0           # push the gate this far PAST the cone. A cone is "passed" only
                                       # once the robot is BOTH past this forward point AND laterally
                                       # clear, so it swings AROUND the cone instead of cutting the
                                       # corner straight through it.
# Slalom guards / motion.
CONE_SCAN_MAX_ADVANCE_MM = 1100.0      # per-cone: fail if we advance this far in SCAN without a cone
# Per-cone gate timeout SCALES with the distance to cover (inter-cone gaps vary),
# like the LAPF watchdog's attempt_cap_s(). A fixed 8 s fired ~1 s too early on a
# wider gap (robot was moving steadily the whole time, just had ~1010 mm to go at
# 120 mm/s ~= 8.4 s). timeout = max(floor, factor * gate_distance / speed).
CONE_GATE_TIMEOUT_FACTOR = 2.0         # safety multiple over the nominal traverse time
CONE_GATE_TIMEOUT_FLOOR_S = 6.0        # never less than this, for short gaps
CONE_EXIT_ADVANCE_MM = 300.0           # settle distance past cone 3 before the wall maneuver
SLALOM_SCAN_LOOKAHEAD_MM = 1600.0      # straight-ahead pursuit target distance while scanning
# Whole-slalom anti-hang. Must cover ~3 m of weaving travel at SLALOM_SPEED plus
# scans/settles, so it is generous (35 s was too tight: ~3 m at 120 mm/s alone is
# ~25 s before weaving). Failures are caught by the per-cone guards first.
SLALOM_SEGMENT_CEILING_S = 60.0

StepKind = Literal["move_to", "turn_by", "move_forward", "square_to_wall"]


@dataclass(frozen=True)
class MissionStep:
    label: str
    kind: StepKind
    value: tuple[float, float] | float


MISSION_STEPS: tuple[MissionStep, ...] = (
    MissionStep("drive to checkpoint 1 approach", "move_forward", CHECKPOINT_1_APPROACH_DISTANCE_MM),
    MissionStep("turn right toward bridge lane", "turn_by", RIGHT_ANGLE_TURN_DEG),
    MissionStep("drive into bridge lane", "move_forward", BRIDGE_ALIGN_DISTANCE_MM),
    MissionStep("turn right to face bridge", "turn_by", RIGHT_ANGLE_TURN_DEG),

    # Checkpoint 1 reached here.
    # Re-reference heading against the bridge-mouth walls before the long
    # open-loop crossing, so turn error doesn't drift us off the ramp.
    MissionStep("square up to bridge axis", "square_to_wall", 0.0),
    MissionStep("cross bridge", "move_forward", BRIDGE_CROSS_DISTANCE_MM),
    MissionStep("turn left after bridge", "turn_by", -RIGHT_ANGLE_TURN_DEG),
    MissionStep("drive past bridge exit toward obstacle section", "move_forward", BRIDGE_EXIT_DISTANCE_MM),
    MissionStep("turn left toward obstacle course", "turn_by", -RIGHT_ANGLE_TURN_DEG),

    # Checkpoint 2 reached here. Robot should be facing the cones/obstacle
    # course. The FSM switches to LAPF obstacle avoidance after this scripted
    # checkpoint 1/2 sequence finishes.
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
    print("[sensor] lidar enabled for checkpoint 2+ obstacle avoidance")


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


def safe_stop_to_idle(robot: Robot, reason: str) -> None:
    """Stop the base and show idle LEDs after an unrecoverable failure."""
    robot.stop()
    show_idle_leds(robot)
    print(f"[FSM] IDLE - {reason}")


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

    if step.kind == "square_to_wall":
        # Measure the heading error against the bridge-mouth walls and null it.
        # When the read is ambiguous the estimator returns 0.0, so this becomes
        # a turn_by(0) that completes immediately and the FSM advances normally.
        error_deg = estimate_bridge_heading_error_deg(robot)
        return robot.turn_by(
            -error_deg,
            tolerance_deg=TURN_TOLERANCE_DEG,
            blocking=False,
        )

    raise ValueError(f"Unknown mission step kind: {step.kind}")


def straight_ahead_goal_from_current_pose(robot: Robot, distance_mm: float) -> tuple[float, float]:
    """Return a world-frame goal straight ahead of the current robot heading."""
    x_mm, y_mm, theta_deg = robot.get_pose()
    theta_rad = math.radians(theta_deg)
    return (
        x_mm + math.cos(theta_rad) * distance_mm,
        y_mm + math.sin(theta_rad) * distance_mm,
    )


def issue_lapf(robot: Robot, label: str, goal_mm: tuple[float, float]):
    """Start a non-blocking LAPF run toward an explicit world-frame goal."""
    goal_x_mm, goal_y_mm = goal_mm
    print(f"[route] {label}: LAPF obstacle avoidance to goal=({goal_x_mm:.0f}, {goal_y_mm:.0f})")
    return robot.lapf_to_goal(
        goal_x_mm,
        goal_y_mm,
        velocity=OBSTACLE_AVOIDANCE_SPEED_MM_S,
        tolerance=OBSTACLE_AVOIDANCE_TOLERANCE_MM,
        leash_length_mm=LAPF_LEASH_LENGTH_MM,
        repulsion_range_mm=LAPF_REPULSION_RANGE_MM,
        target_speed_mm_s=LAPF_TARGET_SPEED_MM_S,
        max_angular_rad_s=OBSTACLE_AVOIDANCE_MAX_ANGULAR_RAD_S,
        repulsion_gain=LAPF_REPULSION_GAIN,
        attraction_gain=LAPF_ATTRACTION_GAIN,
        force_ema_alpha=LAPF_FORCE_EMA_ALPHA,
        inflation_margin_mm=LAPF_INFLATION_MARGIN_MM,
        leash_half_angle_deg=LAPF_LEASH_HALF_ANGLE_DEG,
        blocking=False,
    )


def remaining_to_goal_mm(robot: Robot, goal_mm: tuple[float, float]) -> float:
    x_mm, y_mm, _ = robot.get_pose()
    return math.hypot(goal_mm[0] - x_mm, goal_mm[1] - y_mm)


def attempt_cap_s(remaining_mm: float) -> float:
    """Per-attempt wall-clock cap, scaled to the remaining distance."""
    nominal_s = remaining_mm / max(OBSTACLE_AVOIDANCE_SPEED_MM_S, 1e-6)
    return max(WALLCLOCK_CAP_FLOOR_S, WALLCLOCK_CAP_FACTOR * nominal_s)


def rear_clearance_mm(robot: Robot) -> float:
    """Distance to the nearest confirmed obstacle edge in the rear sector.

    Returns +inf when no confirmed track sits behind the robot. Used to decide
    whether reversing during recovery is safe (the planner only models forward
    obstacles, so a blind reverse must be guarded).
    """
    x_mm, y_mm, theta_deg = robot.get_pose()
    rear_dir_rad = math.radians(theta_deg) + math.pi
    half_rad = math.radians(REAR_SECTOR_HALF_ANGLE_DEG)
    nearest = math.inf
    for track in robot.get_obstacle_tracks():
        dx = float(track["x"]) - x_mm
        dy = float(track["y"]) - y_mm
        dist = math.hypot(dx, dy)
        if dist <= 1e-6:
            return 0.0
        rel = abs((math.atan2(dy, dx) - rear_dir_rad + math.pi) % (2.0 * math.pi) - math.pi)
        if rel <= half_rad:
            nearest = min(nearest, dist - float(track["radius"]))
    return nearest


def front_clearance_mm(
    robot: Robot,
    half_width_mm: float = FRONT_CONE_HALF_WIDTH_MM,
    max_range_mm: float = FRONT_CLEARANCE_MAX_RANGE_MM,
) -> float:
    """Nearest forward obstacle distance in a narrow cone straight ahead.

    Reads the live robot-frame lidar cloud (self-footprint already excluded by
    Fix #1, see Robot.SELF_FOOTPRINT_*). Keeps points ahead of the axle
    (fwd > 0) within +/- half_width_mm of the centerline and closer than
    max_range_mm, and returns the smallest forward distance. Returns +inf when
    nothing is in the cone. Used to detect and approach the outer course wall
    and to verify the cp3 straightaway is clear. fwd = +x, left = +y in the
    robot body frame (same axes as the obstacle dump).

    NOTE: uses the raw lidar point cloud, not the tracked disks, so a flat wall
    reads directly instead of being split into many small tracks. Depends on
    Fix #1 keeping the forward cone phantom-free.
    """
    nearest = math.inf
    for px, py in robot.get_obstacles():
        if px <= 0.0 or px > max_range_mm:
            continue
        if abs(py) > half_width_mm:
            continue
        if px < nearest:
            nearest = px
    return nearest


def _fit_line_angle_deg(points: list[tuple[float, float]]) -> float | None:
    """Principal-axis angle of a 2D point set, in degrees, CCW from robot +x.

    Total-least-squares fit (covariance principal axis), so it handles
    near-vertical walls that an ordinary y=mx+b fit blows up on. Returns the
    angle in [-90, 90], or None if the cloud is degenerate (collinear-free /
    too few points).
    """
    n = len(points)
    if n < 2:
        return None
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    sxx = sum((p[0] - mx) ** 2 for p in points)
    syy = sum((p[1] - my) ** 2 for p in points)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in points)
    if sxx + syy <= 1e-9:
        return None
    return math.degrees(0.5 * math.atan2(2.0 * sxy, sxx - syy))


def estimate_bridge_heading_error_deg(robot: Robot) -> float:
    """Heading error (deg) at the bridge mouth, measured against the walls.

    A positive result means the robot is rotated such that turn_by(-error)
    squares it up onto the bridge axis (SIGN UNVERIFIED — confirm on the
    venue). Uses the LEFT wall (parallel to travel) as the primary reference
    and the REAR wall (perpendicular) as an independent cross-check.

    Returns 0.0 (caller should not turn) whenever the read is ambiguous: too
    few wall points, an implausibly large fit, or the two walls disagreeing.
    Corrects HEADING only; lateral offset in the lane is not observed here.
    """
    if not BRIDGE_SQUARE_ENABLED:
        return 0.0

    cloud = robot.get_obstacles()  # robot frame, mm: fwd=+x, left=+y

    # Left wall: points off the +y side, near the expected standoff, within a
    # forward/back window so we fit the local segment beside the robot.
    left_pts = [
        (x, y) for (x, y) in cloud
        if y > 0.0
        and abs(y - BRIDGE_LEFT_WALL_DIST_MM) <= BRIDGE_WALL_BAND_MM
        and abs(x) <= BRIDGE_WALL_SAMPLE_DEPTH_MM
    ]
    if len(left_pts) < BRIDGE_SQUARE_MIN_POINTS:
        print(f"[bridge-square] left wall: {len(left_pts)} pts "
              f"(<{BRIDGE_SQUARE_MIN_POINTS}); skipping correction")
        return 0.0

    left_angle = _fit_line_angle_deg(left_pts)
    if left_angle is None:
        print("[bridge-square] left wall fit degenerate; skipping correction")
        return 0.0
    # Left wall runs parallel to travel: its principal axis ~= robot +x when
    # squared, so the line's tilt off 0 deg IS the heading error.
    error_deg = left_angle

    # Rear wall cross-check (perpendicular to travel: principal axis ~= +/-90
    # deg when squared, so deviation from 90 is the heading error).
    rear_pts = [
        (x, y) for (x, y) in cloud
        if x < 0.0
        and abs(abs(x) - BRIDGE_REAR_WALL_DIST_MM) <= BRIDGE_WALL_BAND_MM
        and abs(y) <= BRIDGE_WALL_SAMPLE_DEPTH_MM
    ]
    if len(rear_pts) >= BRIDGE_SQUARE_MIN_POINTS:
        rear_angle = _fit_line_angle_deg(rear_pts)
        if rear_angle is not None:
            rear_error = rear_angle - 90.0 if rear_angle >= 0.0 else rear_angle + 90.0
            if abs(error_deg - rear_error) > BRIDGE_SQUARE_CROSSCHECK_DEG:
                print(f"[bridge-square] left ({error_deg:+.1f}) and rear "
                      f"({rear_error:+.1f}) disagree > {BRIDGE_SQUARE_CROSSCHECK_DEG}; "
                      "skipping correction")
                return 0.0
            error_deg = 0.5 * (error_deg + rear_error)  # steadier averaged estimate

    if abs(error_deg) > BRIDGE_SQUARE_MAX_CORRECTION_DEG:
        print(f"[bridge-square] correction {error_deg:+.1f} deg exceeds clamp "
              f"{BRIDGE_SQUARE_MAX_CORRECTION_DEG}; skipping (suspect fit)")
        return 0.0
    if abs(error_deg) < BRIDGE_SQUARE_DEADBAND_DEG:
        print(f"[bridge-square] heading error {error_deg:+.1f} deg within "
              "deadband; no turn")
        return 0.0

    print(f"[bridge-square] squaring up: heading error {error_deg:+.1f} deg")
    return error_deg


def advance_along_axis_mm(robot: Robot, av: dict) -> float:
    """Forward progress from the segment start along the approach heading.

    Projection of (current pose - cp2 start) onto the segment heading direction,
    so lateral drift / weaving does not inflate it (unlike straight-line distance
    or odometry magnitude). Used by the cp3 minimum-advance gate so the terminal
    turn can't fire before the robot is genuinely far enough up the field.
    """
    x_mm, y_mm, _ = robot.get_pose()
    start_x_mm, start_y_mm = av["start_mm"]
    heading_rad = av["seg_heading_rad"]
    return (x_mm - start_x_mm) * math.cos(heading_rad) + (y_mm - start_y_mm) * math.sin(heading_rad)


def start_cp3_terminal_maneuver(robot: Robot):
    """Begin the wall-referenced cp3 terminal maneuver: square up to the wall.

    Shared by both entry paths into CP3_FACE_WALL (the wall-detected early
    termination and the goal-coordinate fallback).
    """
    print("[FSM] checkpoint 3 approach done - squaring up to the outer wall")
    return robot.turn_by(
        CP3_FACE_WALL_TURN_DEG,
        tolerance_deg=TURN_TOLERANCE_DEG,
        blocking=False,
    )


# ===========================================================================
# Checkpoint 3 deterministic 3-cone slalom helpers
# ===========================================================================

def narrow_lidar_for_slalom(robot: Robot) -> None:
    """Layer A: restrict the LiDAR to a forward lane window so the tracker isn't
    saturated by walls/bridge. Restored via restore_lidar_full() at handoff."""
    robot.set_lidar_filter(
        range_max_mm=CONE_RANGE_MAX_MM,
        fov_deg=(-CONE_FOV_HALF_DEG, CONE_FOV_HALF_DEG),
    )


def restore_lidar_full(robot: Robot) -> None:
    """Restore the full configured LiDAR filter (undo narrow_lidar_for_slalom)."""
    robot.set_lidar_filter(
        range_min_mm=LIDAR_RANGE_MIN_MM,
        range_max_mm=LIDAR_RANGE_MAX_MM,
        fov_deg=LIDAR_FOV_DEG,
    )


def project_onto_axis(x_mm: float, y_mm: float, sl: dict) -> tuple[float, float]:
    """Return (S, D) of a world point in the approach-axis frame: S = forward
    along the slalom heading from the cp2 start, D = lateral (+left)."""
    dx = x_mm - sl["start_mm"][0]
    dy = y_mm - sl["start_mm"][1]
    heading = sl["heading_rad"]
    s = dx * math.cos(heading) + dy * math.sin(heading)
    d = -dx * math.sin(heading) + dy * math.cos(heading)
    return s, d


def robot_axis_sd(robot: Robot, sl: dict) -> tuple[float, float]:
    """(S, D) of the robot itself in the approach-axis frame."""
    x_mm, y_mm, _ = robot.get_pose()
    return project_onto_axis(x_mm, y_mm, sl)


def cone_candidates(robot: Robot, sl: dict) -> list[dict]:
    """Confirmed tracks that look like real cones ahead in the lane window.

    Layer B filter: cone-sized radius, inside the lateral lane window, ahead of
    the robot within the search distance, and ISOLATED (no other track within
    CONE_ISOLATION_MIN_MM — rejects wall/bridge segments, which cluster). Returned
    sorted nearest-ahead first. Each entry: {x, y, r, S, D, ahead}.
    """
    tracks = robot.get_obstacle_tracks()
    s_robot, _ = robot_axis_sd(robot, sl)
    candidates: list[dict] = []
    for track in tracks:
        tx, ty, tr = float(track["x"]), float(track["y"]), float(track["radius"])
        if not (CONE_MIN_RADIUS_MM <= tr <= CONE_MAX_RADIUS_MM):
            continue
        s, d = project_onto_axis(tx, ty, sl)
        if abs(d) > CONE_LANE_HALF_WIDTH_MM:
            continue
        ahead = s - s_robot
        if ahead <= 0.0 or ahead > CONE_DETECT_MAX_AHEAD_MM:
            continue
        isolated = True
        for other in tracks:
            if int(other["id"]) == int(track["id"]):
                continue
            if math.hypot(float(other["x"]) - tx, float(other["y"]) - ty) < CONE_ISOLATION_MIN_MM:
                isolated = False
                break
        if not isolated:
            continue
        candidates.append({"x": tx, "y": ty, "r": tr, "S": s, "D": d, "ahead": ahead})
    candidates.sort(key=lambda c: c["ahead"])
    return candidates


def cone_candidate_summary(robot: Robot, sl: dict) -> str:
    """One-line dump of cone candidates vs. raw track count (first-run tuning)."""
    candidates = cone_candidates(robot, sl)
    raw = len(robot.get_obstacle_tracks())
    if not candidates:
        return f"cones: none in window ({raw} raw tracks)"
    parts = [
        f"[S={c['S']:5.0f} D={c['D']:5.0f} ahead={c['ahead']:5.0f} r={c['r']:3.0f}]"
        for c in candidates
    ]
    return f"cones {len(candidates)}/{raw}: " + " ".join(parts)


def cone_open_side_sign(sl: dict, cone: dict) -> int:
    """+1 = pass on the lane's LEFT, -1 = right.

    The known physical R/L/R cone pattern (CONE_PASS_SIDES) is the PRIMARY source.
    The approach-axis D=0 is the robot's ENTRY point, which drifts >1 m run-to-run,
    so the measured lateral sign misreads which side a cone is really on (a left
    cone can read as right). We therefore trust the pattern and only use the
    measured sign to flag a disagreement (bad detection, or a wrong pattern), not
    to override.
    """
    idx = sl["cone_index"] - 1
    pattern_sign = CONE_PASS_SIDES[idx] if 0 <= idx < len(CONE_PASS_SIDES) else 1
    d = cone["D"]
    if abs(d) >= CONE_SIDE_MIN_D_MM:
        measured_sign = -1 if d > 0.0 else 1
        if measured_sign != pattern_sign:
            print(
                f"[warn] cp3 slalom - cone {idx + 1} measured open side "
                f"({'left' if measured_sign > 0 else 'right'}, D={d:.0f}) disagrees with "
                f"R/L/R pattern ({'left' if pattern_sign > 0 else 'right'}) - trusting pattern"
            )
    return pattern_sign


def begin_slalom(robot: Robot, sl: dict, now: float) -> None:
    """Initialize slalom context at cp2 and narrow the LiDAR to the lane window."""
    x_mm, y_mm, theta_deg = robot.get_pose()
    sl.clear()
    sl.update(
        {
            "start_mm": (x_mm, y_mm),
            "heading_rad": math.radians(theta_deg),
            "cone_index": 1,
            "started_at": now,
            "scan_started_s": 0.0,
            "candidate": None,
            "candidate_count": 0,
            "current_cone_s": 0.0,
            "gate_started_at": now,
        }
    )
    narrow_lidar_for_slalom(robot)


def start_scan(robot: Robot, sl: dict, now: float):
    """(Re)start the SCAN phase for the current cone: creep straight along the
    approach axis while looking for the next cone. Returns the motion handle."""
    s_robot, _ = robot_axis_sd(robot, sl)
    sl["scan_started_s"] = s_robot
    sl["candidate"] = None
    sl["candidate_count"] = 0
    heading = sl["heading_rad"]
    target = s_robot + SLALOM_SCAN_LOOKAHEAD_MM
    tx = sl["start_mm"][0] + math.cos(heading) * target
    ty = sl["start_mm"][1] + math.sin(heading) * target
    print(f"[FSM] cp3 slalom - scanning for cone {sl['cone_index']}/3 (advance {s_robot:.0f} mm)")
    return robot.move_to(tx, ty, velocity=SLALOM_SPEED_MM_S, tolerance=DRIVE_TOLERANCE_MM, blocking=False)


def start_gate(robot: Robot, sl: dict, cone: dict, now: float):
    """Begin pure-pursuit to the gate beside+past the identified cone, offset to
    the open side. Returns the motion handle."""
    sign = cone_open_side_sign(sl, cone)
    heading = sl["heading_rad"]
    perp = heading + sign * (math.pi / 2.0)
    gate_x = cone["x"] + math.cos(heading) * CONE_GATE_FORWARD_MM + math.cos(perp) * CONE_PASS_CLEARANCE_MM
    gate_y = cone["y"] + math.sin(heading) * CONE_GATE_FORWARD_MM + math.sin(perp) * CONE_PASS_CLEARANCE_MM
    sl["current_cone_s"] = cone["S"]
    sl["current_cone_d"] = cone["D"]
    sl["gate_started_at"] = now
    # Scale the gate timeout to the distance still to cover (gate forward point
    # minus current advance), so wider inter-cone gaps get proportionally more time.
    s_robot, _ = robot_axis_sd(robot, sl)
    gate_distance = (cone["S"] + CONE_GATE_FORWARD_MM) - s_robot
    sl["gate_timeout_s"] = max(
        CONE_GATE_TIMEOUT_FLOOR_S,
        CONE_GATE_TIMEOUT_FACTOR * gate_distance / max(SLALOM_SPEED_MM_S, 1e-6),
    )
    side = "left" if sign > 0 else "right"
    print(
        f"[FSM] cp3 slalom - cone {sl['cone_index']}/3 at S={cone['S']:.0f} D={cone['D']:.0f} "
        f"r={cone['r']:.0f} mm - passing {side}, gate=({gate_x:.0f}, {gate_y:.0f}), "
        f"timeout={sl['gate_timeout_s']:.1f} s"
    )
    return robot.move_to(gate_x, gate_y, velocity=SLALOM_SPEED_MM_S, tolerance=DRIVE_TOLERANCE_MM, blocking=False)


def settle_forward(robot: Robot, sl: dict, s_robot: float):
    """Issue a short straight settle move CONE_EXIT_ADVANCE_MM further up-axis."""
    heading = sl["heading_rad"]
    target = s_robot + CONE_EXIT_ADVANCE_MM
    tx = sl["start_mm"][0] + math.cos(heading) * target
    ty = sl["start_mm"][1] + math.sin(heading) * target
    return robot.move_to(tx, ty, velocity=SLALOM_SPEED_MM_S, tolerance=DRIVE_TOLERANCE_MM, blocking=False)


def slalom_fail(robot: Robot, sl: dict, reason: str) -> None:
    """Restore the LiDAR and safe-stop to IDLE on a slalom failure."""
    restore_lidar_full(robot)
    safe_stop_to_idle(robot, reason)


def perturbed_goal_mm(
    base_goal_mm: tuple[float, float],
    seg_heading_rad: float,
    sign: int,
) -> tuple[float, float]:
    """Shift the segment goal laterally (perpendicular to the segment heading).

    `sign` alternates the side per retry. Working in the software theta frame
    keeps this consistent with turn_by()/get_pose() regardless of the firmware's
    physical left/right convention.
    """
    perp_rad = seg_heading_rad + sign * (math.pi / 2.0)
    return (
        base_goal_mm[0] + math.cos(perp_rad) * RECOVERY_LATERAL_OFFSET_MM,
        base_goal_mm[1] + math.sin(perp_rad) * RECOVERY_LATERAL_OFFSET_MM,
    )


def begin_avoidance_segment(
    robot: Robot,
    av: dict,
    label: str,
    distance_mm: float,
    next_state: str,
    now: float,
    terminate_on_wall: bool = False,
):
    """Start a LAPF segment and (re)initialize its watchdog context in `av`.

    terminate_on_wall=True (cp2->cp3 approach) ends the LAPF run as soon as the
    outer course wall is within WALL_DETECT_STANDOFF_MM ahead, instead of waiting
    to reach the (drift-corrupted) goal coordinate. The cp3->cp4 segment leaves
    it False and keeps the goal-coordinate termination.
    """
    goal_mm = straight_ahead_goal_from_current_pose(robot, distance_mm)
    start_x_mm, start_y_mm, theta_deg = robot.get_pose()
    handle = issue_lapf(robot, label, goal_mm)
    remaining = remaining_to_goal_mm(robot, goal_mm)
    av.clear()
    av.update(
        {
            "label": label,
            "base_goal_mm": goal_mm,
            "start_mm": (start_x_mm, start_y_mm),
            "seg_heading_rad": math.radians(theta_deg),
            "goal_mm": goal_mm,
            "next_state": next_state,
            "terminate_on_wall": terminate_on_wall,
            "last_gate_log_at": 0.0,
            "retry_count": 0,
            "recovery_sign": 1,
            "started_at": now,
            "attempt_started_at": now,
            "attempt_cap_s": attempt_cap_s(remaining),
            "best_remaining_mm": remaining,
            "best_remaining_at": now,
            "recovery_started_at": now,
        }
    )
    return handle


def evaluate_watchdog(robot: Robot, av: dict, now: float) -> str:
    """Decide the fate of an in-progress LAPF attempt.

    Returns 'continue', 'recover', or 'giveup'. Updates progress tracking.
    """
    remaining = remaining_to_goal_mm(robot, av["goal_mm"])
    if remaining < av["best_remaining_mm"] - NO_PROGRESS_EPS_MM:
        av["best_remaining_mm"] = remaining
        av["best_remaining_at"] = now

    # Global backstop over the whole segment (all attempts + recovery actions).
    if (now - av["started_at"]) >= SEGMENT_HARD_CEILING_S:
        return "giveup"

    # No-progress is authoritative: a genuine stall trips this within seconds.
    if (now - av["best_remaining_at"]) >= NO_PROGRESS_WINDOW_S:
        return "recover" if av["retry_count"] < MAX_AVOIDANCE_RETRIES else "giveup"

    # Slow-orbit last resort: still netting progress but not converging in time.
    # Retrying an orbit doesn't help, so give up rather than retry.
    if (now - av["attempt_started_at"]) >= av["attempt_cap_s"]:
        return "giveup"

    return "continue"


def start_recovery(robot: Robot, av: dict, now: float):
    """Begin a recovery attempt (current LAPF motion must already be cancelled).

    Returns (next_state, motion_handle). Reverses if the rear is clear, else
    skips straight to reorientation.
    """
    av["retry_count"] += 1
    av["recovery_sign"] = 1 if (av["retry_count"] % 2 == 1) else -1
    av["recovery_started_at"] = now

    rear = rear_clearance_mm(robot)
    if math.isinf(rear):
        reverse_dist = RECOVERY_REVERSE_MM
    elif rear >= RECOVERY_MIN_REAR_CLEARANCE_MM:
        reverse_dist = min(RECOVERY_REVERSE_MM, rear - RECOVERY_REVERSE_MARGIN_MM)
    else:
        reverse_dist = 0.0

    if reverse_dist >= RECOVERY_MIN_REVERSE_MM:
        print(
            f"[FSM] recovery {av['retry_count']}/{MAX_AVOIDANCE_RETRIES}: "
            f"reversing {reverse_dist:.0f} mm (rear clearance="
            f"{'inf' if math.isinf(rear) else f'{rear:.0f} mm'})"
        )
        handle = robot.move_backward(
            reverse_dist,
            velocity=RECOVERY_REVERSE_SPEED_MM_S,
            tolerance=RECOVERY_REVERSE_TOLERANCE_MM,
            blocking=False,
        )
        return "OBSTACLE_RECOVERY_REVERSE", handle

    print(
        f"[FSM] recovery {av['retry_count']}/{MAX_AVOIDANCE_RETRIES}: "
        f"rear clearance {rear:.0f} mm too low - skipping reverse, reorienting only"
    )
    handle = robot.turn_by(
        av["recovery_sign"] * RECOVERY_HEADING_OFFSET_DEG,
        tolerance_deg=TURN_TOLERANCE_DEG,
        blocking=False,
    )
    return "OBSTACLE_RECOVERY_REORIENT", handle


def reissue_after_recovery(robot: Robot, av: dict, now: float):
    """Re-issue LAPF toward a perturbed goal and reset the per-attempt watchdog."""
    new_goal = perturbed_goal_mm(av["base_goal_mm"], av["seg_heading_rad"], av["recovery_sign"])
    av["goal_mm"] = new_goal
    handle = issue_lapf(robot, f"{av['label']} (retry {av['retry_count']})", new_goal)
    remaining = remaining_to_goal_mm(robot, new_goal)
    av["attempt_started_at"] = now
    av["attempt_cap_s"] = attempt_cap_s(remaining)
    av["best_remaining_mm"] = remaining
    av["best_remaining_at"] = now
    return handle


def run_manipulator_placeholder(robot: Robot) -> None:
    """Checkpoint 5 scaffold: aim + fire one ball with PLACEHOLDER values.

    The yaw/pitch/distance below are UNTUNED guesses (see shooter.py); replace
    with the real cp5 sequence and tune on the physical build. Imported lazily
    so a missing/incomplete manipulator never blocks the navigation mission.
    NOTE: Shooter.aim_and_shoot() blocks (internal time.sleep), so BTN_2 cannot
    interrupt during the shot. This is acceptable here because cp5 runs after the
    robot has already stopped at checkpoint 4.
    """
    print("[FSM] CHECKPOINT 5 - manipulator placeholder (UNTUNED scaffold)")
    try:
        from robot.shooter import Shooter

        # TODO(cp5): replace placeholder aim/distance with tuned, mission-correct
        # values and the real shot sequence (target selection, multiple shots…).
        PLACEHOLDER_YAW_DEG = 120.0
        PLACEHOLDER_PITCH_DEG = 130.0
        PLACEHOLDER_TARGET_DISTANCE_M = 1.5

        shooter = Shooter(robot)
        shooter.enable()
        try:
            shooter.aim_and_shoot(
                PLACEHOLDER_YAW_DEG,
                PLACEHOLDER_PITCH_DEG,
                PLACEHOLDER_TARGET_DISTANCE_M,
            )
        finally:
            shooter.disable()
    except Exception as exc:  # noqa: BLE001 - scaffold must never crash the mission
        print(f"[warn] manipulator placeholder skipped: {exc}")


def print_status(robot: Robot, step_index: int) -> None:
    x, y, theta = robot.get_odometry_pose()
    step = MISSION_STEPS[step_index]
    print(
        f"  step={step_index + 1}/{len(MISSION_STEPS)} "
        f"{step.label} "
        f"odom=({x:6.0f}, {y:6.0f}) mm "
        f"theta={theta:5.1f} deg"
    )


def nearest_obstacle_summary(robot: Robot, count: int = 6) -> str:
    """Robot-frame dump of the nearest tracked obstacles (the ones the LAPF
    planner actually uses). Each is reported as fwd/left/edge distances so we can
    tell a genuinely blocked route (disks clustered straight ahead) from a field
    polluted by walls/the bridge/the arm (disks beside and behind the robot).

    fwd>0 ahead, fwd<0 behind; left>0 to the robot's left; edge = clearance from
    robot center to the disk surface.
    """
    x, y, theta_deg = robot.get_pose()
    theta = math.radians(theta_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)

    ranked = []
    for track in robot.get_obstacle_tracks():
        dx = float(track["x"]) - x
        dy = float(track["y"]) - y
        radius = float(track["radius"])
        fwd = dx * cos_t + dy * sin_t
        left = -dx * sin_t + dy * cos_t
        edge = math.hypot(dx, dy) - radius
        ranked.append((edge, fwd, left, radius))

    ranked.sort(key=lambda item: item[0])
    if not ranked:
        return "obstacles: none"
    parts = [
        f"[fwd={fwd:5.0f} left={left:5.0f} edge={edge:4.0f} r={radius:3.0f}]"
        for edge, fwd, left, radius in ranked[:count]
    ]
    return f"nearest {min(count, len(ranked))}/{len(ranked)} obstacles: " + " ".join(parts)


def print_obstacle_avoidance_status(robot: Robot, goal_mm: tuple[float, float] | None) -> None:
    x, y, theta = robot.get_pose()
    virtual_target = robot.get_virtual_target()
    obstacle_tracks = robot.get_obstacle_tracks()
    if goal_mm is None:
        remaining_summary = "goal=(unknown)"
    else:
        remaining_mm = math.hypot(goal_mm[0] - x, goal_mm[1] - y)
        remaining_summary = f"remaining={remaining_mm:6.0f} mm"

    if virtual_target is None:
        vt_summary = "vt=(none)"
    else:
        vt_summary = f"vt=({virtual_target[0]:6.0f}, {virtual_target[1]:6.0f}) mm"

    print(
        f"  obstacle avoidance odom=({x:6.0f}, {y:6.0f}) mm "
        f"theta={theta:5.1f} deg {remaining_summary} "
        f"{vt_summary} tracked={len(obstacle_tracks)}"
    )
    print("    " + nearest_obstacle_summary(robot, getattr(robot, "LAPF_MAX_PLANNER_TRACKS", 6)))


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    motion_handle = None
    av: dict = {}  # active obstacle-avoidance segment + watchdog context
    sl: dict = {}  # active cp3 slalom context
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
            av = {}
            print("[FSM] IDLE - press BTN_1 to start checkpoint 3 mission, BTN_2 to cancel")
            print(
                f"[CFG] scripted steps ({len(MISSION_STEPS)}): "
                + ", ".join(step.label for step in MISSION_STEPS)
            )
            print(
                "[CFG] checkpoint 2+ obstacle avoidance: "
                f"cp3_approach={CHECKPOINT_3_APPROACH_DISTANCE_MM:.0f} mm "
                f"cp3_final_straight={CHECKPOINT_3_FINAL_STRAIGHT_DISTANCE_MM:.0f} mm "
                f"cp4={CHECKPOINT_4_DISTANCE_MM:.0f} mm "
                f"speed={OBSTACLE_AVOIDANCE_SPEED_MM_S:.0f} mm/s "
                f"tolerance={OBSTACLE_AVOIDANCE_TOLERANCE_MM:.0f} mm "
                f"leash={LAPF_LEASH_LENGTH_MM:.0f} mm "
                f"repulsion_range={LAPF_REPULSION_RANGE_MM:.0f} mm "
                f"inflation={LAPF_INFLATION_MARGIN_MM:.0f} mm"
            )
            print(
                "[CFG] LAPF watchdog: "
                f"no_progress={NO_PROGRESS_EPS_MM:.0f} mm / {NO_PROGRESS_WINDOW_S:.1f} s "
                f"attempt_cap={WALLCLOCK_CAP_FACTOR:.1f}x (floor {WALLCLOCK_CAP_FLOOR_S:.0f} s) "
                f"segment_ceiling={SEGMENT_HARD_CEILING_S:.0f} s "
                f"max_retries={MAX_AVOIDANCE_RETRIES}"
            )
            print(
                "[CFG] cp3 wall-referenced finish: "
                f"detect_standoff={WALL_DETECT_STANDOFF_MM:.0f} mm "
                f"approach_target={WALL_APPROACH_TARGET_MM:.0f} mm "
                f"approach_speed={WALL_APPROACH_SPEED_MM_S:.0f} mm/s "
                f"clear_path_min={CLEAR_PATH_MIN_MM:.0f} mm "
                f"front_cone_half={FRONT_CONE_HALF_WIDTH_MM:.0f} mm "
                f"(turn signs UNVERIFIED)"
            )
            print(
                "[CFG] cp3 3-cone slalom: "
                f"enabled={SLALOM_ENABLED} speed={SLALOM_SPEED_MM_S:.0f} mm/s "
                f"fov=+/-{CONE_FOV_HALF_DEG:.0f} deg range_max={CONE_RANGE_MAX_MM:.0f} mm "
                f"lane_half={CONE_LANE_HALF_WIDTH_MM:.0f} mm cone_r=[{CONE_MIN_RADIUS_MM:.0f},{CONE_MAX_RADIUS_MM:.0f}] "
                f"isolation={CONE_ISOLATION_MIN_MM:.0f} mm clearance={CONE_PASS_CLEARANCE_MM:.0f} mm "
                "(all UNTUNED)"
            )
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_running_leds(robot)
                step_index = 0
                av = {}
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
                        print_status(robot, len(MISSION_STEPS) - 1)
                        if SLALOM_ENABLED:
                            print("[FSM] CHECKPOINT 2 - starting 3-cone slalom toward checkpoint 3")
                            begin_slalom(robot, sl, now)
                            motion_handle = start_scan(robot, sl, now)
                            last_status_print_at = now
                            state = "CP3_SLALOM_SCAN"
                        else:
                            print("[FSM] CHECKPOINT 2 - starting obstacle avoidance toward checkpoint 3")
                            motion_handle = begin_avoidance_segment(
                                robot,
                                av,
                                "checkpoint 2 -> checkpoint 3 approach",
                                CHECKPOINT_3_APPROACH_DISTANCE_MM,
                                next_state="CP3_FACE_WALL",
                                now=now,
                                terminate_on_wall=True,
                            )
                            last_status_print_at = now
                            state = "OBSTACLE_AVOIDANCE"
                    else:
                        motion_handle = start_step(robot, MISSION_STEPS[step_index])
                        print(
                            f"[FSM] MOVING - started step {step_index + 1}/{len(MISSION_STEPS)}: "
                            f"{MISSION_STEPS[step_index].label}"
                        )

        elif state == "OBSTACLE_AVOIDANCE":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE - obstacle avoidance cancelled")
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_obstacle_avoidance_status(robot, av.get("goal_mm"))
                    last_status_print_at = now

                if motion_handle is not None and motion_handle.is_finished():
                    # A finished handle means goal-reached OR the motion thread
                    # ended early (e.g. error). Confirm we are actually at the
                    # goal before treating it as success.
                    remaining = remaining_to_goal_mm(robot, av["goal_mm"])
                    if remaining <= OBSTACLE_AVOIDANCE_TOLERANCE_MM + SUCCESS_MARGIN_MM:
                        print_obstacle_avoidance_status(robot, av.get("goal_mm"))
                        next_state = av["next_state"]
                        if next_state == "CP3_FACE_WALL":
                            # Reached the goal coordinate before the wall came
                            # into range; hand off to the same wall-referenced
                            # terminal maneuver rather than ending here.
                            motion_handle = start_cp3_terminal_maneuver(robot)
                            last_status_print_at = now
                            state = "CP3_FACE_WALL"
                        else:  # CHECKPOINT_5_MANIPULATOR
                            print("[FSM] CHECKPOINT 4 reached - stopping before checkpoint 5")
                            robot.stop()
                            motion_handle = None
                            state = "CHECKPOINT_5_MANIPULATOR"
                    else:
                        # Ended without reaching the goal - treat as a stall.
                        print(
                            f"[FSM] LAPF ended {remaining:.0f} mm short of goal "
                            f"({av['label']}) - attempting recovery"
                        )
                        if av["retry_count"] < MAX_AVOIDANCE_RETRIES:
                            motion_handle = None
                            state, motion_handle = start_recovery(robot, av, now)
                            last_status_print_at = now
                        else:
                            motion_handle = None
                            safe_stop_to_idle(robot, f"LAPF failed ({av['label']}) - retries exhausted")
                            state = "IDLE"
                else:
                    # cp2->cp3 approach: end the LAPF run as soon as the outer
                    # course wall is within standoff ahead (Fix #2), instead of
                    # chasing the drift-corrupted goal coordinate. The terminal
                    # turn fires only when BOTH guards hold:
                    #   1. wall armed: within WALL_ARM_REMAINING_MM of the goal, so
                    #      a cone in the lower field can't trigger it; and
                    #   2. min advance: at least CHECKPOINT_3_MIN_ADVANCE_MM of
                    #      forward progress along the approach axis, so the turn
                    #      can't fire half a tile early. Until advance clears the
                    #      floor the approach keeps running.
                    # The cp3->cp4 segment leaves terminate_on_wall False and keeps
                    # the goal-coordinate termination handled above.
                    wall_armed = (
                        av.get("terminate_on_wall")
                        and remaining_to_goal_mm(robot, av["goal_mm"]) <= WALL_ARM_REMAINING_MM
                    )
                    front = front_clearance_mm(robot) if wall_armed else math.inf
                    fire_turn = False
                    if front <= WALL_DETECT_STANDOFF_MM:
                        advance = advance_along_axis_mm(robot, av)
                        if advance >= CHECKPOINT_3_MIN_ADVANCE_MM:
                            fire_turn = True
                        elif (now - av.get("last_gate_log_at", 0.0)) >= STATUS_PRINT_INTERVAL_S:
                            # Wall is in range but the advance gate is holding the
                            # turn back — log advance vs. minimum so the constant
                            # can be tuned precisely from the run.
                            print(
                                f"[FSM] cp3 turn held by advance gate: advance {advance:.0f} mm "
                                f"< min {CHECKPOINT_3_MIN_ADVANCE_MM:.0f} mm "
                                f"(wall {front:.0f} mm ahead) - continuing approach"
                            )
                            av["last_gate_log_at"] = now

                    if fire_turn:
                        # Final measured advance is printed here so the next run
                        # gives the exact along-axis distance at which the turn
                        # fires — dial CHECKPOINT_3_MIN_ADVANCE_MM in from this.
                        print(
                            f"[FSM] checkpoint 3 approach - outer wall {front:.0f} mm ahead, "
                            f"advance {advance:.0f} mm (>= min {CHECKPOINT_3_MIN_ADVANCE_MM:.0f} mm) "
                            "- squaring up"
                        )
                        cancel_motion(robot, motion_handle)
                        motion_handle = start_cp3_terminal_maneuver(robot)
                        last_status_print_at = now
                        state = "CP3_FACE_WALL"
                    else:
                        action = evaluate_watchdog(robot, av, now)
                        if action == "recover":
                            print(
                                f"[FSM] LAPF stalled ({av['label']}) - "
                                f"no progress for {NO_PROGRESS_WINDOW_S:.0f} s, recovering"
                            )
                            cancel_motion(robot, motion_handle)
                            motion_handle = None
                            state, motion_handle = start_recovery(robot, av, now)
                            last_status_print_at = now
                        elif action == "giveup":
                            cancel_motion(robot, motion_handle)
                            motion_handle = None
                            safe_stop_to_idle(robot, f"LAPF failed ({av['label']}) - giving up")
                            state = "IDLE"
                        # action == "continue": keep tracking

        elif state == "OBSTACLE_RECOVERY_REVERSE":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE - recovery reverse cancelled")
                state = "IDLE"
            else:
                reverse_done = motion_handle is not None and motion_handle.is_finished()
                reverse_timeout = (now - av["recovery_started_at"]) >= RECOVERY_REVERSE_TIMEOUT_S
                if (now - av["started_at"]) >= SEGMENT_HARD_CEILING_S:
                    cancel_motion(robot, motion_handle)
                    motion_handle = None
                    safe_stop_to_idle(robot, f"LAPF failed ({av['label']}) - segment timeout in recovery")
                    state = "IDLE"
                elif reverse_done or reverse_timeout:
                    if not reverse_done:
                        cancel_motion(robot, motion_handle)
                    motion_handle = robot.turn_by(
                        av["recovery_sign"] * RECOVERY_HEADING_OFFSET_DEG,
                        tolerance_deg=TURN_TOLERANCE_DEG,
                        blocking=False,
                    )
                    av["recovery_started_at"] = now
                    state = "OBSTACLE_RECOVERY_REORIENT"

        elif state == "OBSTACLE_RECOVERY_REORIENT":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE - recovery reorient cancelled")
                state = "IDLE"
            else:
                reorient_done = motion_handle is not None and motion_handle.is_finished()
                reorient_timeout = (now - av["recovery_started_at"]) >= RECOVERY_REORIENT_TIMEOUT_S
                if (now - av["started_at"]) >= SEGMENT_HARD_CEILING_S:
                    cancel_motion(robot, motion_handle)
                    motion_handle = None
                    safe_stop_to_idle(robot, f"LAPF failed ({av['label']}) - segment timeout in recovery")
                    state = "IDLE"
                elif reorient_done or reorient_timeout:
                    if not reorient_done:
                        cancel_motion(robot, motion_handle)
                    motion_handle = reissue_after_recovery(robot, av, now)
                    last_status_print_at = now
                    state = "OBSTACLE_AVOIDANCE"

        elif state == "CP3_SLALOM_SCAN":
            # Creep forward along the approach axis looking for the next cone.
            # Accept only a cone-sized, isolated, persistent track in the lane
            # window ahead; then build its gate and pursue it.
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                restore_lidar_full(robot)
                show_idle_leds(robot)
                print("[FSM] IDLE - cp3 slalom cancelled")
                state = "IDLE"
            else:
                s_robot, d_robot = robot_axis_sd(robot, sl)
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print(
                        f"  cp3 slalom scan cone {sl['cone_index']}/3 "
                        f"S={s_robot:6.0f} D={d_robot:6.0f} mm"
                    )
                    print("    " + cone_candidate_summary(robot, sl))
                    last_status_print_at = now

                candidates = cone_candidates(robot, sl)
                cone = candidates[0] if candidates else None
                if cone is None:
                    sl["candidate"] = None
                    sl["candidate_count"] = 0
                else:
                    prev = sl["candidate"]
                    if prev is not None and math.hypot(cone["x"] - prev[0], cone["y"] - prev[1]) <= CONE_ASSOC_MM:
                        sl["candidate_count"] += 1
                    else:
                        sl["candidate_count"] = 1
                    sl["candidate"] = (cone["x"], cone["y"])

                if (now - sl["started_at"]) >= SLALOM_SEGMENT_CEILING_S:
                    cancel_motion(robot, motion_handle)
                    motion_handle = None
                    slalom_fail(robot, sl, "cp3 slalom timed out (segment ceiling)")
                    state = "IDLE"
                elif cone is not None and sl["candidate_count"] >= CONE_CONFIRM_FRAMES:
                    cancel_motion(robot, motion_handle)
                    motion_handle = start_gate(robot, sl, cone, now)
                    last_status_print_at = now
                    state = "CP3_SLALOM_GATE"
                elif (s_robot - sl["scan_started_s"]) >= CONE_SCAN_MAX_ADVANCE_MM:
                    cancel_motion(robot, motion_handle)
                    motion_handle = None
                    slalom_fail(
                        robot,
                        sl,
                        f"cp3 slalom - cone {sl['cone_index']}/3 not found within "
                        f"{CONE_SCAN_MAX_ADVANCE_MM:.0f} mm ({len(candidates)} candidates) - check cone count",
                    )
                    state = "IDLE"
                # else: keep creeping/scanning (motion continues)

        elif state == "CP3_SLALOM_GATE":
            # Pure-pursuit the gate beside+past the current cone; advance once the
            # robot is past the cone along the approach axis.
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                restore_lidar_full(robot)
                show_idle_leds(robot)
                print("[FSM] IDLE - cp3 slalom cancelled")
                state = "IDLE"
            else:
                s_robot, d_robot = robot_axis_sd(robot, sl)
                lateral_sep = abs(d_robot - sl["current_cone_d"])
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print(
                        f"  cp3 slalom gate cone {sl['cone_index']}/3 "
                        f"S={s_robot:6.0f} D={d_robot:6.0f} mm "
                        f"(cone S={sl['current_cone_s']:.0f} D={sl['current_cone_d']:.0f} "
                        f"sep={lateral_sep:.0f})"
                    )
                    last_status_print_at = now

                # A cone is cleared only when the robot has BOTH driven past the
                # gate's forward point AND achieved lateral clearance from the
                # cone. Forward-only (the old test) let it cut the corner straight
                # through the cone before the sidestep completed.
                past_gate = s_robot >= sl["current_cone_s"] + CONE_GATE_FORWARD_MM
                lateral_clear = lateral_sep >= (CONE_PASS_CLEARANCE_MM - CONE_PASS_CLEARANCE_TOL_MM)
                reached_gate = motion_handle is not None and motion_handle.is_finished()
                gate_done = lateral_clear and (past_gate or reached_gate)
                if (now - sl["started_at"]) >= SLALOM_SEGMENT_CEILING_S:
                    cancel_motion(robot, motion_handle)
                    motion_handle = None
                    slalom_fail(robot, sl, "cp3 slalom timed out (segment ceiling)")
                    state = "IDLE"
                elif gate_done:
                    cancel_motion(robot, motion_handle)
                    motion_handle = None
                    print(
                        f"[FSM] cp3 slalom - cone {sl['cone_index']}/3 passed "
                        f"(advance {s_robot:.0f} mm, lateral sep {lateral_sep:.0f} mm)"
                    )
                    sl["cone_index"] += 1
                    if sl["cone_index"] > 3:
                        print("[FSM] cp3 slalom - all 3 cones passed - settling into finish lane")
                        motion_handle = settle_forward(robot, sl, s_robot)
                        last_status_print_at = now
                        state = "CP3_SLALOM_EXIT"
                    else:
                        motion_handle = start_scan(robot, sl, now)
                        last_status_print_at = now
                        state = "CP3_SLALOM_SCAN"
                elif (now - sl["gate_started_at"]) >= sl["gate_timeout_s"]:
                    cancel_motion(robot, motion_handle)
                    motion_handle = None
                    slalom_fail(
                        robot,
                        sl,
                        f"cp3 slalom - cone {sl['cone_index']}/3 gate not cleared in "
                        f"{sl['gate_timeout_s']:.1f} s",
                    )
                    state = "IDLE"
                # else: keep pursuing the gate

        elif state == "CP3_SLALOM_EXIT":
            # Past cone 3: settle forward, confirm no more cones ahead and the
            # advance floor is met, restore the LiDAR, and hand to the existing
            # wall-square -> finish maneuver.
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                restore_lidar_full(robot)
                show_idle_leds(robot)
                print("[FSM] IDLE - cp3 slalom cancelled")
                state = "IDLE"
            else:
                s_robot, d_robot = robot_axis_sd(robot, sl)
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print(
                        f"  cp3 slalom exit S={s_robot:6.0f} D={d_robot:6.0f} mm"
                    )
                    print("    " + cone_candidate_summary(robot, sl))
                    last_status_print_at = now

                if (now - sl["started_at"]) >= SLALOM_SEGMENT_CEILING_S:
                    cancel_motion(robot, motion_handle)
                    motion_handle = None
                    slalom_fail(robot, sl, "cp3 slalom timed out (segment ceiling)")
                    state = "IDLE"
                elif motion_handle is None or motion_handle.is_finished():
                    cancel_motion(robot, motion_handle)
                    motion_handle = None
                    remaining_cones = cone_candidates(robot, sl)
                    if remaining_cones:
                        slalom_fail(
                            robot,
                            sl,
                            f"cp3 slalom - unexpected cone still ahead after 3 passes "
                            f"({len(remaining_cones)} in window) - check cone count",
                        )
                        state = "IDLE"
                    elif s_robot < CHECKPOINT_3_MIN_ADVANCE_MM:
                        # Belt-and-suspenders advance floor not yet met: settle more.
                        print(
                            f"[FSM] cp3 slalom exit - advance {s_robot:.0f} mm "
                            f"< min {CHECKPOINT_3_MIN_ADVANCE_MM:.0f} mm - settling further"
                        )
                        motion_handle = settle_forward(robot, sl, s_robot)
                        last_status_print_at = now
                    else:
                        restore_lidar_full(robot)
                        print(
                            f"[FSM] cp3 slalom complete (advance {s_robot:.0f} mm, lane clear) "
                            "- handing off to wall-referenced finish"
                        )
                        motion_handle = start_cp3_terminal_maneuver(robot)
                        last_status_print_at = now
                        state = "CP3_FACE_WALL"

        elif state == "CP3_FACE_WALL":
            # Square up to the outer course wall (turn started on entry).
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE - cp3 face-wall turn cancelled")
                state = "IDLE"
            elif motion_handle is not None and motion_handle.is_finished():
                print(
                    "[FSM] squared up to wall - approaching to "
                    f"{WALL_APPROACH_TARGET_MM:.0f} mm standoff"
                )
                motion_handle = None
                av["wall_approach_started_at"] = now
                last_status_print_at = now
                state = "CP3_APPROACH_WALL"

        elif state == "CP3_APPROACH_WALL":
            # Closed-loop creep toward the wall until the front cone reads the
            # target standoff. Velocity-controlled (no MotionHandle), so this
            # state carries its OWN timeout — it is not under the LAPF watchdog.
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop()
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE - cp3 wall approach cancelled")
                state = "IDLE"
            else:
                front = front_clearance_mm(robot)
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    x, y, theta = robot.get_pose()
                    front_str = "inf" if math.isinf(front) else f"{front:.0f}"
                    print(
                        f"  cp3 wall approach odom=({x:6.0f}, {y:6.0f}) mm "
                        f"theta={theta:5.1f} deg front={front_str} mm"
                    )
                    last_status_print_at = now

                if front <= WALL_APPROACH_TARGET_MM:
                    robot.stop()
                    print(
                        f"[FSM] reached wall standoff ({front:.0f} mm) - "
                        "facing finish straightaway"
                    )
                    motion_handle = robot.turn_by(
                        CP3_FACE_STRAIGHTAWAY_TURN_DEG,
                        tolerance_deg=TURN_TOLERANCE_DEG,
                        blocking=False,
                    )
                    last_status_print_at = now
                    state = "CP3_FACE_STRAIGHTAWAY"
                elif (now - av["wall_approach_started_at"]) >= WALL_APPROACH_TIMEOUT_S:
                    safe_stop_to_idle(
                        robot,
                        "cp3 wall approach timed out - wall not reached "
                        "(check detection / standoffs / turn sign)",
                    )
                    state = "IDLE"
                elif math.isfinite(front):
                    robot.set_velocity(WALL_APPROACH_SPEED_MM_S, 0.0)
                else:
                    # Wall not in the forward cone - hold rather than drive blind;
                    # the timeout above will flag it.
                    robot.stop()

        elif state == "CP3_FACE_STRAIGHTAWAY":
            # Turn onto the checkpoint-3 finish straightaway (turn started on entry).
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE - cp3 face-straightaway turn cancelled")
                state = "IDLE"
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                print("[FSM] facing finish straightaway - verifying path is clear")
                state = "CP3_VERIFY_STRAIGHTAWAY"

        elif state == "CP3_VERIFY_STRAIGHTAWAY":
            # One-shot alignment check: a clear forward cone means we are lined up
            # with the straightaway. If blocked, flag and stop rather than drive
            # blind (per design decision).
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop()
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE - cp3 straightaway verify cancelled")
                state = "IDLE"
            else:
                front = front_clearance_mm(robot)
                front_str = "inf" if math.isinf(front) else f"{front:.0f}"
                if front >= CLEAR_PATH_MIN_MM:
                    print(
                        f"[FSM] straightaway clear (nearest {front_str} mm) - "
                        "driving to checkpoint 3"
                    )
                    motion_handle = robot.move_forward(
                        CHECKPOINT_3_FINAL_STRAIGHT_DISTANCE_MM,
                        velocity=DRIVE_VELOCITY_MM_S,
                        tolerance=DRIVE_TOLERANCE_MM,
                        blocking=False,
                    )
                    last_status_print_at = now
                    state = "CP3_DRIVE_STRAIGHTAWAY"
                else:
                    safe_stop_to_idle(
                        robot,
                        f"straightaway blocked ({front_str} mm < {CLEAR_PATH_MIN_MM:.0f} mm) "
                        "- not aligned",
                    )
                    state = "IDLE"

        elif state == "CP3_DRIVE_STRAIGHTAWAY":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE - cp3 straightaway drive cancelled")
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    x, y, theta = robot.get_odometry_pose()
                    print(
                        f"  cp3 straightaway odom=({x:6.0f}, {y:6.0f}) mm "
                        f"theta={theta:5.1f} deg"
                    )
                    last_status_print_at = now

                if motion_handle is not None and motion_handle.is_finished():
                    print("[FSM] CHECKPOINT 3 reached - starting checkpoint 4 obstacle avoidance")
                    motion_handle = begin_avoidance_segment(
                        robot,
                        av,
                        "checkpoint 3 -> checkpoint 4",
                        CHECKPOINT_4_DISTANCE_MM,
                        next_state="CHECKPOINT_5_MANIPULATOR",
                        now=now,
                    )
                    last_status_print_at = now
                    state = "OBSTACLE_AVOIDANCE"

        elif state == "CHECKPOINT_5_MANIPULATOR":
            # Robot is stopped at checkpoint 4. Run the manipulator scaffold once,
            # then return to IDLE. (Placeholder/untuned — see function docstring.)
            run_manipulator_placeholder(robot)
            motion_handle = None
            show_idle_leds(robot)
            print("[FSM] IDLE - press BTN_1 to run again")
            state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
