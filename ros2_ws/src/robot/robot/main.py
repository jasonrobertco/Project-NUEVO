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

# ===========================================================================
# TEMP / TESTING-ONLY -- skip the cp1->bridge->cp2 scripted drive.
# When True, BTN_1 resets odometry to the cp2 start pose (0,0, heading +Y) and
# jumps straight into the cp2->cp3 obstacle-avoidance leg, so the LiDAR/LAPF
# weave can be tested without driving the whole scripted approach each time.
# The robot must be physically placed at the cp2 spot facing up the obstacle
# course (+Y) before pressing BTN_1.
#   >>> SET BACK TO False FOR THE FULL cp1->cp5 RUN <<<  (this line)
# When False the scripted MISSION_STEPS sequence runs unchanged.
START_AT_CP2 = False  # TEMP: True = start at checkpoint 2 (skip scripted steps)
# ===========================================================================

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
CHECKPOINT_1_APPROACH_DISTANCE_MM = 2800.0   # start -> checkpoint 1 approach point
BRIDGE_ALIGN_DISTANCE_MM = 500.0             # short nudge into the bridge lane
BRIDGE_CROSS_DISTANCE_MM = 2350.0            # length of the bridge/ramp crossing
# Post-bridge exit hop: short open-loop drive between the two left turns to round
# the corner onto the obstacle course. Hand-tuned, not a course tile.
BRIDGE_EXIT_DISTANCE_MM = 600.0

# --- Bridge-entry wall-square correction -----------------------------------
# The bridge is crossed open-loop on odometry over a long distance, so any
# heading error left by the two preceding 90-degree turns becomes a large
# lateral deviation (deviation ~= BRIDGE_CROSS_DISTANCE_MM * sin(theta_err)).
# Right after the 2nd turn the LiDAR sees both SIDE walls of the bridge lane,
# each running parallel to the crossing. We fit a line to each side and turn to
# null the heading error (the side walls' tilt off the robot's forward axis)
# before committing to the crossing. The wall standoff is auto-detected per
# side, so no measured distance is needed. This corrects HEADING only, not
# lateral offset.
#
# The correction SIGN is UNVERIFIED (same caveat as the scripted turns) —
# bench-test the direction before trusting it on the bridge. The correction is
# deliberately conservative: it does NOTHING (no turn) whenever the scan is
# ambiguous (no usable side wall, left/right disagree, or fit too large), so a
# noisy read cannot make the crossing worse.
BRIDGE_SQUARE_ENABLED = True
BRIDGE_SQUARE_DEBUG = True                 # dump the per-sector scan at square-up (tuning aid)
BRIDGE_WALL_BAND_MM = 250.0               # keep points within +/- this of the auto-detected standoff
BRIDGE_WALL_SAMPLE_DEPTH_MM = 1200.0      # along-wall window of points to fit (|x| <= this)
BRIDGE_SQUARE_MIN_POINTS = 8              # minimum inliers to trust a line fit
BRIDGE_SQUARE_DEADBAND_DEG = 1.5          # below this error, don't bother turning
BRIDGE_SQUARE_MAX_CORRECTION_DEG = 15.0   # refuse larger corrections (guards a bad fit)
BRIDGE_SQUARE_CROSSCHECK_DEG = 6.0        # left vs right estimates must agree within this

# Checkpoint 2 and later obstacle-avoidance course sections.
# The UI/course grid uses 610 mm cells (see WorldCanvas.tsx / vm_demo.py).
COURSE_TILE_MM = 610.0
CHECKPOINT_3_APPROACH_TILES = 5.0
CHECKPOINT_3_FINAL_STRAIGHT_TILES = 1.0
CHECKPOINT_4_STRAIGHT_TILES = 5.0

CHECKPOINT_3_APPROACH_DISTANCE_MM = COURSE_TILE_MM * CHECKPOINT_3_APPROACH_TILES
# CHECKPOINT_3_FINAL_STRAIGHT_DISTANCE_MM = COURSE_TILE_MM * CHECKPOINT_3_FINAL_STRAIGHT_TILES  # ORIGINAL: 610 mm (1 tile)
# CHECKPOINT_3_FINAL_STRAIGHT_DISTANCE_MM = 2600.0  # prior: full straightaway, stopped at the end wall
CHECKPOINT_3_FINAL_STRAIGHT_DISTANCE_MM = 2525.0  # 2600 to the checkpoint, -75 mm backoff from the end wall
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
LAPF_LEASH_LENGTH_MM = 500.0
LAPF_REPULSION_RANGE_MM = 300.0
LAPF_TARGET_SPEED_MM_S = 200.0
LAPF_REPULSION_GAIN = 550.0
# LAPF_ATTRACTION_GAIN = 1.0           # ORIGINAL (revert here): froze in the force-null between cones
LAPF_ATTRACTION_GAIN = 3.0             # commit-forward bias: drive up through the gap, not balance L/R cones
                                       # (also strengthens the 0.15*attr_gain tangent escape proportionally).
                                       # If it CLIPS cones, back down toward ~2.5; if it still FREEZES, next
                                       # step is the structural forward-arc repulsion filter (not done yet).
LAPF_FORCE_EMA_ALPHA = 0.35
# --- Right-sized cone keep-out + arc-not-pivot tuning (applied together; COUPLED) ---
# These three move as a set. The leash was narrowed to 35 deg to stop the
# spin-in-place pivot, which is ONLY safe because inflation was dropped to 200.
# Fix 2 geometry must hold: leash can place the virtual target outside the
# inflated cone, i.e. reach = leash_length*sin(half_angle) >= eff_radius.
#   reach = 500 * sin(45 deg) = 354 mm  >=  eff_radius = r_track(<=75) + 230 = 305 mm  (49 mm headroom)
#   (was 35 deg = 287 mm, 40 deg = 321 mm; widened to 45 for aggressive dead-ahead-cone dodge room)
# >>> DO NOT narrow the leash further (or raise inflation) without re-checking
#     this inequality, or the target gets re-trapped inside the bubble (nose-in). <<<
# LAPF_INFLATION_MARGIN_MM = 250.0      # ORIGINAL (revert here): oversized ~325 mm keep-out
# LAPF_INFLATION_MARGIN_MM = 200.0      # prior: body-edge clearance ~33-58 mm -> grazed a near-side cone
LAPF_INFLATION_MARGIN_MM = 230.0        # ~280-305 mm keep-out; body-edge-to-cone clearance ~63-88 mm
# LAPF_LEASH_HALF_ANGLE_DEG = 50.0      # ORIGINAL (revert here): wide leash -> target far off-axis -> pivot
# LAPF_LEASH_HALF_ANGLE_DEG = 35.0      # prior: arced but ran out of lateral room on a close dead-ahead cone
# LAPF_LEASH_HALF_ANGLE_DEG = 40.0      # prior: +5 deg, still clipped the 3rd cone
LAPF_LEASH_HALF_ANGLE_DEG = 45.0        # aggressive steering authority to clearly swing around a dead-ahead cone
# Forward-clearance throttle (Fix 3): "see cone -> steer -> confirm clear -> go".
# Scales LINEAR speed only (angular untouched), so when a cone is close dead-ahead
# the robot keeps turning hard toward the committed open side while forward speed
# drops, then ramps back to full as the forward corridor clears -- it won't nose
# in before the turn develops. RE-ENABLED: safe now that the open-side COMMITTED
# escape stops the old pivot/spin (with a committed turn direction, throttling
# forward makes it ARC tightly around the cone instead of spinning in place).
# Floored at 0.35 (not zero) so it never fully stops/freezes -- it keeps creeping
# forward while it swings off the cone.
LAPF_SLOW_CLEARANCE_START_MM = 450.0   # begin easing off when nearest cone edge ahead <= this
LAPF_SLOW_CLEARANCE_STOP_MM = 150.0    # most-slowed (floor) by this edge clearance (~"cone close")
# LAPF_MIN_SPEED_FRAC = 0.5            # earlier mild value
# LAPF_MIN_SPEED_FRAC = 1.0            # PRIOR (revert here): Fix 3 OFF (no throttle)
LAPF_MIN_SPEED_FRAC = 0.35             # Fix 3 ON: floor 35% so it keeps creeping while swinging off the cone

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

# RECOVERY_REVERSE_MM = 300.0            # ORIGINAL (revert here): exceeded ~150 mm forward gain -> walked backward
RECOVERY_REVERSE_MM = 100.0              # nominal back-up distance (small, so a retry can't net rearward)
# Only reverse when the robot is genuinely about to hit something dead ahead.
# A force-null stall BETWEEN cones (cones beside the lane, nothing close in front)
# must NOT reverse: backing up just re-enters the same symmetric minimum and walks
# the robot backward. In that case skip the reverse and let reorient + lateral goal
# perturbation break the symmetry instead.
RECOVERY_REVERSE_FRONT_TRIGGER_MM = 120.0  # reverse only if front clearance is below this
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
# Minimum forward advance before the cp3 terminal turn may fire. Advance is the
# projection of (current pose - cp2 start) onto the approach heading -- drift-free
# along-axis progress, NOT the odometry goal coordinate -- so lateral drift /
# weaving doesn't inflate it. ANDed with the wall/standoff trigger: even once the
# wall is detected, the turn is HELD until advance reaches this floor, and the
# approach keeps running until then.
#
# This is the ONLY "we've driven far enough that what's ahead must be the wall,
# not a cone" guard (the old goal-coordinate WALL_ARM_REMAINING_MM arm gate is
# removed -- no drift-prone distance is in the fire decision any more). The real
# end wall is 5 tiles (5*610=3050 mm) out, so anything seen meaningfully nearer is
# a cone; requiring 4 tiles of advance before trusting the standoff trigger
# rejects every cone in the lower field. NEEDS FIELD TUNING: the fire-time log
# below prints the exact measured advance, so dial this in from one run.
CHECKPOINT_3_MIN_ADVANCE_MM = 4 * COURSE_TILE_MM   # 4 tiles (4*610=2440 mm); end wall sits at 5 tiles
# WALL_APPROACH_TARGET_MM = 350.0      # ORIGINAL (revert here): stopped 75 mm closer
# WALL_APPROACH_TARGET_MM = 425.0      # prior: still a little too close to wall 1
WALL_APPROACH_TARGET_MM = 475.0        # stop this far from the wall (re-zero); +125 mm total backoff (both walls)
# WALL_APPROACH_SPEED_MM_S = 80.0      # ORIGINAL (revert here): too slow, timed out ~640 mm short
WALL_APPROACH_SPEED_MM_S = 150.0       # closed-loop approach speed (closes the full ~1300 mm in time)
# WALL_APPROACH_TIMEOUT_S = 8.0        # ORIGINAL (revert here): too short to close the full distance
WALL_APPROACH_TIMEOUT_S = 16.0         # watchdog for the approach drive (not under LAPF watchdog)
# The standoff the turn actually fires at is variable run-to-run: the creep only
# samples the front cone once per FSM tick and lidar returns jitter, so a single
# noisy frame dipping to the target would otherwise commit the turn at the wrong
# distance. Before turning we require the front clearance to read inside a
# "reasonable range" -- between the target and WALL_APPROACH_BAND_MM nearer than it
# -- for WALL_APPROACH_CONFIRM_FRAMES consecutive ticks. Until then we keep creeping
# (if still beyond the band) or hold (if jittering / too close), and every check is
# logged so the real standoff at turn-time is visible. WALL_APPROACH_TIMEOUT_S is
# the backstop if it never settles.
WALL_APPROACH_BAND_MM = 50.0           # in-range window is [TARGET-BAND, TARGET], e.g. 375-425 mm
WALL_APPROACH_CONFIRM_FRAMES = 3       # consecutive in-range front reads required before turning
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

# Forward gap between the TWO wall approaches. After the turn following wall 1
# there is no wall in the forward cone yet; the robot advances this far before
# wall 2 comes into range and the second approach can run.
CP3_GAP_ADVANCE_DISTANCE_MM = 610.0    # ~1 course tile, open-loop forward move between wall 1 turn and wall 2

# Position sanity check after the turn onto the finish straightaway: with the
# robot squared to the wall at ~standoff then turned 90 deg right, the wall it
# re-zeroed against should now sit ~standoff away BEHIND and to the LEFT. These
# are a logged sanity check only -- they WARN if off but never block the drive
# (turn SIGN is UNVERIFIED, so a flipped sign shows up here as a large/inf read).
# WALL_FINISH_SANITY_TARGET_MM = 300.0 # ORIGINAL (revert here): tripped spuriously once the standoff went up
# WALL_FINISH_SANITY_TARGET_MM = 425.0 # prior: tracked the 425 standoff
WALL_FINISH_SANITY_TARGET_MM = 475.0   # expected rear/left clearance after the final turn (= the 475 mm standoff)
WALL_FINISH_SANITY_BAND_MM = 120.0     # warn (don't fail) if a reading is outside target +/- this (ok 355-595 mm)

# Finish-straightaway wall-following self-correction. While driving the cp3
# straightaway, steer gently toward whichever side wall has MORE room so the
# robot tracks down the middle instead of drifting into a wall. The yaw nudge is
# proportional to (left - right) clearance, capped, and gentle to avoid
# oscillation; applied only when BOTH side walls are visible (else drive straight).
# SIGN is UNVERIFIED: set_velocity is CCW-positive, but if it steers INTO the
# closer wall (left/right gap DIVERGES instead of converging), flip the sign.
CP3_STRAIGHTAWAY_CORRECTION_SIGN = 1               # flip to -1 if it corrects the wrong way
CP3_STRAIGHTAWAY_CORRECTION_KP_DEG_PER_MM = 0.05   # deg/s of yaw per mm of left-right imbalance
CP3_STRAIGHTAWAY_MAX_CORRECTION_DEG_S = 15.0       # cap on the yaw correction (gentle, no oscillation)


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
        slow_clearance_start_mm=LAPF_SLOW_CLEARANCE_START_MM,
        slow_clearance_stop_mm=LAPF_SLOW_CLEARANCE_STOP_MM,
        min_speed_frac=LAPF_MIN_SPEED_FRAC,
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


def directional_clearance_mm(
    robot: Robot,
    axis: str,
    half_width_mm: float = FRONT_CONE_HALF_WIDTH_MM,
    max_range_mm: float = FRONT_CLEARANCE_MAX_RANGE_MM,
) -> float:
    """Nearest raw-cloud return in a narrow body-frame cone along `axis`.

    Generalizes front_clearance_mm to the rear and sides for the cp3 finish
    position sanity check. axis is 'front' (+x), 'rear' (-x), 'left' (+y), or
    'right' (-y); the cone runs `max_range_mm` along that axis within
    +/- half_width_mm laterally. Same raw cloud (self-footprint excluded, robot
    body frame: fwd=+x, left=+y) as front_clearance_mm. Returns +inf when the
    cone is empty (e.g. wrong side / nothing there -> shows up as a warning in
    the caller, which is informative given the unverified turn sign).
    """
    nearest = math.inf
    for px, py in robot.get_obstacles():
        if axis == "front":
            along, lateral = px, py
        elif axis == "rear":
            along, lateral = -px, py
        elif axis == "left":
            along, lateral = py, px
        elif axis == "right":
            along, lateral = -py, px
        else:
            raise ValueError(f"directional_clearance_mm: bad axis {axis!r}")
        if along <= 0.0 or along > max_range_mm:
            continue
        if abs(lateral) > half_width_mm:
            continue
        if along < nearest:
            nearest = along
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


def _log_wall_diagnostics(cloud: list[tuple[float, float]]) -> None:
    """One-shot dump of the LiDAR cloud by robot-frame sector, to reveal the
    true wall standoffs and orientations at the bridge mouth for tuning.

    fwd = +x, left = +y. For each sector prints the point count, the nearest
    return (the standoff to put in BRIDGE_*_WALL_DIST_MM), and the line-fit
    angle (a clean parallel/perpendicular wall reads near 0 / +/-90).
    """
    sectors = {
        "front (+x)": [(x, y) for (x, y) in cloud if x > 0 and abs(y) <= abs(x)],
        "rear  (-x)": [(x, y) for (x, y) in cloud if x < 0 and abs(y) <= abs(x)],
        "left  (+y)": [(x, y) for (x, y) in cloud if y > 0 and abs(x) <= abs(y)],
        "right (-y)": [(x, y) for (x, y) in cloud if y < 0 and abs(x) <= abs(y)],
    }
    print(f"[bridge-square] scan dump: {len(cloud)} pts total")
    for name, pts in sectors.items():
        if not pts:
            print(f"[bridge-square]   {name}:   0 pts")
            continue
        nearest = min(math.hypot(x, y) for (x, y) in pts)
        angle = _fit_line_angle_deg(pts)
        angle_s = f"{angle:+6.1f}" if angle is not None else "   n/a"
        print(f"[bridge-square]   {name}: {len(pts):3d} pts  "
              f"nearest={nearest:6.0f} mm  fit_angle={angle_s} deg")


def _side_wall_error_deg(cloud: list[tuple[float, float]], side: int) -> float | None:
    """Heading error (deg) from one side wall, which runs parallel to travel.

    side = +1 selects the left wall (+y), -1 the right wall (-y). Points are
    taken in a 45-deg wedge toward that side (|x| <= |y|, so a wall must be more
    lateral than forward to count — this isolates the side wall from the
    front/rear walls). The wall standoff is auto-detected as the median lateral
    offset, then only the dominant slab within +/- BRIDGE_WALL_BAND_MM is fit,
    rejecting corner leakage at other distances.

    A parallel wall fits to ~0 deg when the robot is squared, so the fit angle
    IS the heading error. Returns None if there aren't enough clean points.
    """
    cand = [
        (x, y) for (x, y) in cloud
        if (y > 0.0 if side > 0 else y < 0.0)
        and abs(x) <= abs(y)
        and abs(x) <= BRIDGE_WALL_SAMPLE_DEPTH_MM
    ]
    if len(cand) < BRIDGE_SQUARE_MIN_POINTS:
        return None
    lateral = sorted(abs(y) for _, y in cand)
    wall_d = lateral[len(lateral) // 2]  # median standoff = the dominant wall
    pts = [(x, y) for (x, y) in cand if abs(abs(y) - wall_d) <= BRIDGE_WALL_BAND_MM]
    if len(pts) < BRIDGE_SQUARE_MIN_POINTS:
        return None
    return _fit_line_angle_deg(pts)


def estimate_bridge_heading_error_deg(robot: Robot) -> float:
    """Heading error (deg) at the bridge mouth, measured against the side walls.

    A positive result means turn_by(-error) squares the robot onto the bridge
    axis (SIGN UNVERIFIED — confirm on the venue). The two side walls run
    parallel to travel and are independent references: each fits to ~0 deg when
    squared, so each fit angle is a heading-error estimate. When both are
    visible they must agree (cross-check) and are averaged; when only one is
    visible it is used alone.

    Returns 0.0 (caller should not turn) whenever the read is ambiguous: no
    usable side wall, the two walls disagreeing, or an implausibly large fit.
    Corrects HEADING only; lateral offset in the lane is not observed here.
    """
    if not BRIDGE_SQUARE_ENABLED:
        return 0.0

    cloud = robot.get_obstacles()  # robot frame, mm: fwd=+x, left=+y

    if BRIDGE_SQUARE_DEBUG:
        _log_wall_diagnostics(cloud)

    left_err = _side_wall_error_deg(cloud, +1)
    right_err = _side_wall_error_deg(cloud, -1)
    estimates = [(name, e) for name, e in (("left", left_err), ("right", right_err))
                 if e is not None]
    if not estimates:
        print(f"[bridge-square] no usable side wall (<{BRIDGE_SQUARE_MIN_POINTS} pts "
              "either side); skipping correction")
        return 0.0

    if len(estimates) == 2:
        if abs(left_err - right_err) > BRIDGE_SQUARE_CROSSCHECK_DEG:
            print(f"[bridge-square] left ({left_err:+.1f}) and right ({right_err:+.1f}) "
                  f"disagree > {BRIDGE_SQUARE_CROSSCHECK_DEG}; skipping correction")
            return 0.0
        error_deg = 0.5 * (left_err + right_err)
        print(f"[bridge-square] left={left_err:+.1f} right={right_err:+.1f} "
              f"-> error {error_deg:+.1f} deg")
    else:
        name, error_deg = estimates[0]
        print(f"[bridge-square] only {name} wall visible -> error {error_deg:+.1f} deg")

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

    front = front_clearance_mm(robot)
    rear = rear_clearance_mm(robot)
    if front > RECOVERY_REVERSE_FRONT_TRIGGER_MM:
        # Nothing close dead ahead -> this is a force-null stall between cones,
        # not an imminent collision. Reversing would just re-enter the same
        # symmetric minimum and net the robot backward, so skip it entirely and
        # rely on the reorient + lateral goal perturbation below.
        reverse_dist = 0.0
    elif math.isinf(rear):
        reverse_dist = RECOVERY_REVERSE_MM
    elif rear >= RECOVERY_MIN_REAR_CLEARANCE_MM:
        reverse_dist = min(RECOVERY_REVERSE_MM, rear - RECOVERY_REVERSE_MARGIN_MM)
    else:
        reverse_dist = 0.0

    if reverse_dist >= RECOVERY_MIN_REVERSE_MM:
        print(
            f"[FSM] recovery {av['retry_count']}/{MAX_AVOIDANCE_RETRIES}: "
            f"reversing {reverse_dist:.0f} mm (front {front:.0f} mm, rear clearance="
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
        f"skipping reverse (front {front:.0f} mm, rear "
        f"{'inf' if math.isinf(rear) else f'{rear:.0f} mm'}) - reorienting only"
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
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                if START_AT_CP2:
                    # TEMP/testing-only path (START_AT_CP2): skip the scripted
                    # cp1->bridge->cp2 steps and jump straight into the
                    # cp2->cp3 obstacle-avoidance leg. reset_mission_pose() sets
                    # odometry to (0,0, INITIAL_THETA_DEG=+Y), i.e. the cp2 start
                    # pose, so begin_avoidance_segment aims the goal straight up
                    # the obstacle course. Place the robot at the cp2 spot facing
                    # +Y before pressing BTN_1. Set START_AT_CP2=False to restore
                    # the full scripted run.
                    reset_mission_pose(robot)
                    show_running_leds(robot)
                    step_index = len(MISSION_STEPS)
                    av = {}
                    print("[FSM] TEMP START_AT_CP2 - skipping scripted steps; "
                          "starting obstacle avoidance toward checkpoint 3")
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
                    # turn fires only when BOTH drift-free guards hold:
                    #   1. standoff: live LiDAR reads something <= WALL_DETECT_STANDOFF_MM
                    #      straight ahead; and
                    #   2. min advance: at least CHECKPOINT_3_MIN_ADVANCE_MM (4 tiles)
                    #      of forward progress along the approach axis, so a cone in
                    #      the lower field can't trigger it and the turn can't fire
                    #      early. Until advance clears the floor the approach keeps
                    #      running.
                    # No goal-coordinate odometry distance is in this decision any
                    # more (the old WALL_ARM_REMAINING_MM arm gate is gone). The
                    # cp3->cp4 segment leaves terminate_on_wall False and keeps the
                    # goal-coordinate termination handled above.
                    detect_wall = av.get("terminate_on_wall")
                    front = front_clearance_mm(robot) if detect_wall else math.inf
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
                    "[FSM] squared up to wall 1 - approaching to "
                    f"{WALL_APPROACH_TARGET_MM:.0f} mm standoff"
                )
                motion_handle = None
                av["wall_approach_num"] = 1          # first of two wall approaches
                av["wall_approach_started_at"] = now
                av["wall_inrange_count"] = 0
                last_status_print_at = now
                state = "CP3_APPROACH_WALL"

        elif state == "CP3_APPROACH_WALL":
            # Closed-loop creep toward the wall until the front cone reads the
            # target standoff. Velocity-controlled (no MotionHandle). Timeout
            # DISABLED (see below) per request: it keeps creeping until the wall
            # appears and the standoff is reached; BTN_2 is the only stop.
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop()
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE - cp3 wall approach cancelled")
                state = "IDLE"
            else:
                front = front_clearance_mm(robot)
                in_lower = WALL_APPROACH_TARGET_MM - WALL_APPROACH_BAND_MM
                # "In range" only when the front cone reads a plausible final standoff:
                # at/under the target but no nearer than the band's lower edge. This is
                # the checker that keeps the variable stop distance honest before turning.
                in_range = in_lower <= front <= WALL_APPROACH_TARGET_MM

                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    x, y, theta = robot.get_pose()
                    front_str = "inf" if math.isinf(front) else f"{front:.0f}"
                    print(
                        f"  cp3 wall approach odom=({x:6.0f}, {y:6.0f}) mm "
                        f"theta={theta:5.1f} deg front={front_str} mm "
                        f"(want {in_lower:.0f}-{WALL_APPROACH_TARGET_MM:.0f} mm, "
                        f"in-range {av['wall_inrange_count']}/{WALL_APPROACH_CONFIRM_FRAMES})"
                    )
                    last_status_print_at = now

                if in_range:
                    # In the reasonable standoff band: hold and require a few
                    # consecutive in-range reads so a single noisy frame can't fire
                    # the turn at the wrong distance.
                    robot.stop()
                    av["wall_inrange_count"] += 1
                    if av["wall_inrange_count"] >= WALL_APPROACH_CONFIRM_FRAMES:
                        print(
                            f"[FSM] wall {av.get('wall_approach_num', 1)} standoff confirmed "
                            f"at {front:.0f} mm (within {in_lower:.0f}-{WALL_APPROACH_TARGET_MM:.0f} mm "
                            f"for {WALL_APPROACH_CONFIRM_FRAMES} reads) - turning"
                        )
                        motion_handle = robot.turn_by(
                            CP3_FACE_STRAIGHTAWAY_TURN_DEG,
                            tolerance_deg=TURN_TOLERANCE_DEG,
                            blocking=False,
                        )
                        last_status_print_at = now
                        state = "CP3_FACE_STRAIGHTAWAY"
                # TIMEOUT DISABLED per request ("get rid of the timeout timer"): the
                # approach no longer safe-stops if the wall takes a while to appear; it
                # keeps creeping (incl. the inf case below) until it reaches the standoff.
                # BTN_2 is the only stop now. Re-enable by uncommenting this branch AND
                # restoring the inf-case hold below (WALL_APPROACH_TIMEOUT_S still defined).
                # ORIGINAL (revert here):
                # elif (now - av["wall_approach_started_at"]) >= WALL_APPROACH_TIMEOUT_S:
                #     safe_stop_to_idle(
                #         robot,
                #         "cp3 wall approach timed out - front never settled in the "
                #         f"{in_lower:.0f}-{WALL_APPROACH_TARGET_MM:.0f} mm range "
                #         "(check detection / standoffs / turn sign)",
                #     )
                #     state = "IDLE"
                elif math.isfinite(front) and front < in_lower:
                    # Nearer than the band's lower edge: too close to creep further.
                    # Hold and keep sampling - jitter may settle back into range; if it
                    # stays this close the timeout above safe-stops rather than turning
                    # at a bad standoff.
                    robot.stop()
                    av["wall_inrange_count"] = 0
                elif math.isfinite(front):
                    # Still beyond the band (front > target): creep closer.
                    av["wall_inrange_count"] = 0
                    robot.set_velocity(WALL_APPROACH_SPEED_MM_S, 0.0)
                else:
                    # Wall not in the forward cone yet: KEEP GOING (creep forward) so
                    # it closes the gap until the wall comes into range, instead of
                    # holding. This is what was freezing the wall-2 approach (front=inf).
                    # ORIGINAL (revert here): held in place until the (now-disabled)
                    # timeout fired:
                    #   av["wall_inrange_count"] = 0
                    #   robot.stop()
                    av["wall_inrange_count"] = 0
                    robot.set_velocity(WALL_APPROACH_SPEED_MM_S, 0.0)

        elif state == "CP3_FACE_STRAIGHTAWAY":
            # Post-wall turn (turn started on entry). After wall 1 this turn faces
            # the forward gap -> advance CP3_GAP_ADVANCE_DISTANCE_MM before wall 2 is
            # in range. After wall 2 it faces the finish straightaway -> verify+drive.
            # ORIGINAL (revert here): single wall, this always went straight to verify:
            #   elif motion_handle is not None and motion_handle.is_finished():
            #       motion_handle = None
            #       print("[FSM] facing finish straightaway - verifying path is clear")
            #       state = "CP3_VERIFY_STRAIGHTAWAY"
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE - cp3 post-wall turn cancelled")
                state = "IDLE"
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                if av.get("wall_approach_num", 1) == 1:
                    # No wall in front yet right after the wall-1 turn: drive the gap
                    # first, then approach wall 2.
                    print(
                        "[FSM] turned after wall 1 - advancing "
                        f"{CP3_GAP_ADVANCE_DISTANCE_MM:.0f} mm before wall 2 comes into range"
                    )
                    motion_handle = robot.move_forward(
                        CP3_GAP_ADVANCE_DISTANCE_MM,
                        velocity=DRIVE_VELOCITY_MM_S,
                        tolerance=DRIVE_TOLERANCE_MM,
                        blocking=False,
                    )
                    last_status_print_at = now
                    state = "CP3_GAP_ADVANCE"
                else:
                    print("[FSM] turned after wall 2 - facing finish straightaway, verifying path is clear")
                    state = "CP3_VERIFY_STRAIGHTAWAY"

        elif state == "CP3_GAP_ADVANCE":
            # Forward gap between the two wall approaches: drive ~1 tile so wall 2
            # enters the forward cone, then run the second approach to the same
            # 425 mm standoff. Open-loop move_forward (MotionHandle).
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE - cp3 gap advance cancelled")
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    x, y, theta = robot.get_odometry_pose()
                    front = front_clearance_mm(robot)
                    front_str = "inf" if math.isinf(front) else f"{front:.0f}"
                    print(
                        f"  cp3 gap advance odom=({x:6.0f}, {y:6.0f}) mm "
                        f"theta={theta:5.1f} deg front={front_str} mm"
                    )
                    last_status_print_at = now

                if motion_handle is not None and motion_handle.is_finished():
                    print(
                        "[FSM] gap advance done - approaching wall 2 to "
                        f"{WALL_APPROACH_TARGET_MM:.0f} mm standoff"
                    )
                    motion_handle = None
                    av["wall_approach_num"] = 2          # second of two wall approaches
                    av["wall_approach_started_at"] = now
                    av["wall_inrange_count"] = 0
                    last_status_print_at = now
                    state = "CP3_APPROACH_WALL"

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
                # Position sanity check (LOGGED; WARN only, never blocks). After the
                # 90 deg right turn onto the straightaway the re-zero wall should sit
                # ~target away BEHIND and to the LEFT. Turn SIGN is UNVERIFIED, so a
                # flipped sign shows up here as an out-of-band / inf reading -> warn,
                # but still defer the go/no-go to the front-clear gate below.
                rear = directional_clearance_mm(robot, "rear")
                left = directional_clearance_mm(robot, "left")
                lo = WALL_FINISH_SANITY_TARGET_MM - WALL_FINISH_SANITY_BAND_MM
                hi = WALL_FINISH_SANITY_TARGET_MM + WALL_FINISH_SANITY_BAND_MM
                rear_str = "inf" if math.isinf(rear) else f"{rear:.0f}"
                left_str = "inf" if math.isinf(left) else f"{left:.0f}"
                print(
                    f"[FSM] cp3 position check: rear={rear_str} mm left={left_str} mm "
                    f"(want ~{WALL_FINISH_SANITY_TARGET_MM:.0f} mm each, ok {lo:.0f}-{hi:.0f} mm)"
                )
                for name, val in (("rear", rear), ("left", left)):
                    if not (lo <= val <= hi):
                        val_str = "inf" if math.isinf(val) else f"{val:.0f}"
                        print(
                            f"[FSM] WARN cp3 position: {name}={val_str} mm outside "
                            f"{lo:.0f}-{hi:.0f} mm - check turn sign/standoff "
                            "(continuing, not aborting)"
                        )

                front = front_clearance_mm(robot)
                front_str = "inf" if math.isinf(front) else f"{front:.0f}"
                if front >= CLEAR_PATH_MIN_MM:
                    print(
                        f"[FSM] straightaway clear (nearest {front_str} mm) - "
                        f"driving {CHECKPOINT_3_FINAL_STRAIGHT_DISTANCE_MM:.0f} mm to checkpoint 3 "
                        "with wall-following"
                    )
                    # Velocity-controlled wall-following drive (no MotionHandle);
                    # the CP3_DRIVE_STRAIGHTAWAY loop steers + tracks distance.
                    # ORIGINAL (revert here): open-loop fixed-distance move_forward:
                    #   motion_handle = robot.move_forward(
                    #       CHECKPOINT_3_FINAL_STRAIGHT_DISTANCE_MM,
                    #       velocity=DRIVE_VELOCITY_MM_S, tolerance=DRIVE_TOLERANCE_MM,
                    #       blocking=False)
                    motion_handle = None
                    av["drive_start_mm"] = robot.get_odometry_pose()[:2]
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
            # Velocity-controlled wall-following drive to the cp3 checkpoint. Each
            # tick reads the left/right side walls and steers gently toward the side
            # with MORE room so the robot tracks down the middle instead of drifting
            # into a wall; the rear wall is read as a progress/sanity reference. No
            # MotionHandle (own velocity loop), so BTN_2 is polled every tick and the
            # LAPF watchdog does not apply. Terminates on odometry distance.
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop()
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE - cp3 straightaway drive cancelled")
                state = "IDLE"
            else:
                x, y, theta = robot.get_odometry_pose()
                sx, sy = av.get("drive_start_mm", (x, y))
                traveled = math.hypot(x - sx, y - sy)
                remaining = CHECKPOINT_3_FINAL_STRAIGHT_DISTANCE_MM - traveled

                if remaining <= 0.0:
                    robot.stop()
                    print(
                        f"[FSM] CHECKPOINT 3 reached (drove {traveled:.0f} mm) - "
                        "starting checkpoint 4 obstacle avoidance"
                    )
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
                else:
                    left = directional_clearance_mm(robot, "left")
                    right = directional_clearance_mm(robot, "right")
                    behind = directional_clearance_mm(robot, "rear")
                    # Center between the side walls: steer toward the side with more
                    # room (positive error = more room on the left). Only when BOTH
                    # walls are seen; off a single wall we can't center, so go straight.
                    if math.isfinite(left) and math.isfinite(right):
                        error = left - right
                        correction = (
                            CP3_STRAIGHTAWAY_CORRECTION_SIGN
                            * CP3_STRAIGHTAWAY_CORRECTION_KP_DEG_PER_MM
                            * error
                        )
                        correction = max(
                            -CP3_STRAIGHTAWAY_MAX_CORRECTION_DEG_S,
                            min(CP3_STRAIGHTAWAY_MAX_CORRECTION_DEG_S, correction),
                        )
                    else:
                        error = None
                        correction = 0.0
                    robot.set_velocity(DRIVE_VELOCITY_MM_S, correction)

                    if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                        left_str = "inf" if math.isinf(left) else f"{left:.0f}"
                        right_str = "inf" if math.isinf(right) else f"{right:.0f}"
                        behind_str = "inf" if math.isinf(behind) else f"{behind:.0f}"
                        err_str = "n/a" if error is None else f"{error:+.0f}"
                        print(
                            f"  cp3 straightaway: remaining={remaining:5.0f} mm "
                            f"left={left_str} right={right_str} behind={behind_str} mm "
                            f"err(L-R)={err_str} mm corr={correction:+.1f} deg/s "
                            f"(traveled {traveled:.0f}/{CHECKPOINT_3_FINAL_STRAIGHT_DISTANCE_MM:.0f})"
                        )
                        last_status_print_at = now

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
