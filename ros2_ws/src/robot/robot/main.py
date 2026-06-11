"""
main.py - checkpoint 1->5 mission FSM
=====================================
CONTROLS: the route starts from IDLE on a GREEN traffic light (vision-triggered
auto-start, see GREEN_LIGHT_AUTO_START) OR a BTN_1 press (manual fallback). BTN_2
cancels any active motion (including mid-recovery) and returns to IDLE.

Route -- an odometry-only scripted drive to checkpoint 2, then LiDAR/LAPF
obstacle avoidance for the later sections:

  CP1/CP2 (scripted, odometry only):
    drive -> turn R -> drive -> turn R           (cp1: facing the bridge mouth)
    square to bridge axis -> cross bridge ->
    turn L -> drive -> turn L                     (cp2: facing the cones)
  CP3 (LiDAR): obstacle-avoid up the field (LAPF), square to the outer wall and
    approach a fixed standoff (re-zero vs an absolute reference, not drifted
    odometry), turn onto the finish straightaway, verify it is clear, drive it.
  CP4 (LiDAR): obstacle-avoid straight 5 tiles, then stop.
  CP5: manipulator placeholder (scaffold only).

LAPF watchdog/recovery (cp3 & cp4): a LAPF segment can stall in a local minimum,
and the planner loop only ends on goal-or-cancel, so each segment is watched by
a no-progress detector (authoritative) plus a wall-clock cap (last resort). On a
stall the FSM runs a bounded, symmetry-breaking, rear-guarded back-up-and-retry,
then safe-stops to IDLE once the retry budget is spent.
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


# ===========================================================================
# CONTROLS & DRIVE BASICS
# ===========================================================================
# Wheel/motor wiring (preserved from working28/main28backup behavior).
LEFT_WHEEL_MOTOR = 2
LEFT_WHEEL_DIR_INVERTED = True
RIGHT_WHEEL_MOTOR = 1
RIGHT_WHEEL_DIR_INVERTED = False

# Default scripted-drive speeds/tolerances.
DRIVE_VELOCITY_MM_S = 140.0
DRIVE_TOLERANCE_MM = 60.0
TURN_TOLERANCE_DEG = 3.0
STATUS_PRINT_INTERVAL_S = 0.5

# TEMP / TESTING-ONLY: True skips the cp1->bridge->cp2 scripted drive -- BTN_1
# resets odometry to the cp2 start pose (0,0, +Y) and jumps straight into the
# cp2->cp3 leg, so the LiDAR weave can be tested without the full approach.
# Place the robot at the cp2 spot facing +Y first.  >>> False for a full run <<<
START_AT_CP2 = True

# ---------------------------------------------------------------------------
# GREEN-LIGHT AUTO-START (vision-triggered race start)
# ---------------------------------------------------------------------------
# The course starts on a RED light that turns GREEN. Instead of pressing BTN_1
# we watch the vision node's traffic-light detections and auto-start when a
# green light is confirmed. BTN_1 still works as a manual fallback (either one
# starts the mission). Requires `ros2 run vision vision_node` in another
# terminal (it publishes /vision/detections, which robot.enable_vision() reads).
#
# Set GREEN_LIGHT_AUTO_START = False to revert to BTN_1-only starting.
GREEN_LIGHT_AUTO_START = True
# A green reading must persist this many consecutive FSM frames before we go --
# rejects a single mis-classified frame from false-starting the run.
GREEN_START_CONFIRM_FRAMES = 5
# Ignore traffic-light detections weaker than this (0..1 YOLO confidence).
GREEN_START_MIN_CONFIDENCE = 0.50
# Treat vision as unavailable if no detections have arrived within this window.
GREEN_START_VISION_STALE_SEC = 3.0
# While idling, print what the camera currently sees (red / green / none) this
# often, so we can confirm the traffic-light detection is alive on the field.
# The readout persists until a green light or BTN_1 starts the mission.
IDLE_LIGHT_PRINT_INTERVAL_S = 2.0

# Calibrated command for a physical 90-deg turn: a raw 90.0 overshoots by ~10
# deg, so this is reduced to land on a true right angle. One knob for ALL
# scripted right-angle turns (cp1/2 sequence + cp3 terminal turns) -- lower if
# it still turns too far, raise if it falls short.
RIGHT_ANGLE_TURN_DEG = 82.0

# ---------------------------------------------------------------------------
# COLLISION-AWARE TURNS (guarded pivot)
# ---------------------------------------------------------------------------
# A scripted 90-deg turn is an in-place PIVOT: the base spins about its center
# at zero forward velocity (navigation._turn_to_heading). Its CORNERS sweep a
# circle of radius FOOTPRINT_HALF_DIAG_MM (center -> farthest corner), so any
# obstacle inside that circle gets clipped no matter how it turns -- the turn
# itself can't dodge, the robot has to first translate away. guarded_turn_by()
# checks the swept circle against the live LiDAR cloud and, if blocked, NUDGES
# fore/aft to open room, then pivots. If it still can't clear, it FORCES the
# turn anyway and prints a loud warning -- by design, so a tight course turn is
# never silently skipped. Flip PIVOT_GUARD_ENABLED to False to revert to the
# old blind turns instantly on the field.
PIVOT_GUARD_ENABLED = True              # master switch (False = plain blind turns)
FOOTPRINT_HALF_DIAG_MM = 400.0          # robot center -> farthest corner (MEASURED)
PIVOT_CLEARANCE_MARGIN_MM = 40.0        # safety pad added to the swept radius
PIVOT_MAKE_ROOM_NUDGE_MM = 100.0        # per-attempt fore/aft nudge to open room
PIVOT_MAKE_ROOM_MAX_TRIES = 3           # nudges before forcing the turn
PIVOT_MAKE_ROOM_SPEED_MM_S = 80.0       # speed of the make-room nudges
PIVOT_MAKE_ROOM_TOLERANCE_MM = 25.0     # arrival tolerance for a nudge move

# ---------------------------------------------------------------------------
# WAYPOINT CORRECTION (snap back to the ideal map pose after a disrupted turn)
# ---------------------------------------------------------------------------
# Each scripted step has an IDEAL end pose (IDEAL_POSES, computed by integrating
# the commanded steps from the start). In a perfect run odometry lands exactly
# there; a make-room nudge (or a wall bumping the turn) shifts the robot off the
# ideal line. On flagged steps we MEASURE the odometry pose, compare to the ideal
# waypoint, and CORRECT back onto it (turn toward it -> drive -> re-face) before
# continuing. CAVEAT: odometry is the only position sense here (no GPS/absolute
# ref), so this fixes the nudge/bump displacement it can see, NOT true wheel slip
# -- which is why it's scoped to the short PRE-BRIDGE corner (where odom is still
# trustworthy) and the downstream legs stay wall/detection-referenced.
WAYPOINT_CORRECTION_ENABLED = True       # master switch
WAYPOINT_POS_TOL_MM = 60.0               # within this of ideal -> no position fix
WAYPOINT_MAX_CORRECTION_MM = 500.0       # refuse to "correct" a wild deviation (suspect odom)
WAYPOINT_HEADING_TOL_DEG = 4.0           # within this of ideal heading -> no re-face
WAYPOINT_CORRECTION_SPEED_MM_S = 100.0   # speed of the corrective drive
WAYPOINT_CORRECTION_TIMEOUT_S = 8.0      # per-move cap so a correction can't hang

# ===========================================================================
# CHECKPOINT 1 & 2 LOGIC  (scripted approach + bridge crossing, odometry only)
# ===========================================================================
# Scripted distances -- EMPIRICALLY HAND-TUNED on the physical bridge/ramp, not
# derived from the course grid. Don't change without re-tuning on the venue; the
# cp1/2 sequence is meant to stay behaviorally fixed.
CHECKPOINT_1_APPROACH_DISTANCE_MM = 2800.0   # start -> checkpoint 1 approach point
BRIDGE_ALIGN_DISTANCE_MM = 500.0             # short nudge into the bridge lane
BRIDGE_CROSS_DISTANCE_MM = 2350.0            # length of the bridge/ramp crossing
BRIDGE_EXIT_DISTANCE_MM = 600.0              # post-bridge corner hop (hand-tuned, not a tile)

# --- Bridge-entry wall-square correction -----------------------------------
# The bridge is crossed open-loop over a long distance, so heading error from
# the two preceding turns becomes a large lateral deviation (~= cross_dist *
# sin(theta_err)). After the 2nd turn the LiDAR sees both SIDE walls of the lane
# (parallel to the crossing); we fit each and turn to null their tilt off the
# forward axis. Standoff is auto-detected, so no measured distance is needed.
# Corrects HEADING only, not lateral offset.
#
# SIGN is UNVERIFIED -- bench-test before trusting it. Deliberately conservative:
# does NOTHING when the scan is ambiguous (no usable wall, L/R disagree, or fit
# too large), so a noisy read can't make the crossing worse.
BRIDGE_SQUARE_ENABLED = True
BRIDGE_SQUARE_DEBUG = True                 # dump the per-sector scan at square-up (tuning aid)
BRIDGE_WALL_BAND_MM = 250.0               # keep points within +/- this of the auto-detected standoff
BRIDGE_WALL_SAMPLE_DEPTH_MM = 1200.0      # along-wall window of points to fit (|x| <= this)
BRIDGE_SQUARE_MIN_POINTS = 8              # minimum inliers to trust a line fit
BRIDGE_SQUARE_DEADBAND_DEG = 1.5          # below this error, don't bother turning
BRIDGE_SQUARE_MAX_CORRECTION_DEG = 15.0   # refuse larger corrections (guards a bad fit)
BRIDGE_SQUARE_CROSSCHECK_DEG = 6.0        # left vs right estimates must agree within this

# ===========================================================================
# CHECKPOINT 3 & 4 LOGIC  (LiDAR / LAPF obstacle avoidance)
# ===========================================================================
# Course distances. The UI/course grid uses 610 mm cells (WorldCanvas.tsx).
COURSE_TILE_MM = 610.0
CHECKPOINT_3_APPROACH_TILES = 5.0
CHECKPOINT_3_FINAL_STRAIGHT_TILES = 1.0
CHECKPOINT_4_STRAIGHT_TILES = 5.0

CHECKPOINT_3_APPROACH_DISTANCE_MM = COURSE_TILE_MM * CHECKPOINT_3_APPROACH_TILES
# CHECKPOINT_3_FINAL_STRAIGHT_DISTANCE_MM = COURSE_TILE_MM * CHECKPOINT_3_FINAL_STRAIGHT_TILES  # ORIGINAL: 610 mm (1 tile)
# CHECKPOINT_3_FINAL_STRAIGHT_DISTANCE_MM = 2600.0  # prior: full straightaway, stopped at the end wall
CHECKPOINT_3_FINAL_STRAIGHT_DISTANCE_MM = 2525.0  # 2600 to the checkpoint, -75 mm backoff from the end wall
CHECKPOINT_4_DISTANCE_MM = COURSE_TILE_MM * CHECKPOINT_4_STRAIGHT_TILES

# --- LAPF tuning (checkpoint 2+ obstacle-avoidance runs) ---
# Knob legend (raise/lower effect):
#   SPEED            lower if it reacts too late or slips
#   TOLERANCE        larger = looser checkpoint arrival
#   LEASH_LENGTH     how far the virtual target runs ahead (smaller=cautious)
#   REPULSION_RANGE  how early obstacles bend the target (larger=avoid earlier)
#   TARGET_SPEED     virtual-target motion speed
#   REPULSION_GAIN   obstacle push strength (too high oscillates, too low clips)
#   FORCE_EMA_ALPHA  force smoothing (lower=smoother/slower to react)
#   INFLATION_MARGIN extra keep-out radius per obstacle (larger=more clearance)
#   LEASH_HALF_ANGLE forward cone for the target (smaller=straighter)
OBSTACLE_AVOIDANCE_SPEED_MM_S = 140.0
OBSTACLE_AVOIDANCE_TOLERANCE_MM = 60.0
OBSTACLE_AVOIDANCE_MAX_ANGULAR_RAD_S = 1.0
LAPF_LEASH_LENGTH_MM = 500.0
LAPF_REPULSION_RANGE_MM = 300.0
LAPF_TARGET_SPEED_MM_S = 200.0
LAPF_REPULSION_GAIN = 550.0
# Attraction gain: commit-forward bias (drive up through the gap, don't balance
# L/R cones). If it CLIPS cones back toward ~2.5; if it FREEZES, raise.
# LAPF_ATTRACTION_GAIN = 1.0           # revert here: froze in the force-null between cones
LAPF_ATTRACTION_GAIN = 3.0
LAPF_FORCE_EMA_ALPHA = 0.35

# --- Cone keep-out + arc-not-pivot (COUPLED -- inflation, half-width & leash) ---
# The planner now bakes the robot half-width (200 mm) into the keep-out:
#   eff_radius = r_track + LAPF_INFLATION_MARGIN_MM + 200
# so INFLATION is now a TRUE safety margin only -- it EQUALS the body-edge-to-cone
# clearance once r_track ~= the physical cone radius.
# HARD geometry at 75 mm cones / 610 mm spacing (only ~30 mm/side of slack exists):
#   point-corridor open  : 2*eff_radius < 610  -> eff_radius < 305 -> inflation < 30
#   leash reach          : 500*sin(45)=354 >= eff_radius
#   inflation=25 -> eff_radius=300 -> corridor +10 mm, body clearance ~25 mm/side
# >>> Raising inflation past ~30 (or the cone clamp past 55) CLOSES the corridor
#     and the planner can't thread. Centering precision matters more than margin. <<<
LAPF_INFLATION_MARGIN_MM = 25.0          # = body-edge-to-cone clearance per side (~25 mm; 30 mm is the physical max)
# LAPF_LEASH_HALF_ANGLE_DEG = 50.0      # revert here: wide leash -> target off-axis -> pivot
LAPF_LEASH_HALF_ANGLE_DEG = 45.0        # steering authority to swing around a dead-ahead cone

# --- Forward-clearance throttle (Fix 3): "see cone -> steer -> confirm clear -> go" ---
# Scales LINEAR speed only: near a dead-ahead cone it keeps turning hard toward
# the committed open side while forward speed drops, then ramps back as the
# corridor clears -> arcs tightly around the cone instead of nosing in. Floored
# (not zero) so it never fully freezes -- keeps creeping while it swings off.
LAPF_SLOW_CLEARANCE_START_MM = 450.0   # begin easing off when nearest cone edge ahead <= this
LAPF_SLOW_CLEARANCE_STOP_MM = 150.0    # most-slowed (floor) by this edge clearance
# LAPF_MIN_SPEED_FRAC = 1.0            # revert here: Fix 3 OFF (no throttle)
LAPF_MIN_SPEED_FRAC = 0.35             # floor 35% so it keeps creeping while swinging off the cone

# ---------------------------------------------------------------------------
# --- LAPF stall watchdog + recovery (checkpoint 2+ avoidance segments) ---
#
# Detection (per LAPF attempt):
#   - No-progress (AUTHORITATIVE): best dist-to-goal not improving by >= EPS
#     within WINDOW -> recover + retry.
#   - Per-attempt wall-clock (LAST RESORT): catches a slow orbit that nets some
#     progress but never converges; scales with remaining distance. -> safe-stop,
#     NOT retried (retrying an orbit doesn't help).
#   - Segment hard ceiling: global backstop over the whole segment. -> safe-stop.
#
# Recovery (bounded, symmetry-breaking, rear-guarded): re-issuing the IDENTICAL
# goal would walk back into the same minimum, so each retry (a) reverses for
# clearance (guarded), (b) reorients by an alternating offset, (c) re-issues
# toward a laterally-shifted goal. Perturbing heading + goal breaks the
# collinearity that creates the force null; the side alternates per retry.
# Blind reverse is unmodeled, so it's gated on a rear-sector clearance check and
# a hard cap; if the rear isn't clear it skips to reorient-only (or safe-stop).
NO_PROGRESS_WINDOW_S = 4.0
NO_PROGRESS_EPS_MM = 50.0
WALLCLOCK_CAP_FACTOR = 1.5
WALLCLOCK_CAP_FLOOR_S = 12.0
SEGMENT_HARD_CEILING_S = 35.0
MAX_AVOIDANCE_RETRIES = 3

# RECOVERY_REVERSE_MM = 300.0            # revert here: exceeded ~150 mm forward gain -> walked backward
RECOVERY_REVERSE_MM = 100.0              # nominal back-up distance (small, so a retry can't net rearward)
# Only reverse when about to hit something dead ahead. A force-null stall BETWEEN
# cones (nothing close in front) must NOT reverse -- that just re-enters the same
# minimum; reorient + lateral goal perturbation break the symmetry instead.
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
# Guaranteed physical displacement per retry. A between-cones force-null stall
# triggers only the 25 deg reorient pivot (no translation) + a goal shift, so the
# next LAPF run can re-walk into the SAME minimum. After the reorient turn the
# robot is angled toward recovery_sign's open side; a short forward hop then
# translates it off the stalled line onto a different approach line. Alternates
# L/R via recovery_sign. (Diff-drive can't strafe, so this is a forward-lateral
# dogleg; lateral component ~= nudge*sin(RECOVERY_HEADING_OFFSET_DEG).)
RECOVERY_LATERAL_NUDGE_MM = 125.0        # forward hop after reorient (100-150 mm)
RECOVERY_NUDGE_SPEED_MM_S = 100.0
RECOVERY_NUDGE_TOLERANCE_MM = 40.0
RECOVERY_NUDGE_TIMEOUT_S = 3.0
RECOVERY_NUDGE_MIN_FRONT_MM = 250.0      # skip the hop if less than this is clear ahead
REAR_SECTOR_HALF_ANGLE_DEG = 60.0        # +/- around directly-behind

# Slack over the LAPF goal tolerance when confirming a finished LAPF handle
# actually reached the goal (vs the motion thread ending early).
SUCCESS_MARGIN_MM = 20.0

# ---------------------------------------------------------------------------
# --- CHECKPOINT 3 wall-referenced terminal maneuver (Fix #2) ---
# Replaces the old blind turn/drive/turn onto a drift-corrupted goal coordinate.
# The cp2->cp3 LAPF run drives up the field until the outer wall is within
# standoff ahead, squares up, creeps to a fixed standoff (re-zeroing vs an
# absolute physical reference), turns onto the finish straightaway, and drives it
# only once the front LiDAR confirms it's clear (else safe-stop, not blind).
#
# All distances/turns below need FIELD TUNING and depend on the self-footprint
# filter (Fix #1) keeping the forward cone phantom-free -- verify the `nearest
# obstacles` dump is clean before trusting front_clearance_mm().
#
# IMPORTANT: both turn SIGNS are UNVERIFIED (turn_by(+)=right, (-)=left). Which
# way squares to the wall and which faces the finish must be confirmed
# PHYSICALLY -- keep BTN_2 ready on the first run.
# WALL_DETECT_STANDOFF_MM = 500.0      # revert here: narrow-cone read only saw a HEAD-ON wall, so
                                       # as LAPF swung the robot parallel the turn never fired (slid along to a stall).
WALL_DETECT_STANDOFF_MM = 700.0        # end the LAPF run when the wall is this close ahead. Raised + read over a
                                       # WIDE arc (below) so the turn fires while still facing ~the wall, before LAPF
                                       # repulsion swings it parallel. With the 4-tile gate the only thing this close
                                       # ahead is the end wall.
# Half-angle of the forward ARC used to detect the wall during the LAPF approach.
# The narrow +/-150 mm front cone only reads a wall the robot squarely faces; LAPF
# steers ALONG a wall, so the narrow cone reads inf and the turn never fires. This
# wide arc reads an angled wall; <90 deg still excludes a fully-parallel wall.
# Used ONLY for the cp2->cp3 TRIGGER; the squared-up creep keeps the narrow cone.
WALL_DETECT_ARC_HALF_DEG = 60.0
# Minimum forward advance before the cp3 terminal turn may fire. Advance =
# projection of (pose - cp2 start) onto the approach heading -- drift-free
# along-axis progress, so weaving doesn't inflate it. ANDed with the standoff
# trigger: even with the wall detected the turn is HELD until advance hits this
# floor. This is the ONLY "far enough that what's ahead must be the wall, not a
# cone" guard (the end wall is 5 tiles out, so 4 tiles of advance rejects every
# lower-field cone). NEEDS FIELD TUNING -- the fire-time log prints measured advance.
CHECKPOINT_3_MIN_ADVANCE_MM = 4 * COURSE_TILE_MM   # 4 tiles (2440 mm); end wall at 5 tiles
# WALL_APPROACH_TARGET_MM = 350.0      # revert here: stopped 75 mm closer
# WALL_APPROACH_TARGET_MM = 475.0      # prior: +125 mm total backoff
WALL_APPROACH_TARGET_MM = 700.0        # stop this far from the wall (re-zero); doubled-ish approach standoff
# WALL_APPROACH_SPEED_MM_S = 80.0      # revert here: too slow, timed out ~640 mm short
WALL_APPROACH_SPEED_MM_S = 150.0       # closed-loop approach speed (closes the full ~1300 mm in time)
# WALL_APPROACH_TIMEOUT_S = 8.0        # revert here: too short to close the full distance
WALL_APPROACH_TIMEOUT_S = 16.0         # watchdog for the approach drive (not under LAPF watchdog)
# Turn-fire standoff is variable run-to-run (one jittery front sample per tick),
# so before turning we require the front read to sit in-range [TARGET-BAND, TARGET]
# for CONFIRM_FRAMES consecutive ticks; else keep creeping / hold. Every check is
# logged so the real standoff is visible; TIMEOUT is the backstop if it never settles.
WALL_APPROACH_BAND_MM = 50.0           # in-range window is [TARGET-BAND, TARGET], e.g. 375-425 mm
WALL_APPROACH_CONFIRM_FRAMES = 3       # consecutive in-range front reads required before turning
CLEAR_PATH_MIN_MM = 1000.0             # straightaway is "clear" if nearest front return is beyond this
FRONT_CONE_HALF_WIDTH_MM = 150.0       # half-width of the forward cone used to read the wall
FRONT_CLEARANCE_MAX_RANGE_MM = 2000.0  # ignore returns beyond this when reading the wall
# Both turns: SIGN *and* MAGNITUDE need physical verification. FACE_WALL may be
# ~0 (re-zero wall already ahead) or ~90 (perpendicular finish-side wall);
# FACE_STRAIGHTAWAY then turns onto the finish straightaway. Set both on the venue.
CP3_FACE_WALL_TURN_DEG = RIGHT_ANGLE_TURN_DEG          # face the re-zero wall (SIGN UNVERIFIED)
CP3_FACE_STRAIGHTAWAY_TURN_DEG = RIGHT_ANGLE_TURN_DEG  # turn onto the finish straightaway (SIGN UNVERIFIED)

# Forward "onto the straightaway" move after the post-wall turn (single wall
# approach -- no wall 2 any more). Open-loop; ends -> verify + drive straightaway.
# CP3_GAP_ADVANCE_DISTANCE_MM = 610.0  # prior: ~1 tile, used between the old two wall approaches
CP3_GAP_ADVANCE_DISTANCE_MM = 400.0    # shorter forward hop onto the straightaway lane

# Position sanity check after the final turn: the re-zero wall should now sit
# ~standoff BEHIND and to the LEFT. LOGGED WARN only, never blocks (a flipped
# turn sign shows up here as a large/inf read).
# WALL_FINISH_SANITY_TARGET_MM = 300.0 # revert here: tripped spuriously once the standoff went up
# WALL_FINISH_SANITY_TARGET_MM = 475.0 # prior: tracked the old 475 standoff
WALL_FINISH_SANITY_TARGET_MM = 700.0   # expected rear/left clearance after the final turn (= standoff)
WALL_FINISH_SANITY_BAND_MM = 120.0     # warn if a reading is outside target +/- this (ok 580-820 mm)

# Finish-straightaway wall-following: steer gently toward the side wall with MORE
# room so the robot tracks the middle. Proportional to (left-right) clearance,
# capped, only when BOTH walls are visible. SIGN UNVERIFIED (set_velocity is
# CCW-positive); flip if it steers INTO the closer wall.
CP3_STRAIGHTAWAY_CORRECTION_SIGN = 1               # flip to -1 if it corrects the wrong way
CP3_STRAIGHTAWAY_CORRECTION_KP_DEG_PER_MM = 0.05   # deg/s of yaw per mm of left-right imbalance
CP3_STRAIGHTAWAY_MAX_CORRECTION_DEG_S = 15.0       # cap on the yaw correction (gentle, no oscillation)


# ===========================================================================
# MISSION STEPS (cp1/cp2 scripted sequence) + HELPER FUNCTIONS
# ===========================================================================
StepKind = Literal["move_to", "turn_by", "move_forward", "square_to_wall"]


@dataclass(frozen=True)
class MissionStep:
    label: str
    kind: StepKind
    value: tuple[float, float] | float
    # correct=True -> after this step, snap the odom pose back to its ideal
    # waypoint (used on the pre-bridge corner turns that a make-room nudge can
    # shift off-line). tol_deg overrides the turn tolerance for a turn_by step.
    correct: bool = False
    tol_deg: float | None = None


MISSION_STEPS: tuple[MissionStep, ...] = (
    MissionStep("drive to checkpoint 1 approach", "move_forward", CHECKPOINT_1_APPROACH_DISTANCE_MM),
    MissionStep("turn right toward bridge lane", "turn_by", RIGHT_ANGLE_TURN_DEG, correct=True),
    MissionStep("drive into bridge lane", "move_forward", BRIDGE_ALIGN_DISTANCE_MM),
    # A bit more turn tolerance here (tol_deg) -- the waypoint correction right
    # after (correct=True) re-faces the ideal heading anyway, so this turn doesn't
    # need to land tight. (Previously square_to_wall re-fixed it; that's now removed.)
    MissionStep("turn right to face bridge", "turn_by", RIGHT_ANGLE_TURN_DEG, correct=True, tol_deg=5.0),

    # Checkpoint 1 reached here.
    # NOTE: the pre-bridge `square_to_wall` LiDAR heading-square was REMOVED on
    # purpose -- wall detection is now used ONLY in the post-bridge obstacle-course
    # approach + cp3 finish, nowhere else. Heading into the bridge now rests on the
    # guarded turn + the step-4 waypoint correction (odometry), not the walls. The
    # 2350 mm crossing is open-loop, so any residual turn error shows up as lateral
    # drift on the ramp -- watch this on the first run now that the square-up is gone.
    MissionStep("cross bridge", "move_forward", BRIDGE_CROSS_DISTANCE_MM),
    MissionStep("turn left after bridge", "turn_by", -RIGHT_ANGLE_TURN_DEG),
    MissionStep("drive past bridge exit toward obstacle section", "move_forward", BRIDGE_EXIT_DISTANCE_MM),
    MissionStep("turn left toward obstacle course", "turn_by", -RIGHT_ANGLE_TURN_DEG),

    # Checkpoint 2 reached here. Robot should be facing the cones/obstacle
    # course. The FSM switches to LAPF obstacle avoidance after this scripted
    # checkpoint 1/2 sequence finishes.
)


def _compute_ideal_poses(
    steps: tuple[MissionStep, ...],
    start_pose: tuple[float, float, float],
) -> tuple[tuple[float, float, float], ...]:
    """Ideal odom pose (x_mm, y_mm, theta_deg) AFTER each scripted step.

    Integrates the COMMANDED steps (so it matches what odometry reads in a clean,
    un-nudged run -- including the 82deg turn convention, not a true 90). Forward
    motion is along the current heading (x += d*cos, y += d*sin); turns add to
    theta; square_to_wall is an ideal heading-null (no change). These are the
    waypoints correct_to_waypoint() snaps back to.
    """
    x, y, th = start_pose
    poses: list[tuple[float, float, float]] = []
    for step in steps:
        if step.kind == "move_forward":
            x += float(step.value) * math.cos(math.radians(th))
            y += float(step.value) * math.sin(math.radians(th))
        elif step.kind == "turn_by":
            th += float(step.value)
        elif step.kind == "move_to":
            x, y = step.value  # type: ignore[assignment]
        # square_to_wall: ideal correction is 0 -> pose unchanged
        poses.append((x, y, th))
    return poses


IDEAL_POSES: tuple[tuple[float, float, float], ...] = _compute_ideal_poses(
    MISSION_STEPS, (0.0, 0.0, float(INITIAL_THETA_DEG))
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

    # Subscribe to /vision/detections so the IDLE state can auto-start on a
    # green traffic light (see GREEN_LIGHT_AUTO_START). Harmless if the vision
    # node isn't running -- detections simply never arrive and we fall back to
    # the BTN_1 manual start.
    if GREEN_LIGHT_AUTO_START:
        robot.enable_vision()
        print("[sensor] vision enabled for green-light auto-start (BTN_1 still works)")


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


def green_light_visible(robot: Robot) -> bool:
    """True if the vision node currently sees a confident GREEN traffic light.

    Reads the latest /vision/detections (cached by robot.enable_vision()), keeps
    only "traffic light" detections above GREEN_START_MIN_CONFIDENCE, and checks
    their classified "color" attribute (set by vision's classify_traffic_light_color).
    Returns False if vision is stale/unavailable, so a missing vision node simply
    leaves the BTN_1 manual start as the only path.
    """
    if not robot.is_vision_active(timeout_s=GREEN_START_VISION_STALE_SEC):
        return False

    for detection in robot.get_detections("traffic light"):
        if float(detection["confidence"]) < GREEN_START_MIN_CONFIDENCE:
            continue
        color = detection.get("attributes", {}).get("color", {}).get("value")
        if color == "green":
            return True
    return False


def current_light_color(robot: Robot) -> str:
    """What the camera sees right now: "red", "green", or "none".

    Diagnostic counterpart to green_light_visible() -- reads the same cached
    /vision/detections, keeps "traffic light" detections above
    GREEN_START_MIN_CONFIDENCE, and reports the highest-confidence one's color.
    "green" wins ties so it never disagrees with the auto-start check. Returns
    "none" when vision is stale or nothing confident is in view.
    """
    if not robot.is_vision_active(timeout_s=GREEN_START_VISION_STALE_SEC):
        return "none"

    best_color = "none"
    best_conf = -1.0
    for detection in robot.get_detections("traffic light"):
        conf = float(detection["confidence"])
        if conf < GREEN_START_MIN_CONFIDENCE:
            continue
        color = detection.get("attributes", {}).get("color", {}).get("value")
        if color not in ("red", "green"):
            continue
        # Prefer the most confident reading; on a tie, prefer green.
        if conf > best_conf or (conf == best_conf and color == "green"):
            best_color = color
            best_conf = conf
    return best_color


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
        # Collision-aware: make room for the pivot first, then turn (forces the
        # turn with a loud warning if it can't clear). See guarded_turn_by.
        return guarded_turn_by(robot, step.value, tolerance_deg=step.tol_deg)

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


def wall_ahead_clearance_mm(
    robot: Robot,
    half_arc_deg: float = WALL_DETECT_ARC_HALF_DEG,
    max_range_mm: float = FRONT_CLEARANCE_MAX_RANGE_MM,
) -> float:
    """Nearest raw-cloud return within a WIDE forward arc, by straight-line range.

    Unlike front_clearance_mm's narrow +/-150 mm cone (which only reads a wall the
    robot is squarely facing), this keeps any return whose body-frame bearing is
    within +/- half_arc_deg of straight ahead (+x) and closer than max_range_mm,
    and returns the smallest RANGE (hypot). The cp2->cp3 LAPF approach tends to
    steer the robot ALONG the outer wall rather than into it, swinging it off
    head-on before the narrow cone ever reads <= standoff; this arc still reads the
    wall on an angled approach so the terminal turn can fire before the robot goes
    fully parallel. +inf when the arc is empty. fwd=+x, left=+y (obstacle-dump axes).

    half_arc_deg < 90 keeps a fully-parallel wall (returns at ~+/-90 deg) excluded,
    so this can't latch onto a wall the robot is already running beside. Uses the
    raw cloud (self-footprint excluded, Fix #1), same as front_clearance_mm.
    """
    half_rad = math.radians(half_arc_deg)
    nearest = math.inf
    for px, py in robot.get_obstacles():
        if px <= 0.0:
            continue
        if abs(math.atan2(py, px)) > half_rad:
            continue
        rng = math.hypot(px, py)
        if rng > max_range_mm:
            continue
        if rng < nearest:
            nearest = rng
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


def pivot_blocked(robot: Robot, swept_radius_mm: float) -> tuple[bool, str, float]:
    """Is the in-place pivot footprint obstructed?

    An in-place rotation sweeps the robot's corners through a circle of radius
    `swept_radius_mm` (center -> farthest corner + margin). Any obstacle inside
    that circle gets clipped by a corner, so a pivot is unsafe whenever the
    nearest raw-cloud return in ANY direction is closer than the swept radius.

    Reads the four body-frame cones (front/rear/left/right) via
    directional_clearance_mm with a half-width = swept radius, so together the
    four cones tile the plane around the robot. Returns
    (blocked, tightest_axis, nearest_gap_mm); nearest_gap is +inf (blocked
    False) when nothing is within the swept circle on any side. Uses the same
    self-footprint-excluded raw cloud as the other clearance reads, so the
    robot's own structure never trips it.
    """
    gaps = {
        axis: directional_clearance_mm(
            robot, axis,
            half_width_mm=swept_radius_mm,
            max_range_mm=swept_radius_mm,
        )
        for axis in ("front", "rear", "left", "right")
    }
    tight_axis = min(gaps, key=gaps.get)
    gap = gaps[tight_axis]
    return gap < swept_radius_mm, tight_axis, gap


def _make_room_for_pivot(robot: Robot, tight_axis: str, swept_radius_mm: float) -> bool:
    """Nudge the base fore/aft to open up the pivot circle on the blocked side.

    Differential drive can't strafe, so the only way to make room is to
    translate along the heading. Retreat AWAY from the obstruction: back up when
    it's ahead, drive forward when it's behind, and for a side obstruction pick
    whichever of fore/aft currently has more room. The nudge is capped at
    PIVOT_MAKE_ROOM_NUDGE_MM and only fires if that direction is itself clear
    (so making room can't cause a new collision); if the preferred direction is
    blocked it tries the other. Returns True if it moved, False if neither
    direction is safe. Blocks for the (short) duration of the nudge.
    """
    front = directional_clearance_mm(robot, "front", half_width_mm=swept_radius_mm)
    rear = directional_clearance_mm(robot, "rear", half_width_mm=swept_radius_mm)
    need = PIVOT_MAKE_ROOM_NUDGE_MM + PIVOT_CLEARANCE_MARGIN_MM

    if tight_axis == "front":
        go_forward = False
    elif tight_axis == "rear":
        go_forward = True
    else:  # left/right obstruction: retreat toward the roomier end
        go_forward = front >= rear

    avail = front if go_forward else rear
    if avail < need:
        go_forward = not go_forward          # preferred retreat blocked; try the other way
        avail = front if go_forward else rear
        if avail < need:
            return False

    nudge = min(PIVOT_MAKE_ROOM_NUDGE_MM, avail - PIVOT_CLEARANCE_MARGIN_MM)
    direction = "forward" if go_forward else "backward"
    print(f"[guard-turn] making room: nudging {direction} {nudge:.0f} mm "
          f"(front {front:.0f} mm, rear {rear:.0f} mm)")
    mover = robot.move_forward if go_forward else robot.move_backward
    mover(nudge, velocity=PIVOT_MAKE_ROOM_SPEED_MM_S,
          tolerance=PIVOT_MAKE_ROOM_TOLERANCE_MM, blocking=True)
    return True


def guarded_turn_by(robot: Robot, delta_deg: float, tolerance_deg: float | None = None):
    """Collision-aware replacement for robot.turn_by() on scripted 90-deg turns.

    Before pivoting, checks whether the swept footprint is clear (pivot_blocked).
    If blocked, makes room by nudging fore/aft (up to PIVOT_MAKE_ROOM_MAX_TRIES
    times) and re-checks. If room still can't be made, it FORCES the turn anyway
    (by design) and prints a loud warning so the collision risk is visible in the
    console. Returns the non-blocking MotionHandle for the actual pivot, so the
    FSM keeps polling and BTN_2 still cancels during the rotation itself; only the
    short make-room nudges block. tolerance_deg overrides the default turn
    tolerance (per-step, e.g. a looser tolerance on the face-bridge turn).
    """
    tol = TURN_TOLERANCE_DEG if tolerance_deg is None else tolerance_deg
    if not PIVOT_GUARD_ENABLED:
        return robot.turn_by(delta_deg, tolerance_deg=tol, blocking=False)

    swept = FOOTPRINT_HALF_DIAG_MM + PIVOT_CLEARANCE_MARGIN_MM
    side = "right" if delta_deg >= 0 else "left"
    for attempt in range(PIVOT_MAKE_ROOM_MAX_TRIES + 1):
        blocked, axis, gap = pivot_blocked(robot, swept)
        if not blocked:
            if attempt > 0:
                print(f"[guard-turn] {side} {delta_deg:+.0f} deg: clear after {attempt} "
                      f"nudge(s) (nearest {gap:.0f} mm >= swept {swept:.0f} mm) - turning")
            break
        print(f"[guard-turn] {side} {delta_deg:+.0f} deg BLOCKED (try {attempt}/"
              f"{PIVOT_MAKE_ROOM_MAX_TRIES}): nearest obstacle {gap:.0f} mm on {axis} "
              f"side < swept radius {swept:.0f} mm")
        if attempt >= PIVOT_MAKE_ROOM_MAX_TRIES or not _make_room_for_pivot(robot, axis, swept):
            print(f"[guard-turn] !!! could not clear the pivot - FORCING the {side} "
                  f"{delta_deg:+.0f} deg turn anyway (collision risk on {axis} side, "
                  f"nearest {gap:.0f} mm) !!!")
            break

    return robot.turn_by(delta_deg, tolerance_deg=tol, blocking=False)


def _wrap_deg(angle_deg: float) -> float:
    """Wrap an angle to (-180, 180] degrees."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def correct_to_waypoint(robot: Robot, ideal_pose: tuple[float, float, float], label: str) -> None:
    """Snap the odom pose back onto its ideal waypoint after a disrupted step.

    Measures the current odometry pose, compares to `ideal_pose` (x, y,
    theta_deg from IDEAL_POSES), and if it has drifted past WAYPOINT_POS_TOL_MM
    it corrects: turn to face the waypoint (guarded), drive there, then re-face
    the ideal heading. Bounded by WAYPOINT_MAX_CORRECTION_MM (a larger deviation
    is treated as suspect odometry and skipped) and a per-move timeout so it can
    never hang. Blocks for the (short) duration of the correction. See the
    WAYPOINT_CORRECTION_* constants and the odom-only caveat there.
    """
    tx, ty, tth = ideal_pose
    x, y, th = robot.get_pose()
    d = math.hypot(tx - x, ty - y)

    if d > WAYPOINT_MAX_CORRECTION_MM:
        print(f"[waypoint] {label}: deviation {d:.0f} mm > max {WAYPOINT_MAX_CORRECTION_MM:.0f} mm "
              f"- skipping correction (suspect odometry)")
        return

    if d > WAYPOINT_POS_TOL_MM:
        bearing_deg = math.degrees(math.atan2(ty - y, tx - x))
        print(f"[waypoint] {label}: off ideal by {d:.0f} mm "
              f"(at {x:.0f},{y:.0f}; want {tx:.0f},{ty:.0f}) - correcting toward waypoint")
        guarded_turn_by(robot, _wrap_deg(bearing_deg - th)).wait(timeout=WAYPOINT_CORRECTION_TIMEOUT_S)
        robot.move_forward(d, velocity=WAYPOINT_CORRECTION_SPEED_MM_S,
                           tolerance=WAYPOINT_POS_TOL_MM, blocking=True,
                           timeout=WAYPOINT_CORRECTION_TIMEOUT_S)
    else:
        print(f"[waypoint] {label}: on ideal ({d:.0f} mm <= {WAYPOINT_POS_TOL_MM:.0f} mm) - no position fix")

    _, _, th_now = robot.get_pose()
    heading_err = _wrap_deg(tth - th_now)
    if abs(heading_err) > WAYPOINT_HEADING_TOL_DEG:
        print(f"[waypoint] {label}: heading off {heading_err:+.0f} deg - re-aligning to {tth:.0f} deg")
        guarded_turn_by(robot, heading_err).wait(timeout=WAYPOINT_CORRECTION_TIMEOUT_S)


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
    return guarded_turn_by(robot, CP3_FACE_WALL_TURN_DEG)


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


def start_recovery_nudge(robot: Robot, av: dict, now: float):
    """Physically displace the robot onto a new approach line before re-issuing.

    Guarded forward hop so the body actually moves each retry (not just pivots).
    Skips the hop only if something is close dead-ahead (that case already
    reversed) so it never drives into a cone. Returns (next_state, handle).
    """
    front = front_clearance_mm(robot)
    if front <= RECOVERY_NUDGE_MIN_FRONT_MM:
        print(
            f"[FSM] recovery {av['retry_count']}/{MAX_AVOIDANCE_RETRIES}: "
            f"skipping lateral nudge (front {front:.0f} mm) - re-issuing LAPF"
        )
        return "OBSTACLE_AVOIDANCE", reissue_after_recovery(robot, av, now)
    print(
        f"[FSM] recovery {av['retry_count']}/{MAX_AVOIDANCE_RETRIES}: "
        f"lateral nudge {RECOVERY_LATERAL_NUDGE_MM:.0f} mm "
        f"(sign {av['recovery_sign']:+d}, front {front:.0f} mm)"
    )
    av["recovery_started_at"] = now
    handle = robot.move_forward(
        RECOVERY_LATERAL_NUDGE_MM,
        velocity=RECOVERY_NUDGE_SPEED_MM_S,
        tolerance=RECOVERY_NUDGE_TOLERANCE_MM,
        blocking=False,
    )
    return "OBSTACLE_RECOVERY_NUDGE", handle


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


# ===========================================================================
# MISSION FSM  (state machine: INIT -> IDLE -> cp1/cp2 -> cp3 -> cp4 -> cp5)
# ===========================================================================
def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    motion_handle = None
    av: dict = {}  # active obstacle-avoidance segment + watchdog context
    step_index = 0
    last_status_print_at = 0.0
    green_streak = 0  # consecutive IDLE frames seeing green (auto-start confirm)
    last_light_print_at = 0.0  # throttles the IDLE traffic-light readout

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
            green_streak = 0  # require a fresh green confirmation each time we idle
            if GREEN_LIGHT_AUTO_START:
                print("[FSM] IDLE - waiting for GREEN light to auto-start "
                      "(or press BTN_1); BTN_2 to cancel")
            else:
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
            # START TRIGGER: two ways to leave IDLE, whichever fires first --
            #   (1) GREEN-LIGHT AUTO-START: the course begins on a red light that
            #       turns green. We poll vision each frame and require green for
            #       GREEN_START_CONFIRM_FRAMES consecutive frames (a streak) so a
            #       single mis-classified frame can't false-start. Any non-green
            #       frame resets the streak. Gated by GREEN_LIGHT_AUTO_START.
            #   (2) BTN_1 MANUAL START: always available as a fallback (e.g. if the
            #       camera can't see the light), exactly as before.
            # Both lead into the same start sequence below.
            button_start = robot.was_button_pressed(Button.BTN_1)

            green_start = False
            if GREEN_LIGHT_AUTO_START and not button_start:
                if green_light_visible(robot):
                    green_streak += 1
                    if green_streak >= GREEN_START_CONFIRM_FRAMES:
                        green_start = True
                else:
                    green_streak = 0

            # Heartbeat: show what the camera currently sees so we can confirm
            # detection is alive. Persists every IDLE_LIGHT_PRINT_INTERVAL_S until
            # a green light or BTN_1 leaves IDLE (handled just below).
            if now - last_light_print_at >= IDLE_LIGHT_PRINT_INTERVAL_S:
                light = current_light_color(robot)
                seen = {"green": "GREEN light", "red": "RED light"}.get(light, "no light")
                print(f"[FSM] IDLE - detecting: {seen}")
                last_light_print_at = now

            if button_start or green_start:
                green_streak = 0
                print(f"[FSM] START trigger: {'BTN_1' if button_start else 'GREEN light'}")
                if START_AT_CP2:
                    # TEMP/testing path: skip the scripted cp1->cp2 steps and jump
                    # to the cp2->cp3 leg. reset_mission_pose() zeroes odometry to
                    # the cp2 start pose (0,0,+Y), so the goal aims up the course.
                    # Place the robot at the cp2 spot facing +Y first.
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
                    # Before advancing: if this step is flagged, snap the odom
                    # pose back onto its ideal map waypoint (recovers a make-room
                    # nudge / wall bump that shifted the corner off-line). Blocks
                    # briefly; see correct_to_waypoint + the odom-only caveat.
                    finished_step = MISSION_STEPS[step_index]
                    if WAYPOINT_CORRECTION_ENABLED and finished_step.correct:
                        correct_to_waypoint(robot, IDEAL_POSES[step_index], finished_step.label)
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
                    # cp2->cp3 approach: end the LAPF run when the outer wall is in
                    # range (Fix #2), not at the drift-corrupted goal. Fire the turn
                    # only when BOTH drift-free guards hold:
                    #   1. standoff: a return <= WALL_DETECT_STANDOFF_MM in a WIDE
                    #      forward arc (LAPF steers ALONG the wall, so the narrow
                    #      cone would miss it on an angled approach); and
                    #   2. min advance: >= CHECKPOINT_3_MIN_ADVANCE_MM (4 tiles) of
                    #      along-axis progress, so a lower-field cone can't trigger it.
                    # cp3->cp4 leaves terminate_on_wall False (goal-coordinate stop).
                    detect_wall = av.get("terminate_on_wall")
                    front = wall_ahead_clearance_mm(robot) if detect_wall else math.inf
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
                                f"(wall {front:.0f} mm in {WALL_DETECT_ARC_HALF_DEG:.0f} deg arc) - continuing approach"
                            )
                            av["last_gate_log_at"] = now

                    if fire_turn:
                        # Final measured advance is printed here so the next run
                        # gives the exact along-axis distance at which the turn
                        # fires — dial CHECKPOINT_3_MIN_ADVANCE_MM in from this.
                        print(
                            f"[FSM] checkpoint 3 approach - outer wall {front:.0f} mm in "
                            f"{WALL_DETECT_ARC_HALF_DEG:.0f} deg arc, "
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
                    motion_handle = None
                    state, motion_handle = start_recovery_nudge(robot, av, now)
                    last_status_print_at = now

        elif state == "OBSTACLE_RECOVERY_NUDGE":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE - recovery nudge cancelled")
                state = "IDLE"
            else:
                nudge_done = motion_handle is not None and motion_handle.is_finished()
                nudge_timeout = (now - av["recovery_started_at"]) >= RECOVERY_NUDGE_TIMEOUT_S
                if (now - av["started_at"]) >= SEGMENT_HARD_CEILING_S:
                    cancel_motion(robot, motion_handle)
                    motion_handle = None
                    safe_stop_to_idle(robot, f"LAPF failed ({av['label']}) - segment timeout in recovery")
                    state = "IDLE"
                elif nudge_done or nudge_timeout:
                    if not nudge_done:
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
            # Closed-loop velocity creep toward the wall until the front cone reads
            # the target standoff. Timeout DISABLED (below): keeps creeping until
            # the standoff is reached; BTN_2 is the only stop.
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop()
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE - cp3 wall approach cancelled")
                state = "IDLE"
            else:
                front = front_clearance_mm(robot)
                in_lower = WALL_APPROACH_TARGET_MM - WALL_APPROACH_BAND_MM
                # "In range" = at/under the target but no nearer than the band's
                # lower edge -- keeps the variable stop distance honest before turning.
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
                    # Hold and require CONFIRM_FRAMES consecutive in-range reads so
                    # a single noisy frame can't fire the turn at the wrong distance.
                    robot.stop()
                    av["wall_inrange_count"] += 1
                    if av["wall_inrange_count"] >= WALL_APPROACH_CONFIRM_FRAMES:
                        print(
                            f"[FSM] wall {av.get('wall_approach_num', 1)} standoff confirmed "
                            f"at {front:.0f} mm (within {in_lower:.0f}-{WALL_APPROACH_TARGET_MM:.0f} mm "
                            f"for {WALL_APPROACH_CONFIRM_FRAMES} reads) - turning"
                        )
                        motion_handle = guarded_turn_by(robot, CP3_FACE_STRAIGHTAWAY_TURN_DEG)
                        last_status_print_at = now
                        state = "CP3_FACE_STRAIGHTAWAY"
                # TIMEOUT DISABLED: the approach no longer safe-stops if the wall is
                # slow to appear -- it keeps creeping (incl. the inf case below) until
                # the standoff is reached; BTN_2 is the only stop. To re-enable,
                # uncomment this branch AND restore the inf-case hold below.
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
                    # Too close to creep further: hold and keep sampling (jitter may
                    # settle back into range).
                    robot.stop()
                    av["wall_inrange_count"] = 0
                elif math.isfinite(front):
                    # Still beyond the band (front > target): creep closer.
                    av["wall_inrange_count"] = 0
                    robot.set_velocity(WALL_APPROACH_SPEED_MM_S, 0.0)
                else:
                    # Wall not in the cone yet (front=inf): KEEP creeping to close the
                    # gap (holding here was what froze the wall-2 approach).
                    # ORIGINAL (revert here): held in place until the timeout fired:
                    #   av["wall_inrange_count"] = 0
                    #   robot.stop()
                    av["wall_inrange_count"] = 0
                    robot.set_velocity(WALL_APPROACH_SPEED_MM_S, 0.0)

        elif state == "CP3_FACE_STRAIGHTAWAY":
            # Turn #2 (started on entry): from squared-up-at-the-wall onto the finish
            # straightaway. SINGLE wall approach -- after this turn the robot drives
            # forward onto the straightaway lane, then verifies and drives it.
            # End-of-course = turn / straight / turn / straight / straightaway.
            # ORIGINAL (revert here): two-wall hop -- branched on wall_approach_num,
            # driving a gap then a SECOND wall creep+turn. Removed: with the 700 mm
            # standoff the wall-2 approach always sat frozen (wall ended < 650 mm).
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE - cp3 post-wall turn cancelled")
                state = "IDLE"
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                print(
                    "[FSM] turned onto straightaway - advancing "
                    f"{CP3_GAP_ADVANCE_DISTANCE_MM:.0f} mm onto the straightaway lane"
                )
                motion_handle = robot.move_forward(
                    CP3_GAP_ADVANCE_DISTANCE_MM,
                    velocity=DRIVE_VELOCITY_MM_S,
                    tolerance=DRIVE_TOLERANCE_MM,
                    blocking=False,
                )
                last_status_print_at = now
                state = "CP3_GAP_ADVANCE"

        elif state == "CP3_GAP_ADVANCE":
            # Straight #2: forward hop after the post-wall turn to get onto the finish
            # straightaway lane, then verify the path and drive it. (No second wall
            # approach any more -- see CP3_FACE_STRAIGHTAWAY.)
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
                    print("[FSM] gap advance done - verifying straightaway is clear")
                    motion_handle = None
                    last_status_print_at = now
                    state = "CP3_VERIFY_STRAIGHTAWAY"

        elif state == "CP3_VERIFY_STRAIGHTAWAY":
            # One-shot alignment check: a clear forward cone means we're lined up.
            # If blocked, stop rather than drive blind.
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop()
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE - cp3 straightaway verify cancelled")
                state = "IDLE"
            else:
                # Position sanity check (LOGGED, WARN only, never blocks): the
                # re-zero wall should sit ~target away BEHIND and to the LEFT. A
                # flipped (UNVERIFIED) turn sign shows up as out-of-band/inf -> warn;
                # the go/no-go still defers to the front-clear gate below.
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
                    # Velocity-controlled wall-following drive (the
                    # CP3_DRIVE_STRAIGHTAWAY loop steers + tracks distance).
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
            # Velocity-controlled wall-following drive to cp3: each tick steers
            # toward the side wall with MORE room so it tracks down the middle.
            # Own velocity loop (no MotionHandle, no LAPF watchdog); terminates on
            # odometry distance, BTN_2 polled every tick.
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
            # Robot is stopped at checkpoint 4. cp5 manipulator is DISABLED for now
            # -- we just stop at the finish line and go idle. Re-enable by
            # uncommenting run_manipulator_placeholder() below.
            # run_manipulator_placeholder(robot)
            print("[FSM] CHECKPOINT 4 finish - cp5 manipulator disabled, stopping")
            robot.stop()
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
