"""
shooter.py — Nerf ball shooter manipulator
==========================================

Reusable control layer for a 3-servo + 1-DC-motor ball shooter:
  - Servo 1: yaw (left/right)
  - Servo 2: pitch (up/down)
  - Servo 3: loader arm
  - DC_M3: flywheel (open-loop PWM)

All angle and power constants below are starting guesses — tune on the
physical build before relying on distance-based shots.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from robot.hardware_map import DCMotorMode, Motor, ServoChannel
from robot.robot import Robot


# ---------------------------------------------------------------------------
# Tunable hardware mapping — edit to match your build
# ---------------------------------------------------------------------------

YAW_SERVO = ServoChannel.CH_1
PITCH_SERVO = ServoChannel.CH_2
LOADER_SERVO = ServoChannel.CH_4
FLYWHEEL_MOTOR = Motor.DC_M3

# Yaw axis (degrees, 0–180 via set_servo; use set_servo_pulse if you need more)
YAW_CENTER_DEG = 120.0
YAW_MIN_DEG = 60.0
YAW_MAX_DEG = 180.0

# Pitch axis
PITCH_CENTER_DEG = 130.0
PITCH_MIN_DEG = 60.0
PITCH_MAX_DEG = 210.0

# Loader arm — uses set_servo_pulse so angles can go below 0°
LOADER_MIN_DEG = -90.0              # most negative allowed (maps to 500 µs)
LOADER_MAX_DEG = 180.0              # most positive allowed (maps to 2500 µs)
LOADER_REST_DEG = 120.0
LOADER_PUSH_DEG = 190.0
LOADER_PUSH_DWELL_S = 0.3          # used by load() test cycle
LOADER_RETURN_DWELL_S = 0.10
SHOT_LOADER_HOLD_S = 0.20           # pause at push before flywheel spins up
SHOT_FIRE_DWELL_S = 0.30            # hold push while flywheel fires the ball
LOADER_CALIB_STEP_DEG = 90.0
LOADER_CALIB_SETTLE_S = 0.30
# No physical servo encoder — wait for commanded pulse in telemetry, then
# allow time for the arm to physically reach position.
LOADER_PULSE_TOLERANCE_US = 25
LOADER_MOVE_TIMEOUT_S = 1.0
LOADER_MECHANICAL_SETTLE_S = 0.35

# Servo settle after aim moves
AIM_SETTLE_S = 0.30

# Flywheel — open-loop PWM (0–255); sign controls spin direction
FLYWHEEL_PWM_SIGN = -1              # flip to -1 if motor spins backward
FLYWHEEL_SPINUP_S = 0.75
FLYWHEEL_MIN_PWM = 50
FLYWHEEL_MAX_PWM = 110

# Distance-to-power map (linear; swap for a lookup table if needed)
SHOT_MIN_DISTANCE = 0.5   # meters — close target
SHOT_MAX_DISTANCE = 3.0   # meters — far target


@dataclass
class ShooterConfig:
    """Optional overrides for Shooter defaults."""

    yaw_servo: int = int(YAW_SERVO)
    pitch_servo: int = int(PITCH_SERVO)
    loader_servo: int = int(LOADER_SERVO)
    flywheel_motor: int = int(FLYWHEEL_MOTOR)

    yaw_center_deg: float = YAW_CENTER_DEG
    yaw_min_deg: float = YAW_MIN_DEG
    yaw_max_deg: float = YAW_MAX_DEG

    pitch_center_deg: float = PITCH_CENTER_DEG
    pitch_min_deg: float = PITCH_MIN_DEG
    pitch_max_deg: float = PITCH_MAX_DEG

    loader_rest_deg: float = LOADER_REST_DEG
    loader_push_deg: float = LOADER_PUSH_DEG
    loader_min_deg: float = LOADER_MIN_DEG
    loader_max_deg: float = LOADER_MAX_DEG
    loader_push_dwell_s: float = LOADER_PUSH_DWELL_S
    loader_return_dwell_s: float = LOADER_RETURN_DWELL_S
    shot_loader_hold_s: float = SHOT_LOADER_HOLD_S
    shot_fire_dwell_s: float = SHOT_FIRE_DWELL_S
    loader_pulse_tolerance_us: int = LOADER_PULSE_TOLERANCE_US
    loader_move_timeout_s: float = LOADER_MOVE_TIMEOUT_S
    loader_mechanical_settle_s: float = LOADER_MECHANICAL_SETTLE_S

    aim_settle_s: float = AIM_SETTLE_S
    flywheel_spinup_s: float = FLYWHEEL_SPINUP_S
    flywheel_pwm_sign: int = FLYWHEEL_PWM_SIGN
    flywheel_min_pwm: int = FLYWHEEL_MIN_PWM
    flywheel_max_pwm: int = FLYWHEEL_MAX_PWM

    shot_min_distance: float = SHOT_MIN_DISTANCE
    shot_max_distance: float = SHOT_MAX_DISTANCE


class Shooter:
    """
    High-level API for the nerf ball shooter.

    Primitives: set_yaw, set_pitch, load, spin_up, spin_down
    Combined:   aim, shoot, aim_and_shoot
    Future:     track_target (vision hook)
    """

    def __init__(self, robot: Robot, config: ShooterConfig | None = None) -> None:
        self._robot = robot
        self._cfg = config or ShooterConfig()
        self._enabled = False
        self._yaw_deg = self._cfg.yaw_center_deg
        self._pitch_deg = self._cfg.pitch_center_deg
        self._flywheel_pwm = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def enable(self) -> None:
        """Enable servos and flywheel motor; move to safe rest positions."""
        self._robot.enable_servo(self._cfg.yaw_servo)
        self._robot.enable_servo(self._cfg.pitch_servo)
        self._robot.enable_servo(self._cfg.loader_servo)
        self._robot.enable_motor(self._cfg.flywheel_motor, DCMotorMode.PWM)
        self._robot.set_motor_pwm(self._cfg.flywheel_motor, 0)
        self._flywheel_pwm = 0

        self.set_yaw(self._cfg.yaw_center_deg)
        self.set_pitch(self._cfg.pitch_center_deg)
        self.set_loader(self._cfg.loader_rest_deg)
        time.sleep(self._cfg.aim_settle_s)

        self._enabled = True

    def disable(self) -> None:
        """Stop flywheel and disable all shooter actuators."""
        self.spin_down()
        self._robot.disable_motor(self._cfg.flywheel_motor)
        self._robot.disable_servo(self._cfg.yaw_servo)
        self._robot.disable_servo(self._cfg.pitch_servo)
        self._robot.disable_servo(self._cfg.loader_servo)
        self._enabled = False

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def yaw_deg(self) -> float:
        return self._yaw_deg

    @property
    def pitch_deg(self) -> float:
        return self._pitch_deg

    # ------------------------------------------------------------------
    # Primitives — aim axes
    # ------------------------------------------------------------------

    def set_yaw(self, angle_deg: float) -> float:
        """Set yaw angle (clamped). Returns the commanded angle."""
        clamped = max(self._cfg.yaw_min_deg, min(self._cfg.yaw_max_deg, angle_deg))
        self._robot.set_servo(self._cfg.yaw_servo, clamped)
        self._yaw_deg = clamped
        return clamped

    def set_pitch(self, angle_deg: float) -> float:
        """Set pitch angle (clamped). Returns the commanded angle."""
        clamped = max(self._cfg.pitch_min_deg, min(self._cfg.pitch_max_deg, angle_deg))
        self._robot.set_servo(self._cfg.pitch_servo, clamped)
        self._pitch_deg = clamped
        return clamped

    def _loader_angle_to_pulse(self, angle_deg: float) -> int:
        """Map loader angle (LOADER_MIN_DEG … LOADER_MAX_DEG) to 500–2500 µs."""
        lo = self._cfg.loader_min_deg
        hi = self._cfg.loader_max_deg
        clamped = max(lo, min(hi, angle_deg))
        if hi <= lo:
            return 1500
        t = (clamped - lo) / (hi - lo)
        return int(500 + t * 2000)

    def set_loader(self, angle_deg: float) -> float:
        """Set loader arm angle (supports negative degrees). Returns commanded angle."""
        lo = self._cfg.loader_min_deg
        hi = self._cfg.loader_max_deg
        clamped = max(lo, min(hi, angle_deg))
        pulse_us = self._loader_angle_to_pulse(clamped)
        self._robot.set_servo_pulse(self._cfg.loader_servo, pulse_us)
        return clamped

    def _read_loader_pulse_us(self) -> int | None:
        """Return the firmware-reported loader pulse, or None if unavailable."""
        state = self._robot.get_servo_state()
        if state is None or not hasattr(state, "channels"):
            return None
        for channel in state.channels:
            if int(channel.channel_number) == self._cfg.loader_servo:
                return int(channel.pulse_us)
        return None

    def wait_loader_at_angle(self, angle_deg: float) -> bool:
        """
        Wait until loader telemetry matches the target pulse, then settle.

        This confirms the firmware applied the command (not physical position —
        servos have no encoder). Returns False on timeout.
        """
        target_pulse = self._loader_angle_to_pulse(angle_deg)
        tolerance = self._cfg.loader_pulse_tolerance_us
        deadline = time.monotonic() + self._cfg.loader_move_timeout_s

        while time.monotonic() < deadline:
            reported = self._read_loader_pulse_us()
            if reported is not None and abs(reported - target_pulse) <= tolerance:
                time.sleep(self._cfg.loader_mechanical_settle_s)
                return True
            time.sleep(0.02)

        time.sleep(self._cfg.loader_mechanical_settle_s)
        return False

    def loader_rest(self, *, wait: bool = False) -> bool:
        """Return the loader arm to the rest position."""
        angle = self.set_loader(self._cfg.loader_rest_deg)
        if wait:
            return self.wait_loader_at_angle(angle)
        return True

    def loader_push(self, *, wait: bool = False) -> bool:
        """Move the loader arm to the push (feed) position."""
        angle = self.set_loader(self._cfg.loader_push_deg)
        if wait:
            return self.wait_loader_at_angle(angle)
        return True

    def load(self) -> None:
        """Quick push-and-return cycle for loader testing (no flywheel)."""
        self.loader_push(wait=True)
        time.sleep(self._cfg.loader_push_dwell_s)
        self.loader_rest(wait=True)
        time.sleep(self._cfg.loader_return_dwell_s)

    def calibrate_loader(self, step_deg: float = LOADER_CALIB_STEP_DEG) -> float:
        """
        Move loader to rest, then step toward push for mechanical calibration.

        Returns the stepped angle so you can update LOADER_REST_DEG if needed.
        """
        rest = self._cfg.loader_rest_deg
        push = self._cfg.loader_push_deg
        direction = 1.0 if push >= rest else -1.0
        target = rest + direction * abs(step_deg)

        self.set_loader(rest)
        time.sleep(LOADER_CALIB_SETTLE_S)
        self.set_loader(target)
        time.sleep(LOADER_CALIB_SETTLE_S)
        return target

    # ------------------------------------------------------------------
    # Primitives — flywheel
    # ------------------------------------------------------------------

    def distance_to_power(self, distance_m: float) -> int:
        """
        Map target distance (meters) to flywheel PWM.

        Linear interpolation between shot_min_distance → min_pwm and
        shot_max_distance → max_pwm. Replace with a lookup table for
        better accuracy once you've calibrated on the field.
        """
        d_min = self._cfg.shot_min_distance
        d_max = self._cfg.shot_max_distance
        if d_max <= d_min:
            return self._cfg.flywheel_max_pwm

        t = (distance_m - d_min) / (d_max - d_min)
        t = max(0.0, min(1.0, t))
        pwm = int(
            self._cfg.flywheel_min_pwm
            + t * (self._cfg.flywheel_max_pwm - self._cfg.flywheel_min_pwm)
        )
        return max(0, min(255, pwm))

    def set_flywheel_power(self, pwm: int) -> None:
        """Set flywheel PWM directly (-255 … 255), with direction sign applied."""
        pwm = max(-255, min(255, int(pwm)))
        signed_pwm = pwm * self._cfg.flywheel_pwm_sign
        self._robot.set_motor_pwm(self._cfg.flywheel_motor, signed_pwm)
        self._flywheel_pwm = pwm

    def spin_up(self, pwm: int | None = None, *, wait: bool = True) -> None:
        """Spin flywheel to pwm (or current level) and optionally wait for spin-up."""
        if pwm is not None:
            self.set_flywheel_power(pwm)
        if wait and self._flywheel_pwm != 0:
            time.sleep(self._cfg.flywheel_spinup_s)

    def spin_down(self) -> None:
        """Stop the flywheel."""
        self.set_flywheel_power(0)

    # ------------------------------------------------------------------
    # Combined sequences
    # ------------------------------------------------------------------

    def aim(self, yaw_deg: float, pitch_deg: float, *, settle: bool = True) -> None:
        """Move yaw and pitch to the requested angles."""
        self.set_yaw(yaw_deg)
        self.set_pitch(pitch_deg)
        if settle:
            time.sleep(self._cfg.aim_settle_s)

    def shoot(
        self,
        distance_m: float | None = None,
        *,
        power: int | None = None,
        spin_down_after: bool = True,
    ) -> None:
        """
        Fire one ball: push loader and wait for command confirmation,
        hold, spin up flywheel, fire, then return loader to rest.

        Provide either distance_m (maps to PWM) or power (raw PWM).
        """
        if power is None:
            if distance_m is None:
                raise ValueError("shoot() requires distance_m or power")
            power = self.distance_to_power(distance_m)

        self.loader_push(wait=True)
        time.sleep(self._cfg.shot_loader_hold_s)
        self.spin_up(power, wait=True)
        time.sleep(self._cfg.shot_fire_dwell_s)
        if spin_down_after:
            self.spin_down()
        self.loader_rest()
        time.sleep(self._cfg.loader_return_dwell_s)

    def aim_and_shoot(
        self,
        yaw_deg: float,
        pitch_deg: float,
        distance_m: float,
        *,
        spin_down_after: bool = True,
    ) -> None:
        """Aim at yaw/pitch, then shoot at the given distance."""
        self.aim(yaw_deg, pitch_deg)
        self.shoot(distance_m, spin_down_after=spin_down_after)

    # ------------------------------------------------------------------
    # Future vision hook
    # ------------------------------------------------------------------

    def track_target(
        self,
        class_name: str,
        *,
        min_confidence: float = 0.5,
        yaw_gain_deg_per_px: float = 0.05,
        pitch_gain_deg_per_px: float = 0.05,
        center_tolerance_px: float = 20.0,
    ) -> bool:
        """
        Nudge yaw/pitch toward a vision detection (proportional control).

        Requires robot.enable_vision() to have been called elsewhere.
        Returns True when the target bbox center is within tolerance of
        the image center. This is the seam for a future aim-and-track loop:

            robot.enable_vision()
            while not shooter.track_target("target"):
                time.sleep(0.05)
            shooter.shoot(distance_m=2.0)
        """
        if not self._robot.is_vision_active():
            return False

        detections = self._robot.get_detections(class_name)
        if not detections:
            return False

        best = max(detections, key=lambda d: float(d["confidence"]))
        if float(best["confidence"]) < min_confidence:
            return False

        bbox = best["bbox"]
        img_w, img_h = self._robot.get_detection_image_size()
        if img_w <= 0 or img_h <= 0:
            return False

        cx = int(bbox["x"]) + int(bbox["width"]) // 2
        cy = int(bbox["y"]) + int(bbox["height"]) // 2
        err_x = cx - img_w // 2
        err_y = cy - img_h // 2

        if abs(err_x) <= center_tolerance_px and abs(err_y) <= center_tolerance_px:
            return True

        self.set_yaw(self._yaw_deg + err_x * yaw_gain_deg_per_px)
        self.set_pitch(self._pitch_deg - err_y * pitch_gain_deg_per_px)
        return False
