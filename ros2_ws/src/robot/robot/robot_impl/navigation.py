"""
robot_impl/navigation.py — NavigationMixin

Odometry, sensor fusion callbacks, pose API, drive commands, and
higher-level path following. Also defines MotionHandle since it is tightly
coupled to this layer.
All state lives in Robot.__init__; this mixin only defines methods.
"""
from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable

import numpy as np

from bridge_interfaces.msg import (
    FusedPose,
    SensorKinematics,
    SysOdomParamReq,
    SysOdomParamRsp,
    SysOdomParamSet,
    SysOdomReset,
    VirtualTarget,
)

from robot.hardware_map import DEFAULT_NAV_HZ, Unit
from robot.robot_impl.hardware import FirmwareState


class MotionHandle:
    """
    Handle returned by all high-level base-motion commands.

    blocking=True waits before returning; the handle is already finished.
    blocking=False returns immediately; poll or wait as needed.

    Completion contract:
        once is_finished() returns True, the navigation slot has been released
        and it is safe to start the next motion command immediately.

    Example:
        handle = robot.move_to(500, 0, 200, tolerance=15, blocking=False)
        # ... do other things ...
        success = handle.wait(timeout=10.0)
    """

    def __init__(self, finished_event: threading.Event, cancel_event: threading.Event) -> None:
        self._finished = finished_event
        self._cancel = cancel_event

    def wait(self, timeout: float = None) -> bool:
        """Block until motion finishes. Returns True if finished before timeout."""
        return self._finished.wait(timeout=timeout)

    def is_finished(self) -> bool:
        """Non-blocking poll. Returns True once the motion slot is released."""
        return self._finished.is_set()

    def is_done(self) -> bool:
        """Backward-compatible alias for is_finished()."""
        return self.is_finished()

    def cancel(self) -> None:
        """Abort this motion command. The robot will stop."""
        self._cancel.set()


# =============================================================================
# Module-level helpers (used by navigation internals)
# =============================================================================

def _dist2d(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def _wrap_angle(a: float) -> float:
    """Wrap angle to [−π, π]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# =============================================================================
# NavigationMixin
# =============================================================================

class NavigationMixin:
    """Odometry config, pose, drive commands, and path following."""

    # =========================================================================
    # Subscription callbacks
    # =========================================================================

    def _on_kinematics(self, msg: SensorKinematics) -> None:
        import time as _time  # local import to avoid shadowing module-level time
        with self._lock:
            self._pose = (msg.x, msg.y, msg.theta)
            self._vel  = (msg.vx, msg.vy, msg.v_theta)

            _initial_theta_rad = math.radians(self._initial_theta_deg)
            if self._ahrs_heading is not None:
                if self._odom_reset_pending:
                    if abs(math.atan2(
                        math.sin(msg.theta - _initial_theta_rad),
                        math.cos(msg.theta - _initial_theta_rad),
                    )) < math.radians(5.0):
                        self._odom_reset_pending = False
                        self._odom_reset_event.set()
                    relative_ahrs = None
                else:
                    relative_ahrs = self._ahrs_heading
            else:
                if self._odom_reset_pending:
                    if abs(math.atan2(
                        math.sin(msg.theta - _initial_theta_rad),
                        math.cos(msg.theta - _initial_theta_rad),
                    )) < math.radians(5.0):
                        self._odom_reset_pending = False
                        self._odom_reset_event.set()
                relative_ahrs = None

            linear_vel  = math.hypot(float(msg.vx), float(msg.vy))
            angular_vel = float(msg.v_theta)
            gps_fresh = (
                self._gps_last_time > 0.0
                and (_time.monotonic() - self._gps_last_time) < self._gps_timeout_s
            )
            self._fused_theta = self._orientation_fusion.update(
                odom_theta  = msg.theta,
                mag_heading = relative_ahrs,
                linear_vel  = linear_vel,
                angular_vel = angular_vel,
                fused_x     = self._fused_x_mm if gps_fresh else None,
                fused_y     = self._fused_y_mm if gps_fresh else None,
            )
            gps_x = self._gps_x_mm if gps_fresh else None
            gps_y = self._gps_y_mm if gps_fresh else None
            self._fused_x_mm, self._fused_y_mm = self._pos_fusion.update(
                msg.x, msg.y, gps_x, gps_y
            )
            if gps_fresh:
                self._fused_pose_available = True
            _raw_odom  = (float(msg.x), float(msg.y))
            _raw_fused = (self._fused_x_mm, self._fused_y_mm)

        self._odom_traj.append(_raw_odom)
        self._fused_traj.append(_raw_fused)

        _fp = FusedPose()
        _fp.header.stamp = self._node.get_clock().now().to_msg()
        _fp.x     = float(_raw_fused[0])
        _fp.y     = float(_raw_fused[1])
        _fp.theta = float(self._fused_theta)
        _fp.gps_active = bool(gps_fresh)
        self._pub_fused_pose.publish(_fp)

        self._pose_event.set()
        self._pose_event.clear()

    def _on_odom_param_rsp(self, msg: SysOdomParamRsp) -> None:
        if not self._odom_user_configured:
            self._apply_odom_param_snapshot(
                float(msg.wheel_diameter_mm),
                float(msg.wheel_base_mm),
                float(msg.initial_theta_deg),
                int(msg.left_motor_number),
                bool(msg.left_motor_dir_inverted),
                int(msg.right_motor_number),
                bool(msg.right_motor_dir_inverted),
            )
            return

        with self._lock:
            in_sync = (
                abs(float(msg.wheel_diameter_mm) - self._wheel_diameter) < 1e-3
                and abs(float(msg.wheel_base_mm) - self._wheel_base) < 1e-3
                and abs(float(msg.initial_theta_deg) - self._initial_theta_deg) < 1e-3
                and int(msg.left_motor_number) == self._left_wheel_motor
                and bool(msg.left_motor_dir_inverted) == self._left_wheel_dir_inverted
                and int(msg.right_motor_number) == self._right_wheel_motor
                and bool(msg.right_motor_dir_inverted) == self._right_wheel_dir_inverted
            )
            if in_sync:
                snapshot = None
            else:
                snapshot = (
                    self._wheel_diameter,
                    self._wheel_base,
                    self._initial_theta_deg,
                    self._left_wheel_motor,
                    self._left_wheel_dir_inverted,
                    self._right_wheel_motor,
                    self._right_wheel_dir_inverted,
                )

        if in_sync:
            self._odom_confirm_event.set()
        elif snapshot is not None:
            self._publish_odom_params(snapshot)

    # =========================================================================
    # Odometry configuration
    # =========================================================================

    def reset_odometry(self) -> None:
        """Reset firmware odometry pose to (0, 0, initial_theta).

        Captures the AHRS zero reference on the first kinematics tick after the
        firmware confirms the reset, so both references share the same timestamp.
        """
        with self._lock:
            self._odom_reset_pending = True
            self._odom_reset_event.clear()
            self._fused_pose_available = False
            self._gps_last_time = 0.0
            self._pos_fusion.reset()
            if hasattr(self._orientation_fusion, "reset"):
                self._orientation_fusion.reset()
        msg = SysOdomReset()
        msg.flags = 0
        self._odom_pub.publish(msg)

    def set_wheel_diameter_mm(self, wheel_diameter_mm: float) -> None:
        """Update wheel diameter in explicit millimeters."""
        self._update_odometry_params(wheel_diameter_mm=wheel_diameter_mm)

    def set_wheel_base_mm(self, wheel_base_mm: float) -> None:
        """Update wheel base in explicit millimeters."""
        self._update_odometry_params(wheel_base_mm=wheel_base_mm)

    def set_initial_theta(self, theta_deg: float) -> None:
        """Update the heading used by future odometry resets."""
        self._update_odometry_params(initial_theta_deg=theta_deg)

    def set_odom_left_motor(self, motor_id: int) -> None:
        """Select which DC motor is treated as the left wheel."""
        self._update_odometry_params(left_wheel_motor=motor_id)

    def set_odom_right_motor(self, motor_id: int) -> None:
        """Select which DC motor is treated as the right wheel."""
        self._update_odometry_params(right_wheel_motor=motor_id)

    def set_odom_motors(self, left_motor_id: int, right_motor_id: int) -> None:
        """Atomically configure both odometry wheel motors."""
        self._update_odometry_params(
            left_wheel_motor=left_motor_id,
            right_wheel_motor=right_motor_id,
        )

    def set_odom_left_motor_dir_inverted(self, inverted: bool) -> None:
        """Configure whether positive left-wheel counts mean reverse motion."""
        self._update_odometry_params(left_wheel_dir_inverted=inverted)

    def set_odom_right_motor_dir_inverted(self, inverted: bool) -> None:
        """Configure whether positive right-wheel counts mean reverse motion."""
        self._update_odometry_params(right_wheel_dir_inverted=inverted)

    def set_odometry_parameters(
        self,
        *,
        wheel_diameter: float | None = None,
        wheel_base: float | None = None,
        initial_theta_deg: float | None = None,
        left_motor_id: int | None = None,
        left_motor_dir_inverted: bool | None = None,
        right_motor_id: int | None = None,
        right_motor_dir_inverted: bool | None = None,
        timeout: float = 1.0,
    ) -> bool:
        """
        Publish a full odometry-parameter snapshot to firmware and wait for confirmation.

        wheel_diameter and wheel_base are in the current user unit system.
        timeout — seconds to wait for a matching firmware echo. Pass 0 to return
        immediately after publishing.
        Returns True if the firmware confirmed the params within timeout.
        """
        scale = self._unit.value
        return self._update_odometry_params(
            wheel_diameter_mm=None if wheel_diameter is None else float(wheel_diameter) * scale,
            wheel_base_mm=None if wheel_base is None else float(wheel_base) * scale,
            initial_theta_deg=initial_theta_deg,
            left_wheel_motor=left_motor_id,
            left_wheel_dir_inverted=left_motor_dir_inverted,
            right_wheel_motor=right_motor_id,
            right_wheel_dir_inverted=right_motor_dir_inverted,
            timeout=timeout,
        )

    def request_odometry_parameters(self) -> None:
        """Request the current firmware odometry parameter snapshot."""
        msg = SysOdomParamReq()
        msg.target = 0xFF
        self._odom_param_req_pub.publish(msg)

    def get_odometry_parameters(self) -> dict[str, float | int | bool]:
        """Return the latest local odometry parameter snapshot in firmware-native millimeters."""
        with self._lock:
            return {
                "wheel_diameter_mm": self._wheel_diameter,
                "wheel_base_mm": self._wheel_base,
                "initial_theta_deg": self._initial_theta_deg,
                "left_motor_number": self._left_wheel_motor,
                "left_motor_dir_inverted": self._left_wheel_dir_inverted,
                "right_motor_number": self._right_wheel_motor,
                "right_motor_dir_inverted": self._right_wheel_dir_inverted,
            }

    # =========================================================================
    # Pose / odometry
    # =========================================================================

    def get_odometry_pose(self) -> tuple[float, float, float]:
        """
        Return raw odometry as (x, y, theta_deg) in the current unit and degrees.

        This is always the direct `/sensor_kinematics` pose in the local
        odometry-reset frame, without GPS position fusion.
        """
        x_mm, y_mm, theta_rad = self._get_odometry_pose_mm()
        s = self._unit.value
        return x_mm / s, y_mm / s, math.degrees(theta_rad)

    def has_fused_pose(self) -> bool:
        """
        Return True once GPS has been incorporated into the position filter.

        The fused pose can remain available even while `is_gps_active()` is
        false, because the filter dead-reckons from the last fused anchor while
        GPS is temporarily stale.
        """
        with self._lock:
            return bool(self._fused_pose_available)

    def get_pose(self) -> tuple[float, float, float]:
        """
        Return the current navigation pose as (x, y, theta_deg).

        Before GPS has been incorporated, this is raw odometry. After the first
        accepted GPS fix has been incorporated, this becomes the GPS/odometry
        fused position. `theta_deg` is always the current navigation heading.
        """
        x_mm, y_mm, theta_rad = self._get_pose_mm()
        s = self._unit.value
        return x_mm / s, y_mm / s, math.degrees(theta_rad)

    def get_velocity(self) -> tuple[float, float, float]:
        """Return (vx, vy, v_theta_deg_s) in user units/s and degrees/s."""
        with self._lock:
            vx, vy, vt = self._vel
        s = self._unit.value
        return vx / s, vy / s, math.degrees(vt)

    def wait_for_pose_update(self, timeout: float = None) -> bool:
        """Block until the next /sensor_kinematics message arrives (~25 Hz)."""
        return self._pose_event.wait(timeout=timeout)

    def wait_for_odometry_reset(self, timeout: float = 2.0) -> bool:
        """Block until the firmware confirms the odometry reset (theta ≈ initial_theta).

        Returns True if the reset was confirmed within timeout, False on timeout.
        """
        return self._odom_reset_event.wait(timeout=timeout)

    def get_fused_pose(self) -> tuple[float, float, float] | None:
        """
        Return the fused navigation pose, or None if GPS has not been incorporated yet.
        """
        if not self.has_fused_pose():
            return None
        with self._lock:
            x_mm = self._fused_x_mm
            y_mm = self._fused_y_mm
            theta_rad = self._fused_theta
        s = self._unit.value
        return x_mm / s, y_mm / s, math.degrees(theta_rad)

    def get_virtual_target(self) -> tuple[float, float] | None:
        """Return the current leashed APF virtual target in the active user unit."""
        with self._lock:
            target = self._virtual_target_mm
        if target is None:
            return None
        scale = self._unit.value
        return float(target[0]) / scale, float(target[1]) / scale

    # =========================================================================
    # Obstacles
    # =========================================================================

    def set_obstacles(self, obstacles: list[tuple[float, float]]) -> None:
        """Cache APF obstacles in the robot frame using the current user length unit."""
        converted = np.float64([
            (
                self._require_finite_float("obstacle_x", obs_x) * self._unit.value,
                self._require_finite_float("obstacle_y", obs_y) * self._unit.value,
            )
            for obs_x, obs_y in obstacles
        ])
        with self._lock:
            self._obstacles_mm = converted

    def clear_obstacles(self) -> None:
        """Clear the cached APF obstacle list set by set_obstacles()."""
        with self._lock:
            self._obstacles_mm = np.float64([])

    def get_obstacles(self) -> list[tuple[float, float]]:
        """Return the current APF obstacles in the current user length unit."""
        scale = self._unit.value
        return [(x_mm / scale, y_mm / scale) for x_mm, y_mm in self._get_obstacles_mm()]

    def set_obstacle_provider(
        self,
        provider: Callable[[], list[tuple[float, float]]] | None,
    ) -> None:
        """Register a live APF obstacle callback that returns robot-frame positions in mm."""
        if provider is not None and not callable(provider):
            raise TypeError("provider must be callable or None")
        with self._lock:
            self._obstacle_provider = provider

    # =========================================================================
    # Differential drive — velocity commands
    # =========================================================================

    def set_velocity(self, linear: float, angular_deg_s: float) -> None:
        """
        Body-frame velocity command.
          linear        — forward speed in user units/s
          angular_deg_s — rotation rate in degrees/s (CCW positive)
        """
        linear_mm     = linear * self._unit.value
        angular_rad_s = math.radians(angular_deg_s)
        self._send_body_velocity_mm(linear_mm, angular_rad_s)

    def set_motor_velocity(self, motor_id: int, velocity: float) -> None:
        """Command a single DC motor by velocity in user units/s."""
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        self._send_motor_velocity_mm(motor_id, velocity * self._unit.value)

    def stop(self) -> None:
        """Zero velocity on both drive motors. Firmware stays in RUNNING state."""
        self._send_motor_velocity_mm(self._left_wheel_motor, 0.0)
        self._send_motor_velocity_mm(self._right_wheel_motor, 0.0)

    def disable_drive_motors(self) -> None:
        """Disable the currently configured drive motors."""
        with self._lock:
            drive_motors = tuple(dict.fromkeys((self._left_wheel_motor, self._right_wheel_motor)))
        for motor_id in drive_motors:
            self.disable_motor(motor_id)

    def shutdown(self) -> None:
        """Stop navigation and leave the drive motors disabled."""
        self.cancel_motion()
        self.stop()
        time.sleep(self._SHUTDOWN_SETTLE_S)
        self.disable_drive_motors()
        time.sleep(self._SHUTDOWN_SETTLE_S)
        if self.get_state() == FirmwareState.RUNNING:
            self.set_state(FirmwareState.IDLE, timeout=1.0)
        # self.save_trajectory_image()

    def set_left_wheel(self, motor_id: int) -> None:
        """Alias for set_odom_left_motor()."""
        self.set_odom_left_motor(motor_id)

    def set_right_wheel(self, motor_id: int) -> None:
        """Alias for set_odom_right_motor()."""
        self.set_odom_right_motor(motor_id)

    def set_drive_wheels(self, left_motor_id: int, right_motor_id: int) -> None:
        """Alias for set_odom_motors()."""
        self.set_odom_motors(left_motor_id, right_motor_id)

    def get_left_wheel(self) -> int:
        return self._left_wheel_motor

    def get_right_wheel(self) -> int:
        return self._right_wheel_motor

    # =========================================================================
    # Navigation (higher-level, runs a background thread)
    # =========================================================================

    def move_to(
        self,
        x: float,
        y: float,
        velocity: float,
        tolerance: float,
        blocking: bool = True,
        timeout: float = None,
    ) -> MotionHandle:
        """
        Navigate to (x, y) at the given speed using pure-pursuit steering.

        Always returns a MotionHandle.
        blocking=True waits before returning the handle.
        """
        x_mm   = x         * self._unit.value
        y_mm   = y         * self._unit.value
        vel_mm = velocity  * self._unit.value
        tol_mm = tolerance * self._unit.value
        lookahead_mm = max(tol_mm * 2.0, 100.0)

        def target():
            self._nav_follow_purepursuit_path([(x_mm, y_mm)], vel_mm, lookahead_mm, tol_mm, 2.0)

        return self._start_nav(target, blocking, timeout)

    def move_by(
        self,
        dx: float,
        dy: float,
        velocity: float,
        tolerance: float,
        blocking: bool = True,
        timeout: float = None,
    ) -> MotionHandle:
        """Navigate by (dx, dy) relative to current pose."""
        cur_x, cur_y, _ = self.get_pose()
        return self.move_to(cur_x + dx, cur_y + dy, velocity,
                            blocking=blocking, tolerance=tolerance, timeout=timeout)

    def move_forward(
        self,
        distance: float,
        velocity: float,
        tolerance: float,
        blocking: bool = True,
        timeout: float = None,
    ) -> MotionHandle:
        """Drive straight forward by distance along the robot's current heading."""
        distance = self._require_positive_float("distance", distance)
        dist_mm  = distance  * self._unit.value
        vel_mm   = velocity  * self._unit.value
        tol_mm   = tolerance * self._unit.value

        def target():
            self._nav_drive_straight(dist_mm, vel_mm, tol_mm)

        return self._start_nav(target, blocking, timeout)

    def move_backward(
        self,
        distance: float,
        velocity: float,
        tolerance: float,
        blocking: bool = True,
        timeout: float = None,
    ) -> MotionHandle:
        """Drive straight backward by distance along the robot's current heading."""
        distance = self._require_positive_float("distance", distance)
        dist_mm  = distance  * self._unit.value
        vel_mm   = velocity  * self._unit.value
        tol_mm   = tolerance * self._unit.value

        def target():
            self._nav_drive_straight(-dist_mm, vel_mm, tol_mm)

        return self._start_nav(target, blocking, timeout)

    def turn_to(
        self,
        angle_deg: float,
        blocking: bool = True,
        tolerance_deg: float = 2.0,
        timeout: float = None,
    ) -> MotionHandle:
        """
        Rotate to an absolute heading. Firmware must be in RUNNING state.
        angle_deg is in degrees (CCW positive, same convention as the firmware).

        Always returns a MotionHandle.
        """
        target_rad = math.radians(angle_deg)
        tol_rad    = math.radians(tolerance_deg)

        def target():
            self._turn_to_heading(target_rad, tol_rad, max_angular_rad=1.0)

        return self._start_nav(target, blocking, timeout)

    def turn_by(
        self,
        delta_deg: float,
        blocking: bool = True,
        tolerance_deg: float = 2.0,
        timeout: float = None,
    ) -> MotionHandle:
        """Rotate by delta_deg relative to current heading."""
        _, _, cur_deg = self.get_pose()
        return self.turn_to(cur_deg + delta_deg, blocking=blocking,
                            tolerance_deg=tolerance_deg, timeout=timeout)

    def purepursuit_follow_path(
        self,
        waypoints: list[tuple[float, float]],
        velocity: float,
        lookahead: float,
        tolerance: float,
        blocking: bool = True,
        max_angular_rad_s: float = 1.0,
        timeout: float = None,
        *,
        advance_radius: float | None = None,
    ) -> MotionHandle:
        """
        Follow an ordered waypoint path with pure pursuit.

        waypoints           — [(x, y), ...] in the current user unit system
        velocity            — maximum forward speed in user units/s
        lookahead           — lookahead distance in user units
        tolerance           — goal tolerance in user units
        advance_radius      — optional intermediate waypoint acceptance radius
                              in user units; defaults to tolerance
        max_angular_rad_s   — angular-rate clamp in rad/s

        Always returns a MotionHandle.
        """
        if not waypoints:
            raise ValueError("waypoints must not be empty")

        path_mm = [(float(x) * self._unit.value, float(y) * self._unit.value) for x, y in waypoints]
        vel_mm       = float(velocity)   * self._unit.value
        lookahead_mm = float(lookahead)  * self._unit.value
        tolerance_mm = float(tolerance)  * self._unit.value
        advance_radius_mm = (
            tolerance_mm if advance_radius is None
            else float(advance_radius) * self._unit.value
        )
        max_angular = float(max_angular_rad_s)

        def target():
            self._nav_follow_purepursuit_path(
                path_mm, vel_mm, lookahead_mm, advance_radius_mm, tolerance_mm, max_angular
            )

        return self._start_nav(target, blocking, timeout)

    def apf_follow_path(
        self,
        waypoints: list[tuple[float, float]],
        velocity: float,
        lookahead: float,
        tolerance: float,
        repulsion_range: float,
        blocking: bool = True,
        max_angular_rad_s: float = 1.0,
        repulsion_gain: float = 500.0,
        timeout: float = None,
        robot_front_mm: float = 400.0,
        robot_rear_mm: float = 100.0,
        robot_half_width_mm: float = 200.0,
        *,
        advance_radius: float | None = None,
    ) -> MotionHandle:
        """
        Follow an ordered waypoint path with artificial potential fields.

        Obstacles come from set_obstacles() and/or set_obstacle_provider().

        repulsion_range is the physical air-gap clearance (mm) between the
        robot surface and the obstacle surface at which repulsion begins.
        Robot geometry (rectangle in robot frame):
          robot_front_mm     — axle to front face (rear-drive: nose distance)
          robot_rear_mm      — axle to rear face  (front-drive: tail distance)
          robot_half_width_mm — half-width of the body
        """
        if not waypoints:
            raise ValueError("waypoints must not be empty")

        path_mm = [(float(x) * self._unit.value, float(y) * self._unit.value) for x, y in waypoints]
        vel_mm            = float(velocity)        * self._unit.value
        lookahead_mm      = float(lookahead)       * self._unit.value
        tolerance_mm      = float(tolerance)       * self._unit.value
        advance_radius_mm = (
            tolerance_mm if advance_radius is None
            else float(advance_radius) * self._unit.value
        )
        repulsion_range_mm = float(repulsion_range) * self._unit.value
        max_angular        = float(max_angular_rad_s)
        repulsion_gain     = float(repulsion_gain)
        front_mm           = float(robot_front_mm)
        rear_mm            = float(robot_rear_mm)
        half_width_mm      = float(robot_half_width_mm)

        def target():
            self._nav_follow_apf_path(
                path_mm, vel_mm, lookahead_mm, advance_radius_mm, tolerance_mm,
                repulsion_range_mm, max_angular, repulsion_gain,
                front_mm, rear_mm, half_width_mm,
            )

        return self._start_nav(target, blocking, timeout)

    def lapf_to_goal(
        self,
        x: float,
        y: float,
        velocity: float,
        tolerance: float,
        leash_length_mm: float | None = None,
        repulsion_range_mm: float | None = None,
        target_speed_mm_s: float | None = None,
        blocking: bool = True,
        max_angular_rad_s: float = 1.0,
        repulsion_gain: float | None = None,
        attraction_gain: float | None = None,
        force_ema_alpha: float | None = None,
        inflation_margin_mm: float | None = None,
        leash_half_angle_deg: float | None = None,
        timeout: float = None,
    ) -> MotionHandle:
        """
        Navigate to one goal using a leashed APF virtual target.

        The virtual target runs a point-based APF simulation in world frame,
        constrained to remain inside a forward leash cone relative to the robot.
        The real robot tracks that moving target with a single-point
        pure-pursuit-derived controller.
        """
        goal_x_mm = float(x) * self._unit.value
        goal_y_mm = float(y) * self._unit.value
        vel_mm = float(velocity) * self._unit.value
        tolerance_mm = float(tolerance) * self._unit.value
        leash_length_mm = float(
            self.LAPF_LEASH_LENGTH_MM if leash_length_mm is None else leash_length_mm
        )
        repulsion_range_mm = float(
            self.LAPF_REPULSION_RANGE_MM if repulsion_range_mm is None else repulsion_range_mm
        )
        target_speed_mm_s = float(
            self.LAPF_TARGET_SPEED_MM_S if target_speed_mm_s is None else target_speed_mm_s
        )
        max_angular = float(max_angular_rad_s)
        repulsion_gain = float(
            self.LAPF_REPULSION_GAIN if repulsion_gain is None else repulsion_gain
        )
        attraction_gain = float(
            self.LAPF_ATTRACTION_GAIN if attraction_gain is None else attraction_gain
        )
        force_ema_alpha = float(
            self.LAPF_FORCE_EMA_ALPHA if force_ema_alpha is None else force_ema_alpha
        )
        inflation_margin_mm = float(
            self.LAPF_INFLATION_MARGIN_MM if inflation_margin_mm is None else inflation_margin_mm
        )
        leash_half_angle_deg = float(
            self.LAPF_LEASH_HALF_ANGLE_DEG if leash_half_angle_deg is None else leash_half_angle_deg
        )

        def target():
            self._nav_lapf_to_goal(
                (goal_x_mm, goal_y_mm),
                vel_mm,
                tolerance_mm,
                leash_length_mm,
                repulsion_range_mm,
                target_speed_mm_s,
                max_angular,
                repulsion_gain,
                attraction_gain,
                force_ema_alpha,
                inflation_margin_mm,
                leash_half_angle_deg,
            )

        return self._start_nav(target, blocking, timeout)

    def is_moving(self) -> bool:
        """True if a navigation command is active."""
        with self._nav_lock:
            nav_thread = self._nav_thread
        return nav_thread is not None and nav_thread.is_alive()

    def cancel_motion(self) -> None:
        """Abort any active navigation command. The robot stops."""
        self._nav_cancel.set()
        with self._nav_lock:
            nav_thread = self._nav_thread

        if nav_thread is not None:
            nav_thread.join(timeout=1.0)
            if nav_thread.is_alive():
                return

        with self._nav_lock:
            if self._nav_thread is nav_thread:
                self._nav_thread = None
        self._nav_cancel.clear()

    # =========================================================================
    # Trajectory
    # =========================================================================

    def save_trajectory_image(self, path: str = "trajectory.png") -> None:
        """Save a PNG comparing raw odometry vs fused trajectory. Requires matplotlib."""
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            self._node.get_logger().error("matplotlib not installed; cannot save trajectory image")
            return

        odom  = list(self._odom_traj)
        fused = list(self._fused_traj)
        if not odom:
            self._node.get_logger().warn("No trajectory data to save")
            return

        fig, ax = plt.subplots(figsize=(8, 8))
        ox, oy = zip(*odom)
        ax.plot(ox, oy, label="odometry (raw)", color="steelblue", linewidth=1.5)
        if fused:
            fx, fy = zip(*fused)
            ax.plot(fx, fy, label="fused (GPS+odom)", color="darkorange", linewidth=1.5, linestyle="--")
        ax.scatter([ox[0]], [oy[0]], color="green", zorder=5, label="start")
        ax.scatter([ox[-1]], [oy[-1]], color="red",   zorder=5, label="end")
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("y (mm)")
        ax.set_title("Trajectory: raw odometry vs fused")
        ax.legend()
        ax.set_aspect("equal")
        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)
        self._node.get_logger().info(f"Trajectory image saved to {path}")

    # =========================================================================
    # Units
    # =========================================================================

    def set_unit(self, unit: Unit) -> None:
        """Change the unit system. Does not affect already-sent commands."""
        self._unit = unit

    def get_unit(self) -> Unit:
        return self._unit

    # =========================================================================
    # Internal — navigation thread management
    # =========================================================================

    def _start_nav(self, target_fn, blocking: bool, timeout: float) -> MotionHandle:
        """Start a new navigation thread. Only one base-motion command may run at a time."""
        with self._nav_lock:
            if self._nav_thread is not None and self._nav_thread.is_alive():
                raise RuntimeError("Another motion is still running")
            self._nav_done.clear()
            self._nav_cancel.clear()
        self._set_virtual_target_world_mm(None)

        def runner() -> None:
            try:
                target_fn()
            except Exception as exc:
                self._node.get_logger().error(f"motion thread failed: {exc}")
                try:
                    self.stop()
                except Exception:
                    pass
            finally:
                self._set_virtual_target_world_mm(None)
                with self._nav_lock:
                    if self._nav_thread is threading.current_thread():
                        self._nav_thread = None
                self._nav_done.set()

        nav_thread = threading.Thread(target=runner, daemon=True)
        with self._nav_lock:
            self._nav_thread = nav_thread
        nav_thread.start()
        handle = MotionHandle(self._nav_done, self._nav_cancel)
        if blocking:
            handle.wait(timeout=timeout)
        return handle

    def _move_along_heading(
        self,
        signed_distance: float,
        velocity: float,
        tolerance: float,
        blocking: bool = True,
        timeout: float = None,
    ) -> MotionHandle:
        cur_x, cur_y, cur_theta_deg = self.get_pose()
        cur_theta_rad = math.radians(cur_theta_deg)
        dx = math.cos(cur_theta_rad) * signed_distance
        dy = math.sin(cur_theta_rad) * signed_distance
        return self.move_to(cur_x + dx, cur_y + dy, velocity,
                            tolerance=tolerance, blocking=blocking, timeout=timeout)

    def _nav_drive_straight(
        self,
        signed_distance_mm: float,
        velocity_mm: float,
        tolerance_mm: float,
        heading_kp: float = 3.0,
        update_hz: float = float(DEFAULT_NAV_HZ),
    ) -> None:
        """
        Navigation thread body: drive straight along the heading at call time.

        Uses a P-controller on heading error to resist drift — no path planner,
        no lookahead, no steering toward a world-frame target.  This guarantees
        the robot reverses straight back rather than arcing toward a goal point.

        signed_distance_mm > 0 → forward, < 0 → backward.
        velocity_mm is always positive; direction is taken from the sign.
        """
        x0_mm, y0_mm, theta0_rad = self._get_pose_mm()
        direction = 1 if signed_distance_mm >= 0 else -1
        dt = 1.0 / update_hz

        while not self._nav_cancel.is_set():
            x_mm, y_mm, theta_rad = self._get_pose_mm()

            # Signed distance traveled along the initial heading axis
            traveled = ((x_mm - x0_mm) * math.cos(theta0_rad)
                        + (y_mm - y0_mm) * math.sin(theta0_rad))
            remaining = signed_distance_mm - traveled

            if abs(remaining) <= tolerance_mm:
                self.stop()
                return

            # Heading error: positive when robot has turned CCW from the initial heading
            heading_err = _wrap_angle(theta0_rad - theta_rad)

            # Use the same heading correction sign for forward and reverse travel.
            # If the robot has drifted CCW from the launch heading, it should turn
            # back CW regardless of whether it is moving forward or backward.
            angular = heading_kp * heading_err

            # Decelerate smoothly within 3× tolerance, but keep enough speed to move.
            speed = min(velocity_mm, max(velocity_mm * 0.25, abs(remaining) * 2.0))
            linear = direction * speed

            self._send_body_velocity_mm(linear, angular)
            if not self._sleep_with_cancel(dt):
                break

        self.stop()

    def _nav_follow_purepursuit_path(
        self,
        waypoints_mm: list[tuple[float, float]],
        max_vel_mm: float,
        lookahead_mm: float,
        advance_radius_mm: float,
        tolerance_mm: float,
        max_angular_rad_s: float = 1.0,
        update_hz: float = float(DEFAULT_NAV_HZ),
    ) -> None:
        """Navigation thread body: pure-pursuit to an ordered list of waypoints (mm)."""
        from robot.path_planner import PurePursuitPlanner
        planner = PurePursuitPlanner(
            lookahead_dist=lookahead_mm,
            max_angular=max_angular_rad_s,
            goal_tolerance=tolerance_mm,
        )
        self._nav_follow_path(
            waypoints_mm, planner, max_vel_mm, advance_radius_mm, tolerance_mm,
            update_hz=update_hz,
        )

    def _nav_follow_apf_path(
        self,
        waypoints_mm: list[tuple[float, float]],
        max_vel_mm: float,
        lookahead_mm: float,
        advance_radius_mm: float,
        tolerance_mm: float,
        repulsion_range_mm: float,
        max_angular_rad_s: float,
        repulsion_gain: float,
        robot_front_mm: float = 400.0,
        robot_rear_mm: float = 100.0,
        robot_half_width_mm: float = 200.0,
        update_hz: float = float(DEFAULT_NAV_HZ),
    ) -> None:
        """Navigation thread body: APF path following via sequential single-goal APF.

        ``APFPlanner`` itself is a single-goal planner. This wrapper advances
        through the remaining waypoint list and asks APF for the velocity
        command toward the current goal waypoint while avoiding the live lidar
        obstacle cloud.
        """
        from robot.path_planner import APFPlanner
        planner = APFPlanner(
            max_linear=max_vel_mm,
            max_angular=max_angular_rad_s,
            repulsion_gain=repulsion_gain,
            repulsion_range=repulsion_range_mm,
            goal_tolerance=tolerance_mm,
            robot_front_mm=robot_front_mm,
            robot_rear_mm=robot_rear_mm,
            robot_half_width_mm=robot_half_width_mm,
        )
        remaining_path = list(waypoints_mm)
        dt = 1.0 / update_hz

        # Fetch radius: rep_range clearance off the farthest rectangle corner.
        fetch_radius_mm = (repulsion_range_mm + robot_front_mm + robot_half_width_mm
                           + float(self.APF_TRACK_INPUT_MARGIN_MM))

        while not self._nav_cancel.is_set():
            x_mm, y_mm, theta_rad = self._get_pose_mm()
            remaining_path = self._advance_remaining_path(
                remaining_path, x_mm, y_mm, advance_radius_mm
            )

            goal_x_mm, goal_y_mm = remaining_path[0]
            if len(remaining_path) == 1 and _dist2d(x_mm, y_mm, goal_x_mm, goal_y_mm) <= tolerance_mm:
                self.stop()
                return

            obstacle_disks = self._get_nearest_tracked_obstacle_disks_world_mm(
                (x_mm, y_mm, theta_rad),
                fetch_radius_mm,
                int(self.APF_MAX_PLANNER_TRACKS),
            )

            linear_mm, angular_rad_s = planner.navigate_to_goal(
                (x_mm, y_mm, theta_rad),
                (goal_x_mm, goal_y_mm),
                obstacle_disks,
            )
            self._send_body_velocity_mm(linear_mm, angular_rad_s)
            if not self._sleep_with_cancel(dt):
                break

        self.stop()

    def _nav_lapf_to_goal(
        self,
        goal_mm: tuple[float, float],
        max_vel_mm: float,
        tolerance_mm: float,
        leash_length_mm: float,
        repulsion_range_mm: float,
        target_speed_mm_s: float,
        max_angular_rad_s: float,
        repulsion_gain: float,
        attraction_gain: float,
        force_ema_alpha: float,
        inflation_margin_mm: float,
        leash_half_angle_deg: float,
        update_hz: float = float(DEFAULT_NAV_HZ),
    ) -> None:
        from robot.path_planner import LeashedAPFPlanner

        tracker_lookahead_mm = max(100.0, min(leash_length_mm, 250.0))
        planner = LeashedAPFPlanner(
            max_linear=max_vel_mm,
            max_angular=max_angular_rad_s,
            target_speed=target_speed_mm_s,
            repulsion_gain=repulsion_gain,
            repulsion_range=repulsion_range_mm,
            goal_tolerance=tolerance_mm,
            attraction_gain=attraction_gain,
            force_ema_alpha=force_ema_alpha,
            leash_length_mm=leash_length_mm,
            leash_half_angle_deg=leash_half_angle_deg,
            inflation_margin_mm=inflation_margin_mm,
            tracker_lookahead_mm=tracker_lookahead_mm,
        )
        dt = 1.0 / update_hz
        fetch_radius_mm = (
            repulsion_range_mm
            + leash_length_mm
            + inflation_margin_mm
            + float(self.APF_TRACK_INPUT_MARGIN_MM)
        )

        while not self._nav_cancel.is_set():
            pose_mm = self._get_pose_mm()
            x_mm, y_mm, _theta_rad = pose_mm
            goal_x_mm, goal_y_mm = goal_mm

            if _dist2d(x_mm, y_mm, goal_x_mm, goal_y_mm) <= tolerance_mm:
                self.stop()
                self._set_virtual_target_world_mm(None)
                return

            obstacle_disks = self._get_nearest_tracked_obstacle_disks_world_mm(
                pose_mm,
                fetch_radius_mm,
                int(self.LAPF_MAX_PLANNER_TRACKS),
            )

            linear_mm, angular_rad_s = planner.navigate_to_goal(
                pose_mm,
                goal_mm,
                obstacle_disks,
                dt,
            )
            self._set_virtual_target_world_mm(planner.get_virtual_target())
            self._send_body_velocity_mm(linear_mm, angular_rad_s)
            if not self._sleep_with_cancel(dt):
                break

        self.stop()
        self._set_virtual_target_world_mm(None)

    def _nav_follow_path(
        self,
        waypoints_mm: list[tuple[float, float]],
        planner,
        max_vel_mm: float,
        advance_radius_mm: float,
        tolerance_mm: float,
        update_hz: float = float(DEFAULT_NAV_HZ),
    ) -> None:
        """Shared path-following loop for pure pursuit and APF planners."""
        remaining_path = list(waypoints_mm)
        dt = 1.0 / update_hz

        while not self._nav_cancel.is_set():
            x_mm, y_mm, theta_rad = self._get_pose_mm()
            remaining_path = self._advance_remaining_path(
                remaining_path, x_mm, y_mm, advance_radius_mm
            )

            goal_x_mm, goal_y_mm = remaining_path[0]
            if len(remaining_path) == 1 and _dist2d(x_mm, y_mm, goal_x_mm, goal_y_mm) <= tolerance_mm:
                self.stop()
                return

            linear_mm, angular_rad_s = planner.compute_velocity(
                (x_mm, y_mm, theta_rad), remaining_path, max_vel_mm
            )
            self._send_body_velocity_mm(linear_mm, angular_rad_s)
            if not self._sleep_with_cancel(dt):
                break

        self.stop()

    @staticmethod
    def _advance_remaining_path(
        remaining_path: list[tuple[float, float]],
        x_mm: float,
        y_mm: float,
        advance_radius_mm: float,
    ) -> list[tuple[float, float]]:
        """Drop intermediate waypoints once within advance_radius; never drop the last."""
        while len(remaining_path) > 1:
            next_x_mm, next_y_mm = remaining_path[0]
            if _dist2d(x_mm, y_mm, next_x_mm, next_y_mm) > advance_radius_mm:
                break
            remaining_path.pop(0)
        return remaining_path

    def _turn_to_heading(
        self,
        target_rad: float,
        tolerance_rad: float,
        max_angular_rad: float = 2.0,
        update_hz: float = float(DEFAULT_NAV_HZ),
    ) -> None:
        """Navigation thread body: rotate to target_rad in place."""
        dt = 1.0 / update_hz
        while not self._nav_cancel.is_set():
            _, _, theta_rad = self._get_pose_mm()
            error = _wrap_angle(target_rad - theta_rad)
            if abs(error) < tolerance_rad:
                self.stop()
                return
            angular = max(-max_angular_rad, min(max_angular_rad, error * 3.0))
            self._send_body_velocity_mm(0.0, angular)
            if not self._sleep_with_cancel(dt):
                break
        self.stop()

    def _sleep_with_cancel(self, seconds: float) -> bool:
        """Wait for up to seconds, waking early if motion is cancelled.
        Returns True on full interval, False on cancel."""
        return not self._nav_cancel.wait(timeout=seconds)

    # =========================================================================
    # Internal — low-level helpers
    # =========================================================================

    def _get_pose_mm(self) -> tuple[float, float, float]:
        """Return the current navigation pose (raw odom or fused) without unit conversion."""
        if not self.has_fused_pose():
            return self._get_odometry_pose_mm()
        with self._lock:
            return (self._fused_x_mm, self._fused_y_mm, self._fused_theta)

    def _get_odometry_pose_mm(self) -> tuple[float, float, float]:
        """Return raw odometry (x_mm, y_mm, theta_rad) without unit conversion."""
        with self._lock:
            return (
                float(self._pose[0]),
                float(self._pose[1]),
                float(self._pose[2]),
            )

    def _get_obstacles_mm(self) -> list[tuple[float, float]]:
        """Return cached and provider-supplied APF obstacles in robot-frame millimeters."""
        with self._lock:
            cached = list(self._obstacles_mm)
            provider = self._obstacle_provider

        dynamic: list[tuple[float, float]] = []
        if provider is not None:
            try:
                provided = provider() or []
                dynamic = [
                    (
                        self._require_finite_float("obstacle_x_mm", obs_x),
                        self._require_finite_float("obstacle_y_mm", obs_y),
                    )
                    for obs_x, obs_y in provided
                ]
            except Exception as exc:
                self._node.get_logger().error(f"obstacle provider failed: {exc}")

        return cached + dynamic

    def _get_nearest_tracked_obstacle_disks_world_mm(
        self,
        pose_mm: tuple[float, float, float],
        max_distance_mm: float,
        max_count: int,
    ) -> np.ndarray:
        x_mm, y_mm, _theta_rad = pose_mm
        tracks = self.get_obstacle_tracks(include_unconfirmed=False)
        if not tracks:
            return np.empty((0, 3), dtype=float)

        ranked: list[tuple[float, dict[str, float | int]]] = []
        for track in tracks:
            cx = float(track["x"])
            cy = float(track["y"])
            radius = float(track["radius"])
            boundary_distance = max(0.0, math.hypot(cx - x_mm, cy - y_mm) - radius)
            if boundary_distance <= max_distance_mm:
                ranked.append((boundary_distance, track))

        ranked.sort(key=lambda item: item[0])
        selected = ranked[: max(0, int(max_count))]
        if not selected:
            return np.empty((0, 3), dtype=float)

        return np.asarray(
            [
                [float(track["x"]), float(track["y"]), float(track["radius"])]
                for _distance, track in selected
            ],
            dtype=float,
        )

    def _set_virtual_target_world_mm(self, target_mm: tuple[float, float] | None) -> None:
        with self._lock:
            self._virtual_target_mm = None if target_mm is None else (
                float(target_mm[0]),
                float(target_mm[1]),
            )

        msg = VirtualTarget()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        if target_mm is None:
            msg.active = False
            msg.x = 0.0
            msg.y = 0.0
        else:
            msg.active = True
            msg.x = float(target_mm[0])
            msg.y = float(target_mm[1])
        self._pub_virtual_target.publish(msg)

    def _apply_odom_param_snapshot(
        self,
        wheel_diameter_mm: float,
        wheel_base_mm: float,
        initial_theta_deg: float,
        left_motor_number: int,
        left_motor_dir_inverted: bool,
        right_motor_number: int,
        right_motor_dir_inverted: bool,
    ) -> None:
        with self._lock:
            self._wheel_diameter = self._require_positive_float("wheel_diameter_mm", wheel_diameter_mm)
            self._wheel_base = self._require_positive_float("wheel_base_mm", wheel_base_mm)
            self._initial_theta_deg = self._require_finite_float("initial_theta_deg", initial_theta_deg)
            self._left_wheel_motor = self._require_id("left_motor_number", left_motor_number, 1, 4)
            self._right_wheel_motor = self._require_id("right_motor_number", right_motor_number, 1, 4)
            if self._left_wheel_motor == self._right_wheel_motor:
                raise ValueError("left and right odometry motors must be different")
            self._left_wheel_dir_inverted  = bool(left_motor_dir_inverted)
            self._right_wheel_dir_inverted = bool(right_motor_dir_inverted)
            self._ticks_per_mm = self._encoder_ppr / (math.pi * self._wheel_diameter)

    def _update_odometry_params(
        self,
        *,
        wheel_diameter_mm: float | None = None,
        wheel_base_mm: float | None = None,
        initial_theta_deg: float | None = None,
        left_wheel_motor: int | None = None,
        left_wheel_dir_inverted: bool | None = None,
        right_wheel_motor: int | None = None,
        right_wheel_dir_inverted: bool | None = None,
        timeout: float = 0.0,
    ) -> bool:
        with self._lock:
            next_wheel_diameter = self._wheel_diameter if wheel_diameter_mm is None else \
                self._require_positive_float("wheel_diameter_mm", wheel_diameter_mm)
            next_wheel_base = self._wheel_base if wheel_base_mm is None else \
                self._require_positive_float("wheel_base_mm", wheel_base_mm)
            next_initial_theta = self._initial_theta_deg if initial_theta_deg is None else \
                self._require_finite_float("initial_theta_deg", initial_theta_deg)
            next_left_motor = self._left_wheel_motor if left_wheel_motor is None else \
                self._require_id("left_motor_id", left_wheel_motor, 1, 4)
            next_right_motor = self._right_wheel_motor if right_wheel_motor is None else \
                self._require_id("right_motor_id", right_wheel_motor, 1, 4)
            if next_left_motor == next_right_motor:
                raise ValueError("left and right odometry motors must be different")
            next_left_inverted  = self._left_wheel_dir_inverted  if left_wheel_dir_inverted  is None else bool(left_wheel_dir_inverted)
            next_right_inverted = self._right_wheel_dir_inverted if right_wheel_dir_inverted is None else bool(right_wheel_dir_inverted)

        self._apply_odom_param_snapshot(
            next_wheel_diameter, next_wheel_base, next_initial_theta,
            next_left_motor, next_left_inverted, next_right_motor, next_right_inverted,
        )

        snapshot = (
            next_wheel_diameter, next_wheel_base, next_initial_theta,
            next_left_motor, next_left_inverted, next_right_motor, next_right_inverted,
        )

        self._odom_confirm_event.clear()
        self._odom_user_configured = True
        self._publish_odom_params(snapshot)
        if timeout <= 0.0:
            return True

        deadline = time.monotonic() + timeout
        confirmed = False

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            wait_slice = min(0.1, remaining)
            if self._odom_confirm_event.wait(timeout=wait_slice):
                confirmed = True
                break
            self.request_odometry_parameters()

        if not confirmed:
            self._node.get_logger().warning(
                f"[robot] set_odometry_parameters: firmware did not confirm within "
                f"{timeout}s — bridge may not be connected yet or firmware rejected params"
            )
        return confirmed

    def _publish_odom_params(
        self,
        snapshot: tuple[float, float, float, int, bool, int, bool],
    ) -> None:
        msg = SysOdomParamSet()
        (
            msg.wheel_diameter_mm,
            msg.wheel_base_mm,
            msg.initial_theta_deg,
            msg.left_motor_number,
            msg.left_motor_dir_inverted,
            msg.right_motor_number,
            msg.right_motor_dir_inverted,
        ) = snapshot
        self._odom_param_pub.publish(msg)

    def _send_body_velocity_mm(self, linear_mm_s: float, angular_rad_s: float) -> None:
        """Diff-drive mixing and publish. All values in mm/s and rad/s."""
        with self._lock:
            half_wb    = self._wheel_base / 2.0
            left_motor = self._left_wheel_motor
            right_motor = self._right_wheel_motor
            left_dir_inverted  = self._left_wheel_dir_inverted
            right_dir_inverted = self._right_wheel_dir_inverted
        if linear_mm_s or angular_rad_s:
            self._ensure_drive_motors_enabled()
        left_velocity  = linear_mm_s - angular_rad_s * half_wb
        right_velocity = linear_mm_s + angular_rad_s * half_wb
        if left_dir_inverted:
            left_velocity  = -left_velocity
        if right_dir_inverted:
            right_velocity = -right_velocity
        self._send_motor_velocity_mm(left_motor,  left_velocity)
        self._send_motor_velocity_mm(right_motor, right_velocity)
