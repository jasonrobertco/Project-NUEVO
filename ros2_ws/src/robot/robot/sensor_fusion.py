"""
sensor_fusion.py — pluggable sensor fusion strategies
======================================================
Orientation strategies share this interface::

    fused_theta_rad = strategy.update(
        odom_theta,
        mag_heading,
        linear_vel,
        angular_vel,
        fused_x=None,
        fused_y=None,
    )

Position strategies share this interface::

    fused_x_mm, fused_y_mm = strategy.update(odom_x, odom_y, gps_x, gps_y)

Usage in robot.py / main.py
----------------------------
    from robot.sensor_fusion import AdaptiveComplementaryFilter
    robot.set_orientation_fusion_strategy(AdaptiveComplementaryFilter())
"""

from __future__ import annotations

import math


def _wrap(angle: float) -> float:
    """Wrap angle difference to [-π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# =============================================================================
# Abstract base
# =============================================================================

class SensorFusion:
    """Abstract base for all sensor fusion strategies."""
    def __init__(self):
        self.measurement_type = None  # "orientation" or "position"

    def update(self, *args, **kwargs):
        """Return fused estimate. Subclasses define the exact signature."""
        raise NotImplementedError


# =============================================================================
# Complementary filter (fixed alpha)
# =============================================================================

class OrientationComplementaryFilter(SensorFusion):
    """
    Fixed-weight complementary filter.

    Blends the AHRS heading (absolute but noisy) with the odometry
    heading (smooth but drifts) using a constant weight::

        fused = odom_theta + alpha * wrap(mag_heading - odom_theta)

    Parameters
    ----------
    alpha : float, 0.0–1.0
        AHRS heading weight.  0 = pure odometry, 1 = pure AHRS.
        Default 0.02 gives gentle long-term drift correction.
    """

    def __init__(self, alpha: float = 0.0) -> None:
        super().__init__()
        self.alpha = max(0.0, min(1.0, float(alpha)))
        self.measurement_type = "orientation"

    def update(
        self,
        odom_theta: float,
        mag_heading: float | None,
        linear_vel: float,
        angular_vel: float,
        fused_x: float | None = None,
        fused_y: float | None = None,
    ) -> float:
        # fused_x / fused_y are accepted for interface compatibility with
        # GPS-tangent-based orientation fusion and are intentionally ignored.
        if mag_heading is None:
            return odom_theta
        return odom_theta + self.alpha * _wrap(mag_heading - odom_theta)


# =============================================================================
# Position complementary filter
# =============================================================================

class GpsTangentOrientationFusion(SensorFusion):
    """
    Derives heading from the direction of GPS-fused position movement.

    Accumulates fused-position samples and recomputes the heading estimate once
    the robot has travelled at least ``min_displacement_mm`` from the last
    reference point.  The estimate is then blended into the odometry heading
    with a complementary filter, exactly like OrientationComplementaryFilter::

        fused_theta = odom_theta + alpha * wrap(gps_tangent - odom_theta)

    Two stability gates prevent bad updates:

    1. **Displacement gate** – only fires when the robot has moved far enough
       that GPS noise is small relative to the displacement vector.
    2. **Forward-motion gate** – checks the dot product of the displacement
       vector against the current odometry heading.  A negative result means
       the robot is reversing; the reference is advanced but the tangent is
       not updated.

    When GPS is not fresh (``fused_x``/``fused_y`` passed as ``None``), the
    last accepted tangent continues to correct odom drift.  The correction
    drops out entirely before the first valid displacement sample is accepted.

    Parameters
    ----------
    alpha : float
        Heading correction weight.  0 = pure odometry, 1 = snap to GPS
        tangent.  Default 0.15 — gentle long-term drift correction.
    min_displacement_mm : float
        Minimum travel between reference updates before a new tangent is
        accepted.  Default 200 mm.
    """

    def __init__(
        self,
        alpha: float = 0.15,
        min_displacement_mm: float = 200.0,
    ) -> None:
        super().__init__()
        self.alpha = max(0.0, min(1.0, float(alpha)))
        self.min_displacement_mm = float(min_displacement_mm)
        self.measurement_type = "orientation"
        self._ref_x: float | None = None
        self._ref_y: float | None = None
        self._tangent: float | None = None

    def reset(self) -> None:
        """Clear position history — call when odometry is reset."""
        self._ref_x = None
        self._ref_y = None
        self._tangent = None

    def update(
        self,
        odom_theta: float,
        mag_heading: float | None,
        linear_vel: float,
        angular_vel: float,
        fused_x: float | None = None,
        fused_y: float | None = None,
    ) -> float:
        if fused_x is not None and fused_y is not None:
            self._try_update_tangent(odom_theta, fused_x, fused_y)
        if self._tangent is None:
            return odom_theta
        return odom_theta + self.alpha * _wrap(self._tangent - odom_theta)

    def _try_update_tangent(
        self, odom_theta: float, fused_x: float, fused_y: float
    ) -> None:
        if self._ref_x is None:
            self._ref_x = fused_x
            self._ref_y = fused_y
            return

        dx = fused_x - self._ref_x
        dy = fused_y - self._ref_y
        if math.hypot(dx, dy) < self.min_displacement_mm:
            return

        # Forward-motion gate: reject if displacement is opposite odom heading.
        if dx * math.cos(odom_theta) + dy * math.sin(odom_theta) < 0.0:
            self._ref_x = fused_x
            self._ref_y = fused_y
            return

        self._tangent = math.atan2(dy, dx)
        self._ref_x = fused_x
        self._ref_y = fused_y


class PositionComplementaryFilter(SensorFusion):
    """
    Complementary filter for GPS-anchored position fusion.

    Each tick the estimate dead-reckons forward from the last anchor using the
    odometry delta, then blends toward the GPS fix::

        fused = (1 - alpha) * fused_prev + alpha * gps

    where fused_prev is the dead-reckoned position since the last GPS update.
    This converges to the GPS position over time rather than staying fixed at a
    weighted average of odom and GPS.

    When GPS is stale, pure dead-reckoning continues from the last anchor so
    drift only accumulates since the last valid fix. Falls back to raw odometry
    if no GPS fix has ever been received.

    Parameters
    ----------
    alpha : float, 0.0–1.0
        GPS weight per tick.  0 = ignore GPS (pure odometry), 1 = snap to GPS.
        Default 0.10.
    """

    def __init__(self, alpha: float = 0.10) -> None:
        super().__init__()
        self.alpha = max(0.0, min(1.0, float(alpha)))
        self._anchor_x_mm      = 0.0
        self._anchor_y_mm      = 0.0
        self._odom_x_at_anchor = 0.0
        self._odom_y_at_anchor = 0.0
        self._anchor_valid     = False
        self.measurement_type = "position"

    def reset(self) -> None:
        """Clear anchor — call when odometry is reset."""
        self._anchor_valid = False

    def update(
        self,
        odom_x: float,
        odom_y: float,
        gps_x: float | None,
        gps_y: float | None,
    ) -> tuple[float, float]:
        if gps_x is not None and gps_y is not None:
            if self._anchor_valid:
                prev_x = self._anchor_x_mm + (odom_x - self._odom_x_at_anchor)
                prev_y = self._anchor_y_mm + (odom_y - self._odom_y_at_anchor)
            else:
                prev_x, prev_y = odom_x, odom_y
            fused_x = prev_x + self.alpha * (gps_x - prev_x)
            fused_y = prev_y + self.alpha * (gps_y - prev_y)
            self._anchor_x_mm      = fused_x
            self._anchor_y_mm      = fused_y
            self._odom_x_at_anchor = odom_x
            self._odom_y_at_anchor = odom_y
            self._anchor_valid     = True
            return fused_x, fused_y
        if self._anchor_valid:
            return (
                self._anchor_x_mm + (odom_x - self._odom_x_at_anchor),
                self._anchor_y_mm + (odom_y - self._odom_y_at_anchor),
            )
        return odom_x, odom_y
