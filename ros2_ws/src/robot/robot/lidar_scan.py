"""
lidar_scan.py — ROS-agnostic lidar processing helpers
======================================================
Converts a raw sensor_msgs/LaserScan into robot-frame or world-frame point
clouds. Handles sensor mounting position, yaw offset, range filtering, and
optional FOV trimming.

Typical usage in an example script
------------------------------------
    from robot.lidar_scan import LidarConfig, LidarScan

    cfg = LidarConfig(yaw_deg=180.0, range_max_mm=4000.0, fov_deg=270.0)
    scanner = LidarScan(cfg)

    def on_scan(msg):
        pts_robot = scanner.process(msg)           # (N, 2) mm, robot frame
        pts_world = scanner.to_world_frame(pts_robot, robot.get_fused_pose())
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class LidarConfig:
    """Sensor mounting and output configuration."""

    offset_x_mm: float = 0.0
    """Sensor forward offset from robot centre (mm)."""

    offset_y_mm: float = 0.0
    """Sensor left offset from robot centre (mm)."""

    yaw_deg: float = 0.0
    """Sensor mounting rotation in degrees.
    0 = sensor forward aligns with robot forward.
    180 = inverted mount (matches default RPLidar installation on this robot)."""

    range_min_mm: float = 150.0
    """Drop scan points closer than this (mm). Matches RPLidar C1 spec."""

    range_max_mm: float = 12000.0
    """Drop scan points farther than this (mm). Matches RPLidar C1 spec."""

    fov_deg: float = 360.0
    """Symmetric FOV cone to keep, centred on robot forward (degrees).
    360 keeps the full scan. 270 drops 45° on each rear side."""

    units: str = 'mm'
    """Output unit: ``'mm'`` or ``'inch'``. All returned arrays use this unit."""


_INCH_PER_MM = 1.0 / 25.4


class LidarScan:
    """
    Stateless converter from LaserScan messages to Cartesian point clouds.

    The class pre-computes mounting constants at construction time. Reuse a
    single instance for every scan message from the same sensor.
    """

    def __init__(self, config: LidarConfig) -> None:
        self._cfg = config
        self._yaw_rad = math.radians(config.yaw_deg)
        self._half_fov = math.radians(config.fov_deg) / 2.0
        self._full_fov = config.fov_deg >= 360.0
        # m → configured unit (applied after converting from metres)
        self._m_to_unit = 1000.0 if config.units == 'mm' else 1000.0 * _INCH_PER_MM
        self._mm_to_unit = 1.0 if config.units == 'mm' else _INCH_PER_MM

    # ------------------------------------------------------------------

    def process(self, scan_msg) -> np.ndarray:
        """
        Convert a ``sensor_msgs/LaserScan`` to a robot-frame point cloud.

        Applies mounting yaw, offset translation, range filtering, and optional
        FOV cone trimming.

        Returns an ``(N, 2)`` float32 array in the configured unit. Returns
        ``np.empty((0, 2))`` when no points survive filtering.
        """
        ranges = np.asarray(scan_msg.ranges, dtype=np.float64)
        n = len(ranges)
        if n == 0:
            return np.empty((0, 2), dtype=np.float32)

        # Reconstruct per-ray angles from LaserScan header values
        angles = np.linspace(
            float(scan_msg.angle_min),
            float(scan_msg.angle_max),
            n,
            endpoint=False,
        )

        # Range filter (scan_msg uses metres; config params are mm)
        r_min_m = self._cfg.range_min_mm / 1000.0
        r_max_m = self._cfg.range_max_mm / 1000.0
        valid = np.isfinite(ranges) & (ranges >= r_min_m) & (ranges <= r_max_m)

        # FOV filter — apply BEFORE converting to Cartesian (cheap angular check)
        # Robot-frame angle ≈ scan angle + mounting yaw.
        if not self._full_fov:
            robot_angles = angles + self._yaw_rad
            # Wrap to [-π, π]
            robot_angles = (robot_angles + math.pi) % (2 * math.pi) - math.pi
            valid &= np.abs(robot_angles) <= self._half_fov

        if not np.any(valid):
            return np.empty((0, 2), dtype=np.float32)

        r = ranges[valid]
        a = angles[valid]

        # Sensor-local Cartesian (metres): x forward in sensor frame, y left
        # Applying the mounting yaw rotation analytically:
        #   robot_x = r * cos(a + yaw) + offset_x_mm/1000
        #   robot_y = r * sin(a + yaw) + offset_y_mm/1000
        yaw = self._yaw_rad
        robot_x_m = r * np.cos(a + yaw) + self._cfg.offset_x_mm / 1000.0
        robot_y_m = r * np.sin(a + yaw) + self._cfg.offset_y_mm / 1000.0

        points = np.column_stack([
            robot_x_m * self._m_to_unit,
            robot_y_m * self._m_to_unit,
        ])
        return points.astype(np.float32)

    # ------------------------------------------------------------------

    def to_world_frame(self, points: np.ndarray, pose: tuple) -> np.ndarray:
        """
        Rigid-body transform from robot frame to world frame.

        Parameters
        ----------
        points:
            ``(N, 2)`` array of robot-frame coordinates in the configured unit.
        pose:
            ``(x, y, theta)`` — robot position in the same unit, heading in
            radians (counter-clockwise from world +X).

        Returns
        -------
        ``(N, 2)`` float32 array in world frame, same unit as *points*.
        """
        if len(points) == 0:
            return np.empty((0, 2), dtype=np.float32)

        x, y, theta = pose
        c = math.cos(theta)
        s = math.sin(theta)

        wx = c * points[:, 0] - s * points[:, 1] + x
        wy = s * points[:, 0] + c * points[:, 1] + y

        return np.column_stack([wx, wy]).astype(np.float32)
