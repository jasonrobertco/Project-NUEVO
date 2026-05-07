"""
Mock RPLidar node for VM/desktop development (no hardware required).

Publishes a synthetic sensor_msgs/LaserScan on /scan by ray-casting against
a configurable list of circular obstacles in world frame. The robot's current
pose is consumed from /sensor_kinematics so the simulated scan changes as the
(mock) robot moves.

The simulated scan matches real RPLidar hardware conventions:
  - Full 360° scan at 10 Hz
  - Mounting yaw = 180° (inverted mount, matching the physical robot)
  - range_min = 0.15 m, range_max = 12.0 m
  - Points beyond range_max are published as range_max (not inf)

Obstacles are defined as (x_mm, y_mm, radius_mm) tuples in world frame at the
top of this file. Edit OBSTACLES to match your test scenario.

Enable via environment variable:
    NUEVO_MOCK_LIDAR=1

Auto-started by entrypoint.robot.sh when that variable is set.
"""

from __future__ import annotations

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from bridge_interfaces.msg import SensorKinematics

# ---------------------------------------------------------------------------
# Simulated world-frame obstacles — edit to match your test scenario.
# Each entry: (x_mm, y_mm, radius_mm)
# ---------------------------------------------------------------------------
OBSTACLES: list[tuple[float, float, float]] = [
    (600.0,  400.0, 120.0),
    (1100.0, 700.0, 150.0),
    (300.0,  900.0,  80.0),
]

# ---------------------------------------------------------------------------
# Lidar hardware constants — must match real RPLidar + robot configuration
# ---------------------------------------------------------------------------
_ANGLE_MIN       = -math.pi   # rad  — full 360° scan
_ANGLE_MAX       =  math.pi   # rad
_NUM_POINTS      = 360
_RANGE_MIN_M     = 0.15       # metres
_RANGE_MAX_M     = 12.0       # metres
_SCAN_RATE_HZ    = 10.0
_MOUNT_YAW_RAD   = math.pi    # 180° — inverted mount (matches physical robot)


class MockLidarNode(Node):
    def __init__(self) -> None:
        super().__init__('mock_lidar')

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1,
        )

        self._pub = self.create_publisher(LaserScan, '/scan', 10)
        self._sub = self.create_subscription(
            SensorKinematics,
            '/sensor_kinematics',
            self._on_kinematics,
            best_effort,
        )
        self._timer = self.create_timer(1.0 / _SCAN_RATE_HZ, self._publish_scan)

        # Robot pose in world frame (mm, mm, rad) — updated from odometry
        self._rx_mm: float = 0.0
        self._ry_mm: float = 0.0
        self._rtheta: float = 0.0

        self._angles = np.linspace(_ANGLE_MIN, _ANGLE_MAX, _NUM_POINTS, endpoint=False)

        self.get_logger().info(
            f'Mock lidar started — {len(OBSTACLES)} obstacle(s), '
            f'{_NUM_POINTS} pts @ {_SCAN_RATE_HZ:.0f} Hz'
        )

    # ------------------------------------------------------------------
    def _on_kinematics(self, msg: SensorKinematics) -> None:
        self._rx_mm  = float(msg.x)
        self._ry_mm  = float(msg.y)
        self._rtheta = float(msg.theta)

    # ------------------------------------------------------------------
    def _publish_scan(self) -> None:
        ranges = self._raycast()

        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        msg.angle_min       = float(_ANGLE_MIN)
        msg.angle_max       = float(_ANGLE_MAX)
        msg.angle_increment = float((_ANGLE_MAX - _ANGLE_MIN) / _NUM_POINTS)
        msg.time_increment  = 0.0
        msg.scan_time       = float(1.0 / _SCAN_RATE_HZ)
        msg.range_min       = float(_RANGE_MIN_M)
        msg.range_max       = float(_RANGE_MAX_M)
        msg.ranges          = ranges.tolist()
        msg.intensities     = []

        self._pub.publish(msg)

    # ------------------------------------------------------------------
    def _raycast(self) -> np.ndarray:
        """
        For each scan angle, find the nearest circular obstacle and return
        the ray distance in metres. Returns range_max when no obstacle is hit.
        """
        rx_m = self._rx_mm / 1000.0
        ry_m = self._ry_mm / 1000.0

        # World-frame direction for each ray
        world_angles = self._rtheta + self._angles + _MOUNT_YAW_RAD
        dirs_x = np.cos(world_angles)   # (N,)
        dirs_y = np.sin(world_angles)   # (N,)

        ranges = np.full(_NUM_POINTS, _RANGE_MAX_M, dtype=np.float64)

        for ox_mm, oy_mm, orad_mm in OBSTACLES:
            ox_m   = ox_mm   / 1000.0
            oy_m   = oy_mm   / 1000.0
            orad_m = orad_mm / 1000.0

            # Vector from robot to obstacle centre
            fx = rx_m - ox_m
            fy = ry_m - oy_m

            # Ray–circle intersection (quadratic): |P + t*d - C|² = r²
            # a=1 (unit dirs), b = 2*(f·d), c = |f|²-r²
            b    = 2.0 * (fx * dirs_x + fy * dirs_y)
            c    = fx*fx + fy*fy - orad_m*orad_m
            disc = b*b - 4.0*c       # a=1

            hit  = disc >= 0
            t    = np.where(hit, (-b - np.sqrt(np.maximum(disc, 0.0))) * 0.5, np.inf)
            valid = hit & (t > _RANGE_MIN_M) & (t < ranges)
            ranges = np.where(valid, t, ranges)

        return ranges.astype(np.float32)


def main(args=None) -> None:
    if os.getenv('NUEVO_MOCK_LIDAR', '0') != '1':
        print('[mock_lidar] NUEVO_MOCK_LIDAR is not set — exiting.')
        return

    rclpy.init(args=args)
    node = MockLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
