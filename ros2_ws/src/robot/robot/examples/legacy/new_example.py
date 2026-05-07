"""
new_example.py — APF obstacle avoidance to a single goal point
===============================================================
Robot drives toward GOAL_MM using Artificial Potential Fields (APF) while
publishing a live lidar point cloud for the web UI canvas.

Setup
-----
1. Edit GOAL_MM to your desired target (world-frame mm, same origin as odometry).
2. Adjust LIDAR_CONFIG if your RPLidar mounting differs from the default
   (180° yaw = inverted mount, which matches the standard robot configuration).
3. Tune APF_* constants for your environment (start with defaults).

Run
---
Inside the Docker container / on the RPi:

    ros2 run robot new_example

The robot will:
- Move toward GOAL_MM using APF steering
- Deflect around obstacles detected by the lidar
- Stop and log "Reached goal" when within GOAL_THRESHOLD_MM
- Publish /lidar_world_points for the web UI world canvas
"""

from __future__ import annotations

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from bridge_interfaces.msg import FusedPose, LidarWorldPoints

from robot.robot import Robot, Unit
from robot.lidar_scan import LidarConfig, LidarScan
from robot.path_planner import APFPlanner

# ── Configurable parameters (edit these) ──────────────────────────────────────

# Goal position in world frame mm (same frame as robot odometry).
GOAL_MM = (1000.0, 0.0)

# Stop when within this distance of the goal (mm).
GOAL_THRESHOLD_MM = 60.0

# Maximum forward speed (mm/s) and angular rate (rad/s).
MAX_LINEAR_MM_S  = 150.0
MAX_ANGULAR_RAD_S = 1.8

# RPLidar mounting — 180° yaw = inverted mount (standard robot config).
LIDAR_CONFIG = LidarConfig(
    yaw_deg=180.0,
    range_min_mm=150.0,
    range_max_mm=4000.0,   # cap at 4 m for indoor use
    fov_deg=270.0,          # drop 45° on each rear side
    offset_x_mm=0.0,
    offset_y_mm=0.0,
)

# APF gains — increase repulsion_gain to deflect earlier/wider.
APF_ATTRACTION_GAIN   = 1.0
APF_REPULSION_GAIN    = 8000.0
APF_INFLUENCE_RANGE_MM = 400.0  # ignore obstacles farther than this


# ── Node ──────────────────────────────────────────────────────────────────────

class NewExample(Node):
    def __init__(self) -> None:
        super().__init__('new_example')
        self._robot = Robot(self, unit=Unit.MM)

        self._scanner = LidarScan(LIDAR_CONFIG)
        self._planner = APFPlanner(
            attraction_gain=APF_ATTRACTION_GAIN,
            repulsion_gain=APF_REPULSION_GAIN,
            repulsion_range=APF_INFLUENCE_RANGE_MM,
            max_linear=MAX_LINEAR_MM_S,
            max_angular=MAX_ANGULAR_RAD_S,
            heading_gain=2.0,
            goal_tolerance=GOAL_THRESHOLD_MM,
        )

        self._obstacles: np.ndarray = np.empty((0, 2))
        self._lock = threading.Lock()
        self._goal_reached = False

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(LaserScan, '/scan', self._on_scan, best_effort)

        self._lidar_pub = self.create_publisher(LidarWorldPoints, '/lidar_world_points', 10)

        # 10 Hz control loop
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f'new_example started — goal=({GOAL_MM[0]:.0f}, {GOAL_MM[1]:.0f}) mm, '
            f'threshold={GOAL_THRESHOLD_MM:.0f} mm'
        )

    def _pose_rad(self) -> tuple[float, float, float]:
        """Return current pose as (x_mm, y_mm, theta_rad)."""
        x, y, theta_deg = self._robot.get_fused_pose()
        return x, y, math.radians(theta_deg)

    def _on_scan(self, msg: LaserScan) -> None:
        pose = self._pose_rad()
        pts_robot = self._scanner.process(msg)
        pts_world = self._scanner.to_world_frame(pts_robot, pose)

        with self._lock:
            self._obstacles = pts_world

        # Publish LidarWorldPoints for the web UI canvas
        lw = LidarWorldPoints()
        lw.header.stamp = self.get_clock().now().to_msg()
        lw.xs = pts_world[:, 0].tolist() if len(pts_world) else []
        lw.ys = pts_world[:, 1].tolist() if len(pts_world) else []
        lw.robot_x     = float(pose[0])
        lw.robot_y     = float(pose[1])
        lw.robot_theta = float(pose[2])
        self._lidar_pub.publish(lw)

    def _control_loop(self) -> None:
        if self._goal_reached:
            return

        pose = self._pose_rad()
        with self._lock:
            obs = self._obstacles.copy()

        linear, angular = self._planner.navigate_to_goal(pose, GOAL_MM, obs)

        dist = math.hypot(GOAL_MM[0] - pose[0], GOAL_MM[1] - pose[1])
        if dist <= GOAL_THRESHOLD_MM:
            self._robot.set_velocity(0.0, 0.0)
            self._goal_reached = True
            self.get_logger().info(
                f'Reached goal at ({pose[0]:.1f}, {pose[1]:.1f}) mm — '
                f'distance to goal: {dist:.1f} mm'
            )
            return

        self._robot.set_velocity(linear, math.degrees(angular))

    def stop(self) -> None:
        try:
            self._robot.set_velocity(0.0, 0.0)
        except Exception:
            pass


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = NewExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
