"""
path_planner.py — pure-algorithm path planning library
=======================================================
These classes are stateless algorithm helpers. They do NOT own threads or
ROS subscriptions. The Robot class calls compute_velocity() from its own
navigation thread.

To use a planner in your navigation code, just instantiate it and call
compute_velocity() with the current pose and remaining waypoints.
"""

from __future__ import annotations

import math
import numpy as np

# =============================================================================
# Base class
# =============================================================================

class PathPlanner:
    """
    Abstract base for path planning algorithms.

    Subclasses implement compute_velocity() and optionally get_obstacles().
    """

    def compute_velocity(
        self,
        pose: tuple[float, float, float],
        waypoints: list[tuple[float, float]],
        max_linear: float,
    ) -> tuple[float, float]:
        """
        Return (linear, angular) velocity command.

          pose       — (x, y, theta_rad) in any consistent unit
          waypoints  — remaining waypoints in the same unit, nearest first
          max_linear — maximum forward speed in that unit/s

        Returns (linear, angular_rad_s).
        """
        raise NotImplementedError

    def get_obstacles(self) -> list:
        """
        Return a list of obstacle positions in the robot's frame.
        Override when a 2D lidar topic is available.
        """
        return []


# =============================================================================
# Pure Pursuit
# =============================================================================

class PurePursuitPlanner(PathPlanner):
    """
    Pure-pursuit path follower for differential drive.

    Steers toward a lookahead point on the path. Works well for smooth
    curves. The lookahead_dist controls the trade-off between responsiveness
    (small) and smoothness (large).

    Parameters:
        lookahead_dist — how far ahead on the path to aim at (same units as pose)
        max_angular    — maximum angular rate (rad/s)
    """

    def __init__(
        self,
        lookahead_dist: float = 150,
        max_angular: float = 2.0,
        goal_tolerance: float = 20.0,
    ) -> None:
        self._lookahead  = lookahead_dist
        self._max_angular = max_angular
        self.goal_tolerance = goal_tolerance

    def compute_velocity(
        self,
        pose: tuple[float, float, float],
        waypoints: list[tuple[float, float]],
        max_linear: float,
    ) -> tuple[float, float]:
        x, y, theta = pose
        tx, ty = self._lookahead_point(x, y, waypoints)

        angle_to_target = math.atan2(ty - y, tx - x)
        heading_error   = _wrap_angle(angle_to_target - theta)

        linear  = max_linear * math.cos(heading_error)
        angular = self._max_angular * math.tanh(heading_error * 2.0)

        # Don't drive backwards
        linear = max(0.0, linear)
        return linear, angular

    def _lookahead_point(
        self, x: float, y: float, waypoints: list[tuple[float, float]]
    ) -> tuple[float, float]:
        """
        Return the first forward waypoint beyond the lookahead distance.

        Start from the closest waypoint instead of the first waypoint in the
        full path so a long-running demo does not snap back toward the origin
        once the robot has already progressed along the route.
        """
        closest_index = min(
            range(len(waypoints)),
            key=lambda index: math.hypot(waypoints[index][0] - x, waypoints[index][1] - y),
        )
        for wx, wy in waypoints[closest_index:]:
            if math.hypot(wx - x, wy - y) >= self._lookahead:
                return wx, wy
        return waypoints[-1]

    def TargetReached(self, x: float, y: float, waypoints: list[tuple[float, float]]) -> bool:
        goal_x, goal_y = waypoints[-1]
        dist_to_goal = np.hypot(goal_x - x, goal_y - y)
        return dist_to_goal < self.goal_tolerance


class PurePursuitPlanner2(PathPlanner):
    def __init__(self, lookahead_distance=150.0, max_linear_speed=50.0, goal_tolerance=20.0):
        self.Ld = lookahead_distance
        self.v_max = max_linear_speed  # mm/s
        self.goal_tolerance = goal_tolerance

    def _lookahead_point(self, path, x, y):
        for point in path:
            dx = point[0] - x
            dy = point[1] - y
            if np.hypot(dx, dy) >= self.Ld:
                return point
        return path[-1]
    
    def TargetReached(self, path, x, y):
        goal_x, goal_y = path[-1]
        dist_to_goal = np.hypot(goal_x - x, goal_y - y)
        return dist_to_goal < self.goal_tolerance

    def compute_velocity(self, path, pose):
        x, y, theta = pose
        goal_x, goal_y = path[-1]
        dist_to_goal = np.hypot(goal_x - x, goal_y - y)
        if dist_to_goal < self.goal_tolerance:
            return 0.0, 0.0  # Stop if within goal tolerance

        target = self._lookahead_point(path, x, y)
        tx, ty = target

        dx = tx - x
        dy = ty - y

        # Transform to robot frame
        x_r = np.cos(theta) * dx + np.sin(theta) * dy
        y_r = -np.sin(theta) * dx + np.cos(theta) * dy
        Dist2Target = np.hypot(x_r, y_r)
        #curvature = 2.0 * y_r / (self.Ld ** 2)
        curvature = 2.0 * y_r / (Dist2Target ** 2)

        
        v = self.v_max / (1.0 + 2.0 * abs(curvature))  # mm/s
        w = curvature * v  # rad/s
        #print(f"x_r: {x_r:.2f}, y_r: {y_r:.2f}, curvature: {curvature:.4f}, Dist2Target: {Dist2Target:.2f}")
        #print(f"Pose: ({x:.1f}, {y:.1f}, {math.degrees(theta):.1f}°), ", end="")
        #print(f"Target: ({tx:.1f}, {ty:.1f}), Curvature: {curvature:.4f}, v: {v:.2f} mm/s, w: {w:.2f} rad/s")

        return v, w



# =============================================================================
# APF (stub — activated when 2D lidar is ready)
# =============================================================================

class APFPlanner(PathPlanner):
    """
    Artificial Potential Fields planner.

    Combines an attractive force toward the goal with repulsive forces from
    obstacles. Obstacle data comes from get_obstacles(), which reads the
    lidar topic once it is available.

    TODO: implement when /scan or equivalent topic is published by the
    sensors package.
    """

    def __init__(
        self,
        lookahead_dist: float = 200,
        max_linear: float = 200,
        max_angular: float = 2.0,
        repulsion_gain: float = 500.0,
        repulsion_range: float = 300.0,
    ) -> None:
        self._lookahead      = lookahead_dist
        self._max_linear     = max_linear
        self._max_angular    = max_angular
        self._rep_gain       = repulsion_gain
        self._rep_range      = repulsion_range

    def compute_velocity(
        self,
        pose: tuple[float, float, float],
        waypoints: list[tuple[float, float]],
        max_linear: float,
    ) -> tuple[float, float]:
        raise NotImplementedError(
            "APFPlanner.compute_velocity() is not yet implemented. "
            "Use PurePursuitPlanner until the lidar is available."
        )

    def get_obstacles(self) -> list:
        # TODO: subscribe to /scan and return obstacle positions
        return []


# =============================================================================
# Helper
# =============================================================================

def _wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi
