"""
robot_impl/legacy.py — LegacyMixin

Quarantined legacy code: PurePursuitPlannerWithAvoidance-based navigation.
This is kept functional for the obstacle_avoidance.py example but is not
part of the general API and will be removed in a future version.
"""
from __future__ import annotations

import math

import numpy as np


class LegacyMixin:

    @property
    def planner(self):
        """Legacy alias for the current obstacle-avoidance planner instance."""
        return self._obstacle_avoidance_planner

    def _set_obstacle_avoidance_path(self, path: list[tuple[float, float]]) -> None:
        if self._obstacle_avoidance_planner is None:
            raise RuntimeError(
                "Obstacle-avoidance planner is not initialized. "
                "Call _nav_follow_pp_path(...) before setting its path."
            )
        self._obstacle_avoidance_planner.set_path(path)

    def _nav_follow_pp_path(
        self,
        lookahead_distance: float,
        max_linear_speed: float,
        max_angular_speed: float,
        goal_tolerance: float,
        obstacles_range: float,
        view_angle: float,
        safe_dist: float,
        avoidance_delay: int,
        offset: float,
        alpha_Ld: float,
        x_L: float = 0.0,
        lane_width: float = 600.0,
        obstacle_avoidance: bool = True,
    ) -> None:
        from robot.path_planner import PurePursuitPlannerWithAvoidance
        self._obstacle_avoidance_planner = PurePursuitPlannerWithAvoidance(
            lookahead_distance=lookahead_distance,
            max_linear_speed=max_linear_speed,
            max_angular_speed=max_angular_speed,
            goal_tolerance=goal_tolerance,
            obstacles_range=obstacles_range,
            view_angle=view_angle,
            safe_dist=safe_dist,
            avoidance_delay=avoidance_delay,
            offset=offset,
            x_L=x_L,
            lane_width=lane_width,
            alpha_Ld=alpha_Ld,
            obstacle_avoidance=obstacle_avoidance,
        )

    def _nav_follow_pp_path_loop(self):
        with self._lock:
            obstacles = self._obstacles_mm.copy()
            pose = self._pose

        if self._obstacle_avoidance_planner is None:
            raise RuntimeError(
                "Obstacle-avoidance planner is not initialized. "
                "Call _nav_follow_pp_path(...) before entering the loop."
            )

        v, w = self._obstacle_avoidance_planner.compute_velocity(pose, obstacles)
        self.set_velocity(v, math.degrees(w))

        if self._obstacle_avoidance_planner.TargetReached(
            self._obstacle_avoidance_planner.remaining_path,
            pose[0],
            pose[1],
        ):
            print("MOVING: Target reached! Stopping.")
            print(self._obstacle_avoidance_planner.remaining_path, pose[0], pose[1])
            self.stop()
            return "IDLE"

        return "MOVING"

    def _draw_lidar_obstacles(self):
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            self._node.get_logger().error("matplotlib not installed; cannot draw lidar obstacles")
            return

        with self._lock:
            obstacles_mm = self._obstacles_mm.copy()
            pose = self._pose
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.clear()
        if obstacles_mm.size != 0:
            lidar_offset_mm = 100.0
            obstacles = (
                np.array([[np.cos(np.pi), -np.sin(np.pi)],
                          [np.sin(np.pi),  np.cos(np.pi)]]) @ obstacles_mm.T
            ).T + np.array([[lidar_offset_mm, 0]])
            obstacles = obstacles[obstacles[:, 0] > 0, :]
            obstacles = (
                np.array([[np.cos(pose[2]), -np.sin(pose[2])],
                          [np.sin(pose[2]),  np.cos(pose[2])]]) @ obstacles.T
            ).T + np.array([[pose[0], pose[1]]])
            self.ax.scatter(obstacles[:, 0] / 1000.0, obstacles[:, 1] / 1000.0, s=5)
        self.ax.set_xlim(-2.5, 2.5)
        self.ax.set_ylim(-2.5, 2.5)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title("LiDAR Scan")
        self.ax.set_aspect("equal")
        plt.grid()
        plt.draw()
        plt.pause(0.001)
        plt.savefig("/data/plot.png")
        plt.close()
