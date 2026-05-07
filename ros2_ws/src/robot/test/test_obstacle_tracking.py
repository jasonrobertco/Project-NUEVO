from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np


package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

from robot.obstacle_tracking import ObstacleTracker


class ObstacleTrackerTests(unittest.TestCase):
    def test_compact_cluster_becomes_single_disk_track(self) -> None:
        tracker = ObstacleTracker(
            cluster_neighbor_mm=50.0,
            cluster_min_points=3,
            max_disk_radius_mm=75.0,
            disk_radius_margin_mm=10.0,
            ttl_s=1.0,
        )
        points = np.array([
            [100.0, 100.0],
            [110.0, 100.0],
            [105.0, 110.0],
            [95.0, 105.0],
        ])

        tracker.update(points, now_s=0.0)
        tracks = tracker.update(points, now_s=0.1)

        self.assertEqual(len(tracks), 1)
        self.assertGreaterEqual(tracks[0].hit_count, 2)
        self.assertLessEqual(tracks[0].radius_mm, 75.0)

    def test_wall_cluster_splits_into_multiple_small_tracks(self) -> None:
        tracker = ObstacleTracker(
            cluster_neighbor_mm=60.0,
            cluster_min_points=3,
            max_disk_radius_mm=75.0,
            disk_radius_margin_mm=10.0,
            ttl_s=1.0,
            max_tracks=12,
        )
        wall = np.array([[float(x), 0.0] for x in range(0, 451, 25)])

        tracker.update(wall, now_s=0.0)
        tracks = tracker.update(wall, now_s=0.1)

        self.assertGreaterEqual(len(tracks), 3)
        self.assertTrue(all(track.radius_mm <= 75.0 for track in tracks))

    def test_tracks_persist_briefly_after_measurement_dropout(self) -> None:
        tracker = ObstacleTracker(ttl_s=1.0, max_disk_radius_mm=75.0)
        points = np.array([
            [300.0, 0.0],
            [320.0, 10.0],
            [310.0, -10.0],
        ])

        tracker.update(points, now_s=0.0)
        tracker.update(points, now_s=0.1)
        tracks = tracker.update(np.empty((0, 2)), now_s=0.6)
        self.assertEqual(len(tracks), 1)

        tracks = tracker.update(np.empty((0, 2)), now_s=1.3)
        self.assertEqual(tracks, [])


if __name__ == "__main__":
    unittest.main()
