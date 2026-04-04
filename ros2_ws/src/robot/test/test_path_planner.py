from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np


package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

from robot.path_planner import PurePursuitPlanner


class PurePursuitPlannerTests(unittest.TestCase):
    def test_lookahead_starts_from_closest_waypoint(self) -> None:
        planner = PurePursuitPlanner(lookahead_dist=50.0)
        path = np.array([
            [0.0, 0.0],
            [0.0, 500.0],
            [500.0, 500.0],
            [500.0, 0.0],
        ])

        lookahead = planner._lookahead_point(513.48, 2924.50, path)

        self.assertEqual(tuple(lookahead), (500.0, 500.0))


if __name__ == "__main__":
    unittest.main()
