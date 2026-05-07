from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - local env may not have OpenCV installed
    cv2 = None


package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

if cv2 is not None:
    from vision.stop_sign import classify_stop_sign_visibility
else:  # pragma: no cover - test is skipped when OpenCV is unavailable
    classify_stop_sign_visibility = None


@unittest.skipIf(cv2 is None, "OpenCV is not installed in this environment")
class StopSignTests(unittest.TestCase):
    def test_empty_crop_returns_unknown(self) -> None:
        label, score = classify_stop_sign_visibility(np.zeros((0, 0, 3), dtype=np.uint8))

        self.assertEqual(label, "unknown")
        self.assertEqual(score, 0.0)

    def test_red_octagon_returns_visible_label(self) -> None:
        image = np.zeros((220, 220, 3), dtype=np.uint8)
        points = np.array(
            [
                [80, 20],
                [140, 20],
                [200, 80],
                [200, 140],
                [140, 200],
                [80, 200],
                [20, 140],
                [20, 80],
            ],
            dtype=np.int32,
        )
        cv2.fillPoly(image, [points], (0, 0, 255))

        label, score = classify_stop_sign_visibility(image)

        self.assertIn(label, {"clear", "partial"})
        self.assertGreater(score, 0.0)


if __name__ == "__main__":
    unittest.main()
