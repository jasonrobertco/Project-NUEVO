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
    from vision.rule_based_detection import detect_yellow_block
else:  # pragma: no cover - test is skipped when OpenCV is unavailable
    detect_yellow_block = None


@unittest.skipIf(cv2 is None, "OpenCV is not installed in this environment")
class RuleBasedDetectionTests(unittest.TestCase):
    def test_disabled_detector_returns_no_detections(self) -> None:
        detections, debug_overlays = detect_yellow_block(
            np.zeros((240, 320, 3), dtype=np.uint8),
            enabled=False,
        )

        self.assertEqual(detections, [])
        self.assertEqual(debug_overlays, [])

    def test_detects_yellow_block_and_emits_overlay(self) -> None:
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.rectangle(image, (80, 60), (220, 180), (0, 255, 255), -1)

        detections, debug_overlays = detect_yellow_block(
            image,
            enabled=True,
        )

        self.assertEqual(len(detections), 1)
        self.assertEqual(len(debug_overlays), 1)
        detection = detections[0]
        self.assertEqual(detection.class_name, "yellow block")
        self.assertGreater(detection.confidence, 0.0)
        self.assertLessEqual(detection.confidence, 1.0)
        self.assertGreaterEqual(detection.x, 75)
        self.assertGreaterEqual(detection.y, 55)
        self.assertLessEqual(detection.x + detection.width, 225)
        self.assertLessEqual(detection.y + detection.height, 185)
        self.assertEqual(len(detection.attributes), 1)
        self.assertEqual(detection.attributes[0].name, "color")
        self.assertEqual(detection.attributes[0].value, "yellow")
        self.assertIsNotNone(debug_overlays[0].contour)


if __name__ == "__main__":
    unittest.main()
