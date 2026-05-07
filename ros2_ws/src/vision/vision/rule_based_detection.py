from __future__ import annotations

import cv2
import numpy as np

from vision.debug_utils import DebugOverlay
from vision.model_utils import DetectedObject


def detect_yellow_block(
    frame_bgr: np.ndarray,
) -> tuple[list[DetectedObject], list[DebugOverlay]]:
    """Detect a simple yellow block and return detections plus debug contours."""
    detections: list[DetectedObject] = []
    debug_overlays: list[DebugOverlay] = []

    blurred = cv2.GaussianBlur(frame_bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Yellow HSV range. These values were chosen to keep the example simple and
    # reasonably robust for bright classroom-colored yellow objects.
    yellow_hsv_low = (20, 110, 80)
    yellow_hsv_high = (38, 255, 255)
    mask = cv2.inRange(hsv, yellow_hsv_low, yellow_hsv_high)

    # Small open, then slightly larger close:
    # - open removes isolated noise specks
    # - close fills small gaps inside the detected block
    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

    # Minimum blob size and compactness filter:
    # - 500 px allows partially-clipped blocks at the frame edge
    # - 0.30 fill ratio allows partial/clipped bounding shapes
    min_area_px = 500
    min_fill_ratio = 0.30

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        contour_area = float(cv2.contourArea(contour))
        if contour_area < min_area_px:
            continue

        x, y, width, height = cv2.boundingRect(contour)
        bounding_box_area = float(max(1, width * height))
        fill_ratio = contour_area / bounding_box_area
        if fill_ratio < min_fill_ratio:
            continue

        confidence = yellow_detection_score(contour_area, min_area_px, fill_ratio)
        detection = DetectedObject(
            class_name="yellow block",
            confidence=confidence,
            x=int(x),
            y=int(y),
            width=int(width),
            height=int(height),
        )
        detection.add_attribute("color", "yellow", 1.0)
        detections.append(detection)
        debug_overlays.append(
            DebugOverlay(
                color=(0, 255, 255),
                contour=contour,
                label="yellow block",
                x=int(x),
                y=int(y),
            )
        )

    return detections, debug_overlays


def yellow_detection_score(contour_area: float, min_area_px: int, fill_ratio: float) -> float:
    """Build a simple 0-1 confidence score from area and shape compactness."""
    area_score = min(1.0, contour_area / float(max(1, min_area_px * 4)))
    score = 0.55 * area_score + 0.45 * max(0.0, min(1.0, fill_ratio))
    return max(0.0, min(1.0, score))
