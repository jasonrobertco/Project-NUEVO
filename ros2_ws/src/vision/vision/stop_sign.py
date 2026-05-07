from __future__ import annotations

import cv2
import numpy as np


def _largest_red_blob(mask: np.ndarray) -> tuple[float, int]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0.0, 0

    largest = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(largest))
    perimeter = float(cv2.arcLength(largest, True))
    if perimeter <= 0.0:
        return area, 0

    approx = cv2.approxPolyDP(largest, 0.04 * perimeter, True)
    return area, len(approx)


def classify_stop_sign_visibility(stop_sign_crop: np.ndarray) -> tuple[str, float]:
    """Return a simple visibility label for a cropped stop-sign image."""
    if stop_sign_crop.size == 0:
        return "unknown", 0.0

    image_area = float(max(1, stop_sign_crop.shape[0] * stop_sign_crop.shape[1]))
    blurred = cv2.GaussianBlur(stop_sign_crop, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    red_mask_1 = cv2.inRange(hsv, (0, 80, 60), (12, 255, 255))
    red_mask_2 = cv2.inRange(hsv, (168, 80, 60), (180, 255, 255))
    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

    red_ratio = float(np.count_nonzero(red_mask)) / image_area
    largest_blob_area, polygon_sides = _largest_red_blob(red_mask)
    largest_blob_ratio = largest_blob_area / image_area

    score = min(1.0, max(red_ratio, largest_blob_ratio * 1.5))
    if largest_blob_ratio < 0.02 and red_ratio < 0.03:
        return "unknown", 0.0
    if largest_blob_ratio >= 0.15 and 6 <= polygon_sides <= 10:
        return "clear", score
    if largest_blob_ratio >= 0.06:
        return "partial", score
    return "weak", score
