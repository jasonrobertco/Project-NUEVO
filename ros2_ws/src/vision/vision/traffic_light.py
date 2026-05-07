from __future__ import annotations

import cv2
import numpy as np


def _largest_blob_score(mask: np.ndarray, image_area: float) -> float:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0.0
    largest_area = max(cv2.contourArea(contour) for contour in contours)
    return float(largest_area / image_area)


def classify_traffic_light_color(traffic_light_crop: np.ndarray) -> tuple[str, float]:
    """Return (color_label, score) for a cropped traffic-light image.

    The implementation is intentionally simple so students can inspect and tune it.
    """
    if traffic_light_crop.size == 0:
        return "other", 0.0

    image_area = float(max(1, traffic_light_crop.shape[0] * traffic_light_crop.shape[1]))
    blurred = cv2.GaussianBlur(traffic_light_crop, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # TODO: tune these HSV ranges for your lighting and traffic-light model.
    red_mask_1 = cv2.inRange(hsv, (0, 80, 80), (12, 255, 255))
    red_mask_2 = cv2.inRange(hsv, (168, 80, 80), (180, 255, 255))
    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)
    green_mask = cv2.inRange(hsv, (35, 60, 60), (95, 255, 255))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

    red_score = _largest_blob_score(red_mask, image_area)
    green_score = _largest_blob_score(green_mask, image_area)

    min_blob_ratio = 0.01
    dominance_ratio = 1.25

    if red_score < min_blob_ratio and green_score < min_blob_ratio:
        return "other", 0.0
    if red_score >= green_score * dominance_ratio:
        return "red", min(1.0, red_score)
    if green_score >= red_score * dominance_ratio:
        return "green", min(1.0, green_score)
    return "other", min(1.0, max(red_score, green_score))
