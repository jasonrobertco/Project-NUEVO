from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import time

import cv2

from vision.model_utils import DetectedObject


@dataclass
class DebugOverlay:
    color: tuple[int, int, int]
    contour: object | None = None
    label: str | None = None
    x: int = 0
    y: int = 0


class DetectionDebugWriter:
    """Write annotated debug images to disk at a controlled rate."""

    def __init__(
        self,
        enabled: bool,
        output_dir: str,
        save_rate_hz: float,
        save_only_on_detection: bool,
        save_latest: bool,
        save_timestamped: bool,
        max_timestamped_images: int,
        logger,
    ) -> None:
        self.enabled = bool(enabled)
        self.output_dir = Path(output_dir).expanduser()
        self.period = 1.0 / float(save_rate_hz) if save_rate_hz > 0.0 else 0.0
        self.save_only_on_detection = bool(save_only_on_detection)
        self.save_latest = bool(save_latest)
        self.save_timestamped = bool(save_timestamped)
        self.max_timestamped_images = max(0, int(max_timestamped_images))
        self.logger = logger
        self._last_save_time = 0.0
        self._announced_output_dir = False

        if self.enabled and not (self.save_latest or self.save_timestamped):
            self.logger.warn(
                "Vision debug saving is enabled but no output mode is selected; disabling debug writer."
            )
            self.enabled = False

    def maybe_write(
        self,
        frame_bgr,
        detected_objects: list[DetectedObject],
        debug_overlays: list[DebugOverlay] | None = None,
    ) -> None:
        if not self.enabled:
            return

        if self.save_only_on_detection and not detected_objects:
            return

        now = time.monotonic()
        if self.period > 0.0 and now - self._last_save_time < self.period:
            return

        try:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            annotated = annotate_detections(frame_bgr, detected_objects, debug_overlays=debug_overlays)

            if self.save_latest:
                latest_path = self.output_dir / "latest.jpg"
                _write_jpeg(latest_path, annotated)

            if self.save_timestamped:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                snapshot_path = self.output_dir / f"vision_{timestamp}.jpg"
                _write_jpeg(snapshot_path, annotated)
                self._prune_timestamped_images()

            self._last_save_time = now
            if not self._announced_output_dir:
                self.logger.info(f"Vision debug images will be written to {self.output_dir}")
                self._announced_output_dir = True
        except Exception as exc:  # noqa: BLE001
            self.logger.error(
                f"Disabling vision debug image saving after write failure: {exc}"
            )
            self.enabled = False

    def _prune_timestamped_images(self) -> None:
        if self.max_timestamped_images <= 0:
            return

        snapshots = sorted(self.output_dir.glob("vision_*.jpg"))
        excess_count = len(snapshots) - self.max_timestamped_images
        if excess_count <= 0:
            return

        for path in snapshots[:excess_count]:
            path.unlink(missing_ok=True)


def annotate_detections(
    frame_bgr,
    detected_objects: list[DetectedObject],
    debug_overlays: list[DebugOverlay] | None = None,
):
    annotated = frame_bgr.copy()
    for detected_object in detected_objects:
        color = _class_color(detected_object.class_name)
        x1 = int(detected_object.x)
        y1 = int(detected_object.y)
        x2 = int(detected_object.x + detected_object.width)
        y2 = int(detected_object.y + detected_object.height)

        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

        label = _build_label(detected_object)
        _draw_label(annotated, label, x1, y1, color)

    for overlay in debug_overlays or ():
        if overlay.contour is not None:
            cv2.drawContours(annotated, [overlay.contour], -1, overlay.color, 2)
        if overlay.label:
            _draw_label(annotated, overlay.label, int(overlay.x), int(overlay.y), overlay.color)

    return annotated


def _write_jpeg(path: Path, image) -> None:
    ok = cv2.imwrite(str(path), image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if not ok:
        raise RuntimeError(f"failed to write debug image: {path}")


def _build_label(detected_object: DetectedObject) -> str:
    parts = [f"{detected_object.class_name} {detected_object.confidence:.2f}"]
    for attribute in detected_object.attributes:
        parts.append(f"{attribute.name}={attribute.value}")
    return " | ".join(parts)


def _draw_label(image, text: str, x: int, y: int, color) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.5
    thickness = 1
    (text_width, text_height), baseline = cv2.getTextSize(text, font, scale, thickness)

    text_x = max(0, x)
    text_y = y - 8
    if text_y - text_height - baseline < 0:
        text_y = y + text_height + 8

    background_top_left = (text_x, text_y - text_height - baseline)
    background_bottom_right = (text_x + text_width + 6, text_y + 2)
    cv2.rectangle(image, background_top_left, background_bottom_right, color, -1)
    cv2.putText(
        image,
        text,
        (text_x + 3, text_y - 2),
        font,
        scale,
        (255, 255, 255),
        thickness,
        cv2.LINE_AA,
    )


def _class_color(class_name: str) -> tuple[int, int, int]:
    seed = sum(ord(char) for char in class_name)
    blue = 80 + (seed * 97) % 176
    green = 80 + (seed * 57) % 176
    red = 80 + (seed * 31) % 176
    return int(blue), int(green), int(red)
