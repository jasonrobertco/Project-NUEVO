from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import time

import cv2
import ncnn
import numpy as np


DEFAULT_MODEL_DIR = "yolo26n_ncnn_imgsz_416"


@dataclass(frozen=True)
class DetectionAttribute:
    name: str
    value: str
    score: float


@dataclass
class DetectedObject:
    class_name: str
    confidence: float
    x: int
    y: int
    width: int
    height: int
    attributes: list[DetectionAttribute] = field(default_factory=list)

    def add_attribute(self, name: str, value: str, score: float) -> None:
        self.attributes.append(DetectionAttribute(name=name, value=value, score=float(score)))


@dataclass(frozen=True)
class LetterboxInfo:
    scale: float
    pad_x: int
    pad_y: int
    input_width: int
    input_height: int
    original_width: int
    original_height: int


class YoloNcnnDetector:
    """Minimal NCNN runtime for Ultralytics-exported YOLO detect models."""

    input_name = "in0"
    output_name = "out0"

    def __init__(
        self,
        model_path: str | Path,
        input_size: int,
        confidence_threshold: float,
        iou_threshold: float,
        max_detections: int,
        class_filter: str,
        ncnn_threads: int = 0,
    ) -> None:
        self.model_path = Path(model_path).expanduser()
        self.input_size = int(input_size)
        self.confidence_threshold = float(confidence_threshold)
        self.iou_threshold = float(iou_threshold)
        self.max_detections = int(max_detections)
        self.ncnn_threads = int(ncnn_threads)

        metadata = _load_ultralytics_metadata(self.model_path / "metadata.yaml")
        self.names = metadata["names"]
        self.class_filter_ids = _resolve_class_filter(self.names, class_filter)

        param_path = self.model_path / "model.ncnn.param"
        bin_path = self.model_path / "model.ncnn.bin"
        if not param_path.is_file() or not bin_path.is_file():
            raise FileNotFoundError(
                "NCNN model folder must contain model.ncnn.param and model.ncnn.bin: "
                f"{self.model_path}"
            )

        self._net = ncnn.Net()
        self._net.opt.use_vulkan_compute = False
        if self.ncnn_threads > 0:
            ncnn.set_omp_dynamic(0)
            ncnn.set_omp_num_threads(self.ncnn_threads)
            self._net.opt.num_threads = self.ncnn_threads
        self._net.load_param(str(param_path))
        self._net.load_model(str(bin_path))
        self.last_preprocess_ms = 0.0
        self.last_inference_ms = 0.0
        self.last_postprocess_ms = 0.0

    @property
    def class_count(self) -> int:
        return len(self.names)

    def predict(self, frame_bgr: np.ndarray) -> list[DetectedObject]:
        preprocess_start = time.monotonic()
        input_tensor, letterbox = _letterbox_bgr_to_ncnn_mat(frame_bgr, self.input_size)
        inference_start = time.monotonic()

        with self._net.create_extractor() as extractor:
            extractor.input(self.input_name, input_tensor)
            _, output = extractor.extract(self.output_name)

        postprocess_start = time.monotonic()
        predictions = _normalize_output(np.array(output), class_count=self.class_count)
        detected_objects = _decode_predictions(
            predictions=predictions,
            names=self.names,
            letterbox=letterbox,
            confidence_threshold=self.confidence_threshold,
            iou_threshold=self.iou_threshold,
            max_detections=self.max_detections,
            class_filter_ids=self.class_filter_ids,
        )
        done = time.monotonic()
        self.last_preprocess_ms = (inference_start - preprocess_start) * 1000.0
        self.last_inference_ms = (postprocess_start - inference_start) * 1000.0
        self.last_postprocess_ms = (done - postprocess_start) * 1000.0
        return detected_objects


def default_model_path(
    source_data_dir: Path,
    share_data_dir: Path,
    default_model_dir: str = DEFAULT_MODEL_DIR,
) -> Path:
    source_model_path = source_data_dir / default_model_dir
    if source_model_path.is_dir():
        return source_model_path
    return share_data_dir / default_model_dir


def resolve_model_path(
    raw_path: str,
    source_data_dir: Path,
    share_data_dir: Path,
) -> str:
    raw_path = raw_path.strip()
    if not raw_path:
        raise ValueError("model_path cannot be empty")

    path = Path(raw_path).expanduser()
    if path.is_absolute():
        if not path.exists():
            raise FileNotFoundError(f"NCNN model path not found: {path}")
        return str(path)

    candidates = [
        Path.cwd() / path,
        source_data_dir / path,
        share_data_dir / path,
    ]
    for candidate in candidates:
        if candidate.exists():
            return str(candidate)

    raise FileNotFoundError(
        "NCNN model path not found: %s. Put exported models under "
        "ros2_ws/src/vision/data or pass model_path:=/absolute/path." % raw_path
    )


def _load_ultralytics_metadata(path: Path) -> dict[str, dict[int, str]]:
    if not path.is_file():
        raise FileNotFoundError(f"Ultralytics metadata file not found: {path}")

    names: dict[int, str] = {}
    in_names = False
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.rstrip()
            stripped = line.strip()
            if stripped == "names:":
                in_names = True
                continue
            if in_names:
                if not raw_line.startswith("  "):
                    break
                if ":" not in stripped:
                    continue
                raw_index, raw_name = stripped.split(":", 1)
                raw_index = raw_index.strip()
                if raw_index.isdigit():
                    names[int(raw_index)] = raw_name.strip().strip("'\"")

    if not names:
        raise ValueError(f"No class names found in Ultralytics metadata: {path}")

    return {"names": names}


def _resolve_class_filter(names: dict[int, str], raw_filter: str) -> set[int] | None:
    requested = {name.strip().lower() for name in raw_filter.split(",") if name.strip()}
    if not requested:
        return None

    class_ids = {
        class_id
        for class_id, class_name in names.items()
        if class_name.strip().lower() in requested
    }
    missing = requested - {names[class_id].strip().lower() for class_id in class_ids}
    if missing:
        raise ValueError(
            "class_filter entries not found in model metadata: "
            + ", ".join(sorted(missing))
        )
    return class_ids


def _letterbox_bgr_to_ncnn_mat(
    frame_bgr: np.ndarray,
    input_size: int,
) -> tuple[ncnn.Mat, LetterboxInfo]:
    original_height, original_width = frame_bgr.shape[:2]
    scale = min(input_size / float(original_width), input_size / float(original_height))
    resized_width = max(1, int(round(original_width * scale)))
    resized_height = max(1, int(round(original_height * scale)))
    pad_x = (input_size - resized_width) // 2
    pad_y = (input_size - resized_height) // 2

    resized = cv2.resize(frame_bgr, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((input_size, input_size, 3), 114, dtype=np.uint8)
    canvas[pad_y : pad_y + resized_height, pad_x : pad_x + resized_width] = resized

    input_mat = ncnn.Mat.from_pixels(
        np.ascontiguousarray(canvas),
        ncnn.Mat.PixelType.PIXEL_BGR2RGB,
        input_size,
        input_size,
    )
    input_mat.substract_mean_normalize(
        [0.0, 0.0, 0.0],
        [1.0 / 255.0, 1.0 / 255.0, 1.0 / 255.0],
    )
    return (
        input_mat,
        LetterboxInfo(
            scale=scale,
            pad_x=pad_x,
            pad_y=pad_y,
            input_width=input_size,
            input_height=input_size,
            original_width=original_width,
            original_height=original_height,
        ),
    )


def _normalize_output(raw_output: np.ndarray, class_count: int) -> np.ndarray:
    output = np.squeeze(raw_output)
    if output.ndim != 2:
        raise ValueError(f"Unexpected NCNN output shape: {raw_output.shape}")

    attribute_count = class_count + 4
    if output.shape[0] == attribute_count:
        output = output.T
    elif output.shape[1] == attribute_count:
        pass
    elif output.shape[0] == 6:
        output = output.T
    elif output.shape[1] == 6:
        pass
    else:
        raise ValueError(f"Unexpected YOLO output shape: {raw_output.shape}")

    return np.ascontiguousarray(output, dtype=np.float32)


def _decode_predictions(
    predictions: np.ndarray,
    names: dict[int, str],
    letterbox: LetterboxInfo,
    confidence_threshold: float,
    iou_threshold: float,
    max_detections: int,
    class_filter_ids: set[int] | None,
) -> list[DetectedObject]:
    if predictions.size == 0:
        return []

    if predictions.shape[1] == 6:
        boxes_xyxy = predictions[:, 0:4]
        confidences = predictions[:, 4]
        class_ids = predictions[:, 5].astype(np.int32)
    else:
        boxes_xywh = predictions[:, 0:4]
        class_scores = predictions[:, 4:]
        class_ids = np.argmax(class_scores, axis=1).astype(np.int32)
        confidences = class_scores[np.arange(class_scores.shape[0]), class_ids]
        boxes_xyxy = _xywh_to_xyxy(boxes_xywh)

    if boxes_xyxy.size and float(np.nanmax(boxes_xyxy)) <= 2.0:
        boxes_xyxy[:, [0, 2]] = boxes_xyxy[:, [0, 2]] * float(letterbox.input_width)
        boxes_xyxy[:, [1, 3]] = boxes_xyxy[:, [1, 3]] * float(letterbox.input_height)

    valid = confidences >= confidence_threshold
    if class_filter_ids is not None:
        valid &= np.array([class_id in class_filter_ids for class_id in class_ids], dtype=bool)

    if not np.any(valid):
        return []

    boxes_xyxy = boxes_xyxy[valid]
    confidences = confidences[valid]
    class_ids = class_ids[valid]

    boxes_xyxy = _scale_boxes_to_original_image(boxes_xyxy, letterbox)
    boxes_xyxy = _clip_boxes(boxes_xyxy, letterbox.original_width, letterbox.original_height)

    keep = _classwise_nms(boxes_xyxy, confidences, class_ids, confidence_threshold, iou_threshold)
    if not keep:
        return []

    keep = sorted(keep, key=lambda index: float(confidences[index]), reverse=True)
    if max_detections > 0:
        keep = keep[:max_detections]

    detected_objects: list[DetectedObject] = []
    for index in keep:
        x1, y1, x2, y2 = [int(round(value)) for value in boxes_xyxy[index].tolist()]
        width = max(0, x2 - x1)
        height = max(0, y2 - y1)
        if width <= 0 or height <= 0:
            continue
        class_id = int(class_ids[index])
        detected_objects.append(
            DetectedObject(
                class_name=names.get(class_id, f"class_{class_id}"),
                confidence=float(confidences[index]),
                x=x1,
                y=y1,
                width=width,
                height=height,
            )
        )
    return detected_objects


def _xywh_to_xyxy(boxes_xywh: np.ndarray) -> np.ndarray:
    boxes_xyxy = np.empty_like(boxes_xywh, dtype=np.float32)
    boxes_xyxy[:, 0] = boxes_xywh[:, 0] - boxes_xywh[:, 2] / 2.0
    boxes_xyxy[:, 1] = boxes_xywh[:, 1] - boxes_xywh[:, 3] / 2.0
    boxes_xyxy[:, 2] = boxes_xywh[:, 0] + boxes_xywh[:, 2] / 2.0
    boxes_xyxy[:, 3] = boxes_xywh[:, 1] + boxes_xywh[:, 3] / 2.0
    return boxes_xyxy


def _scale_boxes_to_original_image(
    boxes_xyxy: np.ndarray,
    letterbox: LetterboxInfo,
) -> np.ndarray:
    scaled = boxes_xyxy.copy()
    scaled[:, [0, 2]] = (scaled[:, [0, 2]] - float(letterbox.pad_x)) / letterbox.scale
    scaled[:, [1, 3]] = (scaled[:, [1, 3]] - float(letterbox.pad_y)) / letterbox.scale
    return scaled


def _clip_boxes(boxes_xyxy: np.ndarray, image_width: int, image_height: int) -> np.ndarray:
    clipped = boxes_xyxy.copy()
    clipped[:, [0, 2]] = np.clip(clipped[:, [0, 2]], 0.0, float(image_width))
    clipped[:, [1, 3]] = np.clip(clipped[:, [1, 3]], 0.0, float(image_height))
    return clipped


def _classwise_nms(
    boxes_xyxy: np.ndarray,
    confidences: np.ndarray,
    class_ids: np.ndarray,
    confidence_threshold: float,
    iou_threshold: float,
) -> list[int]:
    keep: list[int] = []
    for class_id in sorted(set(int(value) for value in class_ids.tolist())):
        class_indices = np.where(class_ids == class_id)[0]
        boxes = boxes_xyxy[class_indices]
        scores = confidences[class_indices]
        boxes_xywh = [
            [
                float(x1),
                float(y1),
                float(max(0.0, x2 - x1)),
                float(max(0.0, y2 - y1)),
            ]
            for x1, y1, x2, y2 in boxes.tolist()
        ]
        selected = cv2.dnn.NMSBoxes(
            boxes_xywh,
            scores.astype(float).tolist(),
            float(confidence_threshold),
            float(iou_threshold),
        )
        if len(selected) == 0:
            continue
        for local_index in np.array(selected).reshape(-1).tolist():
            keep.append(int(class_indices[int(local_index)]))
    return keep
