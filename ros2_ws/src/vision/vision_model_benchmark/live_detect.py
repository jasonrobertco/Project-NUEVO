from __future__ import annotations

import argparse
import time
from typing import Optional

import cv2
from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Print live YOLO detections from a camera stream."
    )
    parser.add_argument(
        "--model",
        default="yolo26n_ncnn_model",
        help="Model path/name, usually an exported NCNN model directory.",
    )
    parser.add_argument("--source", default="/dev/video10", help="Camera device or index.")
    parser.add_argument("--width", type=int, default=640, help="Requested camera width.")
    parser.add_argument("--height", type=int, default=480, help="Requested camera height.")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO inference image size.")
    parser.add_argument("--conf", type=float, default=0.35, help="Detection confidence threshold.")
    parser.add_argument(
        "--classes",
        default="traffic light,stop sign,person",
        help="Comma-separated class names to print, or empty string for all classes.",
    )
    parser.add_argument(
        "--print-interval",
        type=float,
        default=0.5,
        help="Minimum seconds between repeated prints of the same class.",
    )
    parser.add_argument(
        "--show-empty",
        action="store_true",
        help="Print 'no detection' messages when nothing is detected.",
    )
    return parser.parse_args()


def open_camera(source: str, width: int, height: int) -> cv2.VideoCapture:
    camera_source = int(source) if source.isdigit() else source
    cap = cv2.VideoCapture(camera_source, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera source: {source}")
    return cap


def class_indexes(model: YOLO, class_names: str) -> Optional[list[int]]:
    requested = {name.strip().lower() for name in class_names.split(",") if name.strip()}
    if not requested:
        return None

    names = model.names
    if isinstance(names, dict):
        name_items = names.items()
    else:
        name_items = enumerate(names)

    indexes = [int(index) for index, name in name_items if str(name).lower() in requested]
    available = {str(names[index]).lower() for index in indexes}
    missing = sorted(requested - available)
    if missing:
        print(f"WARNING: model does not expose requested classes: {', '.join(missing)}")
    return indexes


def format_detection(name: str, confidence: float, xyxy: list[float]) -> str:
    x1, y1, x2, y2 = [int(round(value)) for value in xyxy]
    return f"{name} conf={confidence:.2f} bbox=({x1},{y1})-({x2},{y2})"


def main() -> None:
    args = parse_args()
    model = YOLO(args.model)
    keep_classes = class_indexes(model, args.classes)
    cap = open_camera(args.source, args.width, args.height)

    last_print_by_class: dict[str, float] = {}
    frame_count = 0
    start = time.monotonic()

    print("Live detection started. Press Ctrl+C to stop.")
    print(f"Model: {args.model}")
    print(f"Source: {args.source} requested={args.width}x{args.height} imgsz={args.imgsz}")
    print(f"Classes: {args.classes or 'all'}")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("camera read failed")
                time.sleep(0.2)
                continue

            result = model.predict(
                frame,
                imgsz=args.imgsz,
                conf=args.conf,
                classes=keep_classes,
                verbose=False,
            )[0]
            frame_count += 1

            detections: list[str] = []
            if result.boxes is not None:
                for box in result.boxes:
                    class_index = int(box.cls.item())
                    name = str(result.names[class_index])
                    confidence = float(box.conf.item())
                    xyxy = box.xyxy[0].tolist()
                    detections.append(format_detection(name, confidence, xyxy))

            now = time.monotonic()
            if detections:
                class_names = {item.split(" ", 1)[0] for item in detections}
                should_print = any(
                    now - last_print_by_class.get(name, 0.0) >= args.print_interval
                    for name in class_names
                )
                if should_print:
                    elapsed = now - start
                    fps = frame_count / elapsed if elapsed > 0.0 else 0.0
                    print(f"{elapsed:7.2f}s fps={fps:4.2f} | " + " | ".join(detections))
                    for name in class_names:
                        last_print_by_class[name] = now
            elif args.show_empty:
                elapsed = now - start
                fps = frame_count / elapsed if elapsed > 0.0 else 0.0
                print(f"{elapsed:7.2f}s fps={fps:4.2f} | no detection")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        cap.release()


if __name__ == "__main__":
    main()
