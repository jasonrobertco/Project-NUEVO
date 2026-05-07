from __future__ import annotations

import argparse
import os
import statistics
import time
from pathlib import Path
from typing import Optional

import cv2
import psutil
from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Benchmark Ultralytics YOLO on a native Raspberry Pi camera source."
    )
    parser.add_argument(
        "--model",
        default="yolo26n_ncnn_model",
        help="Model path/name. Use an exported NCNN directory for Pi tests.",
    )
    parser.add_argument(
        "--source",
        default="/dev/video10",
        help="Camera device, camera index, image, or video path.",
    )
    parser.add_argument("--width", type=int, default=640, help="Requested camera width.")
    parser.add_argument("--height", type=int, default=480, help="Requested camera height.")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO inference image size.")
    parser.add_argument("--conf", type=float, default=0.35, help="Detection confidence threshold.")
    parser.add_argument(
        "--classes",
        default="traffic light,stop sign,person",
        help="Comma-separated COCO class names to keep, or empty string for all classes.",
    )
    parser.add_argument("--duration", type=float, default=60.0, help="Timed benchmark duration in seconds.")
    parser.add_argument("--warmup", type=int, default=5, help="Warmup frames before timing.")
    parser.add_argument("--target-fps", type=float, default=5.0, help="Target processed FPS.")
    parser.add_argument(
        "--max-system-cpu",
        type=float,
        default=80.0,
        help="Target max total system CPU percent, 0-100 across all cores.",
    )
    parser.add_argument(
        "--save-sample",
        default="samples/last_detection.jpg",
        help="Path for one annotated sample image. Use empty string to disable.",
    )
    return parser.parse_args()


def source_is_camera(source: str) -> bool:
    return source.isdigit() or source.startswith("/dev/video")


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
    missing = sorted(requested - {str(names[index]).lower() for index in indexes})
    if missing:
        print(f"WARNING: model does not expose requested classes: {', '.join(missing)}")
    return indexes


def summarize(values: list[float], unit: str) -> str:
    if not values:
        return "n/a"
    ordered = sorted(values)
    p50 = statistics.median(ordered)
    p90 = ordered[min(len(ordered) - 1, int(len(ordered) * 0.90))]
    p95 = ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))]
    return f"avg={statistics.mean(ordered):.2f}{unit} p50={p50:.2f}{unit} p90={p90:.2f}{unit} p95={p95:.2f}{unit}"


def run_camera_benchmark(args: argparse.Namespace, model: YOLO) -> None:
    cap = open_camera(args.source, args.width, args.height)
    keep_classes = class_indexes(model, args.classes)
    process = psutil.Process()
    cpu_count = max(1, os.cpu_count() or 1)

    sample_path = Path(args.save_sample) if args.save_sample else None
    if sample_path:
        sample_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"Model: {args.model}")
    print(f"Source: {args.source} requested={args.width}x{args.height} imgsz={args.imgsz}")
    print(f"Classes: {args.classes or 'all'}")
    print(f"Warmup frames: {args.warmup}")

    for _ in range(args.warmup):
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError("Camera read failed during warmup")
        model.predict(frame, imgsz=args.imgsz, conf=args.conf, classes=keep_classes, verbose=False)

    frame_times_ms: list[float] = []
    system_cpu_samples: list[float] = []
    process_cpu_samples: list[float] = []
    detections_by_class: dict[str, int] = {}
    last_result = None

    psutil.cpu_percent(interval=None)
    process.cpu_percent(interval=None)
    start = time.perf_counter()
    deadline = start + args.duration
    frames = 0

    while time.perf_counter() < deadline:
        ok, frame = cap.read()
        if not ok:
            print("WARNING: camera read failed; skipping frame")
            continue

        t0 = time.perf_counter()
        result = model.predict(
            frame,
            imgsz=args.imgsz,
            conf=args.conf,
            classes=keep_classes,
            verbose=False,
        )[0]
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        frame_times_ms.append(elapsed_ms)
        frames += 1
        last_result = result

        system_cpu_samples.append(psutil.cpu_percent(interval=None))
        process_cpu_samples.append(process.cpu_percent(interval=None) / cpu_count)

        if result.boxes is not None:
            for cls in result.boxes.cls.tolist():
                name = str(result.names[int(cls)])
                detections_by_class[name] = detections_by_class.get(name, 0) + 1

    elapsed = time.perf_counter() - start
    cap.release()

    if sample_path and last_result is not None:
        cv2.imwrite(str(sample_path), last_result.plot())
        print(f"Sample image: {sample_path}")

    fps = frames / elapsed if elapsed > 0.0 else 0.0
    avg_system_cpu = statistics.mean(system_cpu_samples) if system_cpu_samples else 0.0
    avg_process_cpu = statistics.mean(process_cpu_samples) if process_cpu_samples else 0.0

    print()
    print("Results")
    print("-------")
    print(f"Processed frames: {frames}")
    print(f"Elapsed: {elapsed:.2f}s")
    print(f"Processed FPS: {fps:.2f}")
    print(f"Inference latency: {summarize(frame_times_ms, 'ms')}")
    print(f"System CPU: {summarize(system_cpu_samples, '%')}")
    print(f"YOLO process CPU normalized across cores: {summarize(process_cpu_samples, '%')}")
    print(f"Average system CPU: {avg_system_cpu:.1f}%")
    print(f"Average YOLO process CPU normalized: {avg_process_cpu:.1f}%")
    print(f"Detections by class: {detections_by_class or '{}'}")

    fps_ok = fps >= args.target_fps
    cpu_ok = avg_system_cpu <= args.max_system_cpu
    print(f"Target FPS >= {args.target_fps}: {'PASS' if fps_ok else 'FAIL'}")
    print(f"Average system CPU <= {args.max_system_cpu}%: {'PASS' if cpu_ok else 'FAIL'}")


def run_single_input(args: argparse.Namespace, model: YOLO) -> None:
    keep_classes = class_indexes(model, args.classes)
    result = model.predict(
        args.source,
        imgsz=args.imgsz,
        conf=args.conf,
        classes=keep_classes,
        verbose=False,
    )[0]
    print(result.verbose())
    if args.save_sample:
        sample_path = Path(args.save_sample)
        sample_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(sample_path), result.plot())
        print(f"Sample image: {sample_path}")


def main() -> None:
    args = parse_args()
    model = YOLO(args.model)
    if source_is_camera(args.source):
        run_camera_benchmark(args, model)
    else:
        run_single_input(args, model)


if __name__ == "__main__":
    main()
