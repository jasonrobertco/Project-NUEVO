from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export an Ultralytics YOLO model for Raspberry Pi benchmarking."
    )
    parser.add_argument(
        "--model",
        default="yolo26n.pt",
        help="Ultralytics model name or local .pt path, e.g. yolo26n.pt or yolo11n.pt.",
    )
    parser.add_argument(
        "--format",
        default="ncnn",
        choices=("ncnn", "openvino", "onnx"),
        help="Export format. NCNN is the first candidate for Raspberry Pi CPU.",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Export image size.",
    )
    parser.add_argument(
        "--half",
        action="store_true",
        help="Request FP16 export if the selected backend supports it.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    model = YOLO(args.model)
    exported = model.export(format=args.format, imgsz=args.imgsz, half=args.half)
    print(f"Exported model: {Path(exported)}")


if __name__ == "__main__":
    main()
