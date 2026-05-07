# Raspberry Pi YOLO Benchmark

This folder is for testing Ultralytics YOLO on native Ubuntu before changing
the ROS vision node.

Target:

- Raspberry Pi 5
- native Ubuntu host, not Docker
- COCO object detection
- `/dev/video10` from the existing Pi camera loopback service
- at least 5 processed frames per second
- about 80% or less total system CPU

## Model Candidates

Test these in order:

1. `yolo26n` exported to NCNN
2. `yolo11n` exported to NCNN if `yolo26n` is not available in the installed
   Ultralytics version
3. `yolo26s` exported to NCNN only if Nano is too unreliable; it is expected
   to be much heavier

Use NCNN first because it is designed for mobile/embedded CPU inference and
Ultralytics documents it as the recommended Raspberry Pi export path.

COCO includes `traffic light`, `stop sign`, and `person`. It does not include
a dedicated `face` class.

## Quick Start on the Pi

From the project root:

```bash
cd ~/Project-NUEVO/vision_model_benchmark
sudo apt update
sudo apt install -y python3-venv python3-pip libgl1 libglib2.0-0 v4l-utils
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip wheel setuptools
pip install -r requirements.txt
```

Use the plain `ultralytics` package here, not `ultralytics[export]`.
The `[export]` extra installs many optional backends that we do not need for
this Pi benchmark, and some of them may try slow source builds on ARM.

Verify the camera first:

```bash
v4l2-ctl --device=/dev/video10 --all
```

If `/dev/video10` still reports `1280x720`, the benchmark script can request
`640x480`, but the loopback feed may still be produced at the service
resolution. For a real lower-resolution test, set the host camera loopback
service to produce `640x480`, then restart that service before benchmarking.

Export the first candidate:

```bash
python export_model.py --model yolo26n.pt --format ncnn --imgsz 640
```

If the NCNN export asks for one small missing dependency, install only that
dependency instead of installing the full `ultralytics[export]` extra.

If `yolo26n.pt` is not available in your installed Ultralytics version, use:

```bash
python export_model.py --model yolo11n.pt --format ncnn --imgsz 640
```

Run the 60-second camera benchmark:

```bash
python benchmark_ultralytics.py \
  --model yolo26n_ncnn_model \
  --source /dev/video10 \
  --width 640 \
  --height 480 \
  --imgsz 640 \
  --conf 0.35 \
  --classes "traffic light,stop sign,person" \
  --duration 60 \
  --target-fps 5 \
  --max-system-cpu 80
```

For the YOLO11 fallback:

```bash
python benchmark_ultralytics.py \
  --model yolo11n_ncnn_model \
  --source /dev/video10 \
  --width 640 \
  --height 480 \
  --imgsz 640 \
  --conf 0.35 \
  --classes "traffic light,stop sign,person" \
  --duration 60 \
  --target-fps 5 \
  --max-system-cpu 80
```

The benchmark prints:

- processed FPS
- inference latency
- total system CPU
- YOLO process CPU normalized across all Pi CPU cores
- detection counts by class
- PASS/FAIL for the 5 Hz and 80% CPU targets

It also saves one annotated sample image to:

```bash
vision_model_benchmark/samples/last_detection.jpg
```

## Live Detection Print Test

After the benchmark passes, run a live print test to check whether the output
is useful for robot decisions:

```bash
python live_detect.py \
  --model yolo26n_ncnn_model \
  --source /dev/video10 \
  --width 640 \
  --height 480 \
  --imgsz 640 \
  --conf 0.35 \
  --classes "traffic light,stop sign,person"
```

Example output:

```text
  12.42s fps=6.40 | person conf=0.78 bbox=(132,40)-(410,470)
```

To print every processed frame, including empty results:

```bash
python live_detect.py --model yolo26n_ncnn_model --source /dev/video10 --show-empty
```

## Faster Test Matrix

If 640 input is too slow, test smaller model input sizes:

```bash
python benchmark_ultralytics.py --model yolo26n_ncnn_model --source /dev/video10 --imgsz 512 --duration 60
python benchmark_ultralytics.py --model yolo26n_ncnn_model --source /dev/video10 --imgsz 416 --duration 60
python benchmark_ultralytics.py --model yolo26n_ncnn_model --source /dev/video10 --imgsz 320 --duration 60
```

Expected tradeoff:

- lower `imgsz` gives higher FPS
- lower `imgsz` may miss small or distant traffic lights and stop signs
- for classroom use, `416` or `512` may be the useful middle ground

## Interpreting Results

Use a model/settings pair only if both are true:

- processed FPS is consistently at or above 5 Hz
- average total system CPU is near or below 80%

Then check the saved sample image and live detections:

- traffic light boxes should appear on the actual light, not the whole scene
- stop signs should be reliable at the expected classroom distance
- false positives should be rare enough for robot decisions

If Nano meets the speed target but misses too many objects, try:

```bash
python export_model.py --model yolo26s.pt --format ncnn --imgsz 640
python benchmark_ultralytics.py --model yolo26s_ncnn_model --source /dev/video10 --duration 60
```

Only keep the Small model if it still passes the CPU/FPS target.

## Notes

- Do not commit downloaded `.pt` weights or exported model folders.
- This benchmark is intentionally separate from ROS.
- After selecting the model, we can update the ROS vision node to load the
  exported model directory from a documented local path.
