# Vision Model Data

For package-level runtime usage, debugging, and launch integration, see:
- [`../README.md`](../README.md)

The default ROS vision node model is an exported Ultralytics YOLO26n NCNN
model:

```text
ros2_ws/src/vision/data/yolo26n_ncnn_imgsz_416/
```

It was exported with image size 416 and is tracked in Git because the NCNN
runtime files are small enough for this project. The 640 export is also kept as
a quality baseline for comparison.

The ROS runtime loads `model.ncnn.param`, `model.ncnn.bin`, and
`metadata.yaml` directly with the `ncnn` Python package. It does not install or
import the Ultralytics Python package.

Default detection classes:

- `traffic light`
- `stop sign`
- `person`

Default runtime settings use:

- `model_imgsz:=416`
- `camera_fps:=15.0`
- `process_rate_hz:=4.0`
- `confidence_threshold:=0.25`
- `ncnn_threads:=4`

Launch files:

```bash
ros2 launch vision vision_production.launch.py
ros2 launch vision vision_debug.launch.py
```

To try another Ultralytics model, export it to NCNN outside the ROS runtime,
place the exported model folder under this directory, and launch the node with:

```bash
ros2 run vision vision_node --ros-args \
  -p model_path:=/ros2_ws/src/vision/data/<model_folder> \
  -p model_imgsz:=416
```

For all COCO classes, pass an empty class filter:

```bash
ros2 run vision vision_node --ros-args -p class_filter:=""
```

To check live output:

```bash
ros2 topic echo /vision/detections
```

Default debug output path inside the container:

```text
/runtime_output/vision/latest.jpg
```

On the host, Docker bind-mounts that to:

```text
ros2_ws/runtime_output/vision/latest.jpg
```

Ignored local model files remain useful for experiments:

- `*.weights`
- `*.pt`
- `*.onnx`
- `*.engine`
