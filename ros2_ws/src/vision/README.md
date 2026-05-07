# Vision Package

This package provides the ROS 2 vision node used by the robot runtime.

The current node combines:
- YOLO object detection through NCNN
- small rule-based OpenCV checks for selected cases
- optional debug image saving

The published output is one topic:
- `/vision/detections` using `bridge_interfaces/msg/VisionDetectionArray`

The current default classes of interest are:
- `traffic light`
- `stop sign`
- `person`

The current rule-based example is:
- `yellow block`

## Quick Start

Assumptions:
- the workspace is built
- the camera device exists
- you are already inside the ROS 2 runtime environment

Run the production launch:

```bash
ros2 launch vision vision_production.launch.py
```

Run the debug launch:

```bash
ros2 launch vision vision_debug.launch.py
```

Run the node directly with a custom parameter:

```bash
ros2 run vision vision_node --ros-args -p enable_rule_based_detection:=true
```

Check the live topic:

```bash
ros2 topic echo /vision/detections
```

## What Gets Published

Each detection contains:
- `class_name`
- `confidence`
- `x`, `y`, `width`, `height`
- optional attributes

Examples of current attributes:
- traffic light:
  - `color=red|green|other`
- person:
  - `face_lighting=dim|normal|bright|unknown`
- stop sign:
  - `visibility=clear|partial|weak|unknown`
- yellow block:
  - `color=yellow`

## Debugging

The debug launch enables saved annotated images by default.

Inside the container, the latest image is written to:

```text
/runtime_output/vision/latest.jpg
```

On the host, that maps to:

```text
ros2_ws/runtime_output/vision/latest.jpg
```

Useful checks:

```bash
ros2 node list | grep vision
ros2 topic echo /vision/detections --once
```

Useful direct run for tuning:

```bash
ros2 run vision vision_node --ros-args \
  -p debug_save_enabled:=true \
  -p debug_save_only_on_detection:=false \
  -p enable_rule_based_detection:=true
```

If the topic is empty:
1. verify the camera device path
2. check the terminal for camera reconnect logs
3. enable debug image saving and inspect `latest.jpg`
4. temporarily clear the YOLO class filter:

```bash
ros2 run vision vision_node --ros-args -p class_filter:=""
```

## Customizing Detection Behavior

### YOLO layer

The NCNN model path and inference settings come from:
- `model_path`
- `model_imgsz`
- `confidence_threshold`
- `iou_threshold`
- `max_detections`
- `class_filter`
- `ncnn_threads`

Example:

```bash
ros2 run vision vision_node --ros-args \
  -p model_path:=/ros2_ws/src/vision/data/yolo26n_ncnn_imgsz_640 \
  -p model_imgsz:=640
```

The default packaged model files are documented in:
- [`data/README.md`](data/README.md)

### Rule-based yellow block

The current rule-based code lives in:
- [`vision/rule_based_detection.py`](vision/rule_based_detection.py)

That file is intentionally simple:
- fixed HSV thresholds
- fixed morphology kernel sizes
- fixed area and fill-ratio filters

If you want to tune yellow-block behavior, edit the numbers directly in:
- [`vision/rule_based_detection.py`](vision/rule_based_detection.py)

### Object-specific post-processing

Current per-class crop analysis lives in:
- [`vision/traffic_light.py`](vision/traffic_light.py)
- [`vision/stop_sign.py`](vision/stop_sign.py)
- [`vision/vision_node.py`](vision/vision_node.py) for the person-lighting helper

If you want to add a new attribute for a detected YOLO class, the current place
to do that is the object-crop branch in:
- [`vision/vision_node.py`](vision/vision_node.py)

## Launch Integration

There are two current ways to integrate vision into a larger launch file.

### Option 1: include the packaged launch as-is

Use this if the default parameters are fine.

```python
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

vision_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        str(Path(get_package_share_directory("vision")) / "launch" / "vision_production.launch.py")
    )
)
```

Or for debug mode:

```python
vision_debug_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        str(Path(get_package_share_directory("vision")) / "launch" / "vision_debug.launch.py")
    )
)
```

### Option 2: create the node directly in your parent launch

Use this if you need custom parameters, because the packaged launch files
currently do not expose launch arguments.

```python
from launch_ros.actions import Node

vision_node = Node(
    package="vision",
    executable="vision_node",
    name="vision_node",
    prefix=["nice", "-n", "10"],
    output="screen",
    parameters=[
        {
            "camera_device": "/dev/video10",
            "camera_width": 640,
            "camera_height": 480,
            "camera_fps": 15.0,
            "process_rate_hz": 4.0,
            "model_imgsz": 416,
            "confidence_threshold": 0.25,
            "iou_threshold": 0.7,
            "max_detections": 20,
            "ncnn_threads": 4,
            "enable_rule_based_detection": True,
            "debug_save_enabled": True,
            "debug_output_dir": "/runtime_output/vision",
        }
    ],
)
```

## Package Layout

Main runtime files:
- [`vision/vision_node.py`](vision/vision_node.py)
- [`vision/model_utils.py`](vision/model_utils.py)
- [`vision/camera_utils.py`](vision/camera_utils.py)
- [`vision/debug_utils.py`](vision/debug_utils.py)
- [`vision/traffic_light.py`](vision/traffic_light.py)
- [`vision/stop_sign.py`](vision/stop_sign.py)
- [`vision/rule_based_detection.py`](vision/rule_based_detection.py)

Launch files:
- [`launch/vision_production.launch.py`](launch/vision_production.launch.py)
- [`launch/vision_debug.launch.py`](launch/vision_debug.launch.py)

Model files:
- [`data/README.md`](data/README.md)

Benchmark/export scripts:
- [`vision_model_benchmark/README.md`](vision_model_benchmark/README.md)
