# Vision Node Plan

The ROS vision node uses an exported Ultralytics YOLO26n NCNN model. The old
Darknet/YOLOv4 path is removed, and the ROS runtime loads NCNN directly
without importing Ultralytics or Torch.

## Runtime Model

Default model:

```text
ros2_ws/src/vision/data/yolo26n_ncnn_imgsz_416/
```

The model is exported at image size 416 and is tracked in Git because the NCNN
runtime files are small enough for the project repository. The 640 export is
kept in the same data folder as a quality baseline for comparison.

Default runtime parameters:

- `model_path`: exported NCNN model directory
- `model_imgsz`: `416`
- `class_filter`: `traffic light,stop sign,person`
- `confidence_threshold`: `0.25`
- `camera_fps`: `15.0`
- `process_rate_hz`: `4.0`
- `ncnn_threads`: `4`
- `camera_device`: `/dev/video10`
- `camera_width`: `640`
- `camera_height`: `480`
- `debug_output_dir`: `/runtime_output/vision`

Set `class_filter` to an empty string to publish all model classes:

```bash
ros2 run vision vision_node --ros-args -p class_filter:=""
```

Echo the detection topic from another shell inside the ROS container:

```bash
ros2 topic echo /vision/detections
```

Launch files:

```bash
ros2 launch vision vision_production.launch.py
ros2 launch vision vision_debug.launch.py
```

## Message Contract

Topic:

```text
/vision/detections
```

Message type:

```text
bridge_interfaces/msg/VisionDetectionArray
```

Behavior:

- one message is published per processed frame
- empty frames publish an empty `detections` list
- `class_name` uses the Ultralytics/COCO class name, for example
  `traffic light`, `stop sign`, or `person`
- bounding boxes use pixel coordinates in the captured image
- traffic-light detections include a `color` attribute when color
  classification is enabled
- optional debug image output writes annotated frames to `/runtime_output/vision`

## Student Extension Points

The main processing loop keeps object-specific logic visible:

- `traffic light`: classifies red/green/other and attaches a `color` attribute
- `face`: placeholder for student face/customer analysis with a custom model
- `my_object`: placeholder for student custom object-specific checks

Students can add their own object logic by filtering on `detection.class_name`
and attaching attributes with:

```python
detection.add_attribute("name", "value", score)
```

## Trying Another Ultralytics Model

Export the model to NCNN outside the ROS node, then put the exported folder
under:

```text
ros2_ws/src/vision/data/
```

Launch with:

```bash
ros2 run vision vision_node --ros-args \
  -p model_path:=/ros2_ws/src/vision/data/<model_folder> \
  -p model_imgsz:=416 \
  -p class_filter:="traffic light,stop sign,person"
```

The benchmark and export scripts under `vision_model_benchmark/` are kept as a
native Ubuntu reference workflow and are intentionally separate from the ROS
package.
