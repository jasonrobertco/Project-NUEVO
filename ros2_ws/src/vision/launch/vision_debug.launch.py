from launch import LaunchDescription
from launch_ros.actions import Node


COMMON_PARAMETERS = {
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
}


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="vision",
                executable="vision_node",
                name="vision_node",
                prefix=["nice -n 10"],
                output="screen",
                parameters=[
                    COMMON_PARAMETERS,
                    {
                        "debug_save_enabled": True,
                        "debug_output_dir": "/runtime_output/vision",
                        "debug_save_rate_hz": 1.0,
                        "debug_save_only_on_detection": False,
                        "debug_save_latest": True,
                        "debug_save_timestamped": False,
                    },
                ],
            )
        ]
    )
