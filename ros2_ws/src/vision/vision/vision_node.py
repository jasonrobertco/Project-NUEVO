from __future__ import annotations

from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from bridge_interfaces.msg import VisionDetection, VisionDetectionArray
import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node

from vision.camera_utils import ManagedCamera
from vision.debug_utils import DetectionDebugWriter
from vision.model_utils import (
    DetectedObject,
    YoloNcnnDetector,
    default_model_path,
    resolve_model_path,
)
from vision.rule_based_detection import (
    detect_yellow_block,
)
from vision.stop_sign import classify_stop_sign_visibility
from vision.timing_utils import FixedRateScheduler
from vision.traffic_light import classify_traffic_light_color


# User-facing COCO class filter.
# Add standard COCO names here, matching data/yolo26n_ncnn_imgsz_416/metadata.yaml.
CLASSES_OF_INTEREST = [
    "traffic light",
    "stop sign",
    "person",
    # "car",
    # "bus",
]

DEFAULT_CLASS_FILTER = ",".join(CLASSES_OF_INTEREST)


# ===========================================================================
# ArUco marker + laser dot detection (checkpoint-5 laser targeting)
# ===========================================================================
# The lasing code used to open /dev/video10 directly, which conflicts with
# this node owning the camera. These detectors run in the same frame loop and
# publish into /vision/detections, so the mission code drives the laser servo
# entirely through robot.get_detections().
#
# Published classes:
#   "aruco marker" - one per visible marker. bbox = tight quad bounding box.
#       attributes: marker_id, x_m / y_m / z_m (solvePnP pose, meters).
#   "laser dot"    - at most one (best-scoring candidate). bbox = small box
#       centered on the dot. attribute: peak (0-255 detector score).
#
# COORDINATE CONVENTION: the camera is mounted upside-down. The lasing
# calibration (yaw/pitch tables, offset signs) was built on 180-deg-flipped
# frames, so BOTH detectors run on cv2.flip(frame, -1) and publish coordinates
# in that flipped frame. The YOLO detections in the same message are in the
# RAW (unflipped) frame -- do not mix the two coordinate sets.

# Camera intrinsics (full-resolution 4608x2592 calibration, scaled to frame).
LASER_FX_FULL = 3386.34
LASER_FY_FULL = 3384.60
LASER_CX_FULL = 2304.0
LASER_CY_FULL = 1296.0
LASER_FULL_WIDTH_PX = 4608.0
LASER_FULL_HEIGHT_PX = 2592.0

ARUCO_MARKER_SIZE_M = 0.1      # physical marker edge length (match the course!)
ARUCO_DICT_ID = aruco.DICT_4X4_50

LASER_COLOR = "red"            # "red" | "green" | "blue"
LASER_TOPHAT_KSIZE = 21        # odd; bigger than the dot, smaller than a marker cell
LASER_SCORE_THRESH = 45        # threshold on the combined (top-hat + color) score
LASER_COLOR_WEIGHT = 1.0       # color cue weight vs. the top-hat cue
LASER_MIN_AREA = 1             # px^2, reject single-pixel sensor noise
LASER_MAX_AREA = 1200          # px^2, reject large bright blobs
LASER_DOT_BBOX_PX = 6          # published bbox edge length around the dot center

_ARUCO_DICT = aruco.getPredefinedDictionary(ARUCO_DICT_ID)
try:
    _ARUCO_PARAMS = aruco.DetectorParameters_create()
except AttributeError:  # OpenCV >= 4.7 renamed the constructor
    _ARUCO_PARAMS = aruco.DetectorParameters()

_ARUCO_HALF = ARUCO_MARKER_SIZE_M / 2.0
_ARUCO_OBJ_POINTS = np.array(
    [[-_ARUCO_HALF, _ARUCO_HALF, 0], [_ARUCO_HALF, _ARUCO_HALF, 0],
     [_ARUCO_HALF, -_ARUCO_HALF, 0], [-_ARUCO_HALF, -_ARUCO_HALF, 0]],
    dtype=np.float32,
)
_ARUCO_DIST_COEFFS = np.zeros((4, 1))


def _laser_camera_matrix(frame_width: int, frame_height: int) -> np.ndarray:
    """Scale the full-resolution intrinsic matrix down to the actual frame size."""
    sx = frame_width / LASER_FULL_WIDTH_PX
    sy = frame_height / LASER_FULL_HEIGHT_PX
    return np.array(
        [[LASER_FX_FULL * sx, 0, LASER_CX_FULL * sx],
         [0, LASER_FY_FULL * sy, LASER_CY_FULL * sy],
         [0, 0, 1]],
        dtype=np.float32,
    )


def _detect_aruco_markers(flipped_bgr) -> list[DetectedObject]:
    gray = cv2.cvtColor(flipped_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, _ARUCO_DICT, parameters=_ARUCO_PARAMS)
    if ids is None:
        return []

    h, w = flipped_bgr.shape[:2]
    camera_matrix = _laser_camera_matrix(w, h)

    detections: list[DetectedObject] = []
    for i in range(len(ids)):
        pts = corners[i][0]  # 4x2
        x0, y0 = float(np.min(pts[:, 0])), float(np.min(pts[:, 1]))
        x1, y1 = float(np.max(pts[:, 0])), float(np.max(pts[:, 1]))

        detection = DetectedObject(
            class_name="aruco marker",
            confidence=1.0,
            x=int(x0),
            y=int(y0),
            width=max(1, int(round(x1 - x0))),
            height=max(1, int(round(y1 - y0))),
        )
        detection.add_attribute("marker_id", str(int(ids[i][0])), 1.0)

        success, _rvec, tvec = cv2.solvePnP(
            _ARUCO_OBJ_POINTS, pts, camera_matrix, _ARUCO_DIST_COEFFS
        )
        if success:
            detection.add_attribute("x_m", f"{float(tvec[0][0]):.4f}", 1.0)
            detection.add_attribute("y_m", f"{float(tvec[1][0]):.4f}", 1.0)
            detection.add_attribute("z_m", f"{float(tvec[2][0]):.4f}", 1.0)
        detections.append(detection)
    return detections


def _detect_laser_dot(flipped_bgr) -> list[DetectedObject]:
    """Best laser-dot candidate via a combined top-hat + color-difference score.

    The top-hat isolates a SMALL bright spot from LARGE bright regions (white
    marker cells); the color difference helps on dark areas. Returns at most
    one detection (the candidate with the highest peak score).
    """
    b, g, r = cv2.split(flipped_bgr.astype(np.int16))
    if LASER_COLOR == "green":
        diff = g - cv2.max(r, b); chan = g
    elif LASER_COLOR == "blue":
        diff = b - cv2.max(r, g); chan = b
    else:  # red
        diff = r - cv2.max(g, b); chan = r
    diff = np.clip(diff, 0, 255).astype(np.uint8)
    chan = np.clip(chan, 0, 255).astype(np.uint8)

    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (LASER_TOPHAT_KSIZE, LASER_TOPHAT_KSIZE)
    )
    tophat = cv2.morphologyEx(chan, cv2.MORPH_TOPHAT, kernel)

    score = cv2.addWeighted(tophat, 1.0, diff, LASER_COLOR_WEIGHT, 0)
    score = cv2.GaussianBlur(score, (5, 5), 0)

    _, mask = cv2.threshold(score, LASER_SCORE_THRESH, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_xy = None
    best_peak = -1.0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < LASER_MIN_AREA or area > LASER_MAX_AREA:
            continue
        cmask = np.zeros(score.shape, dtype=np.uint8)
        cv2.drawContours(cmask, [contour], -1, 255, -1)
        _, peak, _, _ = cv2.minMaxLoc(score, mask=cmask)
        if peak > best_peak:
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            best_peak = peak
            best_xy = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])

    if best_xy is None:
        return []

    half = LASER_DOT_BBOX_PX // 2
    detection = DetectedObject(
        class_name="laser dot",
        confidence=min(1.0, best_peak / 255.0),
        x=int(round(best_xy[0])) - half,
        y=int(round(best_xy[1])) - half,
        width=LASER_DOT_BBOX_PX,
        height=LASER_DOT_BBOX_PX,
    )
    detection.add_attribute("peak", f"{best_peak:.0f}", min(1.0, best_peak / 255.0))
    return [detection]


def detect_laser_targets(frame_bgr) -> list[DetectedObject]:
    """Run both lasing detectors on one camera frame.

    Flips the frame once (180-deg upside-down mount correction; see the
    convention note above) and returns aruco-marker + laser-dot detections in
    flipped-frame coordinates.
    """
    flipped = cv2.flip(frame_bgr, -1)
    return _detect_aruco_markers(flipped) + _detect_laser_dot(flipped)


def classify_person_face_lighting(person_crop) -> tuple[str, float]:
    if person_crop.size == 0:
        return "unknown", 0.0

    crop_height, crop_width = person_crop.shape[:2]
    face_height = max(1, crop_height // 3)
    face_left = crop_width // 4
    face_right = max(face_left + 1, crop_width - face_left)
    face_crop = person_crop[:face_height, face_left:face_right]
    if face_crop.size == 0:
        return "unknown", 0.0

    brightness = float(face_crop.mean()) / 255.0
    if brightness < 0.25:
        return "dim", min(1.0, (0.25 - brightness) / 0.25)
    if brightness > 0.75:
        return "bright", min(1.0, (brightness - 0.75) / 0.25)
    return "normal", max(0.0, 1.0 - (abs(brightness - 0.5) / 0.25))


class VisionNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_node")

        self._share_data_dir = Path(get_package_share_directory("vision")) / "data"
        self._source_data_dir = Path("/ros2_ws/src/vision/data")
        model_default = default_model_path(self._source_data_dir, self._share_data_dir)

        self.declare_parameter("camera_device", "/dev/video10")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 15.0)
        self.declare_parameter("process_rate_hz", 4.0)
        self.declare_parameter("model_path", str(model_default))
        self.declare_parameter("model_imgsz", 416)
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.7)
        self.declare_parameter("max_detections", 20)
        self.declare_parameter("class_filter", DEFAULT_CLASS_FILTER)
        self.declare_parameter("ncnn_threads", 4)
        self.declare_parameter("reconnect_delay_sec", 1.0)
        self.declare_parameter("log_interval_sec", 5.0)
        self.declare_parameter("debug_save_enabled", False)
        self.declare_parameter("debug_output_dir", "/runtime_output/vision")
        self.declare_parameter("debug_save_rate_hz", 1.0)
        self.declare_parameter("debug_save_only_on_detection", True)
        self.declare_parameter("debug_save_latest", True)
        self.declare_parameter("debug_save_timestamped", False)
        self.declare_parameter("debug_max_timestamped_images", 100)
        # ArUco marker + laser-dot detections for the cp5 lasing phase (see the
        # detector section above). Published into the same /vision/detections
        # stream so the mission code never has to open the camera itself.
        self.declare_parameter("laser_targets_enabled", True)

        self._camera_device = str(self.get_parameter("camera_device").value)
        self._camera_width = int(self.get_parameter("camera_width").value)
        self._camera_height = int(self.get_parameter("camera_height").value)
        self._camera_fps = float(self.get_parameter("camera_fps").value)
        self._process_rate_hz = float(self.get_parameter("process_rate_hz").value)
        self._model_imgsz = int(self.get_parameter("model_imgsz").value)
        self._confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self._iou_threshold = float(self.get_parameter("iou_threshold").value)
        self._max_detections = int(self.get_parameter("max_detections").value)
        self._class_filter = str(self.get_parameter("class_filter").value)
        self._ncnn_threads = int(self.get_parameter("ncnn_threads").value)
        self._reconnect_delay_sec = max(0.1, float(self.get_parameter("reconnect_delay_sec").value))
        self._log_interval_sec = max(1.0, float(self.get_parameter("log_interval_sec").value))
        self._laser_targets_enabled = bool(self.get_parameter("laser_targets_enabled").value)

        self._publisher = self.create_publisher(VisionDetectionArray, "/vision/detections", 10)
        self._camera = ManagedCamera(
            device=self._camera_device,
            width=self._camera_width,
            height=self._camera_height,
            fps=self._camera_fps,
            reconnect_delay_sec=self._reconnect_delay_sec,
            log_interval_sec=self._log_interval_sec,
            logger=self.get_logger(),
        )
        self._debug_writer = DetectionDebugWriter(
            enabled=bool(self.get_parameter("debug_save_enabled").value),
            output_dir=str(self.get_parameter("debug_output_dir").value),
            save_rate_hz=float(self.get_parameter("debug_save_rate_hz").value),
            save_only_on_detection=bool(self.get_parameter("debug_save_only_on_detection").value),
            save_latest=bool(self.get_parameter("debug_save_latest").value),
            save_timestamped=bool(self.get_parameter("debug_save_timestamped").value),
            max_timestamped_images=int(self.get_parameter("debug_max_timestamped_images").value),
            logger=self.get_logger(),
        )
        self._last_loop_summary = 0.0

        model_path = resolve_model_path(
            raw_path=str(self.get_parameter("model_path").value),
            source_data_dir=self._source_data_dir,
            share_data_dir=self._share_data_dir,
        )
        self._detector = YoloNcnnDetector(
            model_path=model_path,
            input_size=self._model_imgsz,
            confidence_threshold=self._confidence_threshold,
            iou_threshold=self._iou_threshold,
            max_detections=self._max_detections,
            class_filter=self._class_filter,
            ncnn_threads=self._ncnn_threads,
        )
        self.get_logger().info(
            "Loaded NCNN YOLO model path=%s max_imgsz=%d classes=%d filter=%s ncnn_threads=%s confidence=%.2f yellow_block=%s"
            % (
                model_path,
                self._model_imgsz,
                self._detector.class_count,
                self._class_filter or "all",
                self._ncnn_threads if self._ncnn_threads > 0 else "auto",
                self._confidence_threshold,
                "on",
            )
        )

    def _infer_yolo_detections(self, frame) -> list[DetectedObject]:
        return self._detector.predict(frame)

    def _detect_yellow_block(self, frame):
        return detect_yellow_block(frame)

    def _build_detection_msg(self, detected_object: DetectedObject) -> VisionDetection:
        detection = VisionDetection()
        detection.class_name = detected_object.class_name
        detection.confidence = float(detected_object.confidence)
        detection.x = int(detected_object.x)
        detection.y = int(detected_object.y)
        detection.width = int(detected_object.width)
        detection.height = int(detected_object.height)
        for attribute in detected_object.attributes:
            detection.attribute_names.append(attribute.name)
            detection.attribute_values.append(attribute.value)
            detection.attribute_scores.append(float(attribute.score))
        return detection

    def _build_detection_array_msg(
        self,
        capture_stamp,
        image_width: int,
        image_height: int,
        detected_objects: list[DetectedObject],
    ) -> VisionDetectionArray:
        message = VisionDetectionArray()
        message.header.stamp = capture_stamp
        message.header.frame_id = "vision_camera"
        message.image_width = int(image_width)
        message.image_height = int(image_height)
        for detected_object in detected_objects:
            message.detections.append(self._build_detection_msg(detected_object))
        return message

    def run(self) -> None:
        scheduler = FixedRateScheduler(self._process_rate_hz)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            if not self._camera.ensure():
                scheduler.reset()
                continue

            scheduler.wait_until_ready()

            ok, frame = self._camera.read() # frame is a numpy array
            if not ok or frame is None:
                self._camera.handle_read_failure()
                scheduler.reset()
                continue

            capture_stamp = self.get_clock().now().to_msg()
            inference_start = time.monotonic()
            try:
                yolo_detections = self._infer_yolo_detections(frame)
                yellow_block_detections, yellow_block_overlays = self._detect_yellow_block(frame)
                
                for detection in yolo_detections:
                    object_crop = frame[
                        detection.y : detection.y + detection.height,
                        detection.x : detection.x + detection.width,
                    ]

                    if detection.class_name == "traffic light":
                        traffic_light_crop = object_crop
                        color_label, color_score = classify_traffic_light_color(traffic_light_crop)
                        
                        # Add attribute to the detection result; we add color here as an example
                        detection.add_attribute("color", color_label, color_score)

                    elif detection.class_name == "stop sign":
                        stop_sign_crop = object_crop
                        visibility_label, visibility_score = classify_stop_sign_visibility(stop_sign_crop)
                        detection.add_attribute("visibility", visibility_label, visibility_score)
                        
                    elif detection.class_name == "person":
                        person_crop = object_crop
                        face_lighting_label, face_lighting_score = classify_person_face_lighting(person_crop)
                        detection.add_attribute("face_lighting", face_lighting_label, face_lighting_score)
                
                laser_target_detections = (
                    detect_laser_targets(frame) if self._laser_targets_enabled else []
                )

                all_detections = (
                    yolo_detections + yellow_block_detections + laser_target_detections
                )

                message = self._build_detection_array_msg(
                    capture_stamp=capture_stamp,
                    image_width=frame.shape[1],
                    image_height=frame.shape[0],
                    detected_objects=all_detections,
                )
                self._publisher.publish(message)
                self._debug_writer.maybe_write(
                    frame_bgr=frame,
                    detected_objects=all_detections,
                    debug_overlays=yellow_block_overlays,
                )
                yolo_count = len(yolo_detections)
                yellow_block_count = len(yellow_block_detections)
                detection_count = len(message.detections)
            except Exception as exc:
                self.get_logger().error(f"Vision inference failed for one frame: {exc}")
                yolo_count = 0
                yellow_block_count = 0
                detection_count = 0
            inference_ms = (time.monotonic() - inference_start) * 1000.0

            now = time.monotonic()
            if now - self._last_loop_summary >= self._log_interval_sec:
                self._last_loop_summary = now
                self.get_logger().info(
                    "Vision frame %dx%d total=%.1fms preprocess=%.1fms ncnn=%.1fms postprocess=%.1fms yolo=%d yellow_block=%d total=%d target_rate=%.1fHz"
                    % (
                        frame.shape[1],
                        frame.shape[0],
                        inference_ms,
                        self._detector.last_preprocess_ms,
                        self._detector.last_inference_ms,
                        self._detector.last_postprocess_ms,
                        yolo_count,
                        yellow_block_count,
                        detection_count,
                        self._process_rate_hz,
                    )
                )

            scheduler.schedule_next()

        self._camera.release()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
