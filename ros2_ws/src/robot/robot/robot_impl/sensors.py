"""
robot_impl/sensors.py — SensorsMixin

Opt-in sensor subscriptions (lidar, GPS/ArUco, IMU, vision) plus all
related callbacks and public API.  Call enable_lidar(), enable_gps(),
enable_imu(), or enable_vision() before using the corresponding API.
All state lives in Robot.__init__; this mixin only defines methods.
"""
from __future__ import annotations

import math
import time as _time

import numpy as np

from bridge_interfaces.msg import (
    LidarWorldPoints,
    SensorImu,
    TagDetectionArray,
    TrackedObstacle,
    TrackedObstacleArray,
)
try:
    from bridge_interfaces.msg import VisionDetectionArray
except (ImportError, ModuleNotFoundError):
    class VisionDetectionArray:  # type: ignore[no-redef]
        detections = ()
        image_width = 0
        image_height = 0

try:
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
except (ImportError, ModuleNotFoundError):
    class ReliabilityPolicy:  # type: ignore[no-redef]
        BEST_EFFORT = "best_effort"

    class HistoryPolicy:  # type: ignore[no-redef]
        KEEP_LAST = "keep_last"

    class QoSProfile:  # type: ignore[no-redef]
        def __init__(self, reliability=None, history=None, depth=1) -> None:
            self.reliability = reliability
            self.history = history
            self.depth = depth

try:
    from sensor_msgs.msg import LaserScan
except (ImportError, ModuleNotFoundError):
    class LaserScan:  # type: ignore[no-redef]
        ranges = ()
        angle_min = 0.0
        angle_max = 0.0
        angle_increment = 0.0
        range_min = 0.0
        range_max = 0.0

from robot.sensor_fusion import GpsTangentOrientationFusion, OrientationComplementaryFilter, SensorFusion


class SensorsMixin:
    """Opt-in sensor subscriptions and related API."""

    # =========================================================================
    # Opt-in subscriptions
    # =========================================================================

    def enable_lidar(self) -> None:
        """Subscribe to /scan and start processing lidar data."""
        self._node.create_subscription(
            LaserScan,
            '/scan',
            self._on_lidar,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

    def enable_gps(self) -> None:
        """Subscribe to /tag_detections and start GPS position fusion."""
        if not self._gps_subscribed:
            self._node.create_subscription(
                TagDetectionArray,
                '/tag_detections',
                self._on_tag_detections,
                10,
            )
            self._gps_subscribed = True
        self._gps_paused = False

    def disable_gps(self) -> None:
        """Pause GPS updates. Call enable_gps() to resume."""
        self._gps_paused = True

    def enable_imu(self) -> None:
        """Subscribe to /sensor_imu and start orientation fusion."""
        self._node.create_subscription(
            SensorImu,
            '/sensor_imu',
            self._on_imu,
            10,
        )

    def enable_gps_tangent_heading(
        self,
        alpha: float = 0.15,
        min_displacement_mm: float = 200.0,
    ) -> None:
        """Use GPS trajectory tangent for heading correction instead of IMU.

        Installs a GpsTangentOrientationFusion strategy that derives a heading
        reference from the direction of movement in the GPS-fused position.
        Requires GPS to be enabled (enable_gps()) and takes effect once the
        robot has moved at least min_displacement_mm with GPS fresh.

        Mutually exclusive with enable_imu() — call one or the other, not both.

        Parameters
        ----------
        alpha : float
            Correction weight (0 = pure odometry, 1 = snap to GPS tangent).
        min_displacement_mm : float
            Minimum travel between tangent updates. Larger values are more
            robust to GPS noise but correct drift less frequently.
        """
        self.set_orientation_fusion_strategy(
            GpsTangentOrientationFusion(
                alpha=alpha,
                min_displacement_mm=min_displacement_mm,
            )
        )

    def enable_vision(self) -> None:
        """Subscribe to /vision/detections and start caching vision results."""
        self._node.create_subscription(
            VisionDetectionArray,
            '/vision/detections',
            self._on_vision_detections,
            10,
        )

    # =========================================================================
    # Subscription callbacks
    # =========================================================================

    def _on_imu(self, msg: SensorImu) -> None:
        with self._lock:
            self._imu = msg
            if msg.mag_calibrated:
                # Use Madgwick AHRS quaternion (9-axis: accel + gyro + mag).
                # Convention: NWU, yaw via ZYX Euler extraction.
                qw = float(msg.quat_w)
                qx = float(msg.quat_x)
                qy = float(msg.quat_y)
                qz = float(msg.quat_z)
                yaw = math.atan2(
                    2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz),
                )
                # When mounted upside-down (Z toward ground) the AHRS converges
                # to an inverted-attitude quaternion, negating the extracted yaw.
                self._ahrs_heading = -yaw if self._imu_z_down else yaw
            else:
                # Calibration lost — clear heading so _on_kinematics falls back
                # to pure odometry rather than fusing against a stale reference.
                self._ahrs_heading = None

    def _on_tag_detections(self, msg: TagDetectionArray) -> None:
        """Cache world-frame robot position from the tracked ArUco tag (metres → mm)."""
        if self._gps_paused:
            return
        for det in msg.detections:
            if self._tracked_tag_id == -1 or det.tag_id == self._tracked_tag_id:
                with self._lock:
                    tag_x = float(det.x) * 1000.0 + self._gps_offset_x_mm
                    tag_y = float(det.y) * 1000.0 + self._gps_offset_y_mm
                    theta = self._fused_theta
                    cos_t, sin_t = math.cos(theta), math.sin(theta)
                    ox = self._tag_body_offset_x_mm
                    oy = self._tag_body_offset_y_mm
                    self._gps_x_mm      = tag_x - (ox * cos_t - oy * sin_t)
                    self._gps_y_mm      = tag_y - (ox * sin_t + oy * cos_t)
                    self._gps_last_time = _time.monotonic()
                break

    def _on_lidar(self, msg: LaserScan) -> None:
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges)

        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        angles = angles[valid]
        ranges = ranges[valid]

        with self._lock:
            ct = math.cos(self._lidar_mount_theta_rad)
            st = math.sin(self._lidar_mount_theta_rad)
            mx = self._lidar_mount_x_mm
            my = self._lidar_mount_y_mm
            r_min = self._lidar_range_min_mm
            r_max = self._lidar_range_max_mm
            fov_min = self._lidar_fov_min_rad
            fov_max = self._lidar_fov_max_rad

        # Step 1 — polar → Cartesian in lidar sensor frame (metres → mm)
        lx = (ranges * np.cos(angles)) * 1000.0
        ly = (ranges * np.sin(angles)) * 1000.0

        # Step 2 — rotate by mount heading (axes align with robot body frame;
        # origin still at lidar centre, no translation yet)
        rx = ct * lx - st * ly
        ry = st * lx + ct * ly

        # Step 3 — FOV filter: angles in robot-aligned frame, centred on lidar origin
        if not (fov_min <= -math.pi and fov_max >= math.pi):
            angle_lidar = np.arctan2(ry, rx)
            keep = (angle_lidar >= fov_min) & (angle_lidar <= fov_max)
            rx = rx[keep]
            ry = ry[keep]

        # Step 4 — range filter: distance from lidar origin (before translation)
        dist2 = rx * rx + ry * ry
        keep = (dist2 >= r_min * r_min) & (dist2 <= r_max * r_max)
        rx = rx[keep]
        ry = ry[keep]

        # Step 5 — translate to robot body origin
        rx = rx + mx
        ry = ry + my

        robot_obstacles = np.float64(
            np.concatenate([rx[:, np.newaxis], ry[:, np.newaxis]], axis=1)
        )
        with self._lock:
            self._obstacles_mm = robot_obstacles

        self._update_obstacle_tracks(robot_obstacles)

    def _obstacles_robot_to_world_mm(
        self,
        obstacles_mm,
        pose_mm: tuple[float, float, float],
    ) -> np.ndarray:
        """Project robot-frame obstacle points into world coordinates."""
        obstacles = np.asarray(obstacles_mm, dtype=float)
        if obstacles.ndim != 2 or obstacles.shape[0] == 0:
            return np.empty((0, 2), dtype=float)

        x_mm, y_mm, theta_rad = pose_mm
        cos_t = math.cos(theta_rad)
        sin_t = math.sin(theta_rad)
        wx = x_mm + obstacles[:, 0] * cos_t - obstacles[:, 1] * sin_t
        wy = y_mm + obstacles[:, 0] * sin_t + obstacles[:, 1] * cos_t
        return np.column_stack((wx, wy))

    def _update_obstacle_tracks(self, robot_obstacles_mm: np.ndarray) -> None:
        """Update tracked obstacle disks from the latest robot-frame lidar cloud."""
        robot_points = np.asarray(robot_obstacles_mm, dtype=float)
        if robot_points.ndim != 2 or robot_points.shape[0] == 0:
            world_points = np.empty((0, 2), dtype=float)
        else:
            distances = np.linalg.norm(robot_points, axis=1)
            in_range = distances <= float(self.OBSTACLE_TRACK_INPUT_RANGE_MM)
            filtered = robot_points[in_range]

            if filtered.shape[0] > int(self.OBSTACLE_TRACK_MAX_INPUT_POINTS):
                nearest = np.argsort(np.linalg.norm(filtered, axis=1))[: int(self.OBSTACLE_TRACK_MAX_INPUT_POINTS)]
                filtered = filtered[nearest]

            world_points = self._obstacles_robot_to_world_mm(filtered, self._get_pose_mm())

        tracks = self._obstacle_tracker.update(world_points, _time.monotonic())
        with self._lock:
            self._obstacle_tracks = list(tracks)
        self._publish_obstacle_tracks()

    def _publish_obstacle_tracks(self) -> None:
        tracks = self.get_obstacle_tracks(include_unconfirmed=False)
        msg = TrackedObstacleArray()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        for track in tracks:
            item = TrackedObstacle()
            item.id = int(track["id"])
            item.x = float(track["x"])
            item.y = float(track["y"])
            item.radius = float(track["radius"])
            msg.obstacles.append(item)
        self._pub_obstacle_tracks.publish(msg)

    def _on_vision_detections(self, msg: VisionDetectionArray) -> None:
        detections: list[dict[str, object]] = []
        for det in msg.detections:
            attribute_scores = list(det.attribute_scores)
            attributes: dict[str, object] = {}
            for index, name in enumerate(det.attribute_names):
                value = det.attribute_values[index] if index < len(det.attribute_values) else ""
                score = attribute_scores[index] if index < len(attribute_scores) else None
                attributes[name] = {"value": value, "score": score}
            detections.append({
                "class_name": det.class_name,
                "confidence": float(det.confidence),
                "bbox": {
                    "x": int(det.x),
                    "y": int(det.y),
                    "width": int(det.width),
                    "height": int(det.height),
                },
                "attributes": attributes,
            })

        with self._lock:
            self._vision_detections = detections
            self._vision_image_size = (int(msg.image_width), int(msg.image_height))
            self._vision_last_time = _time.monotonic()

    # =========================================================================
    # IMU
    # =========================================================================

    def get_imu(self) -> SensorImu:
        """Return cached IMU state (quaternion, accel, gyro, magnetometer)."""
        with self._lock:
            return self._imu

    def get_fused_orientation(self) -> float:
        """
        Return complementary-filter orientation estimate in degrees.

        Blends the absolute AHRS heading with wheel odometry. The filter weight
        controls how quickly the AHRS heading pulls the estimate; it only
        activates once the firmware reports mag_calibrated = True.
        Before calibration, returns odometry theta.
        """
        with self._lock:
            return math.degrees(self._fused_theta)

    def set_orientation_fusion_strategy(self, strategy: SensorFusion) -> None:
        """
        Replace the active heading-fusion strategy.
        Any SensorFusion subclass is accepted.
        The new strategy takes effect on the next kinematics update.
        """
        if strategy.measurement_type != "orientation":
            raise ValueError(
                f"Orientation fusion strategy must have measurement_type='orientation', "
                f"but got '{strategy.measurement_type}'"
            )
        with self._lock:
            self._orientation_fusion = strategy
            self._fusion = strategy

    def set_orientation_fusion_alpha(self, alpha: float) -> None:
        """
        Set the AHRS heading weight for the complementary filter (0.0–1.0).

        Only valid when the active strategy is OrientationComplementaryFilter.
        Raises TypeError if a different strategy is active.
        Lower values trust odometry more; higher values correct drift faster.
        """
        with self._lock:
            if not isinstance(self._orientation_fusion, OrientationComplementaryFilter):
                raise TypeError(
                    f"set_orientation_fusion_alpha() requires OrientationComplementaryFilter, "
                    f"but active strategy is {type(self._orientation_fusion).__name__}. "
                    "Use set_orientation_fusion_strategy(OrientationComplementaryFilter(alpha)) instead."
                )
            self._orientation_fusion.alpha = max(0.0, min(1.0, float(alpha)))

    def set_fusion_strategy(self, strategy: SensorFusion) -> None:
        """Backward-compatible alias for set_orientation_fusion_strategy()."""
        self.set_orientation_fusion_strategy(strategy)

    def set_fusion_alpha(self, alpha: float) -> None:
        """Backward-compatible alias for set_orientation_fusion_alpha()."""
        self.set_orientation_fusion_alpha(alpha)

    def set_imu_z_down(self, z_down: bool) -> None:
        """
        Configure the IMU mounting orientation.
        Set True when the sensor is mounted upside-down (Z-axis toward the ground).
        Default is False (Z-axis points up).
        """
        with self._lock:
            self._imu_z_down = bool(z_down)

    # =========================================================================
    # GPS / ArUco
    # =========================================================================

    def is_gps_active(self) -> bool:
        """Return True if a GPS fix for the tracked tag arrived within the staleness window."""
        with self._lock:
            return (
                self._gps_last_time > 0.0
                and (_time.monotonic() - self._gps_last_time) < self._gps_timeout_s
            )

    def set_tracked_tag_id(self, tag_id: int) -> None:
        """
        Set the ArUco tag ID used for GPS position fusion.
        Pass -1 (default) to accept any tag from /tag_detections.
        """
        with self._lock:
            self._tracked_tag_id = int(tag_id)

    def get_tracked_tag_id(self) -> int:
        """Return the currently tracked ArUco tag ID (-1 means any)."""
        with self._lock:
            return self._tracked_tag_id

    def set_gps_offset(self, offset_x_mm: float, offset_y_mm: float) -> None:
        """
        Set a fixed translation from the incoming GPS/tag frame to the world frame (mm).

        The current global_gps stack already publishes world-frame coordinates,
        so new code should normally leave this at `(0, 0)`. This hook remains
        for backward compatibility with legacy demos and alternate GPS sources.
        """
        with self._lock:
            self._gps_offset_x_mm = float(offset_x_mm)
            self._gps_offset_y_mm = float(offset_y_mm)

    def set_tag_body_offset(self, x_mm: float, y_mm: float) -> None:
        """
        Set the ArUco tag mounting position in the robot body frame (mm).

        Origin is the midpoint of the two drive wheels. +x = robot forward, +y = robot left.
        The correction is applied each time a tag detection arrives.
        Default is (0, 0) — tag assumed to be at the body origin.
        """
        with self._lock:
            self._tag_body_offset_x_mm = float(x_mm)
            self._tag_body_offset_y_mm = float(y_mm)

    def set_position_fusion_strategy(self, strategy: SensorFusion) -> None:
        """Replace the active position-fusion strategy."""
        if strategy.measurement_type != "position":
            raise ValueError(
                f"Position fusion strategy must have measurement_type='position', "
                f"but got '{strategy.measurement_type}'"
            )
        with self._lock:
            self._pos_fusion = strategy

    def set_position_fusion_alpha(self, alpha: float) -> None:
        """
        Set the GPS weight for position fusion (0.0–1.0).
        0.0 = trust odometry only; 1.0 = snap to GPS each tick.
        Default is 0.10.
        """
        with self._lock:
            self._pos_fusion.alpha = max(0.0, min(1.0, float(alpha)))

    def set_pos_fusion_alpha(self, alpha: float) -> None:
        """Backward-compatible alias for set_position_fusion_alpha()."""
        self.set_position_fusion_alpha(alpha)

    # =========================================================================
    # Lidar
    # =========================================================================

    def set_lidar_mount(self, x_mm: float, y_mm: float, theta_deg: float = 0.0) -> None:
        """
        Set the lidar mounting position and heading in the robot body frame.

        Origin is the midpoint of the two drive wheels. +x = robot forward, +y = robot left.

        - ``x_mm``      — lidar x position in body frame (positive = forward of wheel midpoint).
        - ``y_mm``      — lidar y position in body frame (positive = left of wheel midpoint).
        - ``theta_deg`` — lidar heading offset relative to robot forward (CCW positive).
                          Use 180.0 if the lidar is mounted facing backward.

        Default is (0, 0, 0) — lidar at the body origin, facing forward.
        """
        with self._lock:
            self._lidar_mount_x_mm      = float(x_mm)
            self._lidar_mount_y_mm      = float(y_mm)
            self._lidar_mount_theta_rad = math.radians(float(theta_deg))

    def set_lidar_filter(
        self,
        range_min_mm: float | None = None,
        range_max_mm: float | None = None,
        fov_deg: tuple[float, float] | None = None,
    ) -> None:
        """
        Tune lidar point filtering applied after the mount transform.

        - ``range_min_mm`` — discard points closer than this to the lidar origin (default 150 mm).
        - ``range_max_mm`` — discard points farther than this (default 6000 mm).
        - ``fov_deg``      — ``(min_deg, max_deg)`` angular window in the robot-aligned frame,
                             centred on the lidar origin. 0° = robot +x (forward), CCW positive.
                             Example: ``(-90, 90)`` keeps only the front hemisphere.
                             Default ``(-180, 180)`` keeps everything.

        Pass only the parameters you want to change; omit the rest to keep current values.
        """
        with self._lock:
            if range_min_mm is not None:
                self._lidar_range_min_mm = float(range_min_mm)
            if range_max_mm is not None:
                self._lidar_range_max_mm = float(range_max_mm)
            if fov_deg is not None:
                self._lidar_fov_min_rad = math.radians(float(fov_deg[0]))
                self._lidar_fov_max_rad = math.radians(float(fov_deg[1]))

    def start_lidar_world_publisher(
        self,
        topic: str = '/lidar_world_points',
        hz: float = 10.0,
    ) -> None:
        """
        Start publishing lidar obstacle points in the world frame on ``topic`` at ``hz`` Hz.

        The publisher runs as a ROS timer on the spin thread, so it fires continuously
        and independently of whatever the user's FSM loop is doing — including during
        blocking navigation calls.

        The transform each tick: robot-body-frame obstacles → rotate by fused heading →
        translate by fused position.

        Call stop_lidar_world_publisher() to cancel. Calling this again with different
        arguments replaces the existing publisher/timer.
        """
        self.stop_lidar_world_publisher()
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._lidar_world_pub   = self._node.create_publisher(LidarWorldPoints, topic, best_effort)
        self._lidar_world_timer = self._node.create_timer(1.0 / hz, self._publish_lidar_world)

    def stop_lidar_world_publisher(self) -> None:
        """Cancel the lidar world-frame publisher started by start_lidar_world_publisher."""
        if self._lidar_world_timer is not None:
            self._lidar_world_timer.cancel()
            self._lidar_world_timer.destroy()
            self._lidar_world_timer = None
        if self._lidar_world_pub is not None:
            self._lidar_world_pub.destroy()
            self._lidar_world_pub = None

    def _publish_lidar_world(self) -> None:
        """Timer callback — transforms body-frame obstacles to world frame and publishes."""
        with self._lock:
            obstacles = self._obstacles_mm.copy() if self._obstacles_mm is not None else None
            fx     = self._fused_x_mm
            fy     = self._fused_y_mm
            ftheta = self._fused_theta

        pub = self._lidar_world_pub
        if pub is None or obstacles is None or len(obstacles) == 0:
            return

        world_obstacles = self._obstacles_robot_to_world_mm(obstacles, (fx, fy, ftheta))

        msg = LidarWorldPoints()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.xs           = world_obstacles[:, 0].tolist()
        msg.ys           = world_obstacles[:, 1].tolist()
        msg.robot_x      = float(fx)
        msg.robot_y      = float(fy)
        msg.robot_theta  = float(ftheta)
        pub.publish(msg)

    def get_obstacle_tracks(self, include_unconfirmed: bool = False) -> list[dict[str, float | int]]:
        """Return tracked obstacle disks in world frame."""
        tracks = self._obstacle_tracker.get_tracks(
            _time.monotonic(),
            include_unconfirmed=include_unconfirmed,
        )
        return [
            {
                "id": int(track.track_id),
                "x": float(track.x_mm),
                "y": float(track.y_mm),
                "radius": float(track.radius_mm),
                "hits": int(track.hit_count),
            }
            for track in tracks
        ]

    # =========================================================================
    # Vision
    # =========================================================================

    def get_detections(self, class_name: str | None = None) -> list[dict[str, object]]:
        """Return the latest cached vision detections.

        Each detection is a dict with keys: class_name, confidence, bbox, attributes.
        """
        with self._lock:
            detections = [dict(det) for det in self._vision_detections]
        if class_name is None:
            return detections
        return [det for det in detections if det.get("class_name") == class_name]

    def has_detection(self, class_name: str, min_confidence: float = 0.0) -> bool:
        """Return True if the latest vision snapshot contains the given class."""
        with self._lock:
            return any(
                det["class_name"] == class_name and float(det["confidence"]) >= float(min_confidence)
                for det in self._vision_detections
            )

    def get_detection_attribute(
        self,
        class_name: str,
        attribute_name: str,
        default: str | None = None,
        min_confidence: float = 0.0,
    ) -> str | None:
        """Return the highest-confidence attribute value for a matching detection."""
        best_confidence = -1.0
        best_value = default
        with self._lock:
            for det in self._vision_detections:
                confidence = float(det["confidence"])
                if det["class_name"] != class_name or confidence < float(min_confidence):
                    continue
                attributes = det.get("attributes", {})
                attribute = attributes.get(attribute_name)
                if attribute is None:
                    continue
                if confidence > best_confidence:
                    best_confidence = confidence
                    best_value = str(attribute.get("value", default))
        return best_value

    def get_detection_image_size(self) -> tuple[int, int]:
        """Return the latest vision frame size as (width, height)."""
        with self._lock:
            return self._vision_image_size

    def is_vision_active(self, timeout_s: float = 1.0) -> bool:
        """Return True if a vision detection message arrived recently."""
        with self._lock:
            last_time = self._vision_last_time
        return last_time > 0.0 and (_time.monotonic() - last_time) < float(timeout_s)
