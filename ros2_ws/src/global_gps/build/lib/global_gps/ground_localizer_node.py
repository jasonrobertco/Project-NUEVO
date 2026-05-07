"""
Ground localizer node.

Subscribes to a RealSense colour + depth stream, detects ArUco markers, and
publishes 2-D world-frame poses for all detected rover tags.

Coordinate conventions
----------------------
* Camera frame: standard OpenCV (X right, Y down, Z forward).
* World frame:  right-handed, Z = ground-plane normal pointing up,
                X/Y defined by the corner anchor layout.

Topic published
---------------
/global_gps/tag_detections  (bridge_interfaces/msg/TagDetectionArray)
    One entry per detected rover tag.  Corner anchors are excluded.
    The header stamp matches the colour image that triggered the detection.

Parameters
----------
marker_size : float  (default 0.10)
    Physical side length of the ArUco markers in metres.
corner_ids : int[]  (default [0, 1, 2, 3])
    Marker IDs used as fixed field-corner anchors.  All four must be
    visible simultaneously for the initial calibration to succeed.
rover_ids : int[]  (default [11-18])
    Marker IDs that can appear on rovers.  Only these are published.
"""

from __future__ import annotations

import json
import os
import socket
import threading
import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

from bridge_interfaces.msg import TagDetection, TagDetectionArray

from .geometry_utils import project_point_to_plane, rigid_transform_svd


# QoS used for all publishers and the camera-info subscriber.
_RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
)

# Physical layout of a square marker's corners in marker-local frame
# (counter-clockwise from top-left, z=0 in marker plane).
# Overwritten at runtime with the configured marker_size.
_MARKER_OBJ_PTS_UNIT = np.array(
    [[-0.5,  0.5, 0.0],
     [ 0.5,  0.5, 0.0],
     [ 0.5, -0.5, 0.0],
     [-0.5, -0.5, 0.0]],
    dtype=np.float32,
)


class GroundLocalizer(Node):
    """Detect ArUco markers and publish 2-D world-frame rover poses."""

    def __init__(self) -> None:
        super().__init__("ground_localizer")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("marker_size", 0.10)
        self.declare_parameter("corner_ids", [0, 1, 2, 3])
        self.declare_parameter("rover_ids", [11, 12, 13, 14, 15, 16, 17, 18])
        self.declare_parameter("tcp_port", 7777)
        self.declare_parameter("calibration_layout_file", "")

        self._marker_size: float = float(self.get_parameter("marker_size").value)
        self._corner_ids: list[int] = list(self.get_parameter("corner_ids").value)
        self._rover_ids: list[int] = list(self.get_parameter("rover_ids").value)
        self._tcp_port: int = int(self.get_parameter("tcp_port").value)
        self._calibration_layout_file: str = str(
            self.get_parameter("calibration_layout_file").value
        )

        # Load and validate the world-frame calibration-tag layout.
        # Row order matches self._corner_ids so P_world[i] corresponds to
        # the tvec at P_cam[i] during the SVD calibration step.
        self._P_world: np.ndarray = self._load_calibration_layout()

        self._obj_pts = _MARKER_OBJ_PTS_UNIT * self._marker_size

        # ── OpenCV ArUco detector (supports both 4.6 and ≥ 4.7 APIs) ────────
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, "ArucoDetector"):
            # OpenCV ≥ 4.7
            aruco_params = cv2.aruco.DetectorParameters()
            self._detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            self._use_new_aruco_api = True
        else:
            # OpenCV 4.6 (shipped with Ubuntu 24.04 apt python3-opencv)
            self._aruco_dict = aruco_dict
            self._aruco_params = cv2.aruco.DetectorParameters_create()
            self._use_new_aruco_api = False

        # ── State ──────────────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._K: np.ndarray | None = None
        self._D: np.ndarray | None = None
        self._ground_plane: dict | None = None    # {"normal": ..., "d": ...}
        self._T_world_from_cam: np.ndarray | None = None
        self._calibrated: bool = False

        # ── TCP push server (NAT-friendly: robots connect to us) ──────────
        # Robots on WiFi cannot be reached from the wired Jetson because the
        # WiFi AP performs NAT.  Instead, each robot's robot_gps node opens a
        # TCP connection to this server.  We push line-delimited JSON
        # detections over the established connection.
        self._tcp_clients: list[socket.socket] = []
        self._tcp_lock = threading.Lock()
        self._tcp_server_thread = threading.Thread(
            target=self._tcp_server_loop, daemon=True
        )
        self._tcp_server_thread.start()
        self.get_logger().info(
            f"TCP push server listening on port {self._tcp_port}"
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._detections_pub = self.create_publisher(
            TagDetectionArray,
            "/global_gps/tag_detections",
            _RELIABLE_QOS,
        )

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self._on_camera_info,
            10,
        )

        rgb_sub = Subscriber(self, Image, "/camera/camera/color/image_raw")
        depth_sub = Subscriber(self, Image, "/camera/camera/depth/image_rect_raw")
        self._sync = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub],
            queue_size=5,
            slop=0.05,
        )
        self._sync.registerCallback(self._on_images)

        self.get_logger().info(
            f"Ground localizer started | "
            f"marker_size={self._marker_size:.3f} m | "
            f"corner_ids={self._corner_ids} | "
            f"rover_ids={self._rover_ids}"
        )

    # ── Camera info ───────────────────────────────────────────────────────

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._K is not None:
            return
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._D = np.array(msg.d, dtype=np.float64)
        self.get_logger().info("Camera intrinsics received.")

    # ── Main image callback ───────────────────────────────────────────────

    def _on_images(self, rgb_msg: Image, depth_msg: Image) -> None:  # noqa: ARG002
        if self._K is None:
            return

        rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

        if self._use_new_aruco_api:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._aruco_dict, parameters=self._aruco_params
            )

        if ids is None:
            return

        ids = ids.flatten().tolist()
        marker_poses = self._estimate_poses(corners, ids)

        if not self._calibrated:
            if all(mid in marker_poses for mid in self._corner_ids):
                self._calibrate(marker_poses)
            else:
                missing = [m for m in self._corner_ids if m not in marker_poses]
                self.get_logger().warn(
                    f"Waiting for calibration — missing corner IDs: {missing}",
                    throttle_duration_sec=5.0,
                )
            return

        self._publish_detections(rgb_msg, marker_poses)

    # ── Pose estimation ───────────────────────────────────────────────────

    def _estimate_poses(
        self,
        corners: list,
        ids: list[int],
    ) -> dict[int, dict]:
        """Run solvePnP for every detected marker and return a pose dict."""
        poses: dict[int, dict] = {}
        for corner, mid in zip(corners, ids):
            success, rvec, tvec = cv2.solvePnP(
                self._obj_pts,
                corner[0],
                self._K,
                self._D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not success:
                continue
            poses[mid] = {"rvec": rvec, "tvec": tvec.reshape(3)}
        return poses

    # ── Calibration ───────────────────────────────────────────────────────

    def _load_calibration_layout(self) -> np.ndarray:
        """Resolve, parse and validate the calibration-tag world layout.

        Returns a ``(4, 3)`` float ndarray ordered to match
        ``self._corner_ids`` (row ``i`` is the world-frame ``(x, y, z)``
        of corner id ``self._corner_ids[i]``).
        """
        # Resolution order: explicit param wins; otherwise packaged default.
        path = self._calibration_layout_file
        if not path:
            try:
                share = get_package_share_directory("global_gps")
            except Exception as exc:
                self.get_logger().error(
                    f"Cannot locate global_gps share directory: {exc}"
                )
                rclpy.shutdown()
                raise
            path = os.path.join(share, "config", "calibration_layout.yaml")

        if not os.path.isfile(path):
            self.get_logger().error(
                f"Calibration layout file not found: {path}"
            )
            rclpy.shutdown()
            raise FileNotFoundError(path)

        try:
            with open(path, "r") as f:
                doc = yaml.safe_load(f)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to parse calibration layout {path}: {exc}"
            )
            rclpy.shutdown()
            raise

        raw = (doc or {}).get("calibration_tags")
        if not isinstance(raw, dict):
            self.get_logger().error(
                f"Calibration layout {path} missing 'calibration_tags' map."
            )
            rclpy.shutdown()
            raise ValueError("calibration_tags missing")

        # PyYAML loads `0:` as int but quoted `"0":` becomes str — coerce.
        coerced: dict[int, list[float]] = {}
        for k, v in raw.items():
            try:
                coerced[int(k)] = [float(x) for x in v]
            except (TypeError, ValueError) as exc:
                self.get_logger().error(
                    f"Bad calibration entry {k!r} -> {v!r}: {exc}"
                )
                rclpy.shutdown()
                raise

        # Validate that every required corner id is present.
        missing = [cid for cid in self._corner_ids if cid not in coerced]
        if missing:
            self.get_logger().error(
                f"Calibration layout {path} is missing corner_ids: {missing}"
            )
            rclpy.shutdown()
            raise KeyError(f"missing corner ids: {missing}")

        # Build (N, 3) array in self._corner_ids order — must match P_cam row order.
        P_world = np.array(
            [coerced[cid] for cid in self._corner_ids],
            dtype=np.float64,
        )
        if P_world.shape[1] != 3:
            self.get_logger().error(
                f"Calibration layout entries must be 3-vectors, got shape {P_world.shape}"
            )
            rclpy.shutdown()
            raise ValueError("layout entries must be length-3")

        # Reject near-collinear layouts: smaller singular value of centered
        # XY layout must be at least ~1% of the larger one.
        xy = P_world[:, :2] - P_world[:, :2].mean(axis=0)
        sv = np.linalg.svd(xy, compute_uv=False)
        if sv[0] <= 0.0 or sv[-1] / sv[0] < 0.01:
            self.get_logger().error(
                "Calibration layout is nearly collinear "
                f"(singular values {sv.tolist()}); fix calibration_layout.yaml."
            )
            rclpy.shutdown()
            raise ValueError("collinear calibration layout")

        self.get_logger().info(
            f"Loaded calibration layout from {path} ({len(self._corner_ids)} tags)."
        )
        return P_world

    def _calibrate(self, marker_poses: dict[int, dict]) -> None:
        """SVD-align measured tag tvecs to the known world layout."""
        self.get_logger().info("Calibrating world frame via SVD…")

        # Row order must match self._P_world (which was built in
        # self._corner_ids order).
        P_cam = np.array(
            [marker_poses[mid]["tvec"] for mid in self._corner_ids],
            dtype=np.float64,
        )
        P_world = self._P_world

        T_cam_from_world = rigid_transform_svd(P_world, P_cam)
        self._T_world_from_cam = np.linalg.inv(T_cam_from_world)

        # Ground plane in camera frame is the image of the world z = 0 plane.
        R_cw = T_cam_from_world[:3, :3]
        t_cw = T_cam_from_world[:3, 3]
        normal = R_cw @ np.array([0.0, 0.0, 1.0])
        d = -float(normal @ t_cw)
        self._ground_plane = {"normal": normal, "d": d}

        # Per-tag residuals in millimetres for operator feedback.
        P_world_h = np.hstack([P_world, np.ones((P_world.shape[0], 1))])
        P_cam_pred = (T_cam_from_world @ P_world_h.T).T[:, :3]
        errs = np.linalg.norm(P_cam_pred - P_cam, axis=1)
        max_mm = float(errs.max() * 1000.0)
        rms_mm = float(np.sqrt(np.mean(errs ** 2)) * 1000.0)
        per_tag = ", ".join(
            f"{cid}:{e * 1000.0:.1f}mm"
            for cid, e in zip(self._corner_ids, errs)
        )

        self._calibrated = True
        self.get_logger().info(
            f"Calibration complete. Residuals max={max_mm:.1f}mm "
            f"rms={rms_mm:.1f}mm per-tag=[{per_tag}]"
        )

    # ── Detection publishing ──────────────────────────────────────────────

    def _publish_detections(
        self,
        rgb_msg: Image,
        marker_poses: dict[int, dict],
    ) -> None:
        rover_ids_seen = [
            mid for mid in marker_poses
            if mid in self._rover_ids and mid not in self._corner_ids
        ]
        if not rover_ids_seen:
            return

        detections: list[TagDetection] = []
        for mid in rover_ids_seen:
            result = self._compute_world_pose(marker_poses[mid])
            if result is None:
                continue
            x, y, theta = result

            det = TagDetection()
            det.tag_id = int(mid)
            det.x = float(x)
            det.y = float(y)
            det.theta = float(theta)
            detections.append(det)

            self.get_logger().debug(
                f"Tag {mid}: x={x:.3f} m, y={y:.3f} m, "
                f"theta={np.degrees(theta):.1f}°"
            )

        if not detections:
            return

        msg = TagDetectionArray()
        msg.header = rgb_msg.header
        msg.detections = detections
        self._detections_pub.publish(msg)

        stamp_sec = (
            rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
        )
        self._tcp_push(detections, stamp_sec)

        self.get_logger().info(
            f"Published {len(detections)} rover tag(s): "
            f"{[d.tag_id for d in detections]}"
        )

    # ── World-pose computation ────────────────────────────────────────────

    def _compute_world_pose(
        self,
        marker_data: dict,
    ) -> tuple[float, float, float] | None:
        """Project a marker pose into the world frame.

        Returns (x, y, theta) in world coordinates, or None if the marker
        orientation is degenerate (parallel to the world Z axis).
        """
        tvec: np.ndarray = marker_data["tvec"]
        rvec: np.ndarray = marker_data["rvec"]

        normal = self._ground_plane["normal"]
        d = self._ground_plane["d"]

        # Project marker centre onto the ground plane.
        p_ground = project_point_to_plane(tvec, normal, d)
        p_world = self._T_world_from_cam @ np.append(p_ground, 1.0)

        # Project marker X axis onto the ground plane to get heading.
        R_cam_marker, _ = cv2.Rodrigues(rvec)
        marker_x_cam = R_cam_marker[:, 0]
        R_world_from_cam = self._T_world_from_cam[:3, :3]
        marker_x_world = R_world_from_cam @ marker_x_cam

        # Remove the normal component so the direction is in-plane.
        marker_x_world -= np.dot(marker_x_world, normal) * normal
        norm = np.linalg.norm(marker_x_world)
        if norm < 1e-6:
            self.get_logger().warn("Degenerate marker orientation — skipping.")
            return None
        marker_x_world /= norm

        theta = float(np.arctan2(marker_x_world[1], marker_x_world[0]))
        return float(p_world[0]), float(p_world[1]), theta


    # ── TCP server ────────────────────────────────────────────────────────

    def _tcp_server_loop(self) -> None:
        """Accept robot connections in a background thread."""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self._tcp_port))
        srv.listen(16)
        srv.settimeout(1.0)
        while rclpy.ok():
            try:
                conn, addr = srv.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self._tcp_lock:
                    self._tcp_clients.append(conn)
                self.get_logger().info(f"Robot connected from {addr}")
            except socket.timeout:
                continue
            except Exception as exc:
                self.get_logger().warn(f"TCP server error: {exc}")
        srv.close()

    def _tcp_push(self, detections: list[TagDetection], stamp_sec: float) -> None:
        """Serialize detections to JSON and push to all connected robots."""
        payload = json.dumps({
            "stamp": stamp_sec,
            "detections": [
                {"tag_id": d.tag_id, "x": d.x, "y": d.y, "theta": d.theta}
                for d in detections
            ],
        }) + "\n"
        data = payload.encode()
        dead: list[socket.socket] = []
        with self._tcp_lock:
            for conn in self._tcp_clients:
                try:
                    conn.sendall(data)
                except OSError:
                    dead.append(conn)
            for conn in dead:
                self._tcp_clients.remove(conn)
                conn.close()
                self.get_logger().info("Robot disconnected")


def main() -> None:
    rclpy.init()
    node = GroundLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
