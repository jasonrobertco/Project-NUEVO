from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ObstacleDiskMeasurement:
    x_mm: float
    y_mm: float
    radius_mm: float


@dataclass
class ObstacleTrack:
    track_id: int
    x_mm: float
    y_mm: float
    radius_mm: float
    covariance: np.ndarray
    last_seen_time: float
    last_update_time: float
    hit_count: int = 1
    miss_count: int = 0


class ObstacleTracker:
    """Track lidar-derived obstacle disks in world frame.

    Pipeline:
      world points -> clusters -> disk measurements -> track association

    Each track models a small static obstacle disk. Long objects such as walls
    are represented by multiple small tracks rather than one oversized track.
    """

    def __init__(
        self,
        *,
        cluster_neighbor_mm: float = 90.0,
        cluster_min_points: int = 3,
        max_disk_radius_mm: float = 75.0,
        disk_radius_margin_mm: float = 20.0,
        association_dist_mm: float = 180.0,
        ttl_s: float = 1.0,
        min_hits_to_confirm: int = 2,
        max_tracks: int = 12,
        process_noise_mm: float = 40.0,
        measurement_noise_mm: float = 30.0,
        radius_smoothing_alpha: float = 0.35,
    ) -> None:
        self._cluster_neighbor_mm = float(cluster_neighbor_mm)
        self._cluster_min_points = max(1, int(cluster_min_points))
        self._max_disk_radius_mm = float(max_disk_radius_mm)
        self._disk_radius_margin_mm = float(disk_radius_margin_mm)
        self._association_dist_mm = float(association_dist_mm)
        self._ttl_s = float(ttl_s)
        self._min_hits_to_confirm = max(1, int(min_hits_to_confirm))
        self._max_tracks = max(1, int(max_tracks))
        self._process_noise_mm = float(process_noise_mm)
        self._measurement_noise_mm = float(measurement_noise_mm)
        self._radius_alpha = max(0.0, min(1.0, float(radius_smoothing_alpha)))

        self._tracks: list[ObstacleTrack] = []
        self._next_track_id = 1

    def update(self, world_points_mm: np.ndarray, now_s: float) -> list[ObstacleTrack]:
        points = np.asarray(world_points_mm, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2:
            points = np.empty((0, 2), dtype=float)

        self._predict_tracks(now_s)
        measurements = self._extract_measurements(points)
        self._associate_and_update(measurements, now_s)
        self._expire_tracks(now_s)
        return self.get_tracks(now_s, include_unconfirmed=True)

    def get_tracks(
        self,
        now_s: float,
        *,
        include_unconfirmed: bool = False,
    ) -> list[ObstacleTrack]:
        tracks = []
        for track in self._tracks:
            if (now_s - track.last_seen_time) > self._ttl_s:
                continue
            if not include_unconfirmed and track.hit_count < self._min_hits_to_confirm:
                continue
            tracks.append(track)
        return list(tracks)

    def _predict_tracks(self, now_s: float) -> None:
        for track in self._tracks:
            dt = max(0.0, now_s - track.last_update_time)
            if dt <= 0.0:
                continue
            track.covariance = track.covariance + np.eye(2, dtype=float) * (self._process_noise_mm ** 2) * dt
            track.last_update_time = now_s

    def _extract_measurements(self, points: np.ndarray) -> list[ObstacleDiskMeasurement]:
        measurements: list[ObstacleDiskMeasurement] = []
        for cluster in self._cluster_points(points):
            measurements.extend(self._cluster_to_disks(cluster))
        return measurements

    def _cluster_points(self, points: np.ndarray) -> list[np.ndarray]:
        if points.shape[0] < self._cluster_min_points:
            return []

        clusters: list[np.ndarray] = []
        visited = np.zeros(points.shape[0], dtype=bool)
        neighbor_radius2 = self._cluster_neighbor_mm * self._cluster_neighbor_mm

        for start_idx in range(points.shape[0]):
            if visited[start_idx]:
                continue
            queue = [start_idx]
            visited[start_idx] = True
            members: list[int] = []

            while queue:
                idx = queue.pop()
                members.append(idx)
                deltas = points - points[idx]
                dist2 = np.einsum("ij,ij->i", deltas, deltas)
                neighbor_indices = np.flatnonzero((dist2 <= neighbor_radius2) & (~visited))
                for neighbor_idx in neighbor_indices.tolist():
                    visited[neighbor_idx] = True
                    queue.append(int(neighbor_idx))

            if len(members) >= self._cluster_min_points:
                clusters.append(points[np.asarray(members, dtype=int)])

        return clusters

    def _cluster_to_disks(self, cluster: np.ndarray) -> list[ObstacleDiskMeasurement]:
        if cluster.shape[0] == 0:
            return []

        center = np.mean(cluster, axis=0)
        if cluster.shape[0] == 1:
            return [ObstacleDiskMeasurement(float(center[0]), float(center[1]), self._max_disk_radius_mm)]

        centered = cluster - center
        cov = centered.T @ centered / max(1, cluster.shape[0] - 1)
        eigvals, eigvecs = np.linalg.eigh(cov)
        major_axis = eigvecs[:, int(np.argmax(eigvals))]
        projections = centered @ major_axis
        span = float(np.max(projections) - np.min(projections))
        max_diameter = 2.0 * self._max_disk_radius_mm

        if span <= max_diameter:
            return [self._fit_disk(cluster)]

        segment_count = max(2, int(math.ceil(span / max_diameter)))
        min_proj = float(np.min(projections))
        max_proj = float(np.max(projections))
        edges = np.linspace(min_proj, max_proj, segment_count + 1)

        disks: list[ObstacleDiskMeasurement] = []
        for idx in range(segment_count):
            if idx == segment_count - 1:
                mask = (projections >= edges[idx]) & (projections <= edges[idx + 1])
            else:
                mask = (projections >= edges[idx]) & (projections < edges[idx + 1])
            segment = cluster[mask]
            if segment.shape[0] == 0:
                continue
            disks.append(self._fit_disk(segment))
        return disks

    def _fit_disk(self, points: np.ndarray) -> ObstacleDiskMeasurement:
        center = np.mean(points, axis=0)
        distances = np.linalg.norm(points - center, axis=1)
        radius = min(
            self._max_disk_radius_mm,
            float(np.max(distances) + self._disk_radius_margin_mm),
        )
        radius = max(10.0, radius)
        return ObstacleDiskMeasurement(float(center[0]), float(center[1]), radius)

    def _associate_and_update(
        self,
        measurements: list[ObstacleDiskMeasurement],
        now_s: float,
    ) -> None:
        matched_track_indices: set[int] = set()
        matched_measurement_indices: set[int] = set()
        candidate_pairs: list[tuple[float, int, int]] = []

        for track_idx, track in enumerate(self._tracks):
            for meas_idx, meas in enumerate(measurements):
                distance = math.hypot(track.x_mm - meas.x_mm, track.y_mm - meas.y_mm)
                if distance <= self._association_dist_mm:
                    candidate_pairs.append((distance, track_idx, meas_idx))

        for _distance, track_idx, meas_idx in sorted(candidate_pairs, key=lambda item: item[0]):
            if track_idx in matched_track_indices or meas_idx in matched_measurement_indices:
                continue
            self._kalman_update(self._tracks[track_idx], measurements[meas_idx], now_s)
            matched_track_indices.add(track_idx)
            matched_measurement_indices.add(meas_idx)

        for track_idx, track in enumerate(self._tracks):
            if track_idx in matched_track_indices:
                continue
            track.miss_count += 1

        for meas_idx, meas in enumerate(measurements):
            if meas_idx in matched_measurement_indices:
                continue
            if len(self._tracks) >= self._max_tracks:
                break
            self._tracks.append(self._new_track(meas, now_s))

    def _kalman_update(
        self,
        track: ObstacleTrack,
        measurement: ObstacleDiskMeasurement,
        now_s: float,
    ) -> None:
        z = np.array([[measurement.x_mm], [measurement.y_mm]], dtype=float)
        x = np.array([[track.x_mm], [track.y_mm]], dtype=float)
        p = track.covariance
        r = np.eye(2, dtype=float) * (self._measurement_noise_mm ** 2)

        innovation = z - x
        s = p + r
        k = p @ np.linalg.inv(s)
        x_new = x + k @ innovation
        p_new = (np.eye(2, dtype=float) - k) @ p

        track.x_mm = float(x_new[0, 0])
        track.y_mm = float(x_new[1, 0])
        track.covariance = p_new
        track.radius_mm = (
            (1.0 - self._radius_alpha) * track.radius_mm
            + self._radius_alpha * measurement.radius_mm
        )
        track.radius_mm = min(self._max_disk_radius_mm, max(10.0, track.radius_mm))
        track.last_seen_time = now_s
        track.last_update_time = now_s
        track.hit_count += 1
        track.miss_count = 0

    def _new_track(self, measurement: ObstacleDiskMeasurement, now_s: float) -> ObstacleTrack:
        track = ObstacleTrack(
            track_id=self._next_track_id,
            x_mm=float(measurement.x_mm),
            y_mm=float(measurement.y_mm),
            radius_mm=float(min(self._max_disk_radius_mm, measurement.radius_mm)),
            covariance=np.eye(2, dtype=float) * (self._measurement_noise_mm ** 2),
            last_seen_time=now_s,
            last_update_time=now_s,
            hit_count=1,
            miss_count=0,
        )
        self._next_track_id += 1
        return track

    def _expire_tracks(self, now_s: float) -> None:
        self._tracks = [
            track
            for track in self._tracks
            if (now_s - track.last_seen_time) <= self._ttl_s
        ]
