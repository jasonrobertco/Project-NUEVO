"""
Geometry utilities for the ground-plane localizer.

All functions operate in the camera's 3-D coordinate frame unless the
docstring states otherwise.
"""

import numpy as np


def fit_plane_svd(points: np.ndarray) -> tuple[np.ndarray, float]:
    """Fit a plane to a set of 3-D points using SVD.

    NOTE: Unused after the tripod/SVD calibration refactor. The ground plane
    is now derived analytically from ``T_cam_from_world`` (the image of the
    world ``z = 0`` plane). Kept here for potential future reuse.

    Parameters
    ----------
    points:
        Array of shape (N, 3) containing the 3-D points to fit.

    Returns
    -------
    normal:
        Unit normal vector of the best-fit plane (shape (3,)).
    d:
        Plane offset such that ``normal @ p + d == 0`` for any point *p*
        on the plane.
    """
    centroid = np.mean(points, axis=0)
    _, _, Vt = np.linalg.svd(points - centroid)
    normal: np.ndarray = Vt[-1]
    normal /= np.linalg.norm(normal)
    d: float = float(-normal.dot(centroid))
    return normal, d


def project_point_to_plane(
    p: np.ndarray,
    normal: np.ndarray,
    d: float,
) -> np.ndarray:
    """Project a 3-D point onto a plane defined by (normal, d).

    Parameters
    ----------
    p:
        Point to project, shape (3,).
    normal:
        Unit normal of the plane, shape (3,).
    d:
        Plane offset (``normal @ p + d == 0`` on the plane).

    Returns
    -------
    Projected point on the plane, shape (3,).
    """
    dist = normal.dot(p) + d
    return p - dist * normal


def rigid_transform_svd(
    P_src: np.ndarray,
    P_dst: np.ndarray,
) -> np.ndarray:
    """Kabsch / Umeyama rigid alignment.

    Given ``N >= 3`` corresponding 3-D points (one per row), return the
    ``4x4`` homogeneous transform ``T`` such that ``T @ P_src ~= P_dst``
    in the least-squares sense, with a reflection guard to ensure
    ``det(R) = +1``.

    Parameters
    ----------
    P_src:
        Source point set, shape ``(N, 3)``.
    P_dst:
        Destination point set, shape ``(N, 3)``. Must align with
        ``P_src`` row-for-row.

    Returns
    -------
    T:
        Homogeneous ``4x4`` matrix mapping points from the source frame to
        the destination frame.
    """
    P_src = np.asarray(P_src, dtype=np.float64)
    P_dst = np.asarray(P_dst, dtype=np.float64)

    mu_src = P_src.mean(axis=0)
    mu_dst = P_dst.mean(axis=0)

    H = (P_src - mu_src).T @ (P_dst - mu_dst)
    U, _, Vt = np.linalg.svd(H)

    # Reflection guard: ensure right-handed rotation (det == +1).
    D = np.eye(3)
    D[2, 2] = float(np.sign(np.linalg.det(Vt.T @ U.T)))
    R = Vt.T @ D @ U.T
    t = mu_dst - R @ mu_src

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T
