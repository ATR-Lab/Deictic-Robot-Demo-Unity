"""Depth-assisted PnP in optical coordinates (x right, y down, z forward).

The correspondence frontend is deliberately separate from geometry so that the
geometry can be tested independently of model downloads, ROS, and hardware.
"""

from dataclasses import dataclass

import cv2
import numpy as np


class RegistrationError(ValueError):
    """A measurement is unusable; never substitute an identity observation."""


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    matrix: np.ndarray
    distortion: np.ndarray

    def __post_init__(self):
        k = np.asarray(self.matrix, dtype=np.float64)
        d = np.asarray(self.distortion, dtype=np.float64).reshape(-1)
        if (self.width <= 0 or self.height <= 0 or k.shape != (3, 3)
                or not np.all(np.isfinite(k)) or not np.all(np.isfinite(d))
                or k[0, 0] <= 0 or k[1, 1] <= 0
                or not np.allclose(k[2], [0, 0, 1])
                or not np.isclose(k[0, 1], 0)
                or not np.isclose(k[1, 0], 0)
                or d.size not in (0, 4, 5, 8, 12, 14)):
            raise RegistrationError("Missing or invalid pinhole camera calibration")
        object.__setattr__(self, "matrix", k)
        object.__setattr__(self, "distortion", d)


@dataclass(frozen=True)
class RegistrationConfig:
    min_inliers: int = 12
    min_inlier_ratio: float = 0.35
    ransac_pixels: float = 3.0
    max_median_pixels: float = 2.0
    ransac_iterations: int = 500
    ransac_confidence: float = 0.999
    min_depth_m: float = 0.2
    max_depth_m: float = 5.0
    huber_pixels: float = 1.5

    def __post_init__(self):
        if (self.min_inliers < 6 or not 0 < self.min_inlier_ratio <= 1
                or self.ransac_pixels <= 0 or self.max_median_pixels <= 0
                or self.ransac_iterations < 1 or not 0 < self.ransac_confidence < 1
                or not 0 < self.min_depth_m < self.max_depth_m
                or self.huber_pixels <= 0):
            raise ValueError("Invalid registration thresholds")


@dataclass(frozen=True)
class RegistrationResult:
    camera_from_headset: np.ndarray
    inliers: int
    matches: int
    median_reprojection_error: float


def _points(value):
    value = np.asarray(value, dtype=np.float64)
    if value.ndim != 2 or value.shape[1] != 2 or not np.all(np.isfinite(value)):
        raise RegistrationError("Feature coordinates must be a finite Nx2 array")
    return value


def depth_correspondences(headset_pixels, wrist_pixels, depth_z, headset_camera,
                          wrist_camera, config):
    """Backproject only valid aligned depth; depth values are optical z in meters.

    Nearest-neighbor depth avoids mixing foreground/background depths at edges.
    Invalid holes are discarded, never filled with a guessed plane or range.
    """
    source, target = _points(headset_pixels), _points(wrist_pixels)
    depth = np.asarray(depth_z)
    if source.shape != target.shape:
        raise RegistrationError("Feature arrays have different lengths")
    if depth.shape != (headset_camera.height, headset_camera.width):
        raise RegistrationError("Depth is not aligned to the RGB camera resolution")
    xy = np.rint(source).astype(np.int64)
    valid = ((xy[:, 0] >= 0) & (xy[:, 0] < headset_camera.width)
             & (xy[:, 1] >= 0) & (xy[:, 1] < headset_camera.height)
             & (target[:, 0] >= 0) & (target[:, 0] < wrist_camera.width)
             & (target[:, 1] >= 0) & (target[:, 1] < wrist_camera.height))
    selected = np.flatnonzero(valid)
    z = depth[xy[selected, 1], xy[selected, 0]]
    good = np.isfinite(z) & (z >= config.min_depth_m) & (z <= config.max_depth_m)
    selected, z = selected[good], z[good].astype(np.float64)
    if len(selected) < config.min_inliers:
        raise RegistrationError(f"Insufficient matches with valid aligned metric depth: {len(selected)}/{len(source)}, need {config.min_inliers}")
    normalized = cv2.undistortPoints(
        source[selected].reshape(-1, 1, 2), headset_camera.matrix,
        headset_camera.distortion).reshape(-1, 2)
    points3d = np.column_stack([normalized * z[:, None], z])
    # Reject collinear geometry. A plane is allowed: the task is tabletop reaching.
    spread = np.linalg.svd(points3d - points3d.mean(axis=0), compute_uv=False)
    if spread[1] < 1e-4:
        raise RegistrationError("Degenerate or collinear 3D feature geometry")
    return points3d, target[selected]


def _huber_cost(residual, width):
    norm = np.linalg.norm(residual, axis=1)
    return np.sum(np.where(norm <= width, 0.5 * norm * norm,
                           width * (norm - 0.5 * width)))


def _robust_refine(points3d, pixels, camera, rvec, tvec, huber_width):
    """Damped IRLS reprojection minimization with a vector Huber residual."""
    parameters = np.r_[rvec.reshape(3), tvec.reshape(3)].astype(np.float64)
    damping = 1e-3
    for _ in range(30):
        projected, jacobian = cv2.projectPoints(
            points3d, parameters[:3], parameters[3:], camera.matrix, camera.distortion)
        residual = projected.reshape(-1, 2) - pixels
        norms = np.linalg.norm(residual, axis=1)
        weights = np.repeat(np.minimum(1.0, huber_width / np.maximum(norms, 1e-12)), 2)
        j = jacobian[:, :6]
        hessian = j.T @ (weights[:, None] * j)
        gradient = j.T @ (weights * residual.reshape(-1))
        try:
            step = np.linalg.solve(
                hessian + damping * np.diag(np.maximum(np.diag(hessian), 1e-9)),
                -gradient)
        except np.linalg.LinAlgError as exc:
            raise RegistrationError("Singular reprojection refinement") from exc
        candidate = parameters + step
        new_projection, _ = cv2.projectPoints(
            points3d, candidate[:3], candidate[3:], camera.matrix, camera.distortion)
        if _huber_cost(new_projection.reshape(-1, 2) - pixels, huber_width) < _huber_cost(residual, huber_width):
            parameters = candidate
            damping = max(damping / 3, 1e-10)
            if np.linalg.norm(step) < 1e-9:
                break
        else:
            damping = min(damping * 10, 1e10)
    return parameters[:3], parameters[3:]


def estimate_registration(headset_pixels, wrist_pixels, depth_z,
                          headset_camera, wrist_camera, config=None):
    """Return T_wristOptical_headsetOptical from measured correspondences."""
    config = config or RegistrationConfig()
    points3d, pixels = depth_correspondences(
        headset_pixels, wrist_pixels, depth_z, headset_camera, wrist_camera, config)
    ok, rvec, tvec, initial_inliers = cv2.solvePnPRansac(
        points3d, pixels, wrist_camera.matrix, wrist_camera.distortion,
        iterationsCount=config.ransac_iterations, reprojectionError=config.ransac_pixels,
        confidence=config.ransac_confidence, flags=cv2.SOLVEPNP_EPNP)
    if not ok or initial_inliers is None or len(initial_inliers) < config.min_inliers:
        count = 0 if initial_inliers is None else len(initial_inliers)
        raise RegistrationError(f"RANSAC PnP did not find sufficient inliers: {count}/{len(points3d)}, need {config.min_inliers}")
    selected = initial_inliers.reshape(-1)
    rvec, tvec = _robust_refine(points3d[selected], pixels[selected], wrist_camera,
                               rvec, tvec, config.huber_pixels)
    rotation = cv2.Rodrigues(rvec)[0]
    projected, _ = cv2.projectPoints(points3d, rvec, tvec, wrist_camera.matrix,
                                      wrist_camera.distortion)
    errors = np.linalg.norm(projected.reshape(-1, 2) - pixels, axis=1)
    positive_depth = (points3d @ rotation.T + tvec)[:, 2] > 0.01
    accepted = positive_depth & np.isfinite(errors) & (errors <= config.ransac_pixels)
    count = int(accepted.sum())
    if count < config.min_inliers or count / len(points3d) < config.min_inlier_ratio:
        raise RegistrationError(f"Refined PnP failed inlier/cheirality gate: {count}/{len(points3d)}")
    median = float(np.median(errors[accepted]))
    if median > config.max_median_pixels:
        raise RegistrationError("Refined PnP exceeds median reprojection limit")
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rotation, tvec
    if not np.all(np.isfinite(transform)):
        raise RegistrationError("Non-finite PnP solution")
    # Count includes only geometrically eligible matches (valid source depth).
    return RegistrationResult(transform, count, len(points3d), median)


def matrix_to_pose(transform):
    """Serialize a valid homogeneous transform as xyz, quaternion xyzw."""
    transform = np.asarray(transform, dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise RegistrationError("Invalid output transform")
    vector, _ = cv2.Rodrigues(transform[:3, :3])
    angle = float(np.linalg.norm(vector))
    quat = (np.r_[vector.reshape(3) * (np.sin(angle / 2) / angle), np.cos(angle / 2)]
            if angle > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0]))
    return np.r_[transform[:3, 3], quat].tolist()


def pose_to_matrix(pose):
    pose = np.asarray(pose, dtype=np.float64)
    if pose.shape != (7,) or not np.all(np.isfinite(pose)) or not np.isclose(np.linalg.norm(pose[3:]), 1, atol=1e-3):
        raise RegistrationError("Pose must contain xyz and a unit xyzw quaternion")
    x, y, z, w = pose[3:] / np.linalg.norm(pose[3:])
    transform = np.eye(4)
    transform[:3, 3] = pose[:3]
    transform[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    return transform
