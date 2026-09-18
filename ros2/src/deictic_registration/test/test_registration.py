import cv2
import numpy as np
import pytest

from deictic_registration.core import (
    CameraModel, RegistrationConfig, RegistrationError, estimate_registration,
    matrix_to_pose, pose_to_matrix,
)


def camera():
    return CameraModel(640, 480, np.array([[420., 0, 320], [0, 415., 240], [0, 0, 1]]),
                       np.zeros(5))


def observations(planar=False):
    rng = np.random.default_rng(7)
    model = camera()
    pixels = np.column_stack([rng.integers(100, 540, 120), rng.integers(80, 400, 120)]).astype(float)
    depth = np.full((480, 640), np.nan, dtype=np.float32)
    z = np.full(len(pixels), 1.5) if planar else rng.uniform(1.0, 2.5, len(pixels))
    depth[pixels[:, 1].astype(int), pixels[:, 0].astype(int)] = z
    norm = cv2.undistortPoints(pixels[:, None, :], model.matrix, model.distortion)[:, 0]
    xyz = np.column_stack([norm * z[:, None], z])
    rvec, tvec = np.array([0.025, -0.06, 0.035]), np.array([0.12, -0.02, 0.06])
    target = cv2.projectPoints(xyz, rvec, tvec, model.matrix, model.distortion)[0][:, 0]
    expected = np.eye(4)
    expected[:3, :3] = cv2.Rodrigues(rvec)[0]
    expected[:3, 3] = tvec
    return pixels, target, depth, model, expected, rng


@pytest.mark.parametrize("planar", [False, True])
def test_recovers_transform_with_noise_and_outliers(planar):
    source, target, depth, model, expected, rng = observations(planar)
    target += rng.normal(0, 0.15, target.shape)
    target[:25] = rng.uniform([20, 20], [620, 460], (25, 2))
    result = estimate_registration(source, target, depth, model, model)
    assert result.inliers >= 85
    assert result.median_reprojection_error < 0.5
    assert np.linalg.norm(result.camera_from_headset[:3, 3] - expected[:3, 3]) < 0.006
    delta = result.camera_from_headset[:3, :3] @ expected[:3, :3].T
    assert np.linalg.norm(cv2.Rodrigues(delta)[0]) < 0.006


def test_missing_depth_does_not_create_a_registration():
    source, target, depth, model, _, _ = observations()
    depth[:] = np.nan
    with pytest.raises(RegistrationError, match="valid aligned metric depth"):
        estimate_registration(source, target, depth, model, model)


def test_depth_resolution_must_match_rgb():
    source, target, depth, model, _, _ = observations()
    with pytest.raises(RegistrationError, match="aligned"):
        estimate_registration(source, target, depth[::2, ::2], model, model)


def test_random_feature_pairs_fail_inlier_gate():
    source, target, depth, model, _, rng = observations()
    target = rng.uniform([20, 20], [620, 460], target.shape)
    with pytest.raises(RegistrationError):
        estimate_registration(source, target, depth, model, model)


def test_invalid_calibration_is_rejected():
    with pytest.raises(RegistrationError):
        CameraModel(640, 480, np.zeros((3, 3)), np.zeros(5))


def test_quaternion_output_preserves_rotation():
    _, _, _, _, expected, _ = observations()
    pose = matrix_to_pose(expected)
    assert np.allclose(pose[:3], expected[:3, 3])
    q = np.array(pose[3:])
    assert np.isclose(np.linalg.norm(q), 1)
    angle = 2 * np.arccos(q[3])
    rotation = cv2.Rodrigues(q[:3] / np.sin(angle / 2) * angle)[0]
    assert np.allclose(rotation, expected[:3, :3])
    assert np.allclose(pose_to_matrix(pose), expected)
