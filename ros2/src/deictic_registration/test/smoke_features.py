"""Optional real-network smoke test on a generated calibrated planar image pair.

Uses actual pretrained SuperPoint + LightGlue weights, not injected feature
correspondences. Synthetic truth is used only to construct/score the image pair.
Run manually with the frontend dependencies installed; may download weights.
"""

import json
import time

import cv2
import numpy as np

from deictic_registration.core import CameraModel, estimate_registration
from deictic_registration.features import SuperPointMatcher


def main():
    rng = np.random.default_rng(37)
    image = np.full((480, 640, 3), 205, np.uint8)
    for index in range(140):
        center = tuple(rng.integers([18, 18], [620, 460]).tolist())
        color = tuple(rng.integers(0, 170, 3).tolist())
        cv2.circle(image, center, int(rng.integers(3, 14)), color, -1)
        if index % 3 == 0:
            cv2.putText(image, str(index), center, cv2.FONT_HERSHEY_SIMPLEX,
                        0.35, (20, 20, 20), 1, cv2.LINE_AA)
    k = np.array([[420., 0, 320], [0, 415., 240], [0, 0, 1]])
    z = 1.5
    r = cv2.Rodrigues(np.array([0.015, -0.025, 0.035]))[0]
    translation = np.array([0.055, -0.025, 0.035])
    homography = k @ (r + np.outer(translation, [0, 0, 1]) / z) @ np.linalg.inv(k)
    wrist = cv2.warpPerspective(image, homography, (640, 480))
    model = CameraModel(640, 480, k, np.zeros(5))
    matcher = SuperPointMatcher("cpu", 512)
    start = time.perf_counter()
    source_pixels, target_pixels = matcher.match(image, wrist)
    result = estimate_registration(source_pixels, target_pixels,
                                   np.full((480, 640), z, np.float32), model, model)
    translation_error = float(np.linalg.norm(result.camera_from_headset[:3, 3] - translation))
    rotation_error = float(np.linalg.norm(cv2.Rodrigues(result.camera_from_headset[:3, :3] @ r.T)[0]))
    report = {
        "test": "synthetic_planar_pair_real_superpoint_lightglue",
        "matches_with_depth": result.matches,
        "inliers": result.inliers,
        "median_reprojection_pixels": result.median_reprojection_error,
        "translation_error_m": translation_error,
        "rotation_error_rad": rotation_error,
        "processing_s": time.perf_counter() - start,
    }
    print(json.dumps(report, indent=2))
    assert result.inliers >= 40, "Insufficient real feature matches"
    assert translation_error < 0.025, "Estimated translation disagrees with rendered truth"
    assert rotation_error < 0.025, "Estimated rotation disagrees with rendered truth"


if __name__ == "__main__":
    main()
