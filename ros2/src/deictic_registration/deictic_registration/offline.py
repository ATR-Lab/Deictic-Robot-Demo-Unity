"""Run the real frontend on an exported synchronized camera capture.

manifest.json schema:
  headset: {rgb, depth, width, height, K, D, pose}
  wrist:   {rgb, width, height, K, D, pose}
  optional truth_base_from_headset_world: xyz+xyzw (scoring only)

RGB paths are PNG/JPEG, depth is an aligned 2D float32 optical-z NPY in meters.
K is a nine-element row-major camera matrix, D is pinhole distortion.
Poses are the same capture-pose contract used by the ROS node.
"""

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from .core import CameraModel, estimate_registration, matrix_to_pose, pose_to_matrix
from .features import SuperPointMatcher


def run_capture(directory, device="cpu", max_keypoints=512, match_filter_threshold=0.1):
    directory = Path(directory)
    manifest = json.loads((directory / "manifest.json").read_text())
    head, wrist = manifest["headset"], manifest["wrist"]

    def image(description):
        pixels = cv2.imread(str(directory / description["rgb"]), cv2.IMREAD_COLOR)
        if pixels is None:
            raise ValueError("Cannot decode camera image: " + description["rgb"])
        if pixels.shape[:2] != (description["height"], description["width"]):
            raise ValueError("Camera image resolution differs from calibration")
        return cv2.cvtColor(pixels, cv2.COLOR_BGR2RGB)

    def model(description):
        return CameraModel(description["width"], description["height"],
                           np.asarray(description["K"]).reshape(3, 3),
                           np.asarray(description.get("D", [])))

    headset_rgb, wrist_rgb = image(head), image(wrist)
    depth = np.load(directory / head["depth"], allow_pickle=False)
    head_model, wrist_model = model(head), model(wrist)
    head_pose, camera_pose = pose_to_matrix(head["pose"]), pose_to_matrix(wrist["pose"])
    frontend = SuperPointMatcher(device, max_keypoints, match_filter_threshold=match_filter_threshold)
    start = time.perf_counter()
    source, target = frontend.match(headset_rgb, wrist_rgb)
    result = estimate_registration(source, target, depth, head_model, wrist_model)
    estimated_x = camera_pose @ result.camera_from_headset @ np.linalg.inv(head_pose)
    report = {
        "status": "registration_estimated",
        "source": "superpoint_pnp",
        "capture": str(directory.resolve()),
        "input_provider": manifest.get("input_provider", "unspecified"),
        "max_keypoints": max_keypoints,
        "match_filter_threshold": match_filter_threshold,
        "matched_features": len(source),
        "matches_with_depth": result.matches,
        "inliers": result.inliers,
        "median_reprojection_pixels": result.median_reprojection_error,
        "camera_from_headset": matrix_to_pose(result.camera_from_headset),
        "estimated_base_from_headset_world": matrix_to_pose(estimated_x),
        "processing_s": time.perf_counter() - start,
    }
    # Ground truth is accessed only after estimation, exclusively for scoring.
    if "truth_base_from_headset_world" in manifest:
        truth = pose_to_matrix(manifest["truth_base_from_headset_world"])
        error = np.linalg.inv(truth) @ estimated_x
        report["scoring_only"] = {
            "translation_error_m": float(np.linalg.norm(error[:3, 3])),
            "rotation_error_rad": float(np.linalg.norm(cv2.Rodrigues(error[:3, :3])[0])),
        }
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_dir", type=Path)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--max-keypoints", type=int, default=512)
    parser.add_argument("--match-filter-threshold", type=float, default=0.1)
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args()
    try:
        report = run_capture(arguments.capture_dir, arguments.device, arguments.max_keypoints,
                             arguments.match_filter_threshold)
    except Exception as exc:
        report = {"status": "registration_failed", "source": "superpoint_pnp", "reason": str(exc),
                  "capture": str(arguments.capture_dir.resolve()), "max_keypoints": arguments.max_keypoints,
                  "match_filter_threshold": arguments.match_filter_threshold}
    serialized = json.dumps(report, indent=2, allow_nan=False)
    print(serialized)
    if arguments.output:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(serialized + "\n")
    raise SystemExit(0 if report["status"] == "registration_estimated" else 1)


if __name__ == "__main__":
    main()
