"""Select on development confidence, freeze settings, then validate new captures."""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'ros2/src/deictic_registration'))
sys.path.insert(0, str(root / 'ros2/src/deictic_control'))
from deictic_registration.features import SuperPointMatcher
from deictic_registration.core import CameraModel, estimate_registration, matrix_to_pose
from deictic_control.fusion import FusionConfig

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('development', type=Path)
parser.add_argument('validation', type=Path, nargs='*')
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--thresholds', type=float, nargs='+', default=[.1, .5])
args = parser.parse_args()
if len(args.validation) not in (0, 3):
    parser.error('Provide either no validation captures or exactly three')
cfg = FusionConfig()
frontend = SuperPointMatcher('cpu', 512, 4, 0.1)


def load(directory):
    pair = json.loads((directory / 'camera_pair.json').read_text())
    if pair.get('synthetic') is not True:
        raise ValueError('This test requires explicitly synthetic captures')
    images = [cv2.cvtColor(cv2.imread(str(directory / (name + '_camera.png'))), cv2.COLOR_BGR2RGB)
              for name in ('headset', 'wrist')]
    models = [CameraModel(image.shape[1], image.shape[0], np.array(pair[name + '_K']), np.zeros(5))
              for name, image in zip(('headset', 'wrist'), images)]
    return pair, images, models, np.load(directory / 'headset_depth_z.npy', allow_pickle=False)


def evaluate(directory, threshold, loaded=None):
    pair, images, models, depth = loaded or load(directory)
    frontend.matcher.conf.filter_threshold = threshold
    record = {'capture': str(directory), 'stamp': pair['capture_wall_time_unix'],
              'device': 'cpu', 'cpu_threads': 4, 'max_keypoints': 512,
              'match_filter_threshold': threshold}
    record['capture_metadata'] = {key: pair.get(key) for key in (
        'fixture', 'pose_source', 'render_reference', 'rendering_time_seconds',
        'headset_calibration_image_size', 'wrist_calibration_image_size')}
    record['image_sha256'] = {name: hashlib.sha256((directory / (name + '_camera.png')).read_bytes()).hexdigest()
                              for name in ('headset', 'wrist')}
    start = time.perf_counter()
    try:
        source, target = frontend.match(*images)
        record['matched_features'] = len(source)
        cv2.setRNGSeed(2345)
        estimate = estimate_registration(source, target, depth, *models)
        x = np.array(pair['T_base_wristOptical']) @ estimate.camera_from_headset @ np.linalg.inv(pair['T_world_headOptical'])
        record['warm_processing_s'] = time.perf_counter() - start
        logit = cfg.alpha * estimate.inliers / estimate.matches - cfg.beta * estimate.median_reprojection_error - cfg.gamma
        confidence = float(1 / (1 + np.exp(-np.clip(logit, -60, 60))))
        record.update(status='estimated', inliers=estimate.inliers, matches_with_depth=estimate.matches,
                      median_reprojection_pixels=estimate.median_reprojection_error,
                      default_graph_confidence=confidence,
                      confidence_pass=confidence >= cfg.threshold and estimate.inliers >= cfg.min_inliers,
                      estimated_base_from_headset_world=matrix_to_pose(x))
        # Truth enters only this scoring block, after estimation and confidence.
        truth = np.array(pair['T_base_headsetWorld'])
        error = np.linalg.inv(truth) @ x
        targets_base = np.array([[v, -.25, .02, 1.] for v in (.08, .10, .12, .14, .16)])
        predictions = targets_base @ np.linalg.inv(truth).T @ x.T
        errors = np.linalg.norm(predictions[:, :3] - targets_base[:, :3], axis=1)
        record['scoring_only'] = {'translation_error_m': float(np.linalg.norm(error[:3, 3])),
            'rotation_error_rad': float(np.linalg.norm(cv2.Rodrigues(error[:3, :3])[0])),
            'five_target_grounding_errors_m': errors.tolist(),
            'five_target_mean_grounding_error_m': float(errors.mean()),
            'five_target_max_grounding_error_m': float(errors.max())}
        record['passes_static_check'] = bool(record['confidence_pass'] and errors.max() < .03
                                            and record['scoring_only']['translation_error_m'] < .03)
    except ValueError as exc:
        record.update(status='rejected', reason=str(exc), confidence_pass=False,
                      passes_static_check=False, warm_processing_s=time.perf_counter() - start)
    print(json.dumps(record), flush=True)
    return record


development_data = load(args.development)
frontend.match(*development_data[1])  # initialization excluded from warm timing
development = []
selected = None
for threshold in args.thresholds:
    result = evaluate(args.development, threshold, development_data)
    development.append(result)
    if result['confidence_pass']:
        selected = threshold  # selection never reads the scoring_only block
        break
validation = [evaluate(path, selected) for path in args.validation] if selected is not None else []
stamps = [record['stamp'] for record in development[:1] + validation]
unique_stamps = len(stamps) == 4 and len(set(stamps)) == 4
report = {'scope': 'one explicitly textured simulation workcell and fixed camera configuration',
          'selection_rule': 'first listed filter threshold with default confidence >= 0.7; truth not used for selection',
          'development_threshold_candidates': args.thresholds,
          'selected_filter_threshold': selected, 'development': development, 'validation': validation,
          'unique_capture_stamps': unique_stamps,
          'all_validation_pass': unique_stamps and all(record['passes_static_check'] for record in validation)}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({'selected_filter_threshold': selected, 'all_validation_pass': report['all_validation_pass']}), flush=True)
