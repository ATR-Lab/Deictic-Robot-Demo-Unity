"""Bounded two-pose GPU development selection; simulator truth only scores outputs.

Selection uses unchanged geometric and confidence gates at both declared poses.
Fresh held-out captures must be evaluated separately before deployment.
"""
import argparse
import gc
import hashlib
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'ros2/src/deictic_registration'))
sys.path.insert(0, str(root / 'ros2/src/deictic_control'))
from deictic_registration.features import SuperPointMatcher
from deictic_registration.core import CameraModel, estimate_registration, matrix_to_pose
from deictic_control.fusion import FusionConfig


def load(directory):
    pair = json.loads((directory / 'camera_pair.json').read_text())
    if pair.get('synthetic') is not True:
        raise ValueError('Only explicitly synthetic development captures are supported')
    images = []
    for name in ('headset', 'wrist'):
        raw = cv2.imread(str(directory / (name + '_camera.png')))
        if raw is None:
            raise ValueError('Missing or invalid image: ' + str(directory))
        images.append(cv2.cvtColor(raw, cv2.COLOR_BGR2RGB))
    models = [CameraModel(image.shape[1], image.shape[0], np.array(pair[name + '_K']), np.zeros(5))
              for name, image in zip(('headset', 'wrist'), images)]
    return pair, images, models, np.load(directory / 'headset_depth_z.npy', allow_pickle=False)


def evaluate(frontend, directory, loaded, keypoints, threshold, cfg):
    pair, images, models, depth = loaded
    record = {'capture': str(directory), 'stamp': pair['capture_wall_time_unix'],
              'device': 'cuda', 'max_keypoints': keypoints,
              'match_filter_threshold': threshold,
              'image_sha256': {name: hashlib.sha256((directory / (name + '_camera.png')).read_bytes()).hexdigest()
                               for name in ('headset', 'wrist')},
              'capture_metadata': {key: pair.get(key) for key in
                  ('fixture', 'floor_profile', 'camera_mount_profile', 'camera_mount_translation',
                   'camera_mount_quaternion_xyzw', 'camera_mount_local_offset',
                   'camera_mount_local_quaternion_xyzw', 'headset_resolution', 'wrist_resolution',
                   'headset_resolution_scale', 'pose_source', 'synchronization')}}
    torch.cuda.synchronize()
    torch.cuda.reset_peak_memory_stats()
    start = time.perf_counter()
    try:
        source, target = frontend.match(*images)
        record['matched_features'] = len(source)
        cv2.setRNGSeed(2345)
        estimate = estimate_registration(source, target, depth, *models)
        torch.cuda.synchronize()
        record['warm_processing_s'] = time.perf_counter() - start
        logit = cfg.alpha * estimate.inliers / estimate.matches - cfg.beta * estimate.median_reprojection_error - cfg.gamma
        confidence = float(1 / (1 + np.exp(-np.clip(logit, -60, 60))))
        x = np.array(pair['T_base_wristOptical']) @ estimate.camera_from_headset @ np.linalg.inv(pair['T_world_headOptical'])
        record.update(status='estimated', inliers=estimate.inliers, matches_with_depth=estimate.matches,
                      median_reprojection_pixels=estimate.median_reprojection_error,
                      default_graph_confidence=confidence,
                      confidence_pass=bool(confidence >= cfg.threshold and estimate.inliers >= cfg.min_inliers),
                      estimated_base_from_headset_world=matrix_to_pose(x))
        # Simulator truth is accessed only here, after estimation/confidence.
        # This block never participates in selecting the next configuration.
        truth = np.array(pair['T_base_headsetWorld'])
        error = np.linalg.inv(truth) @ x
        targets = np.array([[v, -.25, .02, 1.] for v in (.08, .10, .12, .14, .16)])
        predictions = targets @ np.linalg.inv(truth).T @ x.T
        errors = np.linalg.norm(predictions[:, :3] - targets[:, :3], axis=1)
        record['scoring_only'] = {'translation_error_m': float(np.linalg.norm(error[:3, 3])),
            'rotation_error_rad': float(np.linalg.norm(cv2.Rodrigues(error[:3, :3])[0])),
            'five_target_grounding_errors_m': errors.tolist(),
            'five_target_max_grounding_error_m': float(errors.max())}
    except ValueError as exc:
        torch.cuda.synchronize()
        record.update(status='rejected', reason=str(exc), confidence_pass=False,
                      warm_processing_s=time.perf_counter() - start)
    record['pytorch_peak_allocated_bytes'] = torch.cuda.max_memory_allocated()
    record['pytorch_peak_reserved_bytes'] = torch.cuda.max_memory_reserved()
    print(json.dumps(record), flush=True)
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('development_rest', type=Path, nargs='?')
    parser.add_argument('development_postreach', type=Path, nargs='?')
    parser.add_argument('--rest-development', type=Path, nargs=3)
    parser.add_argument('--end-development', type=Path, nargs=3)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.rest_development is not None or args.end_development is not None:
        if args.rest_development is None or args.end_development is None:
            parser.error('Both --rest-development and --end-development need three captures')
        if args.development_rest is not None or args.development_postreach is not None:
            parser.error('Use either the two positional captures or the two three-capture lists')
        rest_paths, end_paths = args.rest_development, args.end_development
    else:
        if args.development_rest is None or args.development_postreach is None:
            parser.error('Provide both positional captures or both three-capture lists')
        rest_paths, end_paths = [args.development_rest], [args.development_postreach]
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA is unavailable; this bounded run has no fallback')
    # Only this process's PyTorch allocator is constrained. Other workloads are untouched.
    torch.cuda.set_per_process_memory_fraction(.25)
    torch.set_num_threads(4)
    paths = rest_paths + end_paths
    loaded = [load(path) for path in paths]
    stamps = [data[0]['capture_wall_time_unix'] for data in loaded]
    if len(set(stamps)) != len(stamps):
        raise ValueError('Every declared development capture must have a distinct timestamp')
    cfg = FusionConfig()
    configurations = [(count, threshold) for threshold in (.8, .9) for count in (512, 1024, 2048)]
    report = {'scope': 'development-only selection at two fixed synthetic poses',
              'development_captures_per_pose': len(rest_paths),
              'ordered_candidates': configurations, 'pytorch_allocator_fraction': .25,
              'torch_version': torch.__version__, 'gpu': torch.cuda.get_device_name(),
              'selection_rule': 'first ordered configuration passing unchanged confidence and geometry on every declared capture at both poses; truth excluded',
              'trials': [], 'selected': None}
    try:
        for count, threshold in configurations:
            free, total = torch.cuda.mem_get_info()
            if free < 2 * 1024**3:
                raise RuntimeError('Less than 2 GiB free GPU memory; bounded run stopped')
            frontend = SuperPointMatcher('cuda', count, 4, threshold)
            for data in loaded:
                frontend.match(*data[1])  # exclude model/device warm-up from measured calls
            records = [evaluate(frontend, path, data, count, threshold, cfg)
                       for path, data in zip(paths, loaded)]
            for index, record in enumerate(records):
                record['pose_group'] = 'rest' if index < len(rest_paths) else 'postreach'
            trial = {'max_keypoints': count, 'match_filter_threshold': threshold,
                     'free_gpu_bytes_before': free, 'total_gpu_bytes': total,
                     'poses': records, 'both_quality_pass': all(r['confidence_pass'] for r in records)}
            report['trials'].append(trial)
            del frontend
            gc.collect()
            torch.cuda.empty_cache()
            if trial['both_quality_pass']:
                report['selected'] = {'device': 'cuda', 'max_keypoints': count,
                                      'match_filter_threshold': threshold}
                report['selection_frozen_at_unix'] = time.time()
                break
    except (torch.cuda.OutOfMemoryError, RuntimeError, ValueError) as exc:
        report['stopped_reason'] = str(exc)
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'selected': report['selected'], 'stopped_reason': report.get('stopped_reason')}), flush=True)


if __name__ == '__main__':
    main()
