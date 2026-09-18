"""Read-only development diagnosis of matching versus depth-boundary errors.

Simulator truth is consulted only AFTER the unchanged learned matcher/PnP run.
Neighborhood depths are oracle diagnostics, never estimator inputs or corrections.
"""
import argparse
import hashlib
import json
import time
from pathlib import Path

import cv2
import numpy as np


def skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def epipolar_distances(source, target, head_k, wrist_k, wrist_from_head):
    fundamental = np.linalg.inv(wrist_k).T @ skew(wrist_from_head[:3, 3]) @ wrist_from_head[:3, :3] @ np.linalg.inv(head_k)
    lines = np.column_stack((source, np.ones(len(source)))) @ fundamental.T
    distances = np.abs(np.sum(lines * np.column_stack((target, np.ones(len(target)))), axis=1))
    norms = np.linalg.norm(lines[:, :2], axis=1)
    return np.divide(distances, norms, out=np.full(len(source), np.nan), where=norms > 1e-12)


def project_depths(ray, depths, transform, camera_k, target):
    xyz = np.asarray(depths)[:, None] * np.asarray(ray)[None, :]
    wrist = xyz @ transform[:3, :3].T + transform[:3, 3]
    projection = wrist @ camera_k.T
    valid = wrist[:, 2] > .01
    errors = np.full(len(xyz), np.inf)
    errors[valid] = np.linalg.norm(projection[valid, :2] / projection[valid, 2:] - target, axis=1)
    return errors


def table_footprint(pixels, camera_k, base_from_camera):
    # Declared fixture bounds only, used for diagnostic labels. Foreground objects
    # and occlusion may project inside this footprint; this is not segmentation.
    corners = np.array([[.005, -.55, -.149, 1], [.555, -.55, -.149, 1],
                        [.555, -.05, -.149, 1], [.005, -.05, -.149, 1]])
    points = corners @ np.linalg.inv(base_from_camera).T
    clipped = []
    for a, b in zip(points, np.roll(points, -1, axis=0)):
        a_in, b_in = a[2] >= 1e-5, b[2] >= 1e-5
        if a_in:
            clipped.append(a)
        if a_in != b_in:
            clipped.append(a + (b-a) * (1e-5-a[2]) / (b[2]-a[2]))
    if len(clipped) < 3:
        return np.zeros(len(pixels), dtype=bool)
    projected = np.asarray(clipped)[:, :3] @ camera_k.T
    polygon = (projected[:, :2] / projected[:, 2:]).astype(np.float32)
    return np.array([cv2.pointPolygonTest(polygon, tuple(map(float, p)), False) >= 0 for p in pixels])


def neighborhood_metrics(depth, rounded, ray, target, truth, wrist_k, size, min_depth, max_depth):
    x, y = rounded
    radius = size // 2
    if not (0 <= x < depth.shape[1] and 0 <= y < depth.shape[0]):
        return {'valid_depth_count': 0}
    window = depth[max(0, y-radius):min(depth.shape[0], y+radius+1),
                   max(0, x-radius):min(depth.shape[1], x+radius+1)]
    candidates = window[np.isfinite(window) & (window >= min_depth) & (window <= max_depth)].astype(float)
    if len(candidates) == 0:
        return {'valid_depth_count': 0}
    errors = project_depths(ray, candidates, truth, wrist_k, target)
    best = int(np.argmin(errors))
    return {'valid_depth_count': len(candidates), 'depth_min_m': float(candidates.min()),
            'depth_max_m': float(candidates.max()), 'depth_span_m': float(np.ptp(candidates)),
            'best_depth_m': float(candidates[best]), 'best_truth_reprojection_pixels': float(errors[best])}


def finite_json(value):
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, list):
        return [finite_json(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    return value


def geometry_self_test():
    k = np.array([[320., 0, 320], [0, 320, 240], [0, 0, 1]])
    t = np.eye(4); t[0, 3] = .1
    source = np.array([[320., 240.], [300., 210.]])
    rays = np.column_stack((source, np.ones(2))) @ np.linalg.inv(k).T
    xyz = rays * np.array([1., 2.])[:, None]
    wrist = xyz + t[:3, 3]
    q = wrist @ k.T
    target = q[:, :2] / q[:, 2:]
    np.testing.assert_allclose(epipolar_distances(source, target, k, k, t), 0, atol=1e-9)
    perturbed = target.copy(); perturbed[:, 1] += 4
    np.testing.assert_allclose(epipolar_distances(source, perturbed, k, k, t), 4, atol=1e-9)
    # A depth edge can change disparity along the line but cannot remove its 4px perpendicular error.
    np.testing.assert_allclose(project_depths(rays[0], np.array([1., 2.]), t, k, target[0]), [0, 16], atol=1e-9)
    assert np.min(project_depths(rays[0], np.linspace(.5, 3, 101), t, k, perturbed[0])) >= 4 - 1e-9
    depth = np.ones((5, 5), dtype=np.float32) * 2
    depth[2, 3] = 1
    window = neighborhood_metrics(depth, (2, 2), rays[0], target[0], t, k, 3, .2, 5)
    assert window['depth_span_m'] == 1 and window['best_truth_reprojection_pixels'] < 1e-9
    return {'passed': True, 'checks': ['true epipolar incidence', '4px depth-independent inconsistency',
                                     'depth disparity', 'depth cannot remove perpendicular residual', 'neighborhood depth-boundary recovery']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--end-development', type=Path, nargs=3)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    checks = geometry_self_test()
    if args.self_test:
        print(json.dumps(checks)); return
    if args.end_development is None or args.output is None:
        parser.error('--end-development needs three capture directories and --output is required')
    import torch
    from select_simulation_registration_profile import load, SuperPointMatcher, FusionConfig
    from deictic_registration.core import RegistrationConfig, estimate_registration, depth_correspondences
    torch.set_num_threads(4)
    if not torch.cuda.is_available() or torch.cuda.mem_get_info()[0] < 2 * 1024**3:
        raise RuntimeError('CUDA with at least 2 GiB free memory is required; no fallback')
    loaded = [load(path) for path in args.end_development]
    if len({data[0]['capture_stamp_ns'] for data in loaded}) != 3:
        raise ValueError('Three distinct declared development captures are required')
    cfg, graph_cfg = RegistrationConfig(), FusionConfig()
    report = {'scope': 'read-only depth-boundary diagnosis on three declared development frames',
              'geometry_self_test': checks, 'profile': {'device': 'cuda', 'max_keypoints': 1024,
              'match_filter_threshold': .8, 'cpu_threads': 4, 'pytorch_allocator_fraction': .25},
              'truth_usage': 'diagnostic only after unchanged matcher/PnP; no depth substitution reaches any estimator or live provider',
              'interpretation': 'A neighborhood depth explaining reprojection is only plausible boundary evidence, not proof of correspondence correctness. Epipolar distance above3px cannot be fixed by changing depth alone along that exact head ray. Table-footprint labels include possible foreground/occlusion.',
              'captures': [], 'complete': False}
    try:
        frontend = SuperPointMatcher('cuda', 1024, 4, .8)
        frontend.match(*loaded[0][1])
        for path, (pair, images, cameras, depth) in zip(args.end_development, loaded):
            record = {'capture': str(path), 'capture_stamp_ns': pair['capture_stamp_ns'],
                      'floor_profile': pair.get('floor_profile'),
                      'image_sha256': {name: hashlib.sha256((path/(name+'_camera.png')).read_bytes()).hexdigest()
                                       for name in ('headset', 'wrist')}}
            if any(np.any(camera.distortion != 0) for camera in cameras):
                raise ValueError('Diagnostic requires the captured undistorted pinhole images')
            torch.cuda.synchronize(); start = time.perf_counter()
            source, target = frontend.match(*images)
            cv2.setRNGSeed(2345)
            estimate = estimate_registration(source, target, depth, *cameras, cfg)
            torch.cuda.synchronize()
            record['warm_match_and_pnp_seconds'] = time.perf_counter()-start
            logit = graph_cfg.alpha*estimate.inliers/estimate.matches-graph_cfg.beta*estimate.median_reprojection_error-graph_cfg.gamma
            record.update(matches=len(source), inliers=estimate.inliers, matches_with_depth=estimate.matches,
                          median_reprojection_pixels=estimate.median_reprojection_error,
                          default_graph_confidence=float(1/(1+np.exp(-np.clip(logit, -60, 60)))))

            # Match the production depth selection exactly, checking indices against its outputs.
            rounded = np.rint(source).astype(int)
            valid = ((rounded[:, 0] >= 0) & (rounded[:, 0] < cameras[0].width) &
                     (rounded[:, 1] >= 0) & (rounded[:, 1] < cameras[0].height) &
                     (target[:, 0] >= 0) & (target[:, 0] < cameras[1].width) &
                     (target[:, 1] >= 0) & (target[:, 1] < cameras[1].height))
            depths = np.full(len(source), np.nan)
            depths[valid] = depth[rounded[valid, 1], rounded[valid, 0]]
            valid &= np.isfinite(depths) & (depths >= cfg.min_depth_m) & (depths <= cfg.max_depth_m)
            rays = np.column_stack((source, np.ones(len(source)))) @ np.linalg.inv(cameras[0].matrix).T
            production_xyz, production_pixels = depth_correspondences(source, target, depth, *cameras, cfg)
            np.testing.assert_allclose(production_xyz, rays[valid]*depths[valid, None], atol=1e-9)
            np.testing.assert_allclose(production_pixels, target[valid], atol=1e-9)

            # First truth access: estimation above is now finished and cannot consume it.
            head_base = np.array(pair['T_base_headsetWorld']) @ np.array(pair['T_world_headOptical'])
            wrist_base = np.array(pair['T_base_wristOptical'])
            truth = np.linalg.inv(wrist_base) @ head_base
            epipolar = epipolar_distances(source, target, cameras[0].matrix, cameras[1].matrix, truth)
            source_table = table_footprint(source, cameras[0].matrix, head_base)
            target_table = table_footprint(target, cameras[1].matrix, wrist_base)
            point_records = []
            for i, (pixel, matched, ray, nearest) in enumerate(zip(source, target, rays, depths)):
                p = {'index': i, 'headset_pixel': pixel.tolist(), 'wrist_pixel': matched.tolist(),
                     'eligible_depth': bool(valid[i]), 'nearest_depth_m': float(nearest),
                     'headset_table_footprint': bool(source_table[i]), 'wrist_table_footprint': bool(target_table[i]),
                     'true_wrist_epipolar_distance_pixels': float(epipolar[i]),
                     'depth_alone_cannot_explain_3px': bool(np.isfinite(epipolar[i]) and epipolar[i] > 3)}
                if valid[i]:
                    fit_error = project_depths(ray, [nearest], estimate.camera_from_headset, cameras[1].matrix, matched)[0]
                    truth_error = project_depths(ray, [nearest], truth, cameras[1].matrix, matched)[0]
                    p.update(fitted_reprojection_pixels=float(fit_error), fitted_outlier=bool(fit_error > 3),
                             nearest_truth_reprojection_pixels=float(truth_error))
                    for size in (3, 5):
                        metrics = neighborhood_metrics(depth, rounded[i], ray, matched, truth, cameras[1].matrix,
                                                       size, cfg.min_depth_m, cfg.max_depth_m)
                        metrics['span_exceeds_2cm'] = metrics.get('depth_span_m', 0) > .02
                        metrics['plausible_depth_boundary'] = bool(metrics['span_exceeds_2cm'] and truth_error > 3
                            and metrics.get('best_truth_reprojection_pixels', np.inf) <= 3
                            and np.isfinite(epipolar[i]) and epipolar[i] <= 3)
                        p[str(size)+'x'+str(size)] = metrics
                point_records.append(p)
            focused = [p for p in point_records if p.get('fitted_outlier') and p['headset_table_footprint'] and p['wrist_table_footprint']]
            record['all_points'] = point_records
            record['fitted_table_to_table_outlier_summary'] = {
                'count': len(focused),
                'epipolar_above_3px': sum(p['depth_alone_cannot_explain_3px'] for p in focused),
                'nearest_truth_error_above_3px': sum(p['nearest_truth_reprojection_pixels'] > 3 for p in focused),
                **{str(size)+'x'+str(size): {'span_exceeds_2cm': sum(p[str(size)+'x'+str(size)]['span_exceeds_2cm'] for p in focused),
                                          'plausible_depth_boundary': sum(p[str(size)+'x'+str(size)]['plausible_depth_boundary'] for p in focused)}
                   for size in (3, 5)}}
            report['captures'].append(record)
            print(json.dumps(finite_json({k: v for k, v in record.items() if k != 'all_points'})), flush=True)
        report['complete'] = len(report['captures']) == 3
    except (torch.cuda.OutOfMemoryError, RuntimeError, ValueError, AssertionError) as exc:
        report['stopped_reason'] = str(exc)
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(finite_json(report), indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
