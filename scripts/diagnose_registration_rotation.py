"""Development-only wrist quarter-turn diagnostic with original-camera PnP.

All four rotations are recorded for each of six declared development captures.
No live profile is selected, and simulator truth only scores finished estimates.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def original_wrist_pixels(rotated_pixels, quarter_turns, original_width, original_height):
    """Invert np.rot90's CCW pixel-center mapping, retaining subpixel coordinates."""
    points = np.asarray(rotated_pixels, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError('Expected an N by 2 array of pixel coordinates')
    if type(quarter_turns) is not int or not 0 <= quarter_turns <= 3:
        raise ValueError('Quarter turns must be an integer in [0, 3]')
    if original_width <= 0 or original_height <= 0:
        raise ValueError('Original image size must be positive')
    x, y = points[:, 0], points[:, 1]
    if quarter_turns == 0:
        return points.copy()
    if quarter_turns == 1:
        return np.column_stack((original_width - 1 - y, x))
    if quarter_turns == 2:
        return np.column_stack((original_width - 1 - x, original_height - 1 - y))
    return np.column_stack((y, original_height - 1 - x))


def check_quarter_turn_coordinates():
    """Check every pixel of a rectangular labeled image plus subpixel round trips."""
    height, width = 5, 8
    image = np.arange(height * width).reshape(height, width)
    subpixels = np.array([[0., 0.], [width-1., height-1.], [1.25, 2.75], [4.5, .125]])
    for turns in range(4):
        rotated = np.rot90(image, turns)
        row, col = np.indices(rotated.shape)
        original = original_wrist_pixels(np.column_stack((col.ravel(), row.ravel())), turns, width, height)
        np.testing.assert_array_equal(image[original[:, 1].astype(int), original[:, 0].astype(int)], rotated.ravel())
        # Forward mapping is independently expressed as repeated single CCW steps.
        forward = subpixels.copy()
        current_width, current_height = width, height
        for _ in range(turns):
            forward = np.column_stack((forward[:, 1], current_width-1-forward[:, 0]))
            current_width, current_height = current_height, current_width
        np.testing.assert_allclose(original_wrist_pixels(forward, turns, width, height), subpixels, atol=1e-12, rtol=0)
        assert original_wrist_pixels(np.empty((0, 2)), turns, width, height).shape == (0, 2)
    return {'passed': True, 'rotations': 4, 'integer_pixel_checks': 160, 'subpixel_roundtrip_checks': 16}


class RotatedWristMatcher:
    def __init__(self, frontend, quarter_turns):
        self.frontend = frontend
        self.quarter_turns = quarter_turns

    def match(self, headset, wrist):
        height, width = wrist.shape[:2]
        rotated = np.ascontiguousarray(np.rot90(wrist, self.quarter_turns))
        source, target = self.frontend.match(headset, rotated)
        # PnP sees ORIGINAL wrist pixels and intrinsics. Headset image/depth are untouched.
        return source, original_wrist_pixels(target, self.quarter_turns, width, height)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rest-development', type=Path, nargs=3)
    parser.add_argument('--end-development', type=Path, nargs=3)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    checks = check_quarter_turn_coordinates()
    if args.self_test:
        print(json.dumps(checks))
        return
    if args.rest_development is None or args.end_development is None or args.output is None:
        parser.error('Both three-capture development lists and --output are required')

    # Heavy learned dependencies are intentionally unnecessary for --self-test.
    import torch
    from select_simulation_registration_profile import FusionConfig, SuperPointMatcher, evaluate, load
    torch.set_num_threads(4)
    paths = args.rest_development + args.end_development
    loaded = [load(path) for path in paths]
    if len({item[0]['capture_wall_time_unix'] for item in loaded}) != 6:
        raise ValueError('All six development captures must have distinct timestamps')
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA unavailable; no alternate device/configuration substituted')
    free, total = torch.cuda.mem_get_info()
    if free < 2 * 1024**3:
        raise RuntimeError('Less than 2 GiB free GPU memory; diagnostic not started')
    report = {'scope': 'development-only quarter-turn augmentation diagnostic, not held-out validation',
              'coordinate_checks': checks, 'device': 'cuda', 'max_keypoints': 1024,
              'match_filter_threshold': .8, 'cpu_threads': 4, 'pytorch_allocator_fraction': .25,
              'free_gpu_bytes_before': free, 'total_gpu_bytes': total,
              'torch_version': torch.__version__, 'gpu': torch.cuda.get_device_name(),
              'selection_rule': 'no deployment/profile selection; all four rotations recorded; truth excluded from inference and quality comparisons',
              'records': [], 'complete': False}
    try:
        frontend = SuperPointMatcher('cuda', 1024, 4, .8)
        cfg = FusionConfig()
        # One model; warm both rectangular orientations before synchronized timing.
        for turns in (0, 1):
            RotatedWristMatcher(frontend, turns).match(*loaded[0][1])
        for index, (path, capture) in enumerate(zip(paths, loaded)):
            for turns in range(4):
                record = evaluate(RotatedWristMatcher(frontend, turns), path, capture, 1024, .8, cfg)
                record.update(wrist_rotation_ccw_degrees=90*turns,
                              pose_group='rest' if index < 3 else 'postreach',
                              pnp_pixel_frame='original_unrotated_wrist_image')
                report['records'].append(record)
        report['quality_pass_count_by_rotation'] = {
            str(angle): sum(r['confidence_pass'] for r in report['records'] if r['wrist_rotation_ccw_degrees'] == angle)
            for angle in (0, 90, 180, 270)}
        report['complete'] = len(report['records']) == 24
    except (torch.cuda.OutOfMemoryError, RuntimeError, ValueError) as exc:
        report['stopped_reason'] = str(exc)
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'complete': report['complete'], 'quality_pass_count_by_rotation': report.get('quality_pass_count_by_rotation'),
                      'stopped_reason': report.get('stopped_reason')}), flush=True)


if __name__ == '__main__':
    main()
