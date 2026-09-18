"""Evaluate a frozen GPU profile on three new rest and three new end captures.

No model or confidence settings are selected using these validation images.
"""
import argparse
import json
from pathlib import Path

import torch

from select_simulation_registration_profile import (
    FusionConfig, SuperPointMatcher, evaluate, load,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('selection', type=Path)
    parser.add_argument('--rest', type=Path, nargs=3, required=True)
    parser.add_argument('--postreach', type=Path, nargs=3, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    selection = json.loads(args.selection.read_text())
    profile = selection.get('selected')
    if not profile or profile.get('device') != 'cuda':
        raise ValueError('A successful CUDA development selection is required')
    frozen = selection.get('selection_frozen_at_unix')
    if frozen is None:
        raise ValueError('Development report must record the configuration freeze timestamp')
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA unavailable; no alternative profile will be substituted')
    torch.cuda.set_per_process_memory_fraction(.25)
    torch.set_num_threads(4)
    free, total = torch.cuda.mem_get_info()
    if free < 2 * 1024**3:
        raise RuntimeError('Less than 2 GiB free GPU memory; validation not started')
    paths = args.rest + args.postreach
    loaded = [load(path) for path in paths]
    stamps = [data[0]['capture_wall_time_unix'] for data in loaded]
    if len(set(stamps)) != 6 or not all(stamp > frozen for stamp in stamps):
        raise ValueError('All six captures must have distinct stamps after configuration freeze')
    development_stamps = {pose['stamp'] for trial in selection['trials'] for pose in trial['poses']}
    if development_stamps.intersection(stamps):
        raise ValueError('Development captures cannot serve as held-out validation')
    count, threshold = profile['max_keypoints'], profile['match_filter_threshold']
    frontend = SuperPointMatcher('cuda', count, 4, threshold)
    cfg = FusionConfig()
    # Exclude initialization. Warm-up does not inspect or select any quality result.
    frontend.match(*loaded[0][1])
    frontend.match(*loaded[3][1])
    report = {'scope': 'six fresh static captures at two fixed poses in one synthetic workcell',
              'selection_report': str(args.selection), 'frozen_profile': profile,
              'selection_frozen_at_unix': frozen,
              'pytorch_allocator_fraction': .25, 'free_gpu_bytes_before': free,
              'total_gpu_bytes': total, 'validation': [], 'all_validation_pass': False}
    try:
        for index, (path, data) in enumerate(zip(paths, loaded)):
            record = evaluate(frontend, path, data, count, threshold, cfg)
            record['pose_group'] = 'rest' if index < 3 else 'postreach'
            score = record.get('scoring_only', {})
            record['passes_static_check'] = bool(record['confidence_pass']
                and score.get('five_target_max_grounding_error_m', float('inf')) < .03
                and score.get('translation_error_m', float('inf')) < .03)
            report['validation'].append(record)
        report['all_validation_pass'] = all(r['passes_static_check'] for r in report['validation'])
    except (torch.cuda.OutOfMemoryError, RuntimeError, ValueError) as exc:
        report['stopped_reason'] = str(exc)
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'all_validation_pass': report['all_validation_pass'],
                      'stopped_reason': report.get('stopped_reason')}), flush=True)


if __name__ == '__main__':
    main()
