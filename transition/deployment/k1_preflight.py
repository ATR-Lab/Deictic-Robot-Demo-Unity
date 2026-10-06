#!/usr/bin/env python3
"""Offline manifest diagnostic. Has no SDK import, connection, arm, or motion path."""
import argparse
import importlib.metadata
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from transition_autonomy.physical.manifest import Manifest


def inspect(path):
    try:
        manifest = Manifest.load(path)
    except (ValueError, OSError, TypeError) as error:
        return {'schema_version': 1, 'valid': False, 'physical_ready': False, 'reason': str(error)}
    try:
        sdk_version = importlib.metadata.version('booster_robotics_sdk_python')
    except importlib.metadata.PackageNotFoundError:
        sdk_version = None
    return {'schema_version': 1, 'valid': True, 'manifest_digest': manifest.digest,
            'unresolved': list(manifest.unresolved), 'installed_sdk_version': sdk_version,
            'physical_ready': False, 'connection_opened': False,
            'reason': 'Offline validation cannot establish commissioned owner, support, stop, sampling or approval authority'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = inspect(args.manifest)
    raw = json.dumps(result, indent=2, allow_nan=False)
    if args.output:
        args.output.write_text(raw + '\n', encoding='utf-8')
    print(raw)
    return 0 if result['valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
