#!/usr/bin/env python3
"""Pinned SDK identity/status queries only, each in a deadline-bounded process.

Only the client's query RPC channels are initialized. No motion, stop, mode
change, controller wrapper or low-level command publisher is used. Success
reports read-only RPC connectivity, never physical release.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib
import importlib.metadata
import importlib.util
import json
import multiprocessing
from pathlib import Path
import platform
import sys
import time
import zipfile


DISTRIBUTION = 'booster_robotics_sdk_python'
VERSION = '1.6.3'
WHEEL_NAME = 'booster_robotics_sdk_python-1.6.3-cp312-cp312-manylinux_2_34_x86_64.whl'
WHEEL_SHA256 = '144d50f13ddbe31de56f508df615d070352eebb2ca439be8f6f3723d98067ebe'
WHEEL_URL = 'https://files.pythonhosted.org/packages/b7/66/f3a92eca4057b673def81ec2749fadb015c57536114909fb2db5ce1f50e5/' + WHEEL_NAME
READ_ONLY_QUERIES = ('GetRobotInfo', 'GetStatus')
# Confirmed Python binding fields. The newer C++ response header also mentions
# edition/region, but the pinned 1.6.3 Python response does not expose them.
IDENTITY_FIELDS = ('name', 'nickname', 'version', 'model', 'serial_number')


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for chunk in iter(lambda: source.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def verify_install(wheel):
    """Inspect metadata/files before importing or initializing SDK transport."""
    wheel = Path(wheel).resolve(strict=True)
    if wheel.name != WHEEL_NAME or sha256_file(wheel) != WHEEL_SHA256:
        raise ValueError('Wheel filename/hash does not match the inspected CPython 3.12 artifact')
    if sys.version_info[:2] != (3, 12) or platform.system() != 'Linux' or platform.machine() != 'x86_64':
        raise ValueError('Pinned artifact requires Linux x86_64 CPython 3.12')
    distribution = importlib.metadata.distribution(DISTRIBUTION)
    version = distribution.version
    if version != VERSION:
        raise ValueError('Installed SDK metadata is missing or is not version 1.6.3')
    verified = verify_wheel_files(wheel, distribution)
    return {'distribution': DISTRIBUTION, 'version': version, 'wheel': str(wheel),
            'wheel_sha256': WHEEL_SHA256, 'python': sys.version, 'machine': platform.machine(), **verified}


def verify_wheel_files(wheel, distribution, *, find_spec=importlib.util.find_spec):
    """Hash wrappers, extension modules, bundled libraries and other wheel files.

    SDK 1.6.3 is a package, not a flat extension: __init__.py loads _core and
    defines controller wrapper classes. Nothing is imported during verification.
    Pip rewrites RECORD and generates entrypoint scripts; RECORD is therefore
    not compared, while every actual wheel payload file is compared directly.
    """
    verified = {}
    native_extensions = []
    with zipfile.ZipFile(wheel) as archive:
        names = [entry.filename for entry in archive.infolist() if not entry.is_dir()]
        if len(set(names)) != len(names):
            raise ValueError('Duplicate wheel paths')
        for name in names:
            member = Path(name)
            if member.is_absolute() or '..' in member.parts:
                raise ValueError('Invalid wheel member path')
            if name.endswith('.dist-info/RECORD'):
                continue
            installed = Path(distribution.locate_file(name)).resolve(strict=True)
            expected = hashlib.sha256(archive.read(name)).hexdigest()
            if sha256_file(installed) != expected:
                raise ValueError('Installed wheel file differs from reviewed artifact: ' + name)
            verified[name] = {'path': str(installed), 'sha256': expected}
            if member.suffix == '.so':
                native_extensions.append(name)
    expected_init = DISTRIBUTION + '/__init__.py'
    if expected_init not in verified or not native_extensions:
        raise ValueError('Reviewed package wrapper/native modules are missing')
    # Verify top-level import resolution too: PYTHONPATH must not shadow the
    # installed package or its top-level internal extension. Looking up a
    # submodule would import __init__, so submodule files are checked above only.
    modules = {DISTRIBUTION: expected_init}
    for name in native_extensions:
        if '/' not in name:
            modules[Path(name).name.split('.')[0]] = name
    for module, member in modules.items():
        spec = find_spec(module)
        if spec is None or not spec.origin or str(Path(spec.origin).resolve()) != verified[member]['path']:
            raise ValueError('SDK module is shadowed or resolves outside verified installation: ' + module)
    return {'package_path': str(Path(verified[expected_init]['path']).parent),
            'verified_file_count': len(verified), 'verified_files': verified,
            'native_extensions': native_extensions, 'record_rewritten_by_installer': True}


def _string_field(response, field):
    value = getattr(response, field)
    if not isinstance(value, str) or len(value) > 512:
        raise ValueError('Invalid identity response field: ' + field)
    return value


def query_using_sdk(sdk, query, *, binding, domain_id, robot_name, discovery_wait_s):
    """Explicit allowlist: there is no user-selected method or raw RPC API."""
    if query not in READ_ONLY_QUERIES:
        raise ValueError('Only GetRobotInfo and GetStatus are permitted')
    sdk.ChannelFactory.Instance().Init(domain_id, binding)
    client = sdk.B1LocoClient()
    if robot_name:
        client.InitWithName(robot_name)
    else:
        client.Init()
    # Discovery settling is bounded separately; no vendor initialization demo.
    time.sleep(discovery_wait_s)
    started = time.monotonic()
    if query == 'GetRobotInfo':
        response = client.GetRobotInfo()
        result = {field: _string_field(response, field) for field in IDENTITY_FIELDS}
        if not result['serial_number'] or not result['version'] or not result['model']:
            raise ValueError('Identity reply contains no usable serial/firmware/model')
    else:
        response = client.GetStatus()
        actions = list(response.current_actions)
        if len(actions) > 128:
            raise ValueError('Status action list exceeds bound')
        result = {'current_mode': int(response.current_mode),
                  'current_body_control': int(response.current_body_control),
                  'current_actions': [int(action) for action in actions]}
    ended = time.monotonic()
    return {'ok': True, 'query': query, 'response': result,
            'request_start_monotonic': started, 'response_end_monotonic': ended,
            'rpc_elapsed_s': ended-started, 'acquisition_timestamp_known': False}


def _query_worker(connection, query, settings):
    try:
        sdk = importlib.import_module(DISTRIBUTION)
        connection.send(query_using_sdk(sdk, query, **settings))
    except Exception as error:
        connection.send({'ok': False, 'query': query, 'error_type': type(error).__name__,
                         'error': str(error)[:2048],
                         'interpretation': 'Read-only API unavailable, rejected, malformed or transport failure; no firmware support inferred'})
    finally:
        connection.close()


def bounded_query(query, settings, timeout_s, *, worker=_query_worker):
    if query not in READ_ONLY_QUERIES:
        raise ValueError('Query is not read-only')
    context = multiprocessing.get_context('spawn')
    parent, child = context.Pipe(duplex=False)
    process = context.Process(target=worker, args=(child, query, settings), daemon=True)
    started = time.monotonic()
    process.start(); child.close()
    try:
        if parent.poll(timeout_s):
            try:
                result = parent.recv()
            except EOFError:
                result = {'ok': False, 'query': query, 'error': 'Diagnostic worker exited without a result'}
        else:
            result = {'ok': False, 'query': query, 'error_type': 'TimeoutError',
                      'error': 'Read-only diagnostic worker exceeded its deadline; it was terminated, not retried'}
    finally:
        # A read-only query can be discarded; no delivered motion RPC exists.
        process.join(.1)
        if process.is_alive():
            process.terminate(); process.join(.3)
        if process.is_alive():
            process.kill(); process.join(.3)
        parent.close()
    result['worker_elapsed_s'] = time.monotonic()-started
    result['worker_exitcode'] = process.exitcode
    return result


def main():
    if (Path(__file__).resolve().parents[1]/'.runtime/k1-diagnostics-hold.json').exists():
        raise SystemExit('K1 diagnostics held after the 2026-09-24 memory/reset incident; see docs/K1_RESET_INCIDENT.md')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wheel', required=True, type=Path)
    parser.add_argument('--network-binding', required=True, help='Explicit verified NIC/address; no autodetection')
    parser.add_argument('--domain-id', required=True, type=int)
    parser.add_argument('--robot-name', required=True, help='Exact vendor RPC suffix; empty selects default robot')
    parser.add_argument('--timeout-s', type=float, default=6.)
    parser.add_argument('--discovery-wait-s', type=float, default=.75)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if (not args.network_binding.strip() or len(args.network_binding) > 256 or len(args.robot_name) > 256
            or not 0 <= args.domain_id <= 232 or not 1 <= args.timeout_s <= 15
            or not 0 <= args.discovery_wait_s <= 2 or args.discovery_wait_s >= args.timeout_s):
        parser.error('Invalid explicit connection setting or deadline')
    # Confirm the evidence destination before connecting; never overwrite receipts.
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as receipt:
        report = {'schema_version': 1, 'mode': 'read_only_sdk_diagnostics', 'physical_ready': False,
                  'motion_capability': False, 'read_only_rpc_connected': False,
                  'network_binding': args.network_binding, 'domain_id': args.domain_id,
                  'robot_name': args.robot_name, 'query_allowlist': list(READ_ONLY_QUERIES), 'results': []}
        try:
            report['installation'] = verify_install(args.wheel)
            settings = dict(binding=args.network_binding, domain_id=args.domain_id,
                            robot_name=args.robot_name, discovery_wait_s=args.discovery_wait_s)
            report['results'] = [bounded_query(query, settings, args.timeout_s) for query in READ_ONLY_QUERIES]
            report['read_only_rpc_connected'] = all(result.get('ok') is True for result in report['results'])
            report['identity_matches_k1'] = any(result.get('ok') is True and result['query'] == 'GetRobotInfo'
                and result['response']['model'] in ('K1', 'Booster K1') for result in report['results'])
        except Exception as error:
            report['preflight_error'] = type(error).__name__ + ': ' + str(error)
        receipt.write(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report['read_only_rpc_connected'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
