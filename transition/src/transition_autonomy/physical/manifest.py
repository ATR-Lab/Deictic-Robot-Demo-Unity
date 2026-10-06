"""Strict, detached commissioning manifests; hashes identify, never authorize."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from types import MappingProxyType


def finite(value, name, *, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(f"{name} must be a finite {'positive ' if positive else ''}number")
    return float(value)


def exact(value, names, name):
    if not isinstance(value, dict) or set(value) != set(names.split()):
        raise ValueError(f"{name} requires exactly: {names}")


def text(value, name):
    if not isinstance(value, str) or not value.strip() or len(value) > 1024:
        raise ValueError(f"{name} must be a nonempty bounded string")
    return value


def freeze(value):
    if isinstance(value, dict):
        return MappingProxyType({key: freeze(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(freeze(item) for item in value)
    return value


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def strict_json(raw):
    if len(raw.encode('utf-8')) > 65536:
        raise ValueError('Manifest exceeds 64 KiB')
    return json.loads(raw, object_pairs_hook=_pairs,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite JSON')))


@dataclass(frozen=True)
class Manifest:
    payload: object
    canonical_json: str
    digest: str
    unresolved: tuple[str, ...]

    @classmethod
    def load(cls, path):
        return cls.parse(Path(path).read_text(encoding='utf-8'))

    @classmethod
    def parse(cls, raw):
        data = strict_json(raw)
        exact(data, 'schema_version deployment_id robot control_host sdk physical_actuation_enabled controller ownership observation profiles commissioning', 'manifest')
        if type(data['schema_version']) is not int or data['schema_version'] != 1:
            raise ValueError('Unsupported manifest schema')
        if type(data['physical_actuation_enabled']) is not bool:
            raise ValueError('physical_actuation_enabled must be boolean')
        sections = {
            'robot': 'serial firmware model',
            'control_host': 'host_id os_release python_abi cpu_architecture',
            'sdk': 'distribution version artifact_sha256 network_binding vendor_dds_domain robot_name',
            'controller': 'allowed_mode allowed_body_control support_record independent_stop_record',
            'ownership': 'enforcement_record watchdog_record',
            'observation': 'max_age_s max_gap_s max_skew_s max_transport_uncertainty_s max_arm_speed_rad_s calibration_digest',
            'profiles': 'point_a point_b home',
            'commissioning': 'receipt_id manifest_sha256 reviewer approved_at expires_at',
        }
        unresolved = []
        if data['deployment_id'] is None:
            unresolved.append('deployment_id')
        else:
            text(data['deployment_id'], 'deployment_id')
        for section, keys in sections.items():
            exact(data[section], keys, section)
            for name, value in data[section].items():
                key = section + '.' + name
                if value is None:
                    unresolved.append(key)
                    continue
                if section == 'profiles':
                    exact(value, 'hand position_m orientation_rpy duration_ms target_label position_tolerance_m settling_seconds', key)
                    if value['hand'] not in ('left', 'right'):
                        raise ValueError('Profile hand must be explicit')
                    if value['target_label'] != {'point_a': 'A', 'point_b': 'B', 'home': None}[name]:
                        raise ValueError('Profile target label does not match the reference task')
                    for vector in ('position_m', 'orientation_rpy'):
                        if not isinstance(value[vector], list) or len(value[vector]) != 3:
                            raise ValueError('Profile vector requires three numbers')
                        for number in value[vector]:
                            finite(number, vector)
                    if type(value['duration_ms']) is not int or value['duration_ms'] <= 0:
                        raise ValueError('duration_ms must be a positive integer')
                    finite(value['position_tolerance_m'], key, positive=True)
                    finite(value['settling_seconds'], key, positive=True)
                elif name in ('allowed_mode', 'allowed_body_control', 'vendor_dds_domain'):
                    if type(value) is not int or value < 0:
                        raise ValueError(key + ' must be a nonnegative integer')
                elif name.startswith('max_') or name in ('approved_at', 'expires_at'):
                    finite(value, key, positive=True)
                elif name == 'robot_name' and value == '':
                    pass  # Empty vendor namespace is an explicit value, not autodetection.
                else:
                    text(value, key)
                    if name.endswith('sha256') or name.endswith('digest'):
                        if len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
                            raise ValueError(key + ' must be a lowercase SHA-256')
        if data['robot']['model'] != 'K1':
            raise ValueError('Manifest must identify K1')
        if data['sdk']['distribution'] != 'booster_robotics_sdk_python' or data['sdk']['version'] != '1.6.3':
            raise ValueError('Unreviewed SDK distribution/version')
        # Canonicalization v1: Python JSON finite decimal serialization, Unicode
        # escaped, sorted keys, compact separators; entire commissioning excluded.
        payload = {key: value for key, value in data.items() if key != 'commissioning'}
        canonical = json.dumps(payload, sort_keys=True, separators=(',', ':'), ensure_ascii=True, allow_nan=False)
        digest = hashlib.sha256(canonical.encode('utf-8')).hexdigest()
        claimed = data['commissioning']['manifest_sha256']
        if claimed is not None and claimed != digest:
            raise ValueError('Commissioning digest mismatch')
        return cls(freeze(data), canonical, digest, tuple(unresolved))

    def require_complete(self):
        if self.unresolved:
            raise ValueError('Unresolved commissioning fields: ' + ', '.join(self.unresolved))
        if not self.payload['physical_actuation_enabled']:
            raise ValueError('Physical actuation disabled by manifest')
