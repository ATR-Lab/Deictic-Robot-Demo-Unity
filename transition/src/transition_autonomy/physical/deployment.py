"""Trusted local assembly; no SDK construction, arming, or run-directory ledger."""
from __future__ import annotations

import hashlib
from pathlib import Path

from .manifest import text


def device_ledger_path(registry_root, serial):
    root = Path(registry_root)
    if not root.is_absolute():
        raise ValueError('Persistent device registry must be an absolute site-configured directory')
    digest = hashlib.sha256(text(serial, 'device serial').encode('utf-8')).hexdigest()
    return root.resolve() / ('device-' + digest) / 'adapter.sqlite'


def assemble_disarmed(manifest, supplied_transport, *, registry_root, release_gate, clock):
    from ..backends.booster_backend import BoosterK1Backend, K1Config, PointProfile
    manifest.require_complete()
    if release_gate.manifest.digest != manifest.digest:
        raise ValueError('Gate/assembly manifest mismatch')
    m = manifest.payload
    profiles = {key: PointProfile(**dict(value)) for key, value in m['profiles'].items()}
    config = K1Config(m['robot']['serial'], m['robot']['firmware'], m['controller']['allowed_mode'],
        m['controller']['allowed_body_control'], profiles, m['commissioning']['receipt_id'],
        physical_actuation_enabled=m['physical_actuation_enabled'],
        max_state_age=m['observation']['max_age_s'], max_arm_speed=m['observation']['max_arm_speed_rad_s'])
    # registry_root comes from trusted site deployment, never a run/UI parameter.
    return BoosterK1Backend(config, supplied_transport,
        device_ledger_path(registry_root, config.expected_serial), release_gate=release_gate, clock=clock)
