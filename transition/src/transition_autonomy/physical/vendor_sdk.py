"""Capability-limited pinned SDK ports for a supervised, commissioned gateway.

These are synchronous vendor calls, not an owner/expiry-enforcing device service.
Run each port in a separately supervised process. A returned RPC is not measured
completion, an emergency stop, or containment of an already submitted request.
No controller wrapper, mode change, low-level publisher, or generic API is exposed.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import importlib
import importlib.metadata
import importlib.util
import json
import math
from pathlib import Path
import platform
import re
import sys
import time
import zipfile

from .manifest import Manifest, text


DISTRIBUTION = 'booster_robotics_sdk_python'
SDK_VERSION = '1.6.3'
WHEEL_NAME = 'booster_robotics_sdk_python-1.6.3-cp312-cp312-manylinux_2_34_x86_64.whl'
WHEEL_SHA256 = '144d50f13ddbe31de56f508df615d070352eebb2ca439be8f6f3723d98067ebe'
IDENTITY_FIELDS = ('name', 'nickname', 'version', 'model', 'serial_number')
MOTION_REFERENCE = 'https://docs.booster.tech/docs/developer-guide/cpp/rpc/motion/'


def _sha(path):
    with Path(path).open('rb') as source:
        return hashlib.file_digest(source, 'sha256').hexdigest()


def load_pinned_sdk(wheel_path):
    """Verify the complete installed wheel before importing any vendor code."""
    wheel = Path(wheel_path).resolve(strict=True)
    if wheel.name != WHEEL_NAME or _sha(wheel) != WHEEL_SHA256:
        raise ValueError('Unreviewed SDK wheel filename/hash')
    if sys.version_info[:2] != (3, 12) or platform.system() != 'Linux' or platform.machine() != 'x86_64':
        raise ValueError('SDK artifact requires Linux x86_64 CPython 3.12')
    distribution = importlib.metadata.distribution(DISTRIBUTION)
    if distribution.version != SDK_VERSION:
        raise ValueError('Installed SDK version mismatch')
    files = {}
    with zipfile.ZipFile(wheel) as archive:
        for entry in archive.infolist():
            if entry.is_dir():
                continue
            name, member = entry.filename, Path(entry.filename)
            if member.is_absolute() or '..' in member.parts or name in files:
                raise ValueError('Invalid or duplicate wheel member')
            installed = Path(distribution.locate_file(name)).resolve(strict=True)
            files[name] = installed
            if not name.endswith('.dist-info/RECORD') and _sha(installed) != hashlib.sha256(archive.read(name)).hexdigest():
                raise ValueError('Installed SDK differs from pinned wheel: ' + name)
    wrapper = DISTRIBUTION + '/__init__.py'
    native = [name for name in files if name.endswith('.so')]
    if wrapper not in files or not native:
        raise ValueError('Pinned package wrapper/native extension missing')
    modules = {DISTRIBUTION: wrapper}
    modules.update({Path(name).name.split('.')[0]: name for name in native if '/' not in name})
    for module, name in modules.items():
        found = importlib.util.find_spec(module)
        if found is None or not found.origin or Path(found.origin).resolve() != files[name]:
            raise ValueError('SDK import shadowed: ' + module)
    # Importing definitions is harmless; constructing ArmController would not be.
    return importlib.import_module(DISTRIBUTION)


def firmware_supports_endpoint(version):
    """The official per-method table records minimum firmware v1.3.1.1."""
    match = re.fullmatch(r'v?(\d+)\.(\d+)\.(\d+)(?:\.(\d+))?(?:-[A-Za-z0-9._-]+)?', version)
    if not match:
        return False
    return tuple(int(value or 0) for value in match.groups()) >= (1, 3, 1, 1)


@dataclass(frozen=True)
class ConnectionSettings:
    wheel_path: str
    network_binding: str
    domain_id: int
    robot_name: str
    expected_serial: str
    expected_firmware: str

    def __post_init__(self):
        for field in ('wheel_path', 'network_binding', 'expected_serial', 'expected_firmware'):
            text(getattr(self, field), field)
        if type(self.domain_id) is not int or not 0 <= self.domain_id <= 232:
            raise ValueError('Explicit DDS domain must be between 0 and 232')
        if not isinstance(self.robot_name, str) or len(self.robot_name) > 128 or '/' in self.robot_name:
            raise ValueError('Invalid explicit robot namespace')


def _connect(settings, loader):
    sdk = loader(settings.wheel_path)
    sdk.ChannelFactory.Instance().Init(settings.domain_id, settings.network_binding)
    client = sdk.B1LocoClient()
    if settings.robot_name:
        client.InitWithName(settings.robot_name)
    else:
        client.Init()
    return sdk, client


def _identity(client, settings):
    response = client.GetRobotInfo()
    result = {name: getattr(response, name) for name in IDENTITY_FIELDS}
    if any(not isinstance(value, str) or len(value) > 512 for value in result.values()):
        raise ValueError('Invalid SDK identity response')
    if (result['model'] not in ('K1', 'Booster K1') or result['serial_number'] != settings.expected_serial
            or result['version'] != settings.expected_firmware):
        raise RuntimeError('Connected robot identity/firmware differs from deployment')
    return result


def _status(client):
    response = client.GetStatus()
    # pybind enums implement int(); bools and floating-point lookalikes are invalid.
    def enum(value):
        if isinstance(value, (bool, float, str)):
            raise ValueError('Invalid SDK status enum')
        return int(value)
    actions = tuple(enum(value) for value in response.current_actions)
    if len(actions) > 64:
        raise ValueError('Invalid SDK action list')
    return dict(current_mode=enum(response.current_mode),
                current_body_control=enum(response.current_body_control), current_actions=actions)


def _receipt(operation, started, **fields):
    return dict(operation=operation, rpc_returned=True, started_at=started, returned_at=time.monotonic(),
                execution_complete=False, pending_work_contained=False, **fields)


class _Port:
    enforcement_record = None
    pending_work_contained = False

    def __init__(self, settings, *, loader=load_pinned_sdk):
        self._settings = settings
        self._sdk, self._client = _connect(settings, loader)
        self._closed = False

    def _check_open(self):
        if self._closed:
            raise RuntimeError('Vendor port is closed')

    def close(self):
        # No SDK Close/stop/mode method is assumed. The supervisor must terminate
        # this dedicated process to dispose of the process-global DDS factory.
        self._closed = True
        self._client = None
        self._sdk = None


class QueryPort(_Port):
    def query_identity(self):
        self._check_open()
        started = time.monotonic()
        return _receipt('GetRobotInfo', started, response=_identity(self._client, self._settings),
                        acquisition_timestamp_known=False)

    def query_status(self):
        self._check_open()
        started = time.monotonic()
        return _receipt('GetStatus', started, response=_status(self._client), acquisition_timestamp_known=False)

    def query_hand_positions(self):
        self._check_open()
        results = {}
        for hand, frame in (('left', self._sdk.Frame.kLeftHand), ('right', self._sdk.Frame.kRightHand)):
            started = time.monotonic()
            p = self._client.GetFrameTransform(self._sdk.Frame.kBody, frame).position
            values = tuple(float(value) for value in (p.x, p.y, p.z))
            if not all(math.isfinite(value) for value in values):
                raise ValueError('Nonfinite SDK hand transform')
            results[hand] = _receipt('GetFrameTransform', started, position_m=values,
                                     frame='body', acquisition_timestamp_known=False)
        # Two sequential RPCs are not a synchronized sensor bundle.
        return dict(hands=results, synchronized=False, source_progression_verified=False)


class StopPort(_Port):
    def stop(self):
        self._check_open()
        started = time.monotonic()
        result = self._client.StopHandEndEffector()
        # Pinned Python bindings return None on success and raise on RPC error.
        if result is not None:
            raise RuntimeError('Unexpected pinned SDK stop return; outcome unknown')
        return _receipt('StopHandEndEffector', started, measured_stop_confirmed=False)

    request_stop = stop


def profile_digest(manifest, profile_id):
    if profile_id not in ('point_a', 'point_b', 'home'):
        raise ValueError('Only reviewed point_a, point_b and home profiles are supported')
    profile = manifest.payload['profiles'][profile_id]
    if profile is None:
        raise ValueError('Named profile is not commissioned')
    raw = json.dumps(dict(profile), sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(raw.encode()).hexdigest()


def _bind_manifest(settings, manifest):
    manifest.require_complete()
    robot, sdk = manifest.payload['robot'], manifest.payload['sdk']
    if (robot['serial'] != settings.expected_serial or robot['firmware'] != settings.expected_firmware
            or sdk['artifact_sha256'] != WHEEL_SHA256 or sdk['network_binding'] != settings.network_binding
            or sdk['vendor_dds_domain'] != settings.domain_id or sdk['robot_name'] != settings.robot_name):
        raise ValueError('SDK connection does not match immutable manifest')
    if not firmware_supports_endpoint(settings.expected_firmware):
        raise ValueError('Firmware does not meet documented endpoint API minimum')


class ProfileMotionPort(_Port):
    def __init__(self, settings, manifest_json, *, loader=load_pinned_sdk):
        self._manifest = Manifest.parse(manifest_json)
        _bind_manifest(settings, self._manifest)
        super().__init__(settings, loader=loader)

    def move_profile(self, profile_id, expected_profile_digest, *, before_dispatch):
        """Exactly one RPC after a trusted local gateway's final admission check.

        before_dispatch MUST synchronously recheck authority/owner/deadline and
        return True. It runs after potentially slow identity/status reads. This
        local callback is not a substitute for independently commissioned fencing.
        """
        self._check_open()
        actual = profile_digest(self._manifest, profile_id)
        if actual != expected_profile_digest or not callable(before_dispatch):
            raise ValueError('Profile digest or final admission callback invalid')
        profile = self._manifest.payload['profiles'][profile_id]
        _identity(self._client, self._settings)
        status = _status(self._client)
        controller = self._manifest.payload['controller']
        if (status['current_mode'] != controller['allowed_mode']
                or status['current_body_control'] != controller['allowed_body_control']
                or status['current_actions']):
            raise RuntimeError('Unexpected mode/body control or active competing action')
        sdk = self._sdk
        # Match the pinned wheel's Python example, whose Posture is default-
        # constructed with public fields (do not assume C++ constructors bind).
        target = sdk.Posture()
        target.position = sdk.Position(*profile['position_m'])
        target.orientation = sdk.Orientation(*profile['orientation_rpy'])
        hand = sdk.HandIndex.kLeftHand if profile['hand'] == 'left' else sdk.HandIndex.kRightHand
        if before_dispatch() is not True:
            raise RuntimeError('Final gateway admission denied')
        started = time.monotonic()
        result = self._client.MoveHandEndEffectorV2(target, profile['duration_ms'], hand)
        if result is not None:
            raise RuntimeError('Unexpected pinned SDK motion return; outcome unknown')
        return _receipt('MoveHandEndEffectorV2', started, profile_id=profile_id,
                        profile_digest=actual, manifest_digest=self._manifest.digest)


@dataclass(frozen=True)
class QueryFactory:
    settings: ConnectionSettings

    def __call__(self):
        return QueryPort(self.settings)


@dataclass(frozen=True)
class StopFactory:
    settings: ConnectionSettings

    def __call__(self):
        return StopPort(self.settings)


@dataclass(frozen=True)
class ProfileMotionFactory:
    settings: ConnectionSettings
    manifest_json: str

    def __call__(self):
        return ProfileMotionPort(self.settings, self.manifest_json)
