"""Guarded K1 reaching adapter; never configures gait, balance, or joint control.

The SDK boundary is verified against booster_robotics_sdk_python 1.6.3. Importing
this module opens no network connection. Real connection is an explicit factory.
Tests using a fake transport prove the protocol only, not physical readiness.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import fcntl
import importlib
import importlib.metadata
import json
import math
from pathlib import Path
import sqlite3
import threading
import time
from typing import Protocol
from types import MappingProxyType
import uuid

from .base import BackendEvent, FactUpdate, Observation, SkillCommand
from ..physical.manifest import finite, text

SDK_VERSION = "1.6.3"
SDK_SOURCE_REVISION = "87a9a26d06a94ddd50968e7576f1170f6fa5a80f"


@dataclass(frozen=True)
class PointProfile:
    hand: str
    position_m: tuple[float, float, float]
    orientation_rpy: tuple[float, float, float]
    duration_ms: int
    target_label: str | None
    position_tolerance_m: float
    settling_seconds: float

    def __post_init__(self):
        object.__setattr__(self, 'position_m', tuple(self.position_m))
        object.__setattr__(self, 'orientation_rpy', tuple(self.orientation_rpy))

    def validate(self) -> None:
        values = (*self.position_m, *self.orientation_rpy,
                  self.position_tolerance_m, self.settling_seconds)
        if (self.hand not in {"left", "right"} or len(self.position_m) != 3
                or len(self.orientation_rpy) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)
                or type(self.duration_ms) is not int or self.duration_ms <= 0 or self.position_tolerance_m <= 0
                or self.settling_seconds <= 0):
            raise ValueError("Invalid approved pointing profile")


@dataclass(frozen=True)
class K1Config:
    expected_serial: str
    expected_firmware: str
    allowed_mode: int
    allowed_body_control: int
    profiles: dict[str, PointProfile]
    commissioning_receipt: str
    physical_actuation_enabled: bool = False
    max_state_age: float = 0.25
    max_arm_speed: float = 0.05

    def __post_init__(self):
        object.__setattr__(self, 'profiles', MappingProxyType(dict(self.profiles)))

    def to_dict(self):
        return {**{name: getattr(self, name) for name in self.__dataclass_fields__ if name != 'profiles'},
                'profiles': {name: asdict(profile) for name, profile in self.profiles.items()}}

    def validate(self) -> None:
        for field in ('expected_serial', 'expected_firmware', 'commissioning_receipt'):
            text(getattr(self, field), field)
        if set(self.profiles) != {'point_a', 'point_b', 'home'}:
            raise ValueError("All approved point_a, point_b and home profiles are required")
        if type(self.physical_actuation_enabled) is not bool or type(self.allowed_mode) is not int or type(self.allowed_body_control) is not int:
            raise ValueError('Invalid enable/mode/body-control types')
        finite(self.max_state_age, 'Freshness threshold', positive=True)
        finite(self.max_arm_speed, 'Stationary threshold', positive=True)
        for name, profile in self.profiles.items():
            profile.validate()
            if profile.target_label != {'point_a': 'A', 'point_b': 'B', 'home': None}[name]:
                raise ValueError('Profile target label mismatch')


@dataclass(frozen=True)
class K1Sample:
    sampled_at: float
    serial: str
    firmware: str
    model: str
    mode: int
    body_control: int
    arm_positions: tuple[float, ...]
    arm_velocities: tuple[float, ...]
    hand_positions: dict[str, tuple[float, float, float]]
    motor_lost: bool = False
    # Site monitor must verify support/workcell, owner and commissioned stop path.
    site_ready: bool = False
    timestamp_source: str = "source_monotonic"
    bundle: object | None = None

    def __post_init__(self):
        object.__setattr__(self, 'arm_positions', tuple(self.arm_positions))
        object.__setattr__(self, 'arm_velocities', tuple(self.arm_velocities))
        object.__setattr__(self, 'hand_positions', MappingProxyType({key: tuple(value) for key, value in self.hand_positions.items()}))


class K1Transport(Protocol):
    def read(self) -> K1Sample: ...
    def move(self, profile: PointProfile) -> None: ...
    def stop(self) -> None: ...


class BoosterK1Backend:
    """Single-owner durable command boundary, disabled until explicitly armed.

    A nonterminal ledger entry at restart is never replayed or auto-cleared.
    Deployment must reconcile its physical effects outside the running adapter.
    The local file lock is additional protection, not a DDS-wide owner lease.
    """

    name = "booster_k1"

    def __init__(self, config: K1Config, transport: K1Transport, journal: str | Path,
                 *, release_gate=None, clock=None):
        config.validate()
        self.config, self.transport = config, transport
        self.release_gate, self.clock = release_gate, clock
        self._owner_epoch = None
        self._closed = False
        self.measured_stillness = False
        self.stop_confirmed = False
        self._last_bundle = None
        self._admission = None
        self._dispatch_uncertain = False
        if release_gate is not None:
            self._check_manifest_binding()
        path = Path(journal)
        path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = open(str(path) + ".lock", "a+")
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self._lock.close()
            raise RuntimeError("K1 adapter journal already has an owner") from None
        # Construction may occur on the main thread before ownership transfers
        # to the console's serialized runtime worker. This is not permission for
        # concurrent adapter calls; Backend retains its single-writer contract.
        try:
            self._db = sqlite3.connect(path, check_same_thread=False)
            self._db.execute("PRAGMA synchronous=FULL")
            self._db.execute("CREATE TABLE IF NOT EXISTS commands (id TEXT PRIMARY KEY, payload TEXT NOT NULL, status TEXT NOT NULL, at REAL NOT NULL, detail TEXT NOT NULL)")
            self._db.execute("CREATE TABLE IF NOT EXISTS events (sequence INTEGER PRIMARY KEY AUTOINCREMENT, command_id TEXT NOT NULL, receipt TEXT NOT NULL)")
            self._db.execute("CREATE TABLE IF NOT EXISTS device (serial TEXT NOT NULL)")
            identity = self._db.execute('SELECT serial FROM device').fetchone()
            if identity is not None and identity[0] != config.expected_serial:
                self._db.close()
                self._lock.close()
                raise RuntimeError('Device ledger belongs to another robot')
            if identity is None:
                self._db.execute('INSERT INTO device VALUES(?)', (config.expected_serial,))
            self._db.commit()
        except Exception:
            if hasattr(self, '_db'):
                self._db.close()
            self._lock.close()
            raise
        self.boot_id = str(uuid.uuid4())
        self._active: SkillCommand | None = None
        self._profile: PointProfile | None = None
        self._completed_profile: PointProfile | None = None
        self._settling_since: float | None = None
        self._last_poll_sample: float | None = None
        self._stop_requested = False
        self._armed = False
        unresolved = self._db.execute("SELECT 1 FROM commands WHERE status NOT IN ('succeeded','failed','canceled') LIMIT 1").fetchone()
        self._fault: str | None = "Startup reconciliation required" if unresolved else None

    def close(self) -> None:
        """Close only after shutdown/reconciliation. Closing never proves a stop."""
        if self._closed:
            return
        if self._active is not None:
            raise RuntimeError('Active/ambiguous command: retain monitoring and device ledger; call shutdown')
        self._armed = False
        self._db.close()
        self._lock.close()
        self._closed = True

    def _check_manifest_binding(self):
        m, c = self.release_gate.manifest.payload, self.config
        if (m['robot']['serial'] != c.expected_serial or m['robot']['firmware'] != c.expected_firmware
                or m['controller']['allowed_mode'] != c.allowed_mode
                or m['controller']['allowed_body_control'] != c.allowed_body_control
                or m['physical_actuation_enabled'] != c.physical_actuation_enabled
                or m['commissioning']['receipt_id'] != c.commissioning_receipt
                or m['observation']['max_age_s'] != c.max_state_age
                or m['observation']['max_arm_speed_rad_s'] != c.max_arm_speed):
            raise ValueError('Adapter configuration does not match reviewed manifest')
        for name, profile in c.profiles.items():
            if any(getattr(profile, key) != value for key, value in m['profiles'][name].items()):
                raise ValueError('Adapter profile does not match reviewed manifest')

    def _now_after(self, now, started):
        return self.clock() if self.clock is not None else now + time.monotonic()-started

    def _stationary(self, sample):
        uncertainty = sample.bundle.velocity_uncertainty_rad_s if sample.bundle else 0.
        return max(map(abs, sample.arm_velocities)) + uncertainty <= self.config.max_arm_speed

    def _reached(self, sample, profile):
        uncertainty = sample.bundle.position_uncertainty_m if sample.bundle else 0.
        return math.dist(sample.hand_positions[profile.hand], profile.position_m) + uncertainty <= profile.position_tolerance_m

    def _payload(self, command: SkillCommand) -> str:
        return json.dumps({"command": command.to_dict(), "profile_config": self.config.to_dict(),
                          'manifest_digest': self.release_gate.manifest.digest if self.release_gate else None},
                          sort_keys=True, separators=(",", ":"), allow_nan=False)

    def _record(self, command: SkillCommand, status: str, now: float, detail: str,
                *, facts: dict[str, FactUpdate] | None = None, evidence: dict | None = None) -> BackendEvent:
        event = BackendEvent(command.command_id, status, now, facts=facts or {}, detail=detail, evidence=evidence or {})
        self._db.execute("INSERT INTO commands VALUES(?,?,?,?,?) ON CONFLICT(id) DO UPDATE SET status=excluded.status,at=excluded.at,detail=excluded.detail",
                         (command.command_id, self._payload(command), status, now, detail))
        self._db.execute("INSERT INTO events(command_id,receipt) VALUES(?,?)",
                         (command.command_id, json.dumps(asdict(event), sort_keys=True, allow_nan=False)))
        self._db.commit()
        return event

    def _checked_sample(self, now: float, *, require_ready=True) -> K1Sample:
        read_start = time.monotonic()
        s = self.transport.read()
        checked_now = self._now_after(now, read_start)
        c = self.config
        if s.serial != c.expected_serial or s.firmware != c.expected_firmware or s.model not in {"K1", "Booster K1"}:
            raise RuntimeError("Robot identity/firmware mismatch")
        if type(s.sampled_at) not in (int, float) or not math.isfinite(s.sampled_at) or not 0 <= checked_now - s.sampled_at <= c.max_state_age:
            raise RuntimeError("Stale/invalid measured state")
        if len(s.arm_positions) != 8 or len(s.arm_velocities) != 8:
            raise RuntimeError("K1 measured arm layout mismatch")
        if not all(type(x) in (int, float) and math.isfinite(x) for x in (*s.arm_positions, *s.arm_velocities)):
            raise RuntimeError("Nonfinite measured state")
        if s.motor_lost is not False or (require_ready and (s.mode != c.allowed_mode or s.body_control != c.allowed_body_control or s.site_ready is not True)):
            raise RuntimeError("K1 mode/state/site readiness failed")
        if set(s.hand_positions) != {'left', 'right'}:
            raise RuntimeError('Both measured hand transforms are required')
        for p in s.hand_positions.values():
            if len(p) != 3 or not all(type(x) in (int, float) and math.isfinite(x) for x in p):
                raise RuntimeError("Invalid hand transform")
        if self.release_gate is not None:
            bundle = s.bundle
            if bundle is None:
                raise RuntimeError('Observation evidence bundle missing')
            bundle.validate(checked_now, self.release_gate.manifest.payload['observation'], require_registration=require_ready)
            if self._last_bundle is not None:
                previous = self._last_bundle
                if bundle.boot_id != previous.boot_id or bundle.owner_epoch != previous.owner_epoch:
                    raise RuntimeError('Observation boot/owner epoch changed')
                if bundle.sequence < previous.sequence or (bundle.sequence == previous.sequence and bundle != previous):
                    raise RuntimeError('Observation sequence regressed or reused with changed evidence')
            self._last_bundle = bundle
        return s

    def arm(self, now: float, *, commissioning_receipt: str) -> None:
        if not self.config.physical_actuation_enabled:
            raise RuntimeError("Physical actuation disabled")
        if commissioning_receipt != self.config.commissioning_receipt:
            raise RuntimeError("Commissioning receipt mismatch")
        if self._db.execute("SELECT 1 FROM commands WHERE status NOT IN ('succeeded','failed','canceled') LIMIT 1").fetchone():
            raise RuntimeError("Unresolved prior command: reconcile before arming")
        s = self._checked_sample(now)
        if not self._stationary(s):
            raise RuntimeError("Arms are not observed stationary")
        if self.release_gate is None or not callable(getattr(self.transport, 'move_admitted', None)):
            raise RuntimeError('Commissioned admission gate and enforcing transport required')
        self._owner_epoch = self.release_gate.arm(s, enforcement_record=getattr(self.transport, 'enforcement_record', None))
        self._fault = None
        self._armed = True

    def start(self, command: SkillCommand, now: float) -> BackendEvent:
        entered = time.monotonic()
        payload = self._payload(command)
        previous = self._db.execute("SELECT payload FROM commands WHERE id=?", (command.command_id,)).fetchone()
        if previous:
            if previous[0] != payload:
                raise ValueError("Duplicate command ID has a different payload/configuration")
            return self.command_status(command.command_id, now)  # type: ignore[return-value]
        if not self._armed or self._fault or self._active:
            return BackendEvent(command.command_id, "failed", now, detail="Not armed, unresolved fault, or occupied")
        if command.kind != "point" or set(command.parameters) != {"profile"}:
            return BackendEvent(command.command_id, "failed", now, detail="Only named approved pointing profiles are accepted")
        profile_id = command.parameters["profile"]
        profile = self.config.profiles.get(profile_id) if isinstance(profile_id, str) else None
        if profile is None or not math.isfinite(command.deadline) or command.deadline <= now + profile.duration_ms / 1000 + profile.settling_seconds:
            return BackendEvent(command.command_id, "failed", now, detail="Unknown profile or insufficient deadline")
        try:
            sample = self._checked_sample(now)
            if not self._stationary(sample):
                raise RuntimeError('Arms newly moving at admission')
            self._admission = self.release_gate.admit(command, profile, sample,
                enforcement_record=self.transport.enforcement_record, expected_owner=self._owner_epoch)
        except Exception as exc:
            self._armed = False
            return BackendEvent(command.command_id, "failed", now, detail=str(exc))
        self._active, self._profile = command, profile
        self._completed_profile = None
        self._settling_since = None
        self._last_poll_sample = None
        self._stop_requested = False
        self.stop_confirmed = False
        self._dispatch_uncertain = False
        # This commit precedes the side effect. A crash here remains ambiguous.
        self._record(command, "unknown", now, "Dispatch intent persisted; awaiting gateway receipt",
                     evidence={'admission': asdict(self._admission)})
        # Reads, fsync and authority resolution may have consumed the entire
        # budget. Recheck AFTER durable intent, immediately before the gateway.
        try:
            checked_now = self._now_after(now, entered)
            sample = self._checked_sample(checked_now)
            if not self._stationary(sample):
                raise RuntimeError('Arms newly moving before send')
            admission = self.release_gate.admit(command, profile, sample,
                enforcement_record=self.transport.enforcement_record, expected_owner=self._owner_epoch)
            if self._stop_requested:
                raise RuntimeError('Stop admitted before send')
        except Exception as exc:
            self._active = None
            self._armed = False
            return self._record(command, 'failed', self._now_after(now, entered), 'No dispatch: ' + str(exc))
        try:
            receipt = self.transport.move_admitted(profile, admission)
            # The enforcing transport must independently re-resolve current
            # owner/authority/expiry at acceptance, not simply echo this token.
            if (not isinstance(receipt, dict) or receipt.get('command_id') != command.command_id
                    or receipt.get('owner_epoch') != admission.owner_epoch
                    or receipt.get('enforcement_record') != admission.enforcement_record):
                raise RuntimeError('Missing correlated enforcing-gateway receipt')
        except Exception as exc:
            self._fault = "Dispatch uncertain: " + str(exc)
            self._armed = False
            self._dispatch_uncertain = True
            return self._record(command, "unknown", now, self._fault)
        if self._stop_requested:
            return self._record(command, 'unknown', self._now_after(now, entered), 'Stop preceded dispatch receipt; awaiting observed containment')
        return self._record(command, "accepted", now, "RPC acknowledged; observed completion pending",
                            facts={"remote.pointed_target": FactUpdate(None, now, status="unknown", source="booster_k1_motion")})

    def _check_running_completion(self, command, sample, now, entered):
        """Trusted site/protection checks followed by a fresh clock read.

        A transport may enforce additional commissioned protection requirements.
        Its resolver can block, so its return never refreshes the measurement or
        execution deadline. Stop/fault monitoring deliberately skips this hook.
        """
        if self.release_gate is not None:
            self.release_gate.check_running(command, sample, expected_owner=self._owner_epoch)
        protection = getattr(self.transport, 'check_running_protection', None)
        if callable(protection):
            protection(command, sample)
        evaluated = self._now_after(now, entered)
        if self._stop_requested or self._fault:
            return None  # The following poll must establish a fresh stop dwell.
        if evaluated >= command.deadline:
            raise RuntimeError('Execution deadline expired during protection check')
        if not 0 <= evaluated-sample.sampled_at <= self.config.max_state_age:
            raise RuntimeError('Measured state expired during protection check')
        if sample.bundle is not None and self.release_gate is not None:
            sample.bundle.validate(evaluated, self.release_gate.manifest.payload['observation'])
        return evaluated

    def poll(self, now: float) -> list[BackendEvent]:
        command, profile = self._active, self._profile
        if command is None or profile is None:
            return []
        entered = time.monotonic()
        try:
            s = self._checked_sample(now, require_ready=not (self._stop_requested or self._fault))
            evaluated = self._now_after(now, entered)
            # A receiver sequence or a callback timestamp is not advancing
            # acquisition. Use conservative intervals for the measured dwell.
            components = [part for part in s.bundle.acquisitions if part.name != 'identity'] if s.bundle else []
            sample_time = min(part.earliest for part in components) if components else s.sampled_at
            dwell_begin = max(part.latest for part in components) if components else s.sampled_at
            if self._last_poll_sample is not None:
                gap = sample_time - self._last_poll_sample
                if gap < 0:
                    raise RuntimeError("Observation clock regressed; reconcile the source epoch")
                max_gap = (self.release_gate.manifest.payload['observation']['max_gap_s']
                           if self.release_gate else self.config.max_state_age)
                if gap > max_gap:
                    self._settling_since = None
            self._last_poll_sample = sample_time
            stationary = self._stationary(s)
            self.measured_stillness = stationary
            if evaluated > command.deadline and not self._stop_requested:
                raise RuntimeError("Execution deadline exceeded; outcome requires reconciliation")
            if not self._stop_requested and not self._fault:
                # Continuing motion and completion need current ownership/site/
                # grant evidence, but no new full-operation dispatch budget.
                evaluated = self._check_running_completion(command, s, now, entered)
                if evaluated is None:
                    return []
            reached = self._reached(s, profile)
            if stationary and (self._stop_requested or (reached and not self._fault)):
                self._settling_since = dwell_begin if self._settling_since is None else self._settling_since
                if sample_time - self._settling_since >= profile.settling_seconds:
                    evidence = {"sampled_at": s.sampled_at, "hand_position_m": s.hand_positions[profile.hand],
                                "profile": command.parameters["profile"], "frame": "torso", "orientation_verified": False,
                                "timestamp_source": s.timestamp_source, 'measured_stillness': True,
                                'observation_bundle_id': s.bundle.bundle_id if s.bundle else None}
                    if self._stop_requested:
                        if self.stop_confirmed:
                            return []
                        self.stop_confirmed = True
                        evidence['stop_confirmed'] = True
                        if self._fault or self._dispatch_uncertain:
                            # Stillness cannot prove an ambiguous RPC's outcome
                            # or fence an outstanding queued command.
                            return [self._record(command, 'unknown', evaluated,
                                'Measured stop; outcome/old work requires reconciliation', evidence=evidence)]
                    if not self._stop_requested:
                        # Recheck immediately before the successful terminal
                        # reduction, not only at the start of a potentially slow
                        # poll. A lost protection proof must never enter this
                        # backend's durable ledger as a successful task effect.
                        evaluated = self._check_running_completion(command, s, now, entered)
                        if evaluated is None:
                            return []
                    status = "canceled" if self._stop_requested else "succeeded"
                    facts = {} if self._stop_requested else {"remote.pointed_target": FactUpdate(
                        profile.target_label, s.sampled_at, s.sampled_at + self.config.max_state_age,
                        source="booster_k1_observed_endpoint")}
                    receipt = self._record(command, status, evaluated,
                        "Fresh arm velocity and torso-frame endpoint postcondition observed", facts=facts, evidence=evidence)
                    self._active = None
                    self._completed_profile = None if self._stop_requested else profile
                    return [receipt]
            else:
                self._settling_since = None
            return []
        except Exception as exc:
            self._armed = False
            self.measured_stillness = False
            self._settling_since = None
            reason = str(exc)
            if self._fault == reason:
                return []
            self._fault = reason
            return [self._record(command, "unknown", self._now_after(now, entered), self._fault)]

    def observe(self, now: float) -> Observation:
        entered = time.monotonic()
        try:
            s = self._checked_sample(now, require_ready=False)
            self.measured_stillness = self._stationary(s)
            site_ready = s.site_ready is True and s.mode == self.config.allowed_mode and s.body_control == self.config.allowed_body_control
            quiet = not self._fault and site_ready and self.measured_stillness and self._active is None
            profile = self._completed_profile
            target_known = quiet and profile is not None and self._reached(s, profile)
            if target_known and self.release_gate is not None:
                s.bundle.validate(self._now_after(now, entered), self.release_gate.manifest.payload['observation'])
            facts = {"remote.pointed_target": FactUpdate(profile.target_label if target_known else None,
                      s.sampled_at, s.sampled_at + self.config.max_state_age,
                      status="known" if target_known else "unknown", source="booster_k1_observed_endpoint")}
            return Observation(True, quiet, s.sampled_at, self.boot_id,
                               facts=facts,
                               active_command_id=self._active.command_id if self._active else None,
                               fault=self._fault or (None if site_ready else 'Site/controller readiness unavailable'))
        except Exception as exc:
            self.measured_stillness = False
            return Observation(False, False, now, self.boot_id,
                               facts={"remote.pointed_target": FactUpdate(None, now, status="unknown", source="booster_k1_unavailable")}, fault=str(exc),
                               active_command_id=self._active.command_id if self._active else None)

    def request_stop(self, now: float) -> None:
        self._armed = False
        if self._active is None or self._stop_requested:
            return
        # Linearization point: later endpoint arrival or RPC return cannot win.
        # Admission is serialized; an independent gateway stop/watchdog must
        # remain callable even when this process or its disk is blocked.
        self._stop_requested = True
        self._settling_since = None
        self._last_poll_sample = None
        try:
            self.transport.stop()
        except Exception as exc:
            self._fault = "Stop outcome unknown: " + str(exc)
            self._record(self._active, "unknown", now, self._fault)

    def shutdown(self, now: float) -> dict:
        """One bounded shutdown step, called repeatedly by the serialized owner.

        Does not wait behind a stuck motion worker or erase ambiguous history.
        The enforcing gateway must independently maintain inhibition on process
        death. A missing containment receipt keeps this adapter open/faulted.
        """
        self._armed = False
        self.request_stop(now)
        events = self.poll(now)
        if self._active is None:
            self.close()
            return {'closed': True, 'stop_confirmed': self.stop_confirmed,
                    'outcome_unknown': False, 'events': [e.status for e in events]}
        inhibit = getattr(self.transport, 'inhibit', None)
        receipt = inhibit(self._owner_epoch) if callable(inhibit) else None
        contained = (isinstance(receipt, dict) and receipt.get('owner_epoch') == self._owner_epoch
                     and receipt.get('pending_work_contained') is True
                     and receipt.get('enforcement_record') == getattr(self.transport, 'enforcement_record', None))
        if contained and self.stop_confirmed:
            # Durable unknown survives resource close and blocks the next run.
            self._active = None
            self.close()
            return {'closed': True, 'stop_confirmed': True, 'outcome_unknown': True,
                    'containment_receipt': receipt}
        return {'closed': False, 'stop_confirmed': self.stop_confirmed,
                'outcome_unknown': True, 'recovery_required': True}

    def command_status(self, command_id: str, now: float) -> BackendEvent | None:
        event = self._db.execute("SELECT receipt FROM events WHERE command_id=? ORDER BY sequence DESC LIMIT 1", (command_id,)).fetchone()
        if event:
            data = json.loads(event[0])
            data["facts"] = {name: FactUpdate(**fact) for name, fact in data["facts"].items()}
            return BackendEvent(**data)
        row = self._db.execute("SELECT status,at,detail FROM commands WHERE id=?", (command_id,)).fetchone()
        return BackendEvent(command_id, row[0], row[1], detail=row[2]) if row else None


class BoosterSDKTransport:
    """Optional real SDK transport. Construct only on the commissioned robot host.

    `site_ready` is an injected local workcell/ownership/stop monitor, not an
    internet/UI boolean. Its installation is a deployment prerequisite.
    LowState exposes no source timestamp here; freshness is receipt-time only.
    Commissioning must bound DDS transport delay before using that evidence.
    """

    def __init__(self, *, network_interface: str, domain_id: int, robot_name: str,
                 permit_connection: bool, site_ready):
        if not permit_connection or not network_interface:
            raise RuntimeError("Explicit authorized interface required before SDK connection")
        if importlib.metadata.version("booster_robotics_sdk_python") != SDK_VERSION:
            raise RuntimeError("Unverified SDK version; expected " + SDK_VERSION)
        s = importlib.import_module("booster_robotics_sdk_python")
        self._sdk = s
        self._lock = threading.Lock()
        self._low = None
        self._site_ready = site_ready
        s.ChannelFactory.Instance().Init(domain_id, network_interface)
        self._client = s.B1LocoClient()
        if robot_name:
            self._client.InitWithName(robot_name)
        else:
            self._client.Init()
        self._subscriber = s.B1LowStateSubscriber(self._on_state)
        if robot_name:
            self._subscriber.InitChannelWithName(robot_name)
        else:
            self._subscriber.InitChannel()

    def _on_state(self, state) -> None:
        motors = state.motor_state_serial
        if len(motors) != 22:
            return
        with self._lock:
            self._low = (time.monotonic(), tuple(float(m.q) for m in motors[2:10]),
                         tuple(float(m.dq) for m in motors[2:10]), any(bool(m.lost) for m in motors))

    def read(self) -> K1Sample:
        # Capture the measurement before synchronous queries, so its sample time
        # cannot advance beyond the caller's observation clock during those RPCs.
        with self._lock:
            low = self._low
        if low is None:
            raise RuntimeError("No measured K1 low state received")
        info, state = self._client.GetRobotInfo(), self._client.GetStatus()
        s = self._sdk
        positions = {}
        for hand, frame in (("left", s.Frame.kLeftHand), ("right", s.Frame.kRightHand)):
            p = self._client.GetFrameTransform(s.Frame.kBody, frame).position
            positions[hand] = (float(p.x), float(p.y), float(p.z))
        return K1Sample(low[0], info.serial_number, info.version, info.model,
                        int(state.current_mode), int(state.current_body_control), low[1], low[2],
                        positions, low[3], bool(self._site_ready()), "callback_receipt_monotonic")

    def move(self, profile: PointProfile) -> None:
        raise RuntimeError("Stock SDK has no commissioned owner/expiry gateway; physical motion is disabled")

    def stop(self) -> None:
        self._client.StopHandEndEffector()

    def close(self) -> None:
        self._subscriber.CloseChannel()
