"""Supervised physical gateway, with runnable observation and shadow modes.

All factories/resolvers are trusted local deployment code. Wire requests cannot
provide approvals, protection evidence, profiles, raw poses, or SDK methods.
Software queue inhibition and an independent stop requester do not prove device
fencing; physical arm additionally needs expiring commissioned protection proof.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import fcntl
import json
from pathlib import Path
import queue
import sqlite3
import threading
import time
import uuid

from ..backends.base import BackendEvent, SkillCommand
from ..journal import canonical, digest
from .manifest import finite, strict_json, text


@dataclass(frozen=True)
class ProtectionEvidence:
    issuer: str
    owner_epoch: str
    manifest_digest: str
    enforcement_record: str
    watchdog_record: str
    observed_at: float
    expires_at: float
    exclusive_command_owner: bool
    host_death_inhibition: bool
    old_rpc_containment: bool


def _check_protection(proof, *, gate, trusted_issuers, owner_epoch, now):
    m = gate.manifest.payload
    if (not isinstance(proof, ProtectionEvidence) or proof.issuer not in trusted_issuers
            or proof.owner_epoch != owner_epoch or proof.manifest_digest != gate.manifest.digest
            or proof.enforcement_record != m['ownership']['enforcement_record']
            or proof.watchdog_record != m['ownership']['watchdog_record']
            or proof.exclusive_command_owner is not True or proof.host_death_inhibition is not True
            or proof.old_rpc_containment is not True
            or not finite(proof.observed_at, 'protection time') <= now < finite(proof.expires_at, 'protection expiry')
            or now-proof.observed_at > m['observation']['max_age_s']):
        raise RuntimeError('Commissioned exclusive owner, host-death inhibition or old-RPC containment unavailable')
    return proof


def _validate_start_sample(sample, manifest, now):
    """Repeat the complete measured starting-state check at the final seam."""
    m = manifest.payload
    if (sample.serial != m['robot']['serial'] or sample.firmware != m['robot']['firmware']
            or sample.model not in {'K1', 'Booster K1'}
            or type(sample.mode) is not int or sample.mode != m['controller']['allowed_mode']
            or type(sample.body_control) is not int or sample.body_control != m['controller']['allowed_body_control']
            or sample.motor_lost is not False or sample.site_ready is not True):
        raise RuntimeError('Final measured identity/mode/motor/site check failed')
    if not 0 <= now-finite(sample.sampled_at, 'measured sample time') <= m['observation']['max_age_s']:
        raise RuntimeError('Final measured state expired')
    if len(sample.arm_positions) != 8 or len(sample.arm_velocities) != 8:
        raise RuntimeError('Final measured arm layout mismatch')
    for value in (*sample.arm_positions, *sample.arm_velocities):
        finite(value, 'measured arm state')
    if set(sample.hand_positions) != {'left', 'right'}:
        raise RuntimeError('Final hand measurements incomplete')
    for values in sample.hand_positions.values():
        if len(values) != 3:
            raise RuntimeError('Final hand measurement layout mismatch')
        for value in values:
            finite(value, 'measured hand coordinate')
    uncertainty = finite(getattr(sample.bundle, 'velocity_uncertainty_rad_s', None), 'velocity uncertainty')
    if uncertainty < 0 or max(map(abs, sample.arm_velocities)) + uncertainty > m['observation']['max_arm_speed_rad_s']:
        raise RuntimeError('Arms are newly moving at final vendor admission')


class ProtectedProfileTransport:
    """Final named-profile boundary; no public unguarded move method.

    The raw vendor client rechecks live identity/status, then invokes
    `before_dispatch` immediately before V2. This boundary re-resolves the gate
    there, after durable intent, and refuses an expired protection receipt.
    A stock SDK's acknowledgement is insufficient protection evidence.
    """
    def __init__(self, *, observer, vendor_motion, gate, stop_worker,
                 protection_resolver, trusted_protection_issuers, ledger, clock=time.monotonic, _store=None):
        self.observer, self.vendor_motion, self.gate = observer, vendor_motion, gate
        self.stop_worker, self.protection_resolver = stop_worker, protection_resolver
        self.trusted_protection_issuers = frozenset(trusted_protection_issuers)
        self.clock = clock
        self.enforcement_record = gate.manifest.payload['ownership']['enforcement_record']
        self._stopped = threading.Event()
        self._store = _store if _store is not None else _Store(
            ledger, "protected_profile_transport", gate.manifest.payload['robot']['serial'])
        self._owner = None

    def read(self):
        return self.observer.read()

    def require_protection(self, owner_epoch):
        proof = _check_protection(self.protection_resolver(), gate=self.gate,
            trusted_issuers=self.trusted_protection_issuers, owner_epoch=owner_epoch, now=self.clock())
        if self.stop_worker.status().get('state') != 'ready' or self._stopped.is_set():
            raise RuntimeError('Independent stop requester is unavailable or already tripped')
        return proof

    def check_running_protection(self, command, sample):
        """Measured backend terminal hook; never renews or creates authority."""
        proof = self.require_protection(self._owner)
        now = self.clock()
        sample.bundle.validate(now, self.gate.manifest.payload['observation'])
        if (sample.bundle.owner_epoch != self._owner or proof.owner_epoch != self._owner
                or now >= command.deadline or self._stopped.is_set()):
            raise RuntimeError('Running protection, observation owner or command deadline invalid')

    def move_admitted(self, profile, admission):
        command = SkillCommand(**json.loads(admission.command_json))
        name = command.parameters['profile']
        expected = dict(self.gate.manifest.payload['profiles'][name])
        if canonical(asdict(profile)) != canonical(expected) or admission.manifest_digest != self.gate.manifest.digest:
            raise ValueError('Profile or admission does not match immutable deployment')
        payload = {'command': command.to_dict(), 'profile': asdict(profile), 'manifest_digest': admission.manifest_digest}
        existing = self._store.lookup(command.command_id, payload)
        if existing is not None:
            if existing['status'] == 'acknowledged':
                return existing['receipt']
            raise RuntimeError('Retained transport intent requires reconciliation; it is never replayed')
        if self._store.unresolved():
            raise RuntimeError('Earlier physical transport intent is unresolved')
        self._owner = admission.owner_epoch
        self.require_protection(self._owner)
        self._store.put(command.command_id, payload, 'intent', {'outcome': 'unknown'})

        def before_dispatch():
            if self._stopped.is_set():
                raise RuntimeError('Stop/inhibition preceded vendor dispatch')
            proof = self.require_protection(admission.owner_epoch)
            sample = self.read()
            _validate_start_sample(sample, self.gate.manifest, self.clock())
            renewed = self.gate.admit(command, profile, sample, enforcement_record=self.enforcement_record,
                                      expected_owner=admission.owner_epoch)
            if (renewed.boot_id != admission.boot_id or renewed.authority_revision != admission.authority_revision
                    or proof.expires_at < command.deadline):
                raise RuntimeError('Physical admission changed or protection expires before command deadline')
            # Resolvers and the cached-read seam can themselves consume time.
            # Refresh protection, then validate every captured age/expiry again
            # after the last potentially blocking call below.
            proof = self.protection_resolver()
            self.stop_worker.renew(min(self.clock() + self.stop_worker.maximum_lease_s * .8, command.deadline))
            if self._stopped.is_set() or self.stop_worker.status().get('state') != 'ready':
                raise RuntimeError('Stop/inhibition arrived during final admission')
            final_now = self.clock()
            _validate_start_sample(sample, self.gate.manifest, final_now)
            sample.bundle.validate(final_now, self.gate.manifest.payload['observation'])
            _check_protection(proof, gate=self.gate, trusted_issuers=self.trusted_protection_issuers,
                              owner_epoch=admission.owner_epoch, now=final_now)
            if final_now + renewed.remaining_budget_s > min(command.deadline, proof.expires_at):
                raise RuntimeError('Remaining dispatch budget expired during final admission')
            return True

        try:
            vendor_receipt = self.vendor_motion.move_profile(name, digest(expected), before_dispatch=before_dispatch)
            if self._stopped.is_set() or self.stop_worker.status().get('state') != 'ready':
                raise RuntimeError('Stop/watchdog raced vendor acknowledgement; reconcile physical outcome')
            receipt = {'command_id': command.command_id, 'owner_epoch': admission.owner_epoch,
                       'enforcement_record': self.enforcement_record, 'vendor_acknowledgement': vendor_receipt,
                       'pending_work_contained': False, 'measured_completion': False}
            self._store.put(command.command_id, payload, 'acknowledged', receipt)
            return receipt
        except Exception:
            self.stop()
            # Durable intent remains unknown even if the final callback denied
            # send: arbitrary vendor exceptions cannot prove no side effect.
            raise

    def terminal(self, event):
        """Only the serialized measured backend reducer calls this method."""
        if event.status not in {'succeeded', 'failed', 'canceled'}:
            return
        entry = self._store.get(event.command_id)
        if entry is None:
            return
        if not event.evidence.get('measured_stillness'):
            raise RuntimeError('Transport cannot retire an intent without measured terminal evidence')
        self._store.put(event.command_id, entry['payload'], event.status, asdict(event))

    def stop(self):
        self._stopped.set()
        return self.stop_worker.request_stop('physical_gateway_inhibited')

    def inhibit(self, owner_epoch):
        self.stop()
        # Even a returned stop RPC cannot claim that old vendor work is fenced.
        return {'owner_epoch': owner_epoch, 'enforcement_record': self.enforcement_record,
                'pending_work_contained': False}

    def close(self):
        self.stop()
        self._store.close()
        close = getattr(self.vendor_motion, 'close', None)
        if callable(close):
            close()


class _Store:
    def __init__(self, path, mode, identity):
        path = Path(path)
        if not path.is_absolute():
            raise ValueError('Gateway ledger must have an absolute persistent path')
        path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(str(path) + '.owner', 'a+')
        try:
            fcntl.flock(self._file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self._file.close()
            raise RuntimeError('Gateway ledger already has a live owner') from None
        self._lock = threading.RLock()
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.executescript('CREATE TABLE IF NOT EXISTS metadata (key TEXT PRIMARY KEY,value TEXT NOT NULL);'
            'CREATE TABLE IF NOT EXISTS commands (id TEXT PRIMARY KEY,payload TEXT NOT NULL,status TEXT NOT NULL,receipt TEXT NOT NULL);')
        binding = canonical({'mode': mode, 'identity': identity})
        old = self.db.execute("SELECT value FROM metadata WHERE key='binding'").fetchone()
        if old and old[0] != binding:
            self.close()
            raise ValueError('Ledger belongs to another device or gateway mode')
        self.db.execute("INSERT OR IGNORE INTO metadata VALUES ('binding',?)", (binding,))
        self.db.commit()

    def get(self, identifier):
        with self._lock:
            row = self.db.execute('SELECT payload,status,receipt FROM commands WHERE id=?', (identifier,)).fetchone()
            return {'payload': json.loads(row[0]), 'status': row[1], 'receipt': json.loads(row[2])} if row else None

    def lookup(self, identifier, payload):
        entry = self.get(identifier)
        if entry is not None and canonical(entry['payload']) != canonical(payload):
            raise ValueError('Command ID already used with another payload')
        return entry

    def put(self, identifier, payload, status, receipt):
        with self._lock:
            self.lookup(identifier, payload)
            self.db.execute('INSERT INTO commands VALUES (?,?,?,?) ON CONFLICT(id) DO UPDATE SET status=excluded.status,receipt=excluded.receipt',
                            (identifier, canonical(payload), status, canonical(receipt)))
            self.db.commit()

    def unresolved(self):
        with self._lock:
            return [row[0] for row in self.db.execute("SELECT id FROM commands WHERE status IN ('queued','intent','acknowledged','accepted','running','unknown')")]

    def close(self):
        self.db.close()
        self._file.close()


def decode_command(value):
    if not isinstance(value, dict) or set(value) != set(SkillCommand.__dataclass_fields__):
        raise ValueError('Exact named SkillCommand schema required')
    for key in ('command_id', 'run_id', 'skill_id', 'authority_id'):
        text(value[key], key)
    if value['kind'] != 'point' or not isinstance(value['parameters'], dict) or set(value['parameters']) != {'profile'}:
        raise ValueError('Only named profile commands are accepted')
    if value['parameters']['profile'] not in {'point_a', 'point_b', 'home'}:
        raise ValueError('Unknown physical profile')
    if type(value['authority_revision']) is not int or value['authority_revision'] < 0:
        raise ValueError('Invalid authority revision')
    if not isinstance(value['dependency_versions'], dict) or len(value['dependency_versions']) > 128:
        raise ValueError('Invalid dependency map')
    for key, version in value['dependency_versions'].items():
        text(key, 'dependency')
        if type(version) is not int or version < 0:
            raise ValueError('Invalid dependency version')
    issued = finite(value['issued_at'], 'issued_at')
    if not issued < finite(value['deadline'], 'deadline') <= issued + 120:
        raise ValueError('Command deadline must be within 120 seconds of issue')
    return SkillCommand(**json.loads(canonical(value)))


class FileObservation:
    """Raw diagnostics only: receipt age cannot certify acquisition freshness."""
    def __init__(self, path, *, clock=time.monotonic, clock_id=None, max_receipt_age_s=2.):
        self.path = Path(path)
        self.clock, self.clock_id = clock, clock_id or Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.max_receipt_age_s = finite(max_receipt_age_s, 'diagnostic receipt age', positive=True)

    def read(self):
        raw = self.path.read_text()
        value = strict_json(raw)
        if (not isinstance(value, dict) or type(value.get('schema_version')) is not int or value['schema_version'] != 1
                or value.get('mode') != 'hardware_observation' or value.get('motion_capability') is not False):
            raise ValueError('Not a read-only hardware observation snapshot')
        if value.get('observer_running') is False:
            raise RuntimeError('Diagnostic producer explicitly stopped')
        if (value.get('clock_id') != self.clock_id or not 0 <= self.clock()-finite(
                value.get('generated_at_monotonic'), 'diagnostic publication time') <= self.max_receipt_age_s):
            raise RuntimeError('Diagnostic producer stopped, clock changed, or receipt expired')
        return json.loads(canonical(value))


class PhysicalGateway:
    """A single backend owner; stop is independent of its queue and disk lock."""
    def __init__(self, *, mode, ledger, identity, observation, backend=None, transport=None,
                 stop_worker=None, clock=time.monotonic, heartbeat_s=.25):
        if mode not in {'observe', 'shadow', 'physical'}:
            raise ValueError('Unknown physical gateway mode')
        if mode != 'physical' and any(x is not None for x in (backend, transport, stop_worker)):
            raise ValueError('Observation/shadow cannot receive an actuation object')
        if mode == 'physical' and any(x is None for x in (backend, transport, stop_worker)):
            raise ValueError('Physical mode requires backend, protected transport, and independent stop worker')
        finite(heartbeat_s, 'heartbeat', positive=True)
        if mode == 'physical' and heartbeat_s >= stop_worker.maximum_lease_s:
            raise ValueError('Owner heartbeat must be shorter than independent stop deadline')
        self.mode, self.observation, self.clock = mode, observation, clock
        self.backend, self.transport, self.stop_worker = backend, transport, stop_worker
        self.heartbeat_s = heartbeat_s
        self.boot_id = str(uuid.uuid4())
        self._store = _Store(ledger, mode, identity)
        self._inhibited = threading.Event()
        self._closing = threading.Event()
        self._intake_lock = threading.Lock()
        self._queue = queue.Queue(maxsize=1)
        self._state_lock = threading.Lock()
        self._armed = False
        self._active = None
        self._owner_epoch = None
        self._closed = False
        self._backend_closed = False
        self._state = {'state': 'disarmed', 'fault': None, 'observation': None}
        if self._store.unresolved():
            self._inhibited.set()
            self._state['fault'] = 'Prior durable intent requires explicit reconciliation; no replay'
        self._thread = threading.Thread(target=self._loop, name='physical-gateway-owner', daemon=True)
        self._thread.start()

    def arm(self, commissioning_receipt):
        """Trusted local startup only; deliberately absent from the ROS API."""
        if self.mode != 'physical' or self._inhibited.is_set():
            raise RuntimeError('Gateway cannot arm in this mode/state')
        done = threading.Event()
        result = {}
        self._queue.put_nowait(('arm', commissioning_receipt, done, result))
        if not done.wait(3):
            self.request_stop('arm_operation_timeout')
            raise RuntimeError('Arm outcome unknown; gateway inhibited')
        if 'error' in result:
            raise RuntimeError(result['error'])
        return result

    def submit(self, value):
        command = decode_command(value)
        payload = command.to_dict()
        with self._intake_lock:
            existing = self._store.lookup(command.command_id, payload)
            if existing is not None:
                return {**existing['receipt'], 'duplicate': True}
            if self.mode != 'physical':
                try:
                    snapshot = self.observation.read()
                    observation = {'available': True, 'snapshot': snapshot, 'sha256': digest(snapshot)}
                except Exception as error:
                    observation = {'available': False, 'error': str(error)}
                receipt = {'command_id': command.command_id, 'status': 'rejected' if self.mode == 'observe' else 'would_propose',
                           'shadow_id': str(uuid.uuid4()) if self.mode == 'shadow' else None,
                           'motion_dispatched': False, 'task_effects': {}, 'task_authority_granted': False,
                           'observation': observation, 'physical_admission': 'blocked',
                           'reason': 'Read-only diagnostics do not certify profile admission or predict completion'}
                self._store.put(command.command_id, payload, receipt['status'], receipt)
                return receipt
            if not self._armed or self._inhibited.is_set() or self._active is not None or self._store.unresolved():
                raise RuntimeError('Physical gateway is disarmed, inhibited, occupied or unreconciled')
            now = self.clock()
            if command.issued_at > now or command.deadline <= now:
                raise ValueError('Command clock/deadline invalid at intake')
            receipt = {'command_id': command.command_id, 'status': 'queued', 'motion_dispatched': False,
                       'task_effects': {}}
            self._store.put(command.command_id, payload, 'queued', receipt)
            self._queue.put_nowait(('command', command, None, None))
            return receipt

    def request_stop(self, reason='operator_stop'):
        # No SQLite write, backend lock, SDK read, or dispatcher join precedes
        # this inhibition/independent stop request.
        self._inhibited.set()
        self._armed = False
        if self.transport is not None:
            self.transport._stopped.set()
        receipt = self.stop_worker.request_stop(reason) if self.stop_worker else {'state': 'not_applicable'}
        return {'inhibited': True, 'queued_work_contained': True, 'physical_containment_verified': False,
                'stop': receipt}

    def command_status(self, command_id):
        text(command_id, 'command_id')
        entry = self._store.get(command_id)
        return entry['receipt'] if entry else None

    def snapshot(self):
        with self._state_lock:
            state = json.loads(canonical(self._state))
        return {'schema_version': 1, 'boot_id': self.boot_id, 'mode': self.mode,
                'armed': self._armed, 'inhibited': self._inhibited.is_set(),
                'motion_capability': self.mode == 'physical', 'physical_containment_verified': False,
                'stop_worker': self.stop_worker.status() if self.stop_worker else None, **state}

    def _check_protection_or_inhibit(self):
        if self._inhibited.is_set() or not (self._armed or self._active is not None):
            return
        try:
            self.transport.require_protection(self._owner_epoch)
        except Exception as error:
            with self._state_lock:
                self._state.update(state='blocked', fault='Protection lost: ' + str(error))
            self.request_stop('protection_lost_before_completion')
            # This is on the serialized owner and establishes backend stop state
            # before it can reduce a terminal event. The independent request has
            # already been issued and does not wait for this method.
            self.backend.request_stop(self.clock())

    def _poll_and_reduce(self):
        self._check_protection_or_inhibit()
        if self._inhibited.is_set():
            self.backend.request_stop(self.clock())
        events = self.backend.poll(self.clock())
        # A slow poll may span loss/expiry after the first protection check.
        self._check_protection_or_inhibit()
        for event in events:
            entry = self._store.get(event.command_id)
            if (event.status == 'succeeded' and entry is not None
                    and self.clock() >= entry['payload']['deadline']):
                with self._state_lock:
                    self._state.update(state='blocked', fault='Command deadline elapsed before completion admission')
                self.request_stop('completion_deadline_elapsed')
                self.backend.request_stop(self.clock())
            if event.status == 'succeeded' and self._inhibited.is_set():
                # Keep measured evidence for investigation without promoting its
                # task effects or retiring the durable transport intent.
                event = BackendEvent(event.command_id, 'unknown', self.clock(), facts={},
                    detail='Completion not admitted: stop/protection loss preceded gateway reduction',
                    evidence={'unadmitted_backend_event': asdict(event),
                              'physical_containment_verified': False})
            self.transport.terminal(event)
            if entry:
                self._store.put(event.command_id, entry['payload'], event.status, asdict(event))
            if event.status in {'succeeded', 'failed', 'canceled'}:
                self._active = None

    def _loop(self):
        while True:
            if self._closing.is_set():
                if self.mode != 'physical':
                    return
                try:
                    # Shutdown stays on the owner. A blocked SDK/sample/ledger
                    # call cannot consume the public close() join deadline.
                    result = self.backend.shutdown(self.clock())
                    if result.get('closed'):
                        self._backend_closed = True
                        return
                    with self._state_lock:
                        self._state.update(state='recovery_required', shutdown=result)
                except Exception as error:
                    with self._state_lock:
                        self._state.update(state='recovery_required', fault=str(error))
                time.sleep(.05)
                continue
            try:
                work = self._queue.get(timeout=.02)
            except queue.Empty:
                work = None
            try:
                if work:
                    operation, value, done, result = work
                    if operation == 'arm':
                        try:
                            sample = self.transport.read()
                            owner = self.backend.release_gate.arm(sample, enforcement_record=self.transport.enforcement_record)
                            self.transport.require_protection(owner)
                            self.backend.arm(self.clock(), commissioning_receipt=value)
                            if self._inhibited.is_set():
                                raise RuntimeError('Stop preceded arm completion')
                            self._armed = True
                            self._owner_epoch = owner
                            result['armed'] = True
                        except Exception as error:
                            result['error'] = str(error)
                        finally:
                            done.set()
                    else:
                        command = value
                        if self._inhibited.is_set():
                            self._store.put(command.command_id, command.to_dict(), 'rejected',
                                {'command_id': command.command_id, 'status': 'rejected', 'motion_dispatched': False,
                                 'reason': 'Stop/inhibition preceded dispatcher claim'})
                        else:
                            self._active = command
                            self._store.put(command.command_id, command.to_dict(), 'intent',
                                {'command_id': command.command_id, 'status': 'unknown', 'motion_dispatched': None,
                                 'reason': 'Durable dispatcher intent; outcome requires backend receipt'})
                            if self._inhibited.is_set():
                                raise RuntimeError('Stop raced dispatch intent; no automatic retry')
                            event = self.backend.start(command, self.clock())
                            self._store.put(command.command_id, command.to_dict(), event.status, asdict(event))
                            if event.status in {'succeeded', 'failed', 'canceled'}:
                                self._active = None
                if self.mode == 'physical':
                    self._poll_and_reduce()
                    observation = asdict(self.backend.observe(self.clock()))
                    if self._inhibited.is_set():
                        # Preserve actual fault/measurement records, but do not
                        # advertise command readiness or accepted task effects.
                        observation['quiescent'] = False
                        observation['fault'] = observation['fault'] or self._state['fault'] or 'Gateway inhibited'
                        for name, fact in observation['facts'].items():
                            if name.startswith('remote.'):
                                fact.update(status='unknown', value=None)
                    if self._armed:
                        if not observation['connected'] or observation['fault']:
                            raise RuntimeError('Live backend observation invalid')
                        # Task/site checks in the backend do not substitute for
                        # current exclusive ownership and independent protection.
                        self.transport.require_protection(self._owner_epoch)
                        deadline = self.clock() + self.heartbeat_s
                        if self._active is not None:
                            deadline = min(deadline, self._active.deadline)
                        self.stop_worker.renew(deadline)
                else:
                    observation = self.observation.read()
                with self._state_lock:
                    self._state.update(state='observing' if self.mode != 'physical' else 'blocked' if self._inhibited.is_set()
                                       else 'running' if self._armed else 'disarmed',
                                       observation=observation)
                    if self.mode != 'physical':
                        self._state['fault'] = None
            except Exception as error:
                with self._state_lock:
                    self._state.update(state='blocked', fault=type(error).__name__ + ': ' + str(error))
                if self.mode == 'physical':
                    self.request_stop('dispatcher_fault')

    def close(self, timeout_s=1.):
        if self._closed:
            return {'closed': True, 'physical_containment_verified': False}
        self.request_stop('gateway_shutdown')
        self._closing.set()
        self._thread.join(timeout_s)
        if self._thread.is_alive():
            return {'closed': False, 'outcome_unknown': True, 'independent_stop_retained': True}
        if self.mode == 'physical' and not self._backend_closed:
            return {'closed': False, 'outcome_unknown': True, 'independent_stop_retained': True}
        if self.mode == 'physical':
            if not self.stop_worker.close(timeout_s):
                return {'closed': False, 'stop_requester_unresolved': True}
            self.transport.close()
        self._store.close()
        self._closed = True
        return {'closed': True, 'physical_containment_verified': False}


def assemble_supervised(*, manifest, registry_root, observer, release_gate,
                        protection_resolver, trusted_protection_issuers, wheel_path,
                        clock=time.monotonic, heartbeat_limit_s=.5):
    """Common physical composition; caller supplies real trusted site services.

    This function connects SDK ports only when called explicitly by the physical
    site factory. It never creates an approval, certifies acquisition timing, or
    fills missing site records. Importing the module connects to nothing.
    """
    from .deployment import assemble_disarmed, device_ledger_path
    from .stop_worker import IndependentStopWorker
    from .vendor_sdk import ConnectionSettings, StopFactory, ProfileMotionFactory

    manifest.require_complete()
    if manifest.payload['physical_actuation_enabled'] is not True:
        raise RuntimeError('Physical actuation remains disabled')
    if release_gate.manifest.digest != manifest.digest:
        raise ValueError('Assembly gate and manifest differ')
    m = manifest.payload
    settings = ConnectionSettings(wheel_path, m['sdk']['network_binding'], m['sdk']['vendor_dds_domain'],
                                  m['sdk']['robot_name'], m['robot']['serial'], m['robot']['firmware'])
    device_path = device_ledger_path(registry_root, settings.expected_serial)
    # Pure trusted gate evaluation before constructing either vendor port.
    sample = observer.read()
    owner = release_gate.arm(sample, enforcement_record=m['ownership']['enforcement_record'])
    _check_protection(protection_resolver(), gate=release_gate,
                      trusted_issuers=frozenset(trusted_protection_issuers), owner_epoch=owner, now=clock())
    # Claim the durable local owner BEFORE constructing the stop client. A
    # duplicate launch must never stop somebody else's already-running owner
    # merely because its own later ledger construction failed.
    store = _Store(device_path.with_name('gateway-transport.sqlite'), 'protected_profile_transport', settings.expected_serial)
    stop_worker = None
    transport = None
    try:
        stop_worker = IndependentStopWorker(StopFactory(settings), maximum_lease_s=heartbeat_limit_s)
        # Defer the motion client until commissioned protection has been checked.
        class DeferredMotion:
            port = None
            def move_profile(self, *args, **kwargs):
                if self.port is None:
                    # Manifest.canonical_json deliberately omits commissioning
                    # from its digest; the actual parser needs the full record.
                    complete_json = json.dumps(manifest.payload, default=dict, allow_nan=False)
                    self.port = ProfileMotionFactory(settings, complete_json)()
                return self.port.move_profile(*args, **kwargs)
            def close(self):
                if self.port is not None:
                    self.port.close()
        transport = ProtectedProfileTransport(observer=observer, vendor_motion=DeferredMotion(), gate=release_gate,
            stop_worker=stop_worker, protection_resolver=protection_resolver,
            trusted_protection_issuers=trusted_protection_issuers,
            ledger=device_path.with_name('gateway-transport.sqlite'), clock=clock, _store=store)
        transport.require_protection(owner)
        backend = assemble_disarmed(manifest, transport, registry_root=registry_root,
                                    release_gate=release_gate, clock=clock)
        return {'backend': backend, 'transport': transport, 'stop_worker': stop_worker}
    except Exception:
        if transport is not None:
            transport.close()
        else:
            store.close()
        if stop_worker is not None:
            stop_worker.close()
        raise
