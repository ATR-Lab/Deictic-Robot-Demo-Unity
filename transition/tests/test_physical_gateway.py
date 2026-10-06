"""Offline gateway/failure tests. All protection evidence below is fictional."""
from dataclasses import dataclass, replace
import importlib.util
import json
from pathlib import Path
import threading
import time

import pytest

from physical_fixtures import config, make_gate
from test_booster_backend import FakeTransport, command
from transition_autonomy.backends.base import BackendEvent, FactUpdate, Observation
from transition_autonomy.physical.gateway import (
    FileObservation, PhysicalGateway, ProtectedProfileTransport, ProtectionEvidence, decode_command,
    assemble_supervised,
)
from transition_autonomy.physical.stop_worker import IndependentStopWorker


def wait_until(predicate, timeout=3):
    end = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < end, 'condition deadline expired'
        time.sleep(.01)


class Stop:
    maximum_lease_s = 2.
    def __init__(self): self.state = 'ready'; self.requests = []; self.renewals = []
    def status(self): return {'state': self.state, 'pending_work_contained': False}
    def renew(self, deadline):
        if self.state != 'ready': raise RuntimeError('stop already requested')
        self.renewals.append(deadline)
    def request_stop(self, reason):
        self.requests.append(reason); self.state = 'stop_requested'; return self.status()
    def close(self, timeout_s=1.): return True


class Motion:
    def __init__(self): self.calls = []; self.before_check = lambda: None; self.lose_reply = False
    def move_profile(self, name, profile_digest, *, before_dispatch):
        self.before_check()
        assert before_dispatch() is True
        self.calls.append((name, profile_digest))
        if self.lose_reply: raise TimeoutError('fictional acknowledgement lost')
        return {'rpc_returned': True, 'execution_complete': False}
    def close(self): pass


def protected(tmp_path, *, proof_change=None, filename='transport.sqlite'):
    samples, cfg, stop, motion = FakeTransport(), config(), Stop(), Motion()
    gate = make_gate(cfg, samples)
    proof = ProtectionEvidence('fictional-monitor', 'fake-owner', gate.manifest.digest,
        'fake-enforcement', 'fake-watchdog', 1., 20., True, True, True)
    if proof_change: proof = replace(proof, **proof_change)
    transport = ProtectedProfileTransport(observer=samples, vendor_motion=motion, gate=gate,
        stop_worker=stop, protection_resolver=lambda: proof, trusted_protection_issuers={'fictional-monitor'},
        ledger=tmp_path/filename, clock=lambda: samples.sample.sampled_at)
    admission = gate.admit(command(), cfg.profiles['point_a'], samples.read(),
                           enforcement_record='fake-enforcement', expected_owner='fake-owner')
    return transport, samples, cfg, stop, motion, admission


def test_named_transport_duplicate_never_replays_and_never_claims_completion(tmp_path):
    transport, _, cfg, stop, motion, admission = protected(tmp_path)
    try:
        receipt = transport.move_admitted(cfg.profiles['point_a'], admission)
        assert receipt['measured_completion'] is False and receipt['pending_work_contained'] is False
        assert transport.move_admitted(cfg.profiles['point_a'], admission) == receipt
        assert len(motion.calls) == 1 and stop.renewals
    finally: transport.close()


@pytest.mark.parametrize('change', [
    {'exclusive_command_owner': False}, {'host_death_inhibition': False}, {'old_rpc_containment': False},
    {'issuer': 'unknown'}, {'owner_epoch': 'new-owner'}, {'manifest_digest': 'wrong'},
    {'enforcement_record': 'wrong'}, {'watchdog_record': 'wrong'}, {'expires_at': 1.},
    {'observed_at': 0.},
])
def test_raw_sdk_and_self_declared_monitor_cannot_supply_protection(tmp_path, change):
    transport, _, cfg, _, motion, admission = protected(tmp_path, proof_change=change)
    try:
        with pytest.raises(RuntimeError, match='Commissioned'):
            transport.move_admitted(cfg.profiles['point_a'], admission)
        assert motion.calls == []
    finally: transport.close()


def test_final_admission_rechecked_after_vendor_queries(tmp_path):
    transport, _, cfg, _, motion, admission = protected(tmp_path)
    motion.before_check = lambda: transport.gate.registry.revoke('test-receipt')
    try:
        with pytest.raises(RuntimeError, match='revoked'):
            transport.move_admitted(cfg.profiles['point_a'], admission)
        assert motion.calls == []
    finally: transport.close()


def test_stop_during_final_observation_cannot_then_dispatch(tmp_path):
    transport, samples, cfg, _, motion, admission = protected(tmp_path)
    original = samples.read
    def stop_while_reading():
        transport.stop()
        return original()
    samples.read = stop_while_reading
    try:
        with pytest.raises(RuntimeError, match='stop already requested|Stop/inhibition'):
            transport.move_admitted(cfg.profiles['point_a'], admission)
        assert motion.calls == []
    finally: transport.close()


@pytest.mark.parametrize('changes', [
    {'motor_lost': True}, {'arm_velocities': (.1,)*8}, {'serial': 'another-robot'},
    {'firmware': 'other'}, {'model': 'other'}, {'mode': True}, {'mode': 9},
    {'body_control': 9}, {'site_ready': False}, {'arm_positions': (0.,)*7},
])
def test_final_state_change_after_vendor_queries_denies_motion(tmp_path, changes):
    transport, samples, cfg, _, motion, admission = protected(tmp_path)
    motion.before_check = lambda: setattr(samples, 'sample', replace(samples.sample, **changes))
    try:
        with pytest.raises((RuntimeError, ValueError)):
            transport.move_admitted(cfg.profiles['point_a'], admission)
        assert motion.calls == []
    finally: transport.close()


def test_slow_final_resolver_cannot_extend_stale_acquisition(tmp_path):
    transport, samples, cfg, _, motion, admission = protected(tmp_path)
    now = [1.]
    transport.clock = lambda: now[0]
    original = transport.gate.grant_resolver
    def delayed_grant(authority):
        result = original(authority)
        now[0] = 1.3  # More than the approved .25-second sample/protection age.
        return result
    transport.gate.grant_resolver = delayed_grant
    try:
        with pytest.raises(RuntimeError, match='expired|unavailable'):
            transport.move_admitted(cfg.profiles['point_a'], admission)
        assert motion.calls == []
    finally: transport.close()


def test_lost_motion_reply_survives_restart_without_replay(tmp_path):
    transport, _, cfg, _, motion, admission = protected(tmp_path)
    motion.lose_reply = True
    with pytest.raises(TimeoutError): transport.move_admitted(cfg.profiles['point_a'], admission)
    assert len(motion.calls) == 1
    transport.close()
    restarted, _, cfg, _, second_motion, admission = protected(tmp_path)
    try:
        with pytest.raises(RuntimeError, match='never replayed'):
            restarted.move_admitted(cfg.profiles['point_a'], admission)
        assert second_motion.calls == []
    finally: restarted.close()


def test_transport_single_owner_and_ledger_device_binding(tmp_path):
    transport, *_ = protected(tmp_path)
    try:
        with pytest.raises(RuntimeError, match='live owner'): protected(tmp_path)
    finally: transport.close()


def raw_observation(at=1.):
    return dict(schema_version=1, mode='hardware_observation', motion_capability=False,
                clock_id='test-clock', generated_at_monotonic=at, boot_id='diagnostic-boot', sequence=7,
                acquisition_freshness_verified=False, device_fencing_proven=False, observations={})


def test_file_observation_does_not_upgrade_diagnostics_and_expires(tmp_path):
    path = tmp_path/'latest.json'; path.write_text(json.dumps(raw_observation()))
    assert FileObservation(path, clock=lambda: 1.5, clock_id='test-clock').read() == raw_observation()
    with pytest.raises(RuntimeError, match='receipt expired'):
        FileObservation(path, clock=lambda: 4., clock_id='test-clock').read()
    with pytest.raises(RuntimeError, match='clock changed'):
        FileObservation(path, clock=lambda: 1.5, clock_id='other').read()
    path.write_text(json.dumps({**raw_observation(), 'observer_running': False}))
    with pytest.raises(RuntimeError, match='explicitly stopped'):
        FileObservation(path, clock=lambda: 1.5, clock_id='test-clock').read()


class Diagnostics:
    def read(self): return raw_observation()


@pytest.mark.parametrize('mode,status', [('observe', 'rejected'), ('shadow', 'would_propose')])
def test_observation_shadow_never_construct_or_dispatch_motion(tmp_path, mode, status):
    path = tmp_path/f'{mode}.sqlite'
    gateway = PhysicalGateway(mode=mode, ledger=path, identity='raw-source', observation=Diagnostics())
    receipt = gateway.submit(command().to_dict())
    assert receipt['status'] == status and receipt['task_effects'] == {}
    assert receipt['motion_dispatched'] is False and receipt['task_authority_granted'] is False
    assert receipt['physical_admission'] == 'blocked'
    assert receipt['observation']['snapshot']['acquisition_freshness_verified'] is False
    with pytest.raises(RuntimeError): gateway.arm('anything')
    assert gateway.close()['closed']
    restarted = PhysicalGateway(mode=mode, ledger=path, identity='raw-source', observation=Diagnostics())
    try:
        assert restarted.submit(command().to_dict()) == {**receipt, 'duplicate': True}
        with pytest.raises(ValueError, match='another payload'):
            restarted.submit(command(run_id='changed').to_dict())
    finally: restarted.close()


def test_observation_mode_rejects_actuation_object(tmp_path):
    with pytest.raises(ValueError, match='actuation object'):
        PhysicalGateway(mode='observe', ledger=tmp_path/'no.sqlite', identity='x',
                        observation=Diagnostics(), backend=object())


class BlockingBackend:
    def __init__(self):
        self.entered = threading.Event(); self.release = threading.Event(); self.moves = []
        self.block_observation = False; self.block_command = False
    def observe(self, now):
        if self.block_observation:
            self.entered.set(); self.release.wait(3)
        return Observation(True, True, now, 'fictional-boot')
    def start(self, command, now):
        self.moves.append(command.command_id)
        if self.block_command:
            self.entered.set(); self.release.wait(3)
        return BackendEvent(command.command_id, 'unknown', now, detail='fictional lost acknowledgement')
    def poll(self, now): return []
    def request_stop(self, now): pass
    def shutdown(self, now): return {'closed': True}


class DummyProtectedTransport:
    """Test-only transport; never used by the real site composition."""
    def __init__(self): self._stopped = threading.Event()
    def terminal(self, event): pass
    def require_protection(self, owner_epoch): pass
    def close(self): pass


def dispatcher(tmp_path, backend):
    stop = Stop()
    gateway = PhysicalGateway(mode='physical', ledger=tmp_path/'owner.sqlite', identity='fictional-device',
        observation=Diagnostics(), backend=backend, transport=DummyProtectedTransport(), stop_worker=stop,
        clock=lambda: 1.)
    gateway._armed = True  # Deliberate offline bypass to isolate dispatcher behavior.
    return gateway, stop


def test_stop_bypasses_blocked_dispatch_and_retains_uncertain_intent(tmp_path):
    backend = BlockingBackend(); backend.block_command = True
    gateway, stop = dispatcher(tmp_path, backend)
    gateway.submit(command().to_dict())
    assert backend.entered.wait(1)
    started = time.monotonic(); receipt = gateway.request_stop('test-stop')
    assert time.monotonic()-started < .1 and stop.requests
    assert receipt['physical_containment_verified'] is False
    assert gateway.close(.01)['closed'] is False
    backend.release.set()
    wait_until(lambda: not gateway._thread.is_alive())
    assert gateway.close()['closed']
    restarted, _ = dispatcher(tmp_path, BlockingBackend())
    try:
        assert restarted.snapshot()['inhibited'] is True
        assert restarted.submit(command().to_dict())['duplicate'] is True
        with pytest.raises(RuntimeError, match='inhibited'):
            restarted.submit(command(command_id='fresh').to_dict())
    finally: restarted.close()


def test_stop_contains_queued_work_before_dispatch_claim(tmp_path):
    backend = BlockingBackend(); backend.block_observation = True
    gateway, _ = dispatcher(tmp_path, backend)
    assert backend.entered.wait(1)
    gateway.submit(command().to_dict())
    gateway.request_stop('before-claim'); backend.release.set()
    wait_until(lambda: gateway.command_status('cmd1')['status'] == 'rejected')
    assert backend.moves == []
    gateway.close()


def test_close_keeps_shutdown_on_supervised_owner_and_bounds_wait(tmp_path):
    backend = BlockingBackend()
    def blocked_shutdown(now):
        backend.entered.set(); backend.release.wait(3)
        return {'closed': True}
    backend.shutdown = blocked_shutdown
    gateway, stop = dispatcher(tmp_path, backend)
    started = time.monotonic()
    assert gateway.close(.02)['closed'] is False
    assert time.monotonic()-started < .15
    assert backend.entered.wait(1) and stop.requests
    backend.release.set()
    assert gateway.close(1.)['closed'] is True


def test_running_protection_loss_trips_independent_stop(tmp_path):
    backend = BlockingBackend()
    gateway, stop = dispatcher(tmp_path, backend)
    def protection_lost(owner): raise RuntimeError('exclusive owner proof revoked')
    gateway.transport.require_protection = protection_lost
    wait_until(lambda: gateway.snapshot()['inhibited'])
    assert stop.requests and 'exclusive owner' in gateway.snapshot()['fault']
    assert gateway.close()['closed']


@pytest.mark.parametrize('revoke_at,expected', [('before_poll', 'canceled'), ('during_poll', 'unknown'),
                                              ('deadline_during_poll', 'unknown')])
def test_protection_loss_cannot_persist_success_before_terminal_reduction(tmp_path, revoke_at, expected):
    proof = {'valid': True}
    current_clock = [1.]
    class CompletingBackend(BlockingBackend):
        pending = None
        stopped = False
        def start(self, value, now):
            self.pending = value
            if revoke_at == 'before_poll': proof['valid'] = False
            return BackendEvent(value.command_id, 'accepted', now)
        def request_stop(self, now): self.stopped = True
        def poll(self, now):
            if self.pending is None: return []
            value = self.pending; self.pending = None
            if self.stopped:
                return [BackendEvent(value.command_id, 'canceled', now,
                    evidence={'measured_stillness': True, 'stop_confirmed': True})]
            if revoke_at == 'during_poll': proof['valid'] = False
            if revoke_at == 'deadline_during_poll': current_clock[0] = 4.1
            return [BackendEvent(value.command_id, 'succeeded', now,
                facts={'remote.pointed_target': FactUpdate('A', now)},
                evidence={'measured_stillness': True})]
        def observe(self, now):
            return Observation(True, True, now, 'fictional-boot',
                facts={'remote.pointed_target': FactUpdate('A', now),
                       'robot.measured_joint': FactUpdate(.2, now)}, fault='genuine measured fault' if self.stopped else None)
    backend = CompletingBackend()
    gateway, stop = dispatcher(tmp_path, backend)
    gateway.clock = lambda: current_clock[0]
    terminal_records = []
    def require(owner):
        if not proof['valid']: raise RuntimeError('protection revoked')
    gateway.transport.require_protection = require
    gateway.transport.terminal = lambda event: terminal_records.append(event.status)
    try:
        gateway.submit(command().to_dict())
        wait_until(lambda: gateway.command_status('cmd1')['status'] == expected)
        wait_until(lambda: gateway.snapshot()['inhibited'])
        receipt = gateway.command_status('cmd1')
        assert receipt['facts'] == {} and 'succeeded' not in terminal_records and stop.requests
        if revoke_at != 'before_poll':
            assert receipt['evidence']['unadmitted_backend_event']['status'] == 'succeeded'
            assert gateway._store.unresolved() == ['cmd1']
        else:
            assert receipt['evidence']['stop_confirmed'] is True
        wait_until(lambda: (gateway.snapshot()['observation'] or {}).get('fault') == 'genuine measured fault')
        observation = gateway.snapshot()['observation']
        assert observation['quiescent'] is False
        assert observation['facts']['remote.pointed_target']['status'] == 'unknown'
        assert observation['facts']['robot.measured_joint']['value'] == .2
    finally: gateway.close()


@dataclass
class StopToFileFactory:
    path: str
    def __call__(self): return StopToFile(self.path)


class StopToFile:
    def __init__(self, path): self.path = path
    def stop(self):
        with open(self.path, 'a') as stream: stream.write('requested\n')
    def close(self): pass


def test_independent_process_expires_while_parent_does_no_dispatch_work(tmp_path):
    target = tmp_path/'stops.txt'
    worker = IndependentStopWorker(StopToFileFactory(str(target)), maximum_lease_s=.2)
    try:
        worker.renew(time.monotonic()+.15)
        wait_until(target.exists)
        wait_until(lambda: worker.status()['state'] == 'stop_rpc_returned')
        assert worker.status()['pending_work_contained'] is False
        assert worker.status()['measured_stillness'] is False
        with pytest.raises(RuntimeError): worker.renew(time.monotonic()+.1)
    finally: assert worker.close(1.)
    assert target.read_text().splitlines() == ['requested']


def test_parent_pipe_eof_requests_stop(tmp_path):
    target = tmp_path/'eof.txt'
    worker = IndependentStopWorker(StopToFileFactory(str(target)), maximum_lease_s=.2)
    worker._connection.close()  # Simulated abrupt loss of dispatcher IPC.
    wait_until(target.exists)
    worker._process.join(1)
    assert not worker._process.is_alive()
    worker._closed = True


def test_delayed_renewal_cannot_resurrect_expired_lease(monkeypatch):
    import transition_autonomy.physical.stop_worker as module
    now = [1.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    class Connection:
        def __init__(self): self.index = 0; self.sent = []
        def send(self, value): self.sent.append(value)
        def poll(self, timeout): return True
        def recv(self):
            items = [('renew', 1.1), ('renew', 1.4), ('close', 'shutdown')]
            if self.index == 1: now[0] = 1.2
            result = items[self.index]; self.index += 1; return result
        def close(self): pass
    class Stopper:
        calls = 0
        def stop(self): self.calls += 1
        def close(self): pass
    connection, stopper = Connection(), Stopper()
    module._serve(connection, lambda: stopper, .5)
    assert stopper.calls == 1
    assert connection.sent[-1]['reason'] == 'dispatcher_heartbeat_expired'


def test_ros_envelope_cannot_arm_and_rejects_duplicate_json(tmp_path):
    path = Path(__file__).resolve().parents[1]/'deployment'/'physical_gateway.py'
    spec = importlib.util.spec_from_file_location('test_physical_ros_wrapper', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    gateway = PhysicalGateway(mode='shadow', ledger=tmp_path/'shadow.sqlite', identity='source', observation=Diagnostics())
    try:
        for raw in ('{"schema_version":1,"schema_version":1}',
                    json.dumps({'schema_version':1, 'clock_id':'c', 'operation':'arm'})):
            with pytest.raises(ValueError): module.handle(gateway, raw, 'c')
        envelope = dict(schema_version=1, clock_id='c', operation='start', command=command().to_dict())
        assert module.handle(gateway, json.dumps(envelope), 'c')['status'] == 'would_propose'
    finally: gateway.close()


def test_assembly_checks_protection_before_constructing_sdk_ports(tmp_path, monkeypatch):
    import transition_autonomy.physical.stop_worker as stop_module
    samples = FakeTransport(); gate = make_gate(config(), samples)
    def must_not_connect(*args, **kwargs): raise AssertionError('no SDK connection allowed')
    monkeypatch.setattr(stop_module, 'IndependentStopWorker', must_not_connect)
    with pytest.raises(RuntimeError, match='Commissioned'):
        assemble_supervised(manifest=gate.manifest, registry_root=tmp_path,
            observer=samples, release_gate=gate, protection_resolver=lambda: None,
            trusted_protection_issuers={'fictional-monitor'}, wheel_path='/fictional.whl', clock=lambda: 1.)


@pytest.mark.parametrize('lose_protection', [False, True])
def test_common_assembly_runs_actual_backend_reducer_with_fictional_ports(tmp_path, monkeypatch, lose_protection):
    import transition_autonomy.physical.stop_worker as stop_module
    import transition_autonomy.physical.vendor_sdk as vendor_module
    samples = FakeTransport(); cfg = config(); gate = make_gate(cfg, samples); stop = Stop(); motion = Motion()
    stop_creations = []
    def make_stop(*args, **kwargs): stop_creations.append('stop-only'); return stop
    monkeypatch.setattr(stop_module, 'IndependentStopWorker', make_stop)
    constructions = []
    def motion_factory(*args):
        from transition_autonomy.physical.manifest import Manifest
        parsed = Manifest.parse(args[1])
        assert parsed.payload['commissioning']['receipt_id'] == 'test-receipt'
        assert parsed.digest == gate.manifest.digest
        def construct(): constructions.append('named-port'); return motion
        return construct
    monkeypatch.setattr(vendor_module, 'ProfileMotionFactory', motion_factory)
    protection_valid = [True]
    def proof():
        return ProtectionEvidence('fictional-monitor', 'fake-owner', gate.manifest.digest,
            'fake-enforcement', 'fake-watchdog', samples.sample.sampled_at, 20., True, True, protection_valid[0])
    assembled = assemble_supervised(manifest=gate.manifest, registry_root=tmp_path,
        observer=samples, release_gate=gate, protection_resolver=proof,
        trusted_protection_issuers={'fictional-monitor'}, wheel_path='/fictional.whl',
        clock=lambda: samples.sample.sampled_at)
    backend, transport = assembled['backend'], assembled['transport']
    assert constructions == []  # Motion SDK connection itself is deferred until guarded dispatch.
    with pytest.raises(RuntimeError, match='live owner'):
        assemble_supervised(manifest=gate.manifest, registry_root=tmp_path,
            observer=samples, release_gate=gate, protection_resolver=proof,
            trusted_protection_issuers={'fictional-monitor'}, wheel_path='/fictional.whl',
            clock=lambda: samples.sample.sampled_at)
    assert stop_creations == ['stop-only'] and stop.requests == []
    backend.arm(1., commissioning_receipt='test-receipt')
    assert backend.start(command(), 1.).status == 'accepted'
    assert constructions == ['named-port'] and len(motion.calls) == 1
    assert backend.poll(1.) == []
    samples.sample = replace(samples.sample, sampled_at=1.2)
    protection_valid[0] = not lose_protection
    event = backend.poll(1.2)[0]
    if lose_protection:
        assert event.status == 'unknown' and event.facts == {}
        assert backend.command_status('cmd1', 1.2).status == 'unknown'
        assert transport._store.unresolved() == ['cmd1']
        # Pure fixture containment is only for test teardown, never deployment.
        backend.request_stop(1.2); backend.poll(1.2)
        samples.sample = replace(samples.sample, sampled_at=1.4)
        backend.poll(1.4)
        transport.inhibit = samples.inhibit
        assert backend.shutdown(1.4)['closed']
    else:
        assert event.status == 'succeeded' and event.evidence['measured_stillness'] is True
        transport.terminal(event)
        assert transport._store.unresolved() == []
        backend.close()
    transport.close()
