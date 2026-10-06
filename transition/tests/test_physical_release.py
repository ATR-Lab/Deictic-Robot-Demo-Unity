"""Release gates/fault injection use fictional evidence and never connect to K1."""
from dataclasses import replace
import json
import time

import pytest

from physical_fixtures import config, manifest_data, make_manifest, make_gate, bundle, finish
from test_booster_backend import setup_backend, command, FakeTransport
from transition_autonomy.backends.booster_backend import BoosterK1Backend, BoosterSDKTransport
from transition_autonomy.physical.admission import ApprovalRegistry, TaskGrant
from transition_autonomy.physical.deployment import device_ledger_path, assemble_disarmed
from transition_autonomy.physical.manifest import Manifest
from transition_autonomy.physical.observer_worker import ReadOnlyWorker
from transition_autonomy.physical.sampling import LatestSampleCache


@pytest.mark.parametrize('mutate', [
    lambda d: d.update(extra=True),
    lambda d: d.update(schema_version=True),
    lambda d: d.update(physical_actuation_enabled=1),
    lambda d: d['profiles']['point_a'].update(duration_ms=True),
    lambda d: d['profiles']['point_a'].update(duration_ms=1.1),
    lambda d: d['profiles']['home'].update(target_label='A'),
    lambda d: d['profiles']['point_a'].update(position_m=[float('nan'), 0, 0]),
    lambda d: d['profiles']['point_a'].update(position_m=[True, 0, 0]),
    lambda d: d['profiles']['point_a'].update(position_m=[0, 0]),
    lambda d: d['observation'].update(max_age_s=0),
    lambda d: d['sdk'].update(vendor_dds_domain=True),
    lambda d: d['sdk'].update(version='unreviewed'),
    lambda d: d['commissioning'].update(manifest_sha256='f'*64),
])
def test_manifest_rejects_malformed_or_unreviewed_values(mutate):
    data = manifest_data(config())
    mutate(data)
    with pytest.raises(ValueError):
        Manifest.parse(json.dumps(data))


def test_duplicate_json_and_detached_immutable_profiles():
    with pytest.raises(ValueError, match='Duplicate'):
        Manifest.parse('{"schema_version":1,"schema_version":1}')
    data = manifest_data(config())
    manifest = Manifest.parse(json.dumps(data))
    data['profiles']['point_a']['position_m'] = [99, 99, 99]
    assert manifest.payload['profiles']['point_a']['position_m'] == (.1, .2, .3)
    with pytest.raises(TypeError):
        manifest.payload['profiles']['point_a']['duration_ms'] = 1
    with pytest.raises(TypeError):
        config().profiles['point_a'] = None


def test_digest_excludes_entire_approval_block_but_does_not_approve():
    data = manifest_data(config())
    first = Manifest.parse(json.dumps(data))
    data['commissioning']['reviewer'] = 'different-self-asserted-reviewer'
    assert Manifest.parse(json.dumps(data)).digest == first.digest
    with pytest.raises(RuntimeError, match='trusted'):
        ApprovalRegistry().require(make_manifest(config()), 'point_a', 150.)


def test_missing_profile_is_observation_only_not_arm_permission():
    data = manifest_data(config())
    data['profiles']['home'] = None
    m = Manifest.parse(json.dumps(data))
    assert 'profiles.home' in m.unresolved
    with pytest.raises(ValueError, match='Unresolved'):
        m.require_complete()


def test_gate_is_required_even_with_true_enable_and_receipt(tmp_path):
    c, t = config(), FakeTransport()
    b = BoosterK1Backend(c, t, tmp_path/'gate-missing.sqlite')
    with pytest.raises(RuntimeError, match='gate'):
        b.arm(1., commissioning_receipt='test-receipt')
    assert t.moves == []
    b.close()


def test_stock_sdk_move_cannot_bypass_gateway_without_even_constructing_client():
    sdk = object.__new__(BoosterSDKTransport)
    with pytest.raises(RuntimeError, match='physical motion is disabled'):
        sdk.move(config().profiles['point_a'])


def test_matching_manifest_and_trusted_approval_both_required(tmp_path):
    c, t = config(), FakeTransport()
    gate = make_gate(c, t)
    with pytest.raises(ValueError, match='manifest'):
        BoosterK1Backend(replace(c, allowed_mode=999), t, tmp_path/'x', release_gate=gate)
    b = BoosterK1Backend(c, t, tmp_path/'x', release_gate=gate)
    gate.registry.revoke('test-receipt')
    with pytest.raises(RuntimeError, match='revoked'):
        b.arm(1., commissioning_receipt='test-receipt')
    b.close()


@pytest.mark.parametrize('fault', ['deadline', 'grant', 'moving', 'owner'])
def test_recheck_after_durable_intent_prevents_late_dispatch(tmp_path, fault):
    b, t, _ = setup_backend(tmp_path)
    b.arm(1., commissioning_receipt='test-receipt')
    real_record = b._record
    def delayed_record(*args, **kwargs):
        receipt = real_record(*args, **kwargs)
        if args[1] == 'unknown':
            if fault == 'deadline':
                t.sample = replace(t.sample, sampled_at=3.)
                b.clock = lambda: 3.
            elif fault == 'grant':
                b.release_gate.grant_resolver = lambda _: TaskGrant('authority', 1, 20., frozenset({'point-a'}), revoked=True)
            elif fault == 'moving':
                t.sample = replace(t.sample, arm_velocities=(.1,)*8)
            else:
                original = b.release_gate.site_resolver
                b.release_gate.site_resolver = lambda: replace(original(), owner_epoch='other-owner')
        return receipt
    b._record = delayed_record
    event = b.start(command(), 1.)
    assert event.status == 'failed'
    assert event.detail.startswith('No dispatch:')
    assert t.moves == []
    finish(b)


def test_read_latency_consumes_admission_budget(tmp_path):
    b, t, _ = setup_backend(tmp_path)
    b.arm(1., commissioning_receipt='test-receipt')
    original = t.read
    def slow_read():
        t.sample = replace(t.sample, sampled_at=3.)
        return original()
    t.read = slow_read
    b.clock = lambda: 3.
    assert b.start(command(), 1.).status == 'failed'
    assert not t.moves
    finish(b)


def test_repeated_stop_ticks_do_not_reset_dwell_or_repeat_rpc(tmp_path):
    b, t, _ = setup_backend(tmp_path)
    b.arm(1., commissioning_receipt='test-receipt')
    b.start(command(), 1.)
    for at in (1., 1.05, 1.1, 1.15):
        b.request_stop(at)
        t.sample = replace(t.sample, sampled_at=at)
        events = b.poll(at)
    assert t.stops == 1
    assert b.command_status('cmd1', 1.15).status == 'canceled'
    finish(b)


def test_fault_does_not_stop_observation_or_invent_success(tmp_path):
    b, t, _ = setup_backend(tmp_path)
    b.arm(1., commissioning_receipt='test-receipt')
    t.fail_dispatch = True
    assert b.start(command(), 1.).status == 'unknown'
    b.request_stop(1.)
    for at in (1.1, 1.25):
        t.sample = replace(t.sample, sampled_at=at, site_ready=False)
        events = b.poll(at)
    assert events[0].status == 'unknown'
    assert events[0].evidence['measured_stillness']
    assert b.observe(1.25).connected
    assert not b.observe(1.25).quiescent
    with pytest.raises(RuntimeError, match='Active/ambiguous'):
        b.close()
    # External fake fencing explicitly contains all pending work; ledger unknown remains.
    assert b.shutdown(1.25)['outcome_unknown']


def test_stop_admitted_before_ack_cannot_be_overwritten_with_success(tmp_path):
    b, t, _ = setup_backend(tmp_path)
    b.arm(1., commissioning_receipt='test-receipt')
    original = t.move_admitted
    def racing(profile, admission):
        b.request_stop(1.)
        return original(profile, admission)
    t.move_admitted = racing
    assert b.start(command(), 1.).status == 'unknown'
    b.poll(1.)
    t.sample = replace(t.sample, sampled_at=1.2)
    assert b.poll(1.2)[0].status == 'canceled'
    finish(b)


def test_device_registry_reused_across_runs_and_identity_bound(tmp_path):
    c, t = config(), FakeTransport()
    gate = make_gate(c, t)
    b = assemble_disarmed(gate.manifest, t, registry_root=tmp_path, release_gate=gate, clock=lambda: t.sample.sampled_at)
    b.arm(1., commissioning_receipt='test-receipt')
    t.fail_dispatch = True
    b.start(command(), 1.)
    finish(b)
    next_run = assemble_disarmed(gate.manifest, t, registry_root=tmp_path, release_gate=gate, clock=lambda: t.sample.sampled_at)
    with pytest.raises(RuntimeError, match='Unresolved'):
        next_run.arm(t.sample.sampled_at, commissioning_receipt='test-receipt')
    assert len(t.moves) == 1
    next_run.close()
    with pytest.raises(RuntimeError, match='another robot'):
        BoosterK1Backend(replace(c, expected_serial='different'), t, device_ledger_path(tmp_path, c.expected_serial))


def test_acquisition_unknown_and_skew_cannot_become_fresh_evidence():
    limits = make_manifest(config()).payload['observation']
    with pytest.raises(RuntimeError, match='uncommissioned'):
        replace(bundle(), source_progression_verified=False).validate(1., limits)
    with pytest.raises(RuntimeError, match='uncommissioned'):
        replace(bundle(), transport_uncertainty_s=None).validate(1., limits)
    values = list(bundle().acquisitions)
    values[0] = replace(values[0], earliest=.9)
    with pytest.raises(RuntimeError, match='skew'):
        replace(bundle(), acquisitions=values).validate(1., limits)


def test_receiver_timestamps_cannot_advance_cached_acquisition_dwell(tmp_path):
    b, t, _ = setup_backend(tmp_path)
    b.arm(1., commissioning_receipt='test-receipt')
    b.start(command(), 1.)
    b.poll(1.)
    old = t.sample
    t._sample = replace(old, sampled_at=1.2, bundle=replace(old.bundle,
        bundle_id='new-receipt-old-pixels', sequence=old.bundle.sequence+1, published_at=1.2))
    assert b.poll(1.2) == []
    assert b.command_status('cmd1', 1.2).status == 'accepted'
    finish(b)


def test_unknown_uncertainty_cannot_arm(tmp_path):
    b, t, _ = setup_backend(tmp_path)
    t._sample = replace(t.sample, bundle=replace(t.sample.bundle, position_uncertainty_m=None))
    with pytest.raises(RuntimeError, match='uncertainty'):
        b.arm(1., commissioning_receipt='test-receipt')
    assert not t.moves
    b.close()


def test_stop_failure_is_not_canceled_by_elapsed_time_or_repeat(tmp_path):
    b, t, _ = setup_backend(tmp_path)
    b.arm(1., commissioning_receipt='test-receipt')
    b.start(command(), 1.)
    t.fail_stop = True
    b.request_stop(1.)
    b.poll(1.)
    t.sample = replace(t.sample, sampled_at=1.2)
    b.request_stop(1.2)
    assert b.poll(1.2)[0].status == 'unknown'
    assert t.stops == 1
    assert b.stop_confirmed  # Measured stillness, distinct from the failed RPC.
    finish(b)


def test_bounded_commissioning_only_admits_exact_reviewed_command_ids(tmp_path):
    b, t, _ = setup_backend(tmp_path)
    approval = b.release_gate.registry._records['test-receipt']
    b.release_gate.registry = ApprovalRegistry([replace(approval, permit_class='bounded_commissioning',
        commissioning_command_ids=frozenset({'reviewed-trial-1'}))], trusted_issuers={'fake-reviewer'})
    b.arm(1., commissioning_receipt='test-receipt')
    assert b.start(command(), 1.).status == 'failed'
    assert not t.moves
    finish(b)


def test_sampler_dead_duplicate_or_new_boot_does_not_refresh_cache():
    now = [1.]
    cache = LatestSampleCache(clock=lambda: now[0], max_age_s=.25)
    t = FakeTransport()
    cache.publish(t.sample)
    with pytest.raises(RuntimeError, match='sequence'):
        cache.publish(t.sample)
    now[0] = 1.3
    with pytest.raises(RuntimeError, match='expired'):
        cache.read()
    changed = replace(t.sample, bundle=bundle(1.3, 2, boot_id='new-boot'))
    with pytest.raises(RuntimeError, match='boot changed'):
        cache.publish(changed)
    with pytest.raises(RuntimeError, match='boot changed'):
        cache.read()


class StuckObserver:
    def read(self):
        time.sleep(60)


def test_stuck_read_is_process_isolated_and_never_retried():
    observer = ReadOnlyWorker(StuckObserver, timeout_s=.15)
    started = time.monotonic()
    with pytest.raises(RuntimeError, match='deadline'):
        observer.read()
    assert time.monotonic()-started < 1.5
    with pytest.raises(RuntimeError, match='deadline'):
        observer.read()
    assert not observer._process.is_alive()
