"""Protocol and failure tests with a fake transport; no robot is contacted."""
from dataclasses import replace
from concurrent.futures import ThreadPoolExecutor

import pytest

from physical_fixtures import config as make_config, make_gate, bundle, finish
from transition_autonomy.backends.base import SkillCommand
from transition_autonomy.backends.booster_backend import (
    BoosterK1Backend, K1Config, K1Sample, PointProfile,
)


class FakeTransport:
    enforcement_record = "fake-enforcement"

    def __init__(self):
        self.sequence = 0
        self.sample = K1Sample(1.0, "test-serial", "test-fw", "K1", 1, 2,
                               (0.0,) * 8, (0.0,) * 8, {"left": (0.1, 0.2, 0.3), "right": (0.1, -0.2, 0.3)}, site_ready=True)
        self.moves = []
        self.stops = 0
        self.fail_dispatch = False
        self.fail_stop = False

    @property
    def sample(self):
        return self._sample

    @sample.setter
    def sample(self, value):
        self.sequence += 1
        self._sample = replace(value, bundle=bundle(value.sampled_at, self.sequence))

    def read(self):
        return self.sample

    def move_admitted(self, profile, admission):
        self.moves.append(profile)
        if self.fail_dispatch:
            raise TimeoutError("ack lost")
        import json
        return dict(command_id=json.loads(admission.command_json)['command_id'],
                    owner_epoch=admission.owner_epoch, enforcement_record=self.enforcement_record)

    def stop(self):
        self.stops += 1
        if self.fail_stop:
            raise TimeoutError('stop reply lost')

    def inhibit(self, owner):
        return dict(owner_epoch=owner, pending_work_contained=True, enforcement_record=self.enforcement_record)


def setup_backend(tmp_path, *, enabled=True):
    transport = FakeTransport()
    config = make_config(enabled)
    return BoosterK1Backend(config, transport, tmp_path / "ledger.db", release_gate=make_gate(config, transport)), transport, config



def command(**changes):
    c = SkillCommand("cmd1", "run1", "point-a", "point", {"profile": "point_a"},
                     {}, "authority", 1, 1.0, 4.0)
    return replace(c, **changes)


def test_disabled_never_dispatches(tmp_path):
    backend, transport, _ = setup_backend(tmp_path, enabled=False)
    with pytest.raises(RuntimeError, match="disabled"):
        backend.arm(1.0, commissioning_receipt="test-receipt")
    assert backend.start(command(), 1.0).status == "failed"
    assert transport.moves == []
    finish(backend)


def test_ack_not_completion_and_dwell_requires_fresh_state(tmp_path):
    backend, transport, _ = setup_backend(tmp_path)
    backend.arm(1.0, commissioning_receipt="test-receipt")
    accepted = backend.start(command(), 1.0)
    assert accepted.status == "accepted"
    assert accepted.facts["remote.pointed_target"].status == "unknown"
    assert backend.poll(1.0) == []
    transport.sample = replace(transport.sample, sampled_at=1.2)
    result = backend.poll(1.2)[0]
    assert result.status == "succeeded"
    assert result.facts["remote.pointed_target"].value == "A"
    assert result.evidence["orientation_verified"] is False
    finish(backend)


def test_duplicate_and_changed_payload(tmp_path):
    backend, transport, _ = setup_backend(tmp_path)
    backend.arm(1.0, commissioning_receipt="test-receipt")
    backend.start(command(), 1.0)
    assert backend.start(command(), 1.0).status == "accepted"
    assert len(transport.moves) == 1
    with pytest.raises(ValueError, match="different payload"):
        backend.start(command(run_id="other"), 1.0)
    finish(backend)


def test_restart_cannot_replay_or_claim_quiescence(tmp_path):
    backend, transport, config = setup_backend(tmp_path)
    backend.arm(1.0, commissioning_receipt="test-receipt")
    transport.fail_dispatch = True
    assert backend.start(command(), 1.0).status == "unknown"
    finish(backend)
    restarted = BoosterK1Backend(config, transport, tmp_path / "ledger.db", release_gate=make_gate(config, transport))
    assert not restarted.observe(1.0).quiescent
    with pytest.raises(RuntimeError, match="Unresolved"):
        restarted.arm(1.0, commissioning_receipt="test-receipt")
    assert restarted.start(command(), 1.0).status == "unknown"
    assert len(transport.moves) == 1
    finish(restarted)


def test_stop_requires_observed_stillness(tmp_path):
    backend, transport, _ = setup_backend(tmp_path)
    backend.arm(1.0, commissioning_receipt="test-receipt")
    backend.start(command(), 1.0)
    backend.request_stop(1.0)
    transport.sample = replace(transport.sample, arm_velocities=(0.2,) * 8)
    assert backend.poll(1.0) == []
    transport.sample = replace(transport.sample, sampled_at=1.1, arm_velocities=(0.0,) * 8)
    assert backend.poll(1.1) == []
    transport.sample = replace(transport.sample, sampled_at=1.3)
    assert backend.poll(1.3)[0].status == "canceled"
    assert transport.stops == 1
    finish(backend)


def test_stale_state_marks_unknown_and_blocks_new_work(tmp_path):
    backend, transport, _ = setup_backend(tmp_path)
    backend.arm(1.0, commissioning_receipt="test-receipt")
    backend.start(command(), 1.0)
    assert backend.poll(2.0)[0].status == "unknown"
    assert backend.start(command(command_id="cmd2"), 2.0).status == "failed"
    finish(backend)


def test_reject_raw_joint_payload_and_identity_mismatch(tmp_path):
    backend, transport, _ = setup_backend(tmp_path)
    backend.arm(1.0, commissioning_receipt="test-receipt")
    assert backend.start(command(parameters={"joints": [0] * 22}), 1.0).status == "failed"
    transport.sample = replace(transport.sample, serial="unexpected")
    assert backend.start(command(), 1.0).status == "failed"
    assert transport.moves == []
    finish(backend)


def test_second_adapter_cannot_own_same_journal(tmp_path):
    backend, transport, config = setup_backend(tmp_path)
    with pytest.raises(RuntimeError, match="already has an owner"):
        BoosterK1Backend(config, transport, tmp_path / "ledger.db")
    finish(backend)


def test_completed_receipt_retains_measured_evidence_after_restart(tmp_path):
    backend, transport, config = setup_backend(tmp_path)
    backend.arm(1.0, commissioning_receipt="test-receipt")
    backend.start(command(), 1.0)
    backend.poll(1.0)
    transport.sample = replace(transport.sample, sampled_at=1.2)
    backend.poll(1.2)
    finish(backend)
    restarted = BoosterK1Backend(config, transport, tmp_path / "ledger.db", release_gate=make_gate(config, transport))
    receipt = restarted.start(command(), 1.2)
    assert receipt.status == "succeeded"
    assert receipt.facts["remote.pointed_target"].value == "A"
    assert receipt.evidence["sampled_at"] == 1.2
    assert len(transport.moves) == 1
    finish(restarted)


def test_repeated_sample_cannot_satisfy_settling_dwell(tmp_path):
    backend, transport, _ = setup_backend(tmp_path)
    backend.arm(1.0, commissioning_receipt="test-receipt")
    backend.start(command(), 1.0)
    assert backend.poll(1.0) == []
    assert backend.poll(1.2) == []  # Same sensor sample, despite elapsed wall time.
    transport.sample = replace(transport.sample, sampled_at=1.2)
    assert backend.poll(1.2)[0].status == "succeeded"
    finish(backend)


def test_completed_target_expires_and_observation_invalidates_drift(tmp_path):
    backend, transport, _ = setup_backend(tmp_path)
    backend.arm(1.0, commissioning_receipt="test-receipt")
    backend.start(command(), 1.0)
    backend.poll(1.0)
    transport.sample = replace(transport.sample, sampled_at=1.2)
    result = backend.poll(1.2)[0]
    assert result.facts["remote.pointed_target"].valid_until == 1.45
    assert backend.observe(1.2).facts["remote.pointed_target"].status == "known"
    transport.sample = replace(transport.sample, sampled_at=1.3, hand_positions={"left": (0.3, 0.2, 0.3), "right": (0.1, -0.2, 0.3)})
    assert backend.observe(1.3).facts["remote.pointed_target"].status == "unknown"
    finish(backend)


def test_sample_gap_restarts_dwell_and_regression_is_unknown(tmp_path):
    backend, transport, _ = setup_backend(tmp_path)
    backend.arm(1.0, commissioning_receipt="test-receipt")
    backend.start(command(), 1.0)
    assert backend.poll(1.0) == []
    transport.sample = replace(transport.sample, sampled_at=2.0)
    assert backend.poll(2.0) == []
    transport.sample = replace(transport.sample, sampled_at=1.9)
    event = backend.poll(2.0)[0]
    assert event.status == "unknown"
    assert "regressed" in event.detail
    finish(backend)


def test_main_thread_construction_can_transfer_to_one_runtime_worker(tmp_path):
    backend, transport, _ = setup_backend(tmp_path)

    def serialized_worker():
        backend.arm(1.0, commissioning_receipt="test-receipt")
        accepted = backend.start(command(), 1.0)
        duplicate = backend.command_status(command().command_id, 1.0)
        return accepted.status, duplicate.status

    with ThreadPoolExecutor(max_workers=1) as executor:
        assert executor.submit(serialized_worker).result() == ("accepted", "accepted")
    assert len(transport.moves) == 1
    finish(backend)


def test_running_protection_loss_never_grants_backend_success(tmp_path):
    backend, transport, _ = setup_backend(tmp_path)
    backend.arm(1., commissioning_receipt='test-receipt')
    backend.start(command(), 1.)
    def lost_protection(command, sample):
        raise RuntimeError('commissioned protection revoked')
    transport.check_running_protection = lost_protection
    event = backend.poll(1.)[0]
    assert event.status == 'unknown' and event.facts == {}
    assert backend.command_status('cmd1', 1.).status == 'unknown'
    # Loss of protection must not prevent measured stop monitoring afterwards.
    finish(backend)


def test_protection_rechecked_immediately_before_terminal_ledger_write(tmp_path):
    backend, transport, _ = setup_backend(tmp_path)
    backend.arm(1., commissioning_receipt='test-receipt')
    backend.start(command(), 1.)
    assert backend.poll(1.) == []
    transport.sample = replace(transport.sample, sampled_at=1.2)
    calls = []
    def lost_at_terminal(command, sample):
        calls.append(sample.sampled_at)
        if len(calls) == 2:
            raise RuntimeError('protection lost during terminal reduction')
    transport.check_running_protection = lost_at_terminal
    event = backend.poll(1.2)[0]
    assert calls == [1.2, 1.2]
    assert event.status == 'unknown' and event.facts == {}
    assert backend.command_status('cmd1', 1.2).status == 'unknown'
    finish(backend)


@pytest.mark.parametrize('late_now,reason', [(1.5, 'Measured state expired'), (4.1, 'Execution deadline expired')])
def test_slow_protection_hook_cannot_refresh_old_sample_or_deadline(tmp_path, late_now, reason):
    backend, transport, _ = setup_backend(tmp_path)
    clock = [1.]
    backend.clock = lambda: clock[0]
    backend.arm(1., commissioning_receipt='test-receipt')
    backend.start(command(), 1.)
    backend.poll(1.)
    transport.sample = replace(transport.sample, sampled_at=1.2)
    clock[0] = 1.2
    def slow_protection(command, sample):
        clock[0] = late_now
    transport.check_running_protection = slow_protection
    event = backend.poll(1.2)[0]
    assert event.status == 'unknown' and reason in event.detail
    assert event.facts == {} and backend.command_status('cmd1', late_now).status == 'unknown'
    backend.clock = lambda: transport.sample.sampled_at
    finish(backend)


def test_cancellation_monitoring_does_not_require_live_motion_protection(tmp_path):
    backend, transport, _ = setup_backend(tmp_path)
    backend.arm(1., commissioning_receipt='test-receipt')
    backend.start(command(), 1.)
    backend.request_stop(1.)
    def no_longer_available(command, sample):
        pytest.fail('cancellation must remain observable without motion protection')
    transport.check_running_protection = no_longer_available
    assert backend.poll(1.) == []
    transport.sample = replace(transport.sample, sampled_at=1.2)
    event = backend.poll(1.2)[0]
    assert event.status == 'canceled' and event.evidence['stop_confirmed']
    assert not event.facts
    finish(backend)
