"""Terminal coherence and immutable Unity return-record acceptance tests."""
from dataclasses import replace

import pytest

from transition_autonomy.backends.base import FactUpdate, Observation
from transition_autonomy.journal import digest
from test_runtime_review import make_runtime, exposed_runtime


@pytest.mark.parametrize("change", ["fault", "boot", "owner", "contradiction", "same_time_contradiction"])
def test_terminal_validation_uses_full_observation_path(monkeypatch, change):
    runtime, backend = make_runtime()
    runtime.dispatch("point", 1)
    backend.finish({"remote.target": FactUpdate("B", 2)}, 2)
    obs = Observation(True, True, 2, "controlled-test")
    if change == "fault":
        obs = replace(obs, fault="motor health invalid")
    elif change == "boot":
        obs = replace(obs, boot_id="restarted")
    elif change == "owner":
        obs = replace(obs, active_command_id="another-command")
    else:
        observed = 2 if change == "same_time_contradiction" else 2.01
        obs = replace(obs, facts={"remote.target": FactUpdate("C", observed)})
    monkeypatch.setattr(backend, "observe", lambda now: obs)
    runtime.tick(2.01, auto_dispatch=False)
    assert not runtime.state.completed
    assert runtime.fault and runtime.active is not None
    assert not any(e["kind"] == "skill_completed" for e in runtime.journal.events())
    if "contradiction" in change:
        assert runtime.state.facts["remote.target"].value == "C"


def test_blocking_terminal_observation_cannot_outlive_authority(monkeypatch):
    clock = [0.0]
    runtime, backend = make_runtime(expires=5.1, clock=lambda: clock[0])
    runtime.dispatch("point", 0)
    backend.finish({"remote.target": FactUpdate("B", 4.9)}, 4.9)
    clock[0] = 4.9

    def slow_observe(now):
        clock[0] = 5.1
        return Observation(True, True, 5.1, "controlled-test", {"remote.target": FactUpdate("B", 5.1)})

    monkeypatch.setattr(backend, "observe", slow_observe)
    runtime.tick(4.9, auto_dispatch=False)
    assert not runtime.state.completed and runtime.fault
    assert runtime.last_now == 5.1


def test_terminal_observation_with_new_grant_cannot_credit_old_command(monkeypatch):
    runtime, backend = make_runtime()
    runtime.dispatch("point", 1)
    backend.finish({"remote.target": FactUpdate("B", 2)}, 2)
    runtime.grant({"point"}, 100, 1.5)
    runtime.tick(2, auto_dispatch=False)
    assert not runtime.state.completed and runtime.fault


def test_success_after_admitted_revoke_never_overwrites_abort():
    runtime, backend = make_runtime()
    runtime.dispatch("point", 1)
    runtime.revoke(1.1)
    backend.finish({"remote.target": FactUpdate("B", 2)}, 2)
    runtime.tick(2, auto_dispatch=False)
    assert not runtime.state.completed
    assert runtime.fault and runtime.active is not None
    assert runtime.state.facts["remote.target"].value == "B"


def test_live_clock_is_refreshed_before_validating_newly_acquired_sample(monkeypatch):
    clock = [0.0]
    runtime, backend = make_runtime(clock=lambda: clock[0])

    def delayed_sample(now):
        clock[0] = .1
        return Observation(True, True, .1, "controlled-test", {"remote.target": FactUpdate("A", .1)})

    monkeypatch.setattr(backend, "observe", delayed_sample)
    runtime.tick(0, auto_dispatch=False)
    assert runtime.fault is None
    assert runtime.state.facts["remote.target"].observed_at == .1


def test_lease_export_and_hash_are_detached_and_retained_after_response():
    runtime, _, lease_id = exposed_runtime()
    exported = runtime.public_state()["lease"]
    snapshot_hash = exported["snapshot_hash"]
    assert snapshot_hash == digest(exported["snapshot"])
    exported["snapshot"]["remote.target"]["value"] = "corrupted by UI"
    assert runtime.lease.snapshot["remote.target"]["value"] == "A"
    result = runtime.respond("defer", "complete", .1, lease_id=lease_id,
                             decision_id="stable", snapshot_hash=snapshot_hash)
    assert result["lease_id"] == lease_id and result["decision_id"] == "stable"
    assert result["snapshot_hash"] == snapshot_hash
    runtime.cue_return("next", 10, .2)
    duplicate = runtime.respond("defer", "complete", .3, lease_id=lease_id,
                                decision_id="stable", snapshot_hash=snapshot_hash)
    assert duplicate == {**result, "duplicate": True}
    assert runtime.leases[0].snapshot_hash == snapshot_hash


def test_wrong_rendered_hash_cannot_acknowledge_or_respond():
    runtime, _ = make_runtime()
    lease = runtime.cue_return("hash", 10, 0)
    with pytest.raises(ValueError, match="hash"):
        runtime.mark_displayed(lease, .1, snapshot_hash="0" * 64)
    assert runtime.lease.displayed_at is None
    runtime.mark_displayed(lease, .1, snapshot_hash=runtime.lease.snapshot_hash)
    with pytest.raises(ValueError, match="hash"):
        runtime.respond("defer", "complete", .2, lease_id=lease, decision_id="first", snapshot_hash="0" * 64)
    assert not any(e["kind"] == "raw_response" for e in runtime.journal.events())


def test_operator_cannot_add_undeclared_local_fact():
    runtime, _ = make_runtime()
    with pytest.raises(ValueError):
        runtime.update_local("local.typo", True, .1)
    assert "local.typo" not in runtime.state.facts


def test_lease_summary_freezes_original_acknowledgement():
    runtime, backend = make_runtime()
    backend.facts = {"remote.target": FactUpdate("B", .1)}
    runtime.tick(.1, auto_dispatch=False)
    runtime.cue_return("summary", 10, .1)
    lease = runtime.public_state()["lease"]
    assert lease["decision_summary"][0]["before"]["value"] == "A"
    assert lease["decision_summary"][0]["current"]["value"] == "B"
    runtime.acknowledge(.2)
    assert runtime.public_state()["decision_summary"] == []
    assert runtime.public_state()["lease"]["decision_summary"] == lease["decision_summary"]
    lease["decision_summary"][0]["current"]["value"] = "mutated"
    assert runtime.public_state()["lease"]["decision_summary"][0]["current"]["value"] == "B"
