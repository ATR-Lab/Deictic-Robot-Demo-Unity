from pathlib import Path
import sqlite3

import pytest

from transition_autonomy.backends.base import FactUpdate
from transition_autonomy.backends.logical import LogicalBackend
from transition_autonomy.journal import Journal
from transition_autonomy.model import load_task
from transition_autonomy.runtime import Runtime, RecoveryRequired
from transition_autonomy.scoring import correct_responses

ROOT = Path(__file__).resolve().parents[1]


def make(tmp_path, policy="consequence", effects=None, extra_facts=None):
    task = load_task(ROOT / "examples/logical_cowork.json")
    task.initial_facts.update(extra_facts or {})
    world = {"read_cue": {"duration": 2, "effects": effects or {"remote.cue": "blue"}},
             "present_reference": {"duration": 1, "effects": {"remote.reference_presented": True}}}
    backend = LogicalBackend(world)
    journal = Journal(tmp_path / (policy + ".sqlite"))
    runtime = Runtime(task, backend, journal, policy=policy, operator_mode="test")
    runtime.grant(set(task.skills), 100, 0)
    runtime.acknowledge(0)
    runtime.set_attention("local", 0)
    return runtime, backend


def test_policies_diverge_with_same_facts_and_no_idle(tmp_path):
    a, _ = make(tmp_path, "ordinary")
    b, _ = make(tmp_path, "consequence")
    a.tick(0); b.tick(0)
    assert a.active.skill_id == "read_cue"
    assert b.active.skill_id == "present_reference"
    for now in (1, 2, 3, 4):
        a.tick(now); b.tick(now)
    assert a.state.completed == b.state.completed == set(a.task.skills)
    assert a.journal.verify()["valid"] and b.journal.verify()["valid"]


def test_return_clock_includes_boundary_and_scoring_precedes_guard(tmp_path):
    r, _ = make(tmp_path, "ordinary")
    r.tick(0)
    r.set_local_quiescent(True, 0.5)
    r.cue_return("checkpoint", 5, 0.5)
    assert r.lease.status == "draining"
    r.tick(1)
    assert r.lease.status == "draining"
    r.tick(2, auto_dispatch=False)
    assert r.lease.exposed_at == 2
    assert r.respond("execute", "select_amber", 2.2)["correct"] is False
    assert r.lease.result["cue_to_snapshot"] == 1.5
    kinds = [e["kind"] for e in r.journal.events()]
    assert kinds.index("raw_response") < kinds.index("semantic_score")


def test_no_snapshot_deadline_is_system_not_human_error(tmp_path):
    r, _ = make(tmp_path, "ordinary")
    r.tick(0); r.set_local_quiescent(True, 0.1)
    r.cue_return("checkpoint", 0.5, 0.1)
    r.tick(0.6, auto_dispatch=False)
    assert r.lease.result["outcome"] == "system_unavailable"
    assert not r.lease.result["human_error"]


def test_duplicate_dispatch_does_not_repeat_effect(tmp_path):
    r, b = make(tmp_path)
    r.dispatch("present_reference", 0, "stable-id")
    r.dispatch("present_reference", 0.1, "stable-id")
    assert b.starts == 1
    with pytest.raises(ValueError):
        r.dispatch("read_cue", 0.2, "stable-id")


def test_link_loss_latches_and_never_replays_motion(tmp_path):
    r, b = make(tmp_path)
    r.tick(0); b.connected = False; r.tick(0.2)
    assert r.fault and r.active
    b.connected = True; r.tick(0.3)
    assert b.starts == 1 and r.fault
    assert r.reconcile(0.4)["automatic_retry"] is False


def test_out_of_support_effects_retained_but_not_success(tmp_path):
    r, _ = make(tmp_path, "ordinary", effects={"remote.cue": "unexpected"})
    r.tick(0); r.tick(2)
    assert "read_cue" not in r.state.completed
    assert r.state.facts["remote.cue"].value == "unexpected"
    assert r.fault == "out-of-model effects"


def test_snapshot_semantic_change_invalidates_but_refresh_does_not(tmp_path):
    r, b = make(tmp_path, extra_facts={"local.unrelated": False})
    r.set_local_quiescent(True, 0); r.cue_return("checkpoint", 5, 0)
    old = r.state.facts["remote.cue"].version
    r._apply_updates({"remote.cue": FactUpdate("pending", 0.1)}, 0.1)
    assert r.state.facts["remote.cue"].version == old and r.lease.status == "exposed"
    r.update_local("local.unrelated", True, 0.2)
    assert r.lease.status == "exposed"
    r.update_local("local.variant", "B", 0.3)
    assert r.lease.result["outcome"] == "technical_failure"


def test_existing_journal_cannot_restart_physical_outbox(tmp_path):
    r, b = make(tmp_path)
    r.dispatch("present_reference", 0, "before-crash")
    with pytest.raises(RecoveryRequired):
        Runtime(r.task, b, r.journal)


def test_unknown_evidence_has_independent_correct_response(tmp_path):
    r, _ = make(tmp_path)
    r.set_local_quiescent(True, 0); r.cue_return("checkpoint", 5, 0)
    assert r.respond("request_evidence", "cue", 0.1)["correct"] is True
    assert not r.state.completed


def test_journal_tampering_detected(tmp_path):
    r, _ = make(tmp_path)
    r.journal.db.execute("UPDATE events SET kind='tampered' WHERE seq=1")
    r.journal.db.commit()
    with pytest.raises(ValueError):
        r.journal.verify()


def test_authority_change_during_snapshot_is_technical(tmp_path):
    r, _ = make(tmp_path)
    r.set_local_quiescent(True, 0); r.cue_return("checkpoint", 5, 0)
    r.grant(set(r.task.skills), 100, 0.1)
    assert r.lease.result["outcome"] == "technical_failure"
