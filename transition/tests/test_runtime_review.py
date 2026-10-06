"""Independent contract regression cases; this backend does not model physics."""

import pytest
import sqlite3

from transition_autonomy.backends.base import BackendEvent, FactUpdate, Observation
from transition_autonomy.journal import Journal
from transition_autonomy.model import Outcome, Skill, TaskSpec
from transition_autonomy.runtime import Runtime
from transition_autonomy.scoring import correct_responses


class ControlledBackend:
    name = "logical_simulation"

    def __init__(self):
        self.active = None
        self.pending = []
        self.facts = {}
        self.stops = 0
        self.starts = 0

    def start(self, command, now):
        self.active = command
        self.starts += 1
        return BackendEvent(command.command_id, "accepted", now)

    def poll(self, now):
        pending, self.pending = self.pending, []
        return pending

    def observe(self, now):
        return Observation(True, self.active is None, now, "controlled-test", dict(self.facts))

    def request_stop(self, now):
        self.stops += 1

    def command_status(self, command_id, now):
        return None

    def finish(self, facts, now):
        command = self.active
        assert command is not None
        self.active = None
        self.pending.append(BackendEvent(command.command_id, "succeeded", now, facts))


def make_runtime(*, outcomes=None, write_set=None, expires=100, ready=True, scored=None, rules=None, clock=None):
    outcomes = outcomes or [Outcome("pointed", {"remote.target": "B"})]
    write_set = write_set or {"remote.target"}
    task = TaskSpec(
        "pointing", {"point": Skill("point", preconditions={"local.ready": True},
            write_set=write_set, resources={"arm"}, outcomes=outcomes, duration_max=5)},
        {"remote.target": "revision"},
        {"local.ready": ready, "remote.target": "A", "remote.quality": "unread"},
        scored or {"remote.target"},
        rules or [{"when": {"remote.target": "A"}, "responses": [{"kind": "defer", "target": "complete"}]}],
    )
    backend = ControlledBackend()
    runtime = Runtime(task, backend, Journal(":memory:"), now=0, clock=clock)
    runtime.grant({"point"}, expires, 0)
    runtime.acknowledge(0)
    runtime.set_local_quiescent(True, 0)
    return runtime, backend


def test_idle_exposed_snapshot_does_not_survive_authority_expiry():
    runtime, _ = make_runtime(expires=1)
    runtime.cue_return("q", 10, 0)
    assert runtime.lease.status == "exposed"
    runtime.tick(2, auto_dispatch=False)
    assert runtime.lease.status == "closed"
    assert runtime.lease.result["human_error"] is False
    with pytest.raises(ValueError):
        runtime.respond("defer", "complete", 2)


def test_observed_effects_must_match_one_outcome_not_mix_branches():
    outcomes = [Outcome("target_only", {"remote.target": "B"}), Outcome("quality_only", {"remote.quality": "pass"})]
    runtime, backend = make_runtime(outcomes=outcomes, write_set={"remote.target", "remote.quality"})
    runtime.dispatch("point", 1)
    backend.finish({"remote.target": FactUpdate("B", 2), "remote.quality": FactUpdate("pass", 2)}, 2)
    runtime.tick(2, auto_dispatch=False)
    assert "point" not in runtime.state.completed
    assert runtime.fault is not None


@pytest.mark.parametrize("reported", [
    FactUpdate("B", -1),
    FactUpdate("B", 1, valid_until=1.5),
])
def test_stale_or_predispatch_terminal_evidence_cannot_complete_skill(reported):
    runtime, backend = make_runtime()
    runtime.dispatch("point", 1)
    backend.finish({"remote.target": reported}, 2)
    runtime.tick(2, auto_dispatch=False)
    assert "point" not in runtime.state.completed
    assert runtime.fault is not None


def test_fresh_exact_terminal_evidence_completes_one_step_without_local_inference():
    runtime, backend = make_runtime()
    initial_local = runtime.state.facts["local.ready"]
    runtime.dispatch("point", 1)
    backend.finish({"remote.target": FactUpdate("B", 2)}, 2)
    runtime.tick(2, auto_dispatch=False)
    assert runtime.state.completed == {"point"}
    assert runtime.state.facts["local.ready"] == initial_local
    assert runtime.fault is None


def test_supported_success_without_quiescence_retains_effects_but_not_completion(monkeypatch):
    runtime, backend = make_runtime()
    runtime.dispatch("point", 1)
    backend.finish({"remote.target": FactUpdate("B", 2)}, 2)
    monkeypatch.setattr(backend, "observe", lambda now: Observation(
        True, False, now, "controlled-test", {}))
    runtime.tick(2, auto_dispatch=False)
    assert runtime.state.facts["remote.target"].value == "B"
    assert "point" not in runtime.state.completed
    assert not any(event["kind"] == "skill_completed" for event in runtime.journal.events())
    assert runtime.active is not None
    assert runtime.fault == "terminal command without observed quiescence"
    assert backend.stops > 0


def test_local_prerequisite_requires_explicit_local_commit():
    runtime, backend = make_runtime(ready=False)
    with pytest.raises(ValueError):
        runtime.dispatch("point", 0)
    assert backend.starts == 0
    runtime.update_local("local.ready", True, 0)
    runtime.dispatch("point", 0)
    assert backend.starts == 1
    assert runtime.state.facts["local.ready"].source == "operator_local_commit"


def test_refresh_renews_evidence_without_manufacturing_semantic_change():
    runtime, backend = make_runtime()
    initial = runtime.state.facts["remote.target"].version
    backend.facts = {"remote.target": FactUpdate("A", 1)}
    runtime.tick(1, auto_dispatch=False)
    assert runtime.state.facts["remote.target"].version == initial
    assert runtime.state.facts["remote.target"].observed_at == 1
    backend.facts = {"remote.target": FactUpdate("B", 2)}
    runtime.tick(2, auto_dispatch=False)
    assert runtime.state.facts["remote.target"].version == initial + 1


def test_lease_expires_when_scored_evidence_loses_validity():
    runtime, backend = make_runtime()
    backend.facts = {"remote.target": FactUpdate("A", 0, valid_until=0.2)}
    runtime.cue_return("q", 10, 0)
    backend.facts = {}
    with pytest.raises(ValueError):
        runtime.respond("defer", "complete", 0.3)
    assert runtime.lease.result["outcome"] == "technical_failure"
    assert runtime.lease.result["human_error"] is False


def test_changed_backend_fact_at_response_invalidates_instead_of_scoring_old_view():
    runtime, backend = make_runtime()
    runtime.cue_return("q", 10, 0)
    backend.facts = {"remote.target": FactUpdate("B", 1)}
    with pytest.raises(ValueError):
        runtime.respond("defer", "complete", 1)
    assert runtime.lease.result["human_error"] is False
    assert not any(e["kind"] == "semantic_score" for e in runtime.journal.events())


def test_delayed_response_is_bound_to_its_original_lease():
    runtime, _ = make_runtime()
    old = runtime.cue_return("old", 1, 0)
    runtime.tick(1, auto_dispatch=False)
    current = runtime.cue_return("new", 10, 1)
    assert current != old
    with pytest.raises(ValueError, match="lease"):
        runtime.respond("defer", "complete", 1.1, lease_id=old)
    assert runtime.lease.id == current
    assert runtime.lease.status == "exposed"
    assert not any(e["kind"] == "semantic_score" for e in runtime.journal.events())


def test_scoring_descriptor_checks_value_as_well_as_known_status():
    rules = [{"when": {"remote.target": {"status": "known", "value": "B"}},
              "responses": [{"kind": "execute", "target": "point"}]}]
    assert correct_responses(rules, {"remote.target": {"status": "known", "value": "A"}}) == set()


def test_authority_revision_invalidates_snapshot_without_user_error():
    runtime, _ = make_runtime()
    runtime.cue_return("q", 10, 0)
    runtime.grant({"point"}, 100, 1)
    assert runtime.lease.status == "closed"
    assert runtime.lease.result["human_error"] is False


def exposed_runtime():
    runtime, backend = make_runtime()
    lease_id = runtime.cue_return("durable-response", 10, 0)
    runtime.mark_displayed(lease_id, 0)
    return runtime, backend, lease_id


def test_scorer_exception_preserves_committed_first_response(monkeypatch):
    runtime, backend, lease_id = exposed_runtime()
    calls = []

    def failing_score(*args):
        calls.append(args)
        assert runtime.journal.db.in_transaction is False
        assert runtime.lease.status == "closed"
        assert {"raw_response", "response_claimed", "return_closed"} <= {
            event["kind"] for event in runtime.journal.events()}
        raise RuntimeError("rubric crashed")

    monkeypatch.setattr("transition_autonomy.runtime.score", failing_score)
    result = runtime.respond("defer", "complete", 0.1, lease_id=lease_id, decision_id="first")
    assert result["label"] == "scoring_error" and result["correct"] is None
    assert runtime.lease.status == "closed"
    assert runtime.lease.result["human_error"] is False
    assert runtime.respond("defer", "complete", 0.2, lease_id=lease_id, decision_id="first")["duplicate"]
    assert len(calls) == 1 and backend.starts == 0
    with pytest.raises(ValueError, match="another response"):
        runtime.respond("execute", "point", 0.2, lease_id=lease_id, decision_id="first")
    with pytest.raises(ValueError, match="open confirmed"):
        runtime.respond("execute", "point", 0.2, lease_id=lease_id, decision_id="second")
    events = runtime.journal.events()
    assert [e["kind"] for e in events].count("raw_response") == 1
    assert events[-1]["kind"] == "scoring_error"
    assert runtime.journal.verify()["valid"]


def test_capture_transaction_failure_rolls_back_claim_and_does_not_score(monkeypatch):
    runtime, backend, lease_id = exposed_runtime()
    original_append = runtime.journal.append
    calls = []

    def fail_capture_close(kind, *args, **kwargs):
        if kind == "return_closed":
            raise sqlite3.OperationalError("database or disk is full")
        return original_append(kind, *args, **kwargs)

    monkeypatch.setattr(runtime.journal, "append", fail_capture_close)
    monkeypatch.setattr("transition_autonomy.runtime.score", lambda *args: calls.append(args))
    with pytest.raises(sqlite3.OperationalError, match="disk is full"):
        runtime.respond("defer", "complete", 0.1, lease_id=lease_id, decision_id="retryable")
    assert runtime.lease.status == "exposed" and runtime.lease.result is None
    assert runtime.lease.response_at is None and runtime.lease.closed_at is None
    assert not calls and backend.starts == 0
    assert "retryable" not in runtime._operator_event_ids
    assert not {"raw_response", "response_claimed", "return_closed"} & {
        event["kind"] for event in runtime.journal.events()}
    assert runtime.journal.verify()["valid"]


def test_interruption_after_capture_leaves_pending_score_and_no_second_choice(monkeypatch):
    runtime, backend, lease_id = exposed_runtime()

    class SimulatedProcessInterruption(BaseException):
        pass

    def interrupted_score(*args):
        raise SimulatedProcessInterruption()

    monkeypatch.setattr("transition_autonomy.runtime.score", interrupted_score)
    with pytest.raises(SimulatedProcessInterruption):
        runtime.respond("defer", "complete", 0.1, lease_id=lease_id, decision_id="interrupted")
    assert runtime.lease.status == "closed"
    assert runtime.lease.result["outcome"] == "response_captured"
    assert runtime.lease.result["scoring_status"] == "pending"
    assert runtime.respond("defer", "complete", 0.2, lease_id=lease_id,
        decision_id="interrupted")["label"] == "scoring_pending"
    assert backend.starts == 0
    assert runtime.journal.verify()["valid"]


def test_captured_inputs_survive_mutating_scorer(monkeypatch):
    runtime, _, lease_id = exposed_runtime()

    def mutating_score(rules, facts, kind, target):
        rules.clear()
        facts["remote.target"]["value"] = "corrupted"
        return {"correct": True, "label": "correct"}

    monkeypatch.setattr("transition_autonomy.runtime.score", mutating_score)
    runtime.respond("defer", "complete", 0.1, lease_id=lease_id)
    claim = next(e["data"] for e in runtime.journal.events() if e["kind"] == "response_claimed")
    assert claim["scoring_inputs"]["snapshot"]["remote.target"]["value"] == "A"
    assert claim["scoring_inputs"]["rules"]
    assert runtime.lease.snapshot["remote.target"]["value"] == "A"
    assert runtime.task.scoring_rules


@pytest.mark.parametrize("ingress", [-0.1, float("nan"), float("inf"), 0.2])
def test_display_receipt_rejects_invalid_timestamp(ingress):
    runtime, _ = make_runtime()
    lease_id = runtime.cue_return("display", 10, 0)
    with pytest.raises(ValueError, match="timestamp"):
        runtime.mark_displayed(lease_id, 0.1, ingress_at=ingress)
    assert runtime.lease.displayed_at is None


def test_response_cannot_predate_snapshot_exposure():
    runtime, _ = make_runtime()
    runtime.set_local_quiescent(False, 0)
    lease_id = runtime.cue_return("delayed-exposure", 10, 0)
    runtime.set_local_quiescent(True, 0.2)
    runtime.tick(0.2, auto_dispatch=False)
    runtime.mark_displayed(lease_id, 0.2)
    with pytest.raises(ValueError, match="precedes snapshot"):
        runtime.respond("defer", "complete", 0.3, ingress_at=0.1, lease_id=lease_id)
    assert runtime.lease.status == "exposed"


def test_human_response_cannot_predate_display_acknowledgement():
    runtime, _ = make_runtime()
    lease_id = runtime.cue_return("display-order", 10, 0)
    runtime.mark_displayed(lease_id, 0.2, ingress_at=0.15)
    with pytest.raises(ValueError, match="precedes client display"):
        runtime.respond("defer", "complete", 0.3, ingress_at=0.1, lease_id=lease_id)
    result = runtime.respond("defer", "complete", 0.4, ingress_at=0.25, lease_id=lease_id)
    assert result["correct"] is True
    assert runtime.lease.result["display_ack_to_response"] == pytest.approx(0.1)


def test_late_older_contradiction_preserves_response_and_withholds_human_blame():
    runtime, backend, lease_id = exposed_runtime()
    backend.facts = {"remote.target": FactUpdate("A", 0.2)}
    result = runtime.respond("execute", "point", 0.2, lease_id=lease_id)
    assert result["correct"] is False and runtime.lease.result["human_error"] is True
    backend.facts = {"remote.target": FactUpdate("B", 0.1)}
    runtime.tick(0.3, auto_dispatch=False)
    assert runtime.state.facts["remote.target"].value == "A"  # Newer evidence retained.
    assert runtime.lease.result["outcome"] == "incorrect"  # Original rubric result retained.
    assert runtime.lease.result["human_error"] is False
    assert runtime.lease.result["coherence_status"] == "uncertain"
    assert len(runtime.lease.coherence_flags) == 1
    runtime.tick(0.4, auto_dispatch=False)
    assert len(runtime.lease.coherence_flags) == 1  # Repeated telemetry is not a new incident.
    events = runtime.journal.events()
    assert len([e for e in events if e["kind"] == "raw_response"]) == 1
    assert len([e for e in events if e["kind"] == "semantic_score"]) == 1
    assert len([e for e in events if e["kind"] == "snapshot_coherence_flag"]) == 1


def test_contradiction_after_response_is_not_retroactive_snapshot_violation():
    runtime, backend, lease_id = exposed_runtime()
    runtime.respond("defer", "complete", 0.1, lease_id=lease_id)
    backend.facts = {"remote.target": FactUpdate("B", 0.2)}
    runtime.tick(0.2, auto_dispatch=False)
    assert not runtime.lease.coherence_flags


@pytest.mark.parametrize(("expires", "reason"), [(0.2, "authority_expired"), (5.2, "authority_horizon_exceeded")])
def test_clock_recheck_after_observe_rejects_expired_or_short_grant(monkeypatch, expires, reason):
    clock = [0.0]
    runtime, backend = make_runtime(expires=expires, clock=lambda: clock[0])
    original_observe = backend.observe

    def slow_observe(now):
        observation = original_observe(now)
        clock[0] = 0.3
        return observation

    monkeypatch.setattr(backend, "observe", slow_observe)
    with pytest.raises(ValueError, match=reason):
        runtime.dispatch("point", 0, "not-admitted")
    assert backend.starts == 0 and runtime.active is None
    assert runtime.journal.command("not-admitted") is None
    assert runtime.last_now == 0.3


def test_clock_recheck_after_intent_withholds_dispatch_without_inflight_effects(monkeypatch):
    clock = [0.0]
    runtime, backend = make_runtime(expires=5.2, clock=lambda: clock[0])
    original_append = runtime.journal.append
    original_fact = runtime.state.facts["remote.target"]

    def slow_intent(kind, *args, **kwargs):
        event = original_append(kind, *args, **kwargs)
        if kind == "dispatch_intent":
            clock[0] = 0.3
        return event

    monkeypatch.setattr(runtime.journal, "append", slow_intent)
    with pytest.raises(ValueError, match="authority_horizon_exceeded"):
        runtime.dispatch("point", 0, "persisted-not-sent")
    assert backend.starts == 0 and runtime.active is None
    assert runtime.state.facts["remote.target"] == original_fact
    assert runtime.journal.command("persisted-not-sent")["status"] == "canceled"
    assert runtime.journal.events()[-1]["kind"] == "dispatch_withheld"
    assert runtime.dispatch("point", 0.3, "persisted-not-sent")["duplicate"]
    assert backend.starts == 0
    assert runtime.journal.verify()["valid"]


def test_refresh_clock_preserves_monotonicity_and_sends_at_actual_time(monkeypatch):
    clock = [0.0]
    runtime, backend = make_runtime(clock=lambda: clock[0])
    original_observe = backend.observe
    original_start = backend.start
    send_times = []

    def slow_observe(now):
        observation = original_observe(now)
        clock[0] = 0.1
        return observation

    def capture_start(command, now):
        send_times.append(now)
        return original_start(command, now)

    monkeypatch.setattr(backend, "observe", slow_observe)
    monkeypatch.setattr(backend, "start", capture_start)
    runtime.dispatch("point", 0)
    assert send_times == [0.1]
    assert runtime.active.issued_at == 0.1
    assert runtime.last_now == 0.1


def test_clock_regression_after_observe_does_not_dispatch(monkeypatch):
    runtime, backend = make_runtime(clock=lambda: -1.0)
    with pytest.raises(ValueError, match="finite and monotonic"):
        runtime.dispatch("point", 0)
    assert backend.starts == 0 and runtime.active is None


def test_predeadline_display_and_response_remain_eligible_when_processed_late():
    runtime, _ = make_runtime()
    lease_id = runtime.cue_return("queued-display", 1, 0)
    runtime.mark_displayed(lease_id, 1.1, ingress_at=0.2)
    assert runtime.lease.status == "exposed" and runtime.lease.displayed_at == 0.2
    result = runtime.respond("defer", "complete", 1.2, ingress_at=0.3, lease_id=lease_id)
    assert result["correct"] is True
    assert runtime.lease.result["cue_to_response"] == 0.3


def test_delayed_timer_uses_admission_cutoff_before_queued_predeadline_response():
    runtime, _ = make_runtime()
    lease_id = runtime.cue_return("queued-timer", 1, 0)
    runtime.mark_displayed(lease_id, 0.1)
    runtime.tick(1.1, auto_dispatch=False, deadline_at=0.2)
    assert runtime.lease.status == "exposed"
    assert runtime.respond("defer", "complete", 1.2, ingress_at=0.3, lease_id=lease_id)["correct"]


def test_timer_admission_at_deadline_closes_with_default_tie_rule():
    runtime, _ = make_runtime()
    runtime.cue_return("timer-tie", 1, 0)
    runtime.tick(1.1, auto_dispatch=False, deadline_at=1.0)
    assert runtime.lease.result["outcome"] == "no_response"


def test_delayed_timer_still_checks_current_authority():
    runtime, _ = make_runtime(expires=1.0)
    runtime.cue_return("authority-vs-timer", 1, 0)
    runtime.tick(1.1, auto_dispatch=False, deadline_at=0.2)
    assert runtime.lease.result["outcome"] == "technical_failure"
    assert runtime.lease.result["human_error"] is False


def test_display_receipt_admitted_at_deadline_is_rejected():
    runtime, _ = make_runtime()
    lease_id = runtime.cue_return("late-display", 1, 0)
    with pytest.raises(ValueError, match="outside"):
        runtime.mark_displayed(lease_id, 1.1, ingress_at=1.0)
    assert runtime.lease.displayed_at is None


def test_delayed_timer_cannot_publish_first_snapshot_after_deadline():
    runtime, _ = make_runtime()
    runtime.set_local_quiescent(False, 0)
    runtime.cue_return("late-boundary", 1, 0)
    runtime.set_local_quiescent(True, 1.1)
    runtime.tick(1.1, auto_dispatch=False, deadline_at=0.2)
    assert runtime.lease.status == "draining"
    assert runtime.lease.exposed_at is None
    runtime.tick(1.2, auto_dispatch=False, deadline_at=1.0)
    assert runtime.lease.result["outcome"] == "system_unavailable"
