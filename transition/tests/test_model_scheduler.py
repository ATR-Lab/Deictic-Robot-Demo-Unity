import json
from dataclasses import replace

import pytest

from transition_autonomy.model import (
    Authority, Commitment, Fact, Outcome, Skill, State, TaskSpec,
    apply_outcome, load_task,
)
from transition_autonomy.scheduler import consequence_counts, select


def skill(name, effects=None, *, priority=0, **kwargs):
    effects = effects or {}
    return Skill(
        name, outcomes=[Outcome("done", effects)], write_set=set(effects),
        priority=priority, **kwargs,
    )


def fixture(skills, *, initial=None, fields=None):
    initial = initial or {"target": "A", "ready": True, "stage_done": False}
    fields = fields or {"target": "revision"}
    task = TaskSpec("task", {s.id: s for s in skills}, fields, initial, set(fields), [])
    state = task.initial_state()
    values = {key: {"value": state.facts[key].value, "status": state.facts[key].status} for key in fields}
    commitment = Commitment("ack", values, 0)
    authority = Authority("auth", 1, set(task.skills), 100)
    return task, state, commitment, authority


def chosen(args, *, policy="proposed", attention="local", **kwargs):
    return select(*args, now=0, policy=policy, attention=attention, **kwargs)


def test_policy_changes_order_but_not_common_candidate_set():
    args = fixture([skill("retarget", {"target": "B"}, priority=0), skill("stage", {"stage_done": True}, priority=1)])
    baseline = chosen(args, policy="baseline")
    proposed = chosen(args)
    assert baseline.candidates == proposed.candidates == ["retarget", "stage"]
    assert baseline.chosen.id == "retarget"
    assert proposed.chosen.id == "stage"
    assert proposed.burdens == {"retarget": [0, 1, 0], "stage": [0, 0, 0]}
    assert args[1].completed == set()  # selection is pure
    assert args[1].facts["stage_done"].value is False


def test_single_high_burden_action_still_executes_without_optional_idle():
    args = fixture([skill("retarget", {"target": "B"})])
    assert chosen(args).chosen.id == "retarget"


def test_remote_attention_uses_ordinary_order():
    args = fixture([skill("retarget", {"target": "B"}, priority=0), skill("stage", priority=1)])
    assert chosen(args, attention="remote").chosen.id == "retarget"


def test_equal_burdens_use_priority_duration_and_stable_id():
    args = fixture([skill("z", duration_max=1), skill("a", duration_max=1), skill("b", duration_max=2)])
    assert chosen(args).chosen.id == "a"


def test_support_uses_worst_possible_outcome_not_first_or_hidden_realization():
    inspect = Skill("inspect", outcomes=[Outcome("pass", {"target": "A"}), Outcome("fail", {"target": "B"})], write_set={"target"})
    args = fixture([inspect, skill("stage", priority=1)])
    result = chosen(args)
    assert result.burdens["inspect"] == [0, 1, 0]
    assert result.chosen.id == "stage"
    # Reordering support does not change the rank; there is no chosen future outcome input.
    reverse = Skill("inspect", outcomes=list(reversed(inspect.outcomes)), write_set={"target"})
    assert chosen(fixture([reverse, skill("stage", priority=1)])).burdens == result.burdens


def test_unknown_observed_quality_does_not_block_modeled_inspection():
    inspect = Skill(
        "inspect", preconditions={"ready": True}, read_set={"quality"},
        write_set={"quality"}, outcomes=[Outcome("pass", {"quality": "pass"}), Outcome("fail", {"quality": "fail"})],
    )
    args = fixture([inspect], initial={"ready": True, "quality": {"value": None, "status": "unknown"}}, fields={"quality": "evidence"})
    assert chosen(args).chosen.id == "inspect"


def test_unknown_effect_model_is_common_exclusion():
    args = fixture([Skill("unmodeled"), skill("known")])
    for policy in ("baseline", "proposed"):
        result = chosen(args, policy=policy)
        assert result.chosen.id == "known"
        assert result.rejections["unmodeled"] == "unknown_outcome_model"


@pytest.mark.parametrize("fact", [Fact(True, 1, 0, valid_until=1), Fact(True, 1, 0, status="unknown"), Fact(True, 1, 0, status="conflict")])
def test_stale_unknown_and_conflicting_prerequisites_block_both(fact):
    task, state, commit, auth = fixture([skill("point", preconditions={"ready": True})])
    state.facts["ready"] = fact
    for policy in ("baseline", "proposed"):
        result = select(task, state, commit, auth, 2, policy, "local")
        assert result.chosen is None
        assert result.rejections["point"] == "stale_or_unknown_precondition:ready"


def test_unknown_is_not_false_and_boolean_is_not_numeric():
    args = fixture([skill("point", preconditions={"ready": 1})])
    assert chosen(args).chosen is None
    assert chosen(args).rejections["point"] == "false_precondition:ready"


def test_done_predecessor_authority_and_resource_guards_are_common():
    args = fixture([skill("first"), skill("next", requires={"first"}), skill("other", resources={"arm"})])
    task, state, commit, auth = args
    state.completed.add("first")
    result = chosen(args, blocked_resources={"arm"})
    assert result.chosen.id == "next"
    assert result.rejections == {"first": "completed", "other": "resource_unavailable"}
    expired = Authority("a", 1, set(task.skills), 0)
    assert select(task, state, commit, expired, 0, "baseline", "local").chosen is None
    revoked = Authority("a", 2, set(task.skills), 100, revoked=True)
    assert select(task, state, commit, revoked, 0, "proposed", "local").chosen is None


def test_lease_blocks_writes_and_unrelated_update_does_not_globally_reject():
    args = fixture([skill("point", {"target": "B"})])
    assert chosen(args, locked_facts={"target"}).chosen is None
    args[1].facts["unrelated"] = Fact("new", 900, 0)
    assert chosen(args, locked_facts={"other"}).chosen.id == "point"


def test_common_urgency_is_nonempty_and_can_override_consequence_preference():
    args = fixture([skill("urgent", {"target": "B"}, deadline=1), skill("easy", priority=1)])
    for policy in ("baseline", "proposed"):
        result = chosen(args, policy=policy)
        assert result.chosen.id == "urgent"
        assert result.candidates == ["urgent"]
        assert result.urgent
        assert result.rejections["easy"] == "lower_common_urgency"


def test_expiry_is_evaluated_at_projected_completion_time():
    args = fixture([skill("slow", duration_max=2), skill("fast", duration_max=0.5, priority=1)], fields={"ready": "evidence"})
    args[1].facts["ready"] = Fact(True, 1, 0, valid_until=1)
    result = chosen(args)
    assert result.burdens["slow"] == [1, 0, 0]
    assert result.burdens["fast"] == [0, 0, 0]
    assert result.chosen.id == "fast"


def test_commitment_is_detached_deeply_immutable_and_json_serializable():
    original = {"object": {"value": ["A"], "status": "known"}}
    commit = Commitment("c", original, 0)
    original["object"]["value"].append("B")
    assert commit.to_dict()["values"]["object"]["value"] == ["A"]
    with pytest.raises(TypeError):
        commit.values["object"]["status"] = "unknown"
    json.dumps(commit.to_dict())


def test_fixed_slots_not_event_or_version_counts_and_missing_reference_rejected():
    args = fixture([skill("point")])
    task, state, commit, auth = args
    state.facts["target"] = Fact("B", 500, 0)
    assert consequence_counts(task, state, commit, 0) == [0, 1, 0]
    with pytest.raises(ValueError, match="lacks fixed decision field"):
        select(task, state, Commitment("bad", {}, 0), auth, 0, "proposed", "local")


def test_observation_refresh_preserves_semantic_version_but_updates_freshness():
    refresh = skill("refresh", {"ready": {"value": True, "status": "known", "valid_for": 5}})
    task, state, *_ = fixture([refresh])
    next_state = apply_outcome(state, refresh, refresh.outcomes[0], 3)
    assert next_state.facts["ready"].version == state.facts["ready"].version
    assert next_state.facts["ready"].observed_at == 3
    assert next_state.facts["ready"].valid_until == 8
    assert state.facts["ready"].observed_at == 0
    assert next_state.completed == {"refresh"}


def test_outcome_outside_support_and_undeclared_writes_rejected():
    step = skill("point", {"target": "A"})
    with pytest.raises(ValueError, match="outside"):
        apply_outcome(State({}), step, Outcome("secret", {"target": "B"}), 0)
    with pytest.raises(ValueError, match="undeclared"):
        Skill("bad", outcomes=[Outcome("x", {"target": "B"})])


def test_json_loader_and_cycle_validation(tmp_path):
    data = {
        "id": "k1", "schema_version": 1, "initial_facts": {"target": "A"},
        "decision_fields": {"target": "revision"}, "scored_facts": ["target"],
        "scoring_rules": [{"when": {"target": "A"}, "responses": [{"kind": "execute", "target": "point"}]}],
        "skills": [{"id": "point", "kind": "point", "write_set": ["target"], "outcomes": [{"id": "done", "effects": {"target": "B"}}]}],
    }
    path = tmp_path / "task.json"
    path.write_text(json.dumps(data))
    task = load_task(path)
    assert task.skills["point"].kind == "point"
    assert task.scoring_rules == data["scoring_rules"]
    data["skills"][0]["requires"] = ["point"]
    with pytest.raises(ValueError, match="acyclic"):
        TaskSpec.from_dict(data)


@pytest.mark.parametrize("policy,attention", [("mystery", "local"), ("baseline", "gone")])
def test_unknown_configuration_fails_explicitly(policy, attention):
    with pytest.raises(ValueError):
        chosen(fixture([skill("point")]), policy=policy, attention=attention)


@pytest.mark.parametrize("bad", [
    {},
    {"id": "bad", "skills": [None]},
    {"id": "bad", "skills": [{"id": "point", "resources": "arm"}]},
    {"id": "bad", "skills": [{"id": "point", "outcomes": [{"effects": {}}]}]},
])
def test_malformed_json_contract_has_explicit_validation_errors(bad):
    with pytest.raises(ValueError):
        TaskSpec.from_dict(bad)


def test_nonfinite_selection_time_is_rejected_even_without_decision_fields():
    task = TaskSpec("empty", {}, {}, {}, set(), [])
    with pytest.raises(ValueError, match="finite"):
        select(task, State({}), Commitment("c", {}, 0), Authority("a", 1, set(), None), float("nan"), "baseline", "local")


def test_authority_must_cover_declared_skill_duration_for_both_policies():
    task, state, commit, _ = fixture([skill("point", duration_max=2)])
    short = Authority("a", 1, {"point"}, 1.5)
    for policy in ("baseline", "proposed"):
        result = select(task, state, commit, short, 0, policy, "local")
        assert result.chosen is None
        assert result.rejections["point"] == "authority_horizon_exceeded"
    exact = Authority("a", 1, {"point"}, 2)
    assert select(task, state, commit, exact, 0, "baseline", "local").chosen.id == "point"


def test_every_rubric_dependency_must_be_in_the_held_scored_fact_set():
    with pytest.raises(ValueError, match="scoring rule dependencies"):
        TaskSpec("bad", {}, {}, {"local.choice": "A", "remote.target": "B"}, {"remote.target"},
                 [{"when": {"local.choice": "A"}, "responses": [{"kind": "defer", "target": "wait"}]}])


def test_lex_worst_and_upper_envelope_can_reverse_action_selection():
    initial = {"e": False, **{f"r{i}": False for i in range(10)}}
    fields = {"e": "evidence", **{f"r{i}": "revision" for i in range(10)}}
    alternative = Skill("A", outcomes=[
        Outcome("one_evidence", {"e": True}),
        Outcome("ten_revisions", {f"r{i}": True for i in range(10)}),
    ], write_set=set(initial))
    middle = skill("B", {"e": True, **{f"r{i}": True for i in range(5)}})
    task, state, commit, auth = fixture([alternative, middle], initial=initial, fields=fields)
    lex = chosen((task, state, commit, auth))
    envelope = chosen((replace(task, aggregation="upper_envelope"), state, commit, auth))
    assert task.aggregation == "lex_worst"
    assert lex.burdens["A"] == [1, 0, 0]
    assert envelope.burdens["A"] == [1, 10, 0]
    assert lex.burdens["B"] == envelope.burdens["B"] == [1, 5, 0]
    assert lex.chosen.id == "A"
    assert envelope.chosen.id == "B"
    assert lex.candidates == envelope.candidates


def test_outcome_duration_preserves_correlation_with_its_effects():
    initial = {"e": True, "r": False}
    fields = {"e": "evidence", "r": "revision"}
    correlated = Skill("correlated", duration_max=2, write_set={"e", "r"}, outcomes=[
        Outcome("quick_revision", {"r": True}, duration_max=0.5),
        Outcome("slow_refresh", {"e": {"value": True, "status": "known", "valid_for": 2}}, duration_max=2),
    ])
    alternative = skill("alternative", {"e": False}, duration_max=0.5)
    task, state, commit, auth = fixture([correlated, alternative], initial=initial, fields=fields)
    state.facts["e"] = Fact(True, 1, 0, valid_until=1)
    result = chosen((task, state, commit, auth))
    assert result.burdens["correlated"] == [0, 1, 0]
    assert result.chosen.id == "correlated"
    # Collapsing every branch to the global bound invents a stale+revision pair.
    uncorrelated = replace(correlated, outcomes=[replace(o, duration_max=None) for o in correlated.outcomes])
    other_task = replace(task, skills={"correlated": uncorrelated, "alternative": alternative})
    other = chosen((other_task, state, commit, auth))
    assert other.burdens["correlated"] == [1, 1, 0]
    assert other.chosen.id == "alternative"


def test_modeled_failed_branch_remains_in_support_without_success_completion():
    step = Skill("inspect", write_set={"target"}, outcomes=[
        Outcome("success", {"target": "A"}),
        Outcome("detected_failure", {"target": "B"}, terminal_status="failed"),
    ])
    task, state, commit, auth = fixture([step, skill("stage", priority=1)])
    result = chosen((task, state, commit, auth))
    assert result.burdens["inspect"] == [0, 1, 0]
    assert result.chosen.id == "stage"
    failed = apply_outcome(state, step, step.outcomes[1], 1)
    assert failed.facts["target"].value == "B"
    assert "inspect" not in failed.completed
    succeeded = apply_outcome(state, step, step.outcomes[0], 1)
    assert "inspect" in succeeded.completed


@pytest.mark.parametrize("fact", [
    None,
    Fact(False, 1, 0),
    Fact(True, 1, 0, status="unknown"),
    Fact(True, 1, 0, status="conflict"),
    Fact(True, 1, 0, valid_until=0),
])
def test_invariants_join_read_dependencies_and_need_fresh_true_evidence(fact):
    step = skill("point", invariants={"local.stable": True})
    args = fixture([step])
    if fact is not None:
        args[1].facts["local.stable"] = fact
    assert "local.stable" in step.read_set
    for policy in ("baseline", "proposed"):
        result = chosen(args, policy=policy)
        assert result.chosen is None
        assert "invariant:local.stable" in result.rejections["point"]
    args[1].facts["local.stable"] = Fact(True, 1, 0)
    assert chosen(args).chosen.id == "point"


def test_new_model_fields_round_trip_and_invalid_bounds_are_rejected():
    task = TaskSpec.from_dict({
        "id": "correlated", "aggregation": "upper_envelope",
        "initial_facts": {"local.stable": True}, "skills": [{
            "id": "point", "invariants": {"local.stable": True}, "duration_max": 2,
            "outcomes": [{"id": "stop", "effects": {}, "duration_max": 1, "terminal_status": "failed"}],
        }],
    })
    assert task.aggregation == "upper_envelope"
    assert task.skills["point"].outcomes[0].duration_max == 1
    assert task.skills["point"].outcomes[0].terminal_status == "failed"
    assert Outcome("ok", {}, terminal_status="success").terminal_status == "succeeded"
    with pytest.raises(ValueError, match="aggregation"):
        replace(task, aggregation="expected_value")
    with pytest.raises(ValueError, match="exceeds"):
        Skill("bad", duration_max=1, outcomes=[Outcome("slow", {}, duration_max=2)])
    with pytest.raises(ValueError, match="terminal_status"):
        Outcome("bad", {}, terminal_status="unmodeled")
    with pytest.raises(ValueError, match="positive"):
        Outcome("bad", {}, duration_max=0)
