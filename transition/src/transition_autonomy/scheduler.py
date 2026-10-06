"""Pure one-skill, work-conserving rankers over a common admissible set."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from .model import Authority, CATEGORIES, Commitment, Skill, State, TaskSpec, _finite, apply_outcome, semantic_equal


@dataclass(frozen=True)
class Selection:
    chosen: Skill | None
    candidates: list[str]
    rejections: dict[str, str]
    burdens: dict[str, list[int]]
    urgent: bool


def _reference(value: Any) -> tuple[Any, str]:
    if isinstance(value, Mapping) and "status" in value and "value" in value:
        return value["value"], value["status"]
    return value, "known"


def consequence_counts(task: TaskSpec, state: State, commitment: Commitment, now: float) -> list[int]:
    """Count fixed canonical slots, never event rows or policy-selected next goals.

    Version 0 fixes each slot's category in TaskSpec. This is a schema heuristic,
    not a cognitive severity estimate. Stale/unknown evidence differs by status.
    """
    counts = [0, 0, 0]
    for key, category in task.decision_fields.items():
        if key not in commitment.values:
            raise ValueError(f"commitment lacks fixed decision field: {key}")
        expected, expected_status = _reference(commitment.values[key])
        fact = state.facts.get(key)
        if fact is None:
            actual, status = None, "unknown"
        else:
            actual = fact.value
            status = fact.status if fact.status != "known" or fact.fresh(now) else "stale"
        if status != expected_status or not semantic_equal(actual, expected):
            counts[CATEGORIES.index(category)] += 1
    return counts


def select(
    task: TaskSpec,
    state: State,
    commitment: Commitment,
    authority: Authority,
    now: float,
    policy: str,
    attention: str,
    blocked_resources: set[str] | frozenset[str] = frozenset(),
    locked_facts: set[str] | frozenset[str] = frozenset(),
) -> Selection:
    _finite(now, "now")
    ordinary = {"ordinary", "baseline", "conventional", "A0"}
    proposed = {"consequence", "proposed", "transition", "transition_aware", "consequence_aware", "A1"}
    if policy not in ordinary | proposed:
        raise ValueError(f"unknown policy: {policy}")
    if attention not in {"local", "remote"}:
        raise ValueError("attention must be local or remote")
    # Validate the immutable reference even when eligibility is currently empty.
    consequence_counts(task, state, commitment, now)
    if now < commitment.issued_at:
        raise ValueError("commitment was issued in the future")
    rejections: dict[str, str] = {}
    ready: list[Skill] = []
    for skill in sorted(task.skills.values(), key=lambda value: value.id):
        reason: str | None = None
        if skill.id in state.completed:
            reason = "completed"
        elif not skill.required_work:
            reason = "not_required_work"
        elif not skill.requires <= state.completed:
            reason = "predecessor_incomplete"
        elif not authority.valid(now):
            reason = "authority_revoked" if authority.revoked else "authority_expired"
        elif skill.id not in authority.skill_ids:
            reason = "not_authorized"
        elif authority.expires_at is not None and authority.expires_at < now + skill.duration_max:
            reason = "authority_horizon_exceeded"
        elif not skill.outcomes:
            reason = "unknown_outcome_model"
        elif skill.resources & blocked_resources:
            reason = "resource_unavailable"
        elif skill.write_set & locked_facts:
            reason = "return_lease_conflict"
        else:
            for guard_kind, predicates in (("precondition", skill.preconditions), ("invariant", skill.invariants)):
                for key, expected in sorted(predicates.items()):
                    fact = state.facts.get(key)
                    if fact is None:
                        reason = f"missing_{guard_kind}:{key}"
                    elif not fact.fresh(now, skill.max_age):
                        reason = f"stale_or_unknown_{guard_kind}:{key}"
                    elif not semantic_equal(fact.value, expected):
                        reason = f"false_{guard_kind}:{key}"
                    if reason:
                        break
                if reason:
                    break
        if reason:
            rejections[skill.id] = reason
        else:
            ready.append(skill)
    if not ready:
        return Selection(None, [], rejections, {}, False)

    # Common nonempty heuristic. deadline is a skill completion due time; this
    # is not a downstream/multi-resource schedulability guarantee.
    horizon = max(skill.duration_max for skill in ready)
    urgent = [skill for skill in ready if skill.deadline is not None and skill.deadline - now - skill.duration_max <= horizon]
    if urgent:
        urgent_ids = {skill.id for skill in urgent}
        for skill in ready:
            if skill.id not in urgent_ids:
                rejections[skill.id] = "lower_common_urgency"
        ready = urgent

    burdens: dict[str, list[int]] = {}
    for skill in ready:
        vectors = []
        for outcome in skill.outcomes:
            completion_at = now + (outcome.duration_max if outcome.duration_max is not None else skill.duration_max)
            projected = apply_outcome(state, skill, outcome, completion_at)
            vectors.append(consequence_counts(task, projected, commitment, completion_at))
        # Primary objective: minimize the lexicographically worst supported
        # vector. Componentwise extrema are a distinct comparison objective.
        burdens[skill.id] = max(vectors) if task.aggregation == "lex_worst" else [max(vector[i] for vector in vectors) for i in range(3)]

    def rank(skill: Skill) -> tuple:
        base = (skill.priority, skill.duration_max, skill.id)
        return (*burdens[skill.id], *base) if policy in proposed and attention == "local" else base

    chosen = min(ready, key=rank)
    return Selection(chosen, sorted(skill.id for skill in ready), rejections, burdens, bool(urgent))
