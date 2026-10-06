"""Finite public task state shared by logical simulation and hardware adapters.

This module does not implement robot motion or establish physical safety.
"""

from __future__ import annotations

import copy
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping


STATUSES = frozenset({"known", "unknown", "conflict"})
CATEGORIES = ("evidence", "revision", "advance")


def _finite(value: float, name: str, *, nonnegative: bool = False) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    if nonnegative and value < 0:
        raise ValueError(f"{name} must be nonnegative")


def _freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        return MappingProxyType({str(k): _freeze(v) for k, v in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(v) for v in value)
    return copy.deepcopy(value)


def to_json_value(value: Any) -> Any:
    """Return a detached JSON-compatible form of immutable snapshot values."""
    if isinstance(value, Mapping):
        return {k: to_json_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_json_value(v) for v in value]
    if isinstance(value, (set, frozenset)):
        return sorted(to_json_value(v) for v in value)
    return copy.deepcopy(value)


def semantic_equal(left: Any, right: Any) -> bool:
    # JSON distinguishes false from zero; Python's ordinary equality does not.
    return json.dumps(to_json_value(left), sort_keys=True, allow_nan=False) == json.dumps(
        to_json_value(right), sort_keys=True, allow_nan=False
    )


@dataclass(frozen=True)
class Fact:
    value: Any
    version: int
    observed_at: float
    valid_until: float | None = None
    status: str = "known"
    source: str = "declared"

    def __post_init__(self) -> None:
        if isinstance(self.version, bool) or not isinstance(self.version, int) or self.version < 0:
            raise ValueError("Fact.version must be a nonnegative integer")
        _finite(self.observed_at, "observed_at")
        if self.valid_until is not None:
            _finite(self.valid_until, "valid_until")
            if self.valid_until < self.observed_at:
                raise ValueError("valid_until cannot precede observed_at")
        if self.status not in STATUSES:
            raise ValueError(f"unsupported fact status: {self.status}")
        # Validate JSON semantics while retaining ordinary values for adapters.
        json.dumps(self.value, allow_nan=False)
        object.__setattr__(self, "value", copy.deepcopy(self.value))

    def fresh(self, now: float, max_age: float | None = None) -> bool:
        _finite(now, "now")
        if max_age is not None:
            _finite(max_age, "max_age", nonnegative=True)
        return (
            self.status == "known"
            and now >= self.observed_at
            and (self.valid_until is None or now < self.valid_until)
            and (max_age is None or now - self.observed_at <= max_age)
        )


@dataclass
class State:
    facts: dict[str, Fact]
    completed: set[str] = field(default_factory=set)

    def copy(self) -> State:
        return State(dict(self.facts), set(self.completed))


@dataclass(frozen=True)
class Outcome:
    id: str
    effects: dict[str, Any]
    duration_max: float | None = None
    terminal_status: str = "succeeded"

    def __post_init__(self) -> None:
        # Keep backend event terminology canonical, while accepting older prose.
        if self.terminal_status == "success":
            object.__setattr__(self, "terminal_status", "succeeded")
        if self.terminal_status not in {"succeeded", "failed"}:
            raise ValueError("outcome terminal_status must be succeeded or failed")
        if self.duration_max is not None:
            _finite(self.duration_max, "outcome duration_max")
            if self.duration_max <= 0:
                raise ValueError("outcome duration_max must be positive")


@dataclass(frozen=True)
class Skill:
    id: str
    kind: str = "point"
    parameters: dict[str, Any] = field(default_factory=dict)
    preconditions: dict[str, Any] = field(default_factory=dict)
    read_set: set[str] = field(default_factory=set)
    write_set: set[str] = field(default_factory=set)
    resources: set[str] = field(default_factory=set)
    outcomes: list[Outcome] = field(default_factory=list)
    duration_max: float = 1.0
    priority: int = 0
    required_work: bool = True
    requires: set[str] = field(default_factory=set)
    deadline: float | None = None
    max_age: float | None = None
    invariants: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.id, str) or not self.id:
            raise ValueError("skill id must be nonempty")
        if not isinstance(self.kind, str) or not self.kind:
            raise ValueError("skill kind must be nonempty")
        if not isinstance(self.parameters, dict) or not isinstance(self.preconditions, dict) or not isinstance(self.invariants, dict):
            raise ValueError("parameters, preconditions, and invariants must be objects")
        if not isinstance(self.outcomes, list) or not all(isinstance(o, Outcome) for o in self.outcomes):
            raise ValueError("outcomes must be a list of Outcome objects")
        for name in ("read_set", "write_set", "resources", "requires"):
            values = getattr(self, name)
            if isinstance(values, str) or not all(isinstance(v, str) and v for v in values):
                raise ValueError(f"{name} must contain nonempty string identifiers")
        _finite(self.duration_max, "duration_max")
        if self.duration_max <= 0:
            raise ValueError("duration_max must be positive")
        if isinstance(self.priority, bool) or not isinstance(self.priority, int):
            raise ValueError("priority must be an integer")
        if self.deadline is not None:
            _finite(self.deadline, "deadline")
        if self.max_age is not None:
            _finite(self.max_age, "max_age", nonnegative=True)
        if not isinstance(self.required_work, bool):
            raise ValueError("required_work must be boolean")
        object.__setattr__(self, "read_set", frozenset(self.read_set) | self.preconditions.keys() | self.invariants.keys())
        object.__setattr__(self, "write_set", frozenset(self.write_set))
        object.__setattr__(self, "resources", frozenset(self.resources))
        object.__setattr__(self, "requires", frozenset(self.requires))
        ids = [o.id for o in self.outcomes]
        if len(ids) != len(set(ids)):
            raise ValueError(f"duplicate outcome id in {self.id}")
        for outcome in self.outcomes:
            if not isinstance(outcome.id, str) or not outcome.id or not isinstance(outcome.effects, dict):
                raise ValueError(f"invalid outcome in {self.id}")
            if not set(outcome.effects) <= self.write_set:
                raise ValueError(f"outcome writes undeclared facts in {self.id}")
            if outcome.duration_max is not None and outcome.duration_max > self.duration_max:
                raise ValueError(f"outcome duration_max exceeds skill bound in {self.id}")
            for effect in outcome.effects.values():
                _effect_parts(effect)


@dataclass(frozen=True)
class Commitment:
    id: str
    values: Mapping[str, Any]
    issued_at: float

    def __post_init__(self) -> None:
        _finite(self.issued_at, "issued_at")
        json.dumps(to_json_value(self.values), allow_nan=False)
        object.__setattr__(self, "values", _freeze(self.values))

    def to_dict(self) -> dict[str, Any]:
        return {"id": self.id, "values": to_json_value(self.values), "issued_at": self.issued_at}


@dataclass(frozen=True)
class Authority:
    id: str
    revision: int
    skill_ids: set[str]
    expires_at: float | None
    revoked: bool = False

    def __post_init__(self) -> None:
        if isinstance(self.revision, bool) or not isinstance(self.revision, int) or self.revision < 0:
            raise ValueError("authority revision must be a nonnegative integer")
        if self.expires_at is not None:
            _finite(self.expires_at, "expires_at")
        object.__setattr__(self, "skill_ids", frozenset(self.skill_ids))

    def valid(self, now: float) -> bool:
        _finite(now, "now")
        return not self.revoked and (self.expires_at is None or now < self.expires_at)


def _effect_parts(effect: Any) -> tuple[Any, str, float | None]:
    """An explicit status key distinguishes fact descriptors from raw JSON values."""
    if isinstance(effect, Mapping) and "status" in effect:
        if "value" not in effect or effect["status"] not in STATUSES:
            raise ValueError("fact descriptor requires value and a supported status")
        unknown = set(effect) - {"value", "status", "valid_for"}
        if unknown:
            raise ValueError(f"unknown fact descriptor fields: {sorted(unknown)}")
        lifetime = effect.get("valid_for")
        if lifetime is not None:
            _finite(lifetime, "valid_for", nonnegative=True)
        value, status = effect["value"], effect["status"]
    else:
        value, status, lifetime = effect, "known", None
    json.dumps(to_json_value(value), allow_nan=False)
    return to_json_value(value), status, lifetime


def apply_outcome(state: State, skill: Skill, outcome: Outcome, now: float) -> State:
    """Apply declared logical effects, not inferred physical effects; return a new state."""
    _finite(now, "now")
    if outcome not in skill.outcomes:
        raise ValueError("outcome is outside the skill's declared support")
    result = state.copy()
    for key, effect in outcome.effects.items():
        value, status, lifetime = _effect_parts(effect)
        previous = result.facts.get(key)
        changed = previous is None or previous.status != status or not semantic_equal(previous.value, value)
        version = (previous.version if previous else 0) + int(changed)
        result.facts[key] = Fact(
            value, version, now, None if lifetime is None else now + lifetime,
            status, f"outcome:{skill.id}:{outcome.id}",
        )
    if outcome.terminal_status == "succeeded":
        result.completed.add(skill.id)
    return result


@dataclass(frozen=True)
class TaskSpec:
    id: str
    skills: dict[str, Skill]
    decision_fields: dict[str, str]
    initial_facts: dict[str, Any]
    scored_facts: set[str]
    scoring_rules: list[dict[str, Any]]
    backend_kind: str = "point"
    aggregation: str = "lex_worst"

    def __post_init__(self) -> None:
        if not isinstance(self.id, str) or not self.id:
            raise ValueError("task id must be nonempty")
        if self.aggregation not in {"lex_worst", "upper_envelope"}:
            raise ValueError("aggregation must be lex_worst or upper_envelope")
        object.__setattr__(self, "scored_facts", frozenset(self.scored_facts))
        if any(key != skill.id for key, skill in self.skills.items()):
            raise ValueError("skill dictionary keys must equal skill ids")
        if any(category not in CATEGORIES for category in self.decision_fields.values()):
            raise ValueError("decision categories must be evidence, revision, or advance")
        if not isinstance(self.scoring_rules, list) or not all(isinstance(r, dict) for r in self.scoring_rules):
            raise ValueError("scoring_rules must be a list of independent rule objects")
        for rule in self.scoring_rules:
            dependencies = rule.get("when", {})
            if not isinstance(dependencies, dict):
                raise ValueError("scoring rule when must be an object")
            if not set(dependencies) <= self.scored_facts:
                raise ValueError("all scoring rule dependencies must be included in scored_facts")
        all_facts = set(self.initial_facts)
        for skill in self.skills.values():
            if not skill.requires <= self.skills.keys():
                raise ValueError(f"unknown predecessor in {skill.id}")
            all_facts.update(skill.write_set)
        if not set(self.decision_fields) <= all_facts or not self.scored_facts <= all_facts:
            raise ValueError("decision/scored fields must be declared task facts")
        for value in self.initial_facts.values():
            _effect_parts(value)
        visiting: set[str] = set()
        visited: set[str] = set()

        def visit(key: str) -> None:
            if key in visiting:
                raise ValueError("task requires graph must be acyclic")
            if key in visited:
                return
            visiting.add(key)
            for parent in self.skills[key].requires:
                visit(parent)
            visiting.remove(key)
            visited.add(key)

        for key in self.skills:
            visit(key)

    def initial_state(self, now: float = 0.0) -> State:
        facts: dict[str, Fact] = {}
        for key, effect in self.initial_facts.items():
            value, status, lifetime = _effect_parts(effect)
            facts[key] = Fact(value, 1, now, None if lifetime is None else now + lifetime, status, "initial")
        return State(facts)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> TaskSpec:
        allowed = {"id", "skills", "decision_fields", "initial_facts", "scored_facts", "scoring_rules", "backend_kind", "aggregation", "schema_version", "description"}
        unknown = set(data) - allowed
        if unknown:
            raise ValueError(f"unknown task fields: {sorted(unknown)}")
        if data.get("schema_version", 1) != 1:
            raise ValueError("unsupported task schema_version")
        if "id" not in data:
            raise ValueError("task id is required")
        raw_skills = data.get("skills", {})
        if isinstance(raw_skills, list):
            if not all(isinstance(item, Mapping) for item in raw_skills):
                raise ValueError("each skill must be an object")
            entries = [(item.get("id"), item) for item in raw_skills]
        elif isinstance(raw_skills, Mapping):
            entries = list(raw_skills.items())
        else:
            raise ValueError("skills must be a mapping or list")
        skills: dict[str, Skill] = {}
        for key, raw in entries:
            if not isinstance(raw, Mapping):
                raise ValueError("each skill must be an object")
            params = copy.deepcopy(dict(raw))
            params.setdefault("id", key)
            if not key or key in skills or key != params["id"]:
                raise ValueError("duplicate or inconsistent skill id")
            outcomes = params.get("outcomes", [])
            if not isinstance(outcomes, list) or not all(isinstance(item, Mapping) for item in outcomes):
                raise ValueError(f"outcomes in {key} must be a list of objects")
            try:
                params["outcomes"] = [Outcome(**item) for item in outcomes]
            except TypeError as exc:
                raise ValueError(f"invalid outcome fields in {key}: {exc}") from exc
            for field_name in ("read_set", "write_set", "resources", "requires"):
                items = params.get(field_name, [])
                if not isinstance(items, list) or not all(isinstance(item, str) and item for item in items):
                    raise ValueError(f"{field_name} in {key} must be a list of identifiers")
                params[field_name] = set(items)
            try:
                skills[key] = Skill(**params)
            except TypeError as exc:
                raise ValueError(f"invalid skill fields in {key}: {exc}") from exc
        return cls(
            id=data["id"], skills=skills,
            decision_fields=dict(data.get("decision_fields", {})),
            initial_facts=copy.deepcopy(dict(data.get("initial_facts", {}))),
            scored_facts=set(data.get("scored_facts", [])),
            scoring_rules=copy.deepcopy(data.get("scoring_rules", [])),
            backend_kind=data.get("backend_kind", "point"),
            aggregation=data.get("aggregation", "lex_worst"),
        )


def load_task(path: str | Path) -> TaskSpec:
    with Path(path).open(encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError("task JSON must be an object")
    return TaskSpec.from_dict(data)


load_task_spec = load_task
