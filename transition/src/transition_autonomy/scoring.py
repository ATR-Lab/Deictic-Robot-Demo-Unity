"""Independent authored task rubric. Does not import the scheduler."""
from __future__ import annotations

from typing import Any
from .model import semantic_equal


def correct_responses(rules: list[dict], facts: dict[str, dict]) -> set[tuple[str, str]]:
    answers: set[tuple[str, str]] = set()
    for rule in rules:
        matches = True
        for key, expected in rule.get("when", {}).items():
            fact = facts.get(key, {"status": "unknown", "value": None})
            if isinstance(expected, dict) and "status" in expected:
                matches &= fact["status"] == expected["status"]
                if "value" in expected:
                    matches &= semantic_equal(fact["value"], expected["value"])
            else:
                matches &= fact["status"] == "known" and semantic_equal(fact["value"], expected)
        if matches:
            answers.update((r["kind"], r["target"]) for r in rule["responses"])
    return answers


def score(rules: list[dict], facts: dict[str, dict], kind: str, target: str) -> dict[str, Any]:
    allowed = correct_responses(rules, facts)
    if not allowed:
        return {"correct": None, "label": "unscorable", "reason": "authored rubric has no justified response for snapshot"}
    return {"correct": (kind, target) in allowed, "label": "correct" if (kind, target) in allowed else "incorrect"}
