"""Offline acceptance reports, not commissioning permits or motion authorization.

The reporter checks declared coverage, attempt accounting and artifact integrity.
It cannot certify the scientific or physical truth of an author's predicates.
No robot, ROS, SDK, runtime owner or command client is imported or constructed.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

from .journal import digest


REQUIRED_PREDICATES = {
    "S0": ("software_regressions", "configuration_validation", "ledger_recovery", "no_implicit_connection"),
    "S1": ("native_profiles", "advancing_sample_dwell", "duplicate_suppression", "stop_confirmation",
           "lost_response_reconciliation", "runtime_decisions", "journal_integrity"),
    "S2": ("device_identity", "controller_mode", "joint_frame_map", "observation_timing",
           "velocity_uncertainty", "transform_error_bounds", "motor_health", "support_evidence", "read_only_capability"),
    "S3": ("real_observation_provenance", "motion_incapable_transport", "no_task_authority",
           "distinct_shadow_ids", "proposed_actions_and_rejections", "readiness_changes", "return_snapshots"),
    "S4": ("software_gate", "independent_protection", "commissioning_permit", "starting_region",
           "approved_transition", "endpoint_uncertainty", "valid_dwell", "stop_bounds"),
    "S5": ("transition_commissioning", "integration_permit", "measured_sequence", "local_dependency_gates",
           "raw_responses", "return_protocol", "matched_policy_configuration", "journal_integrity"),
    "S6": ("offline_fault_matrix", "scoped_fault_permit", "no_motion_after_inhibition",
           "stop_bounds", "watchdog_enforcement", "ambiguity_retained", "no_replay"),
    "S7": ("competing_useful_choices", "bidirectional_dependencies", "observable_outcomes",
           "public_outcome_support", "independent_rubric", "progress_and_quality_measures", "pilot_protocol"),
}
SCOPES = {"software_test", "mock_transport", "native_simulation", "read_only_hardware",
          "shadow_hardware", "commanded_hardware", "human_participant"}
PERMITS = {"offline", "simulation", "observation_discovery", "shadow",
           "bounded_commissioning", "integration_commissioning", "operational_release"}
STAGE_SCOPES = {
    "S0": {"software_test", "mock_transport"}, "S1": {"native_simulation"},
    "S2": {"read_only_hardware"}, "S3": {"shadow_hardware"},
    "S4": {"commanded_hardware"}, "S5": {"commanded_hardware"},
    "S6": {"software_test", "mock_transport", "native_simulation", "commanded_hardware"},
    "S7": {"native_simulation", "commanded_hardware", "human_participant"},
}


def _object(value, required, optional=(), label="object"):
    if not isinstance(value, dict) or set(value) - set(required) - set(optional) or not set(required) <= set(value):
        raise ValueError(f"invalid {label} fields")


def _text(value, label):
    if not isinstance(value, str) or not 1 <= len(value) <= 4096 or any(ord(c) < 32 for c in value):
        raise ValueError(f"invalid {label}")
    return value


def _count(value, label):
    if type(value) is not int or not 0 <= value <= 100000:
        raise ValueError(f"{label} must be a nonnegative bounded integer")


def _strings(value, label):
    if not isinstance(value, list) or len(value) > 10000:
        raise ValueError(f"{label} must be a bounded array")
    for item in value:
        _text(item, label)
    if len(set(value)) != len(value):
        raise ValueError(f"duplicate {label}")


def load_plan(path):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError("duplicate JSON key: " + key)
            result[key] = value
        return result

    def reject_constant(value):
        raise ValueError("nonfinite JSON is not accepted")

    raw = Path(path).read_bytes()
    if len(raw) > 2_000_000:
        raise ValueError("stage plan exceeds 2 MB")
    plan = json.loads(raw, object_pairs_hook=pairs, parse_constant=reject_constant)
    # Reject exponent overflow nested anywhere, before recording a plan digest.
    json.dumps(plan, allow_nan=False)
    return plan


def build_stage_report(plan, artifact_root):
    _object(plan, {"schema_version", "stage", "permit_class", "scope", "run_started",
        "planned_count", "attempted_count", "refused_count", "attempts", "refusals",
        "predicates", "artifacts", "final_state"}, {"notes"}, "stage plan")
    if type(plan["schema_version"]) is not int or plan["schema_version"] != 1:
        raise ValueError("unsupported stage schema")
    stage = plan["stage"]
    for name in ("stage", "scope", "permit_class"):
        _text(plan[name], name)
    if stage not in REQUIRED_PREDICATES or plan["scope"] not in SCOPES or plan["permit_class"] not in PERMITS:
        raise ValueError("unknown stage, evidence scope or permit class")
    if plan["scope"] not in STAGE_SCOPES[stage]:
        raise ValueError("evidence scope does not match the requested stage")
    _strings(plan.get("notes", []), "notes")
    if type(plan["run_started"]) is not bool:
        raise ValueError("run_started must be boolean")
    for name in ("planned_count", "attempted_count", "refused_count"):
        _count(plan[name], name)
    if not plan["run_started"] and (plan["attempted_count"] or plan["refused_count"]):
        raise ValueError("unstarted stage cannot contain attempts or refusals")
    if plan["run_started"] and not plan["planned_count"]:
        raise ValueError("started stage requires a predeclared planned count")
    root = Path(artifact_root).resolve(strict=True)
    if not root.is_dir():
        raise ValueError("artifact root must be a directory")
    issues = []

    def issue(code, reason, *, failure=False):
        issues.append({"code": code, "reason": reason, "severity": "failed" if failure else "blocked"})

    if plan["attempted_count"] > plan["planned_count"]:
        issue("attempt_quota_exceeded", "Actual attempts exceed the predeclared quota; retain every attempt.", failure=True)
    elif plan["run_started"] and plan["attempted_count"] < plan["planned_count"]:
        issue("attempts_incomplete", "Declared attempted count has not reached the predeclared count.")
    all_ids = set()
    for key, count in (("attempts", "attempted_count"), ("refusals", "refused_count")):
        entries = plan[key]
        if not isinstance(entries, list) or len(entries) != plan[count]:
            raise ValueError(f"{count} does not match {key} records")
        for item in entries:
            _object(item, {"id", "stratum", "outcome", "possible_actuation", "evidence_refs", "reason"}, label=key)
            for name in ("id", "stratum", "reason"):
                _text(item[name], name)
            if item["id"] in all_ids:
                raise ValueError("duplicate attempt/refusal identifier")
            all_ids.add(item["id"])
            _strings(item["evidence_refs"], "evidence_refs")
            if type(item["possible_actuation"]) is not bool:
                raise ValueError("possible_actuation must be explicit boolean")
            if key == "refusals":
                if item["outcome"] != "refused" or item["possible_actuation"]:
                    raise ValueError("pre-actuation refusal cannot contain possible actuation")
            elif item["outcome"] not in {"passed", "failed", "interrupted", "unknown", "invalid_observation", "invalid_artifact"}:
                raise ValueError("invalid attempt outcome")
            elif item["outcome"] != "passed":
                issue("attempt_" + item["outcome"], item["reason"], failure=item["outcome"] == "failed")
            if item["possible_actuation"] and plan["scope"] not in {"commanded_hardware", "human_participant", "native_simulation"}:
                raise ValueError("read-only/software scope cannot conceal a possible actuation attempt")

    if not isinstance(plan["artifacts"], dict):
        raise ValueError("artifacts must be a named object")
    artifacts = {}
    for identifier, item in plan["artifacts"].items():
        _text(identifier, "artifact id")
        _object(item, {"path", "role", "required"}, {"sha256"}, "artifact")
        _text(item["path"], "artifact path")
        if item["role"] not in {"source", "configuration", "evidence", "permit"} or type(item["required"]) is not bool:
            raise ValueError("invalid artifact role or required flag")
        expected = item.get("sha256")
        if expected is not None and (not isinstance(expected, str) or len(expected) != 64
                                    or any(c not in "0123456789abcdef" for c in expected)):
            raise ValueError("invalid expected SHA256")
        path = (root / item["path"]).resolve()
        if not path.is_relative_to(root):
            raise ValueError("artifact path escapes the declared root")
        actual = None
        try:
            hasher = hashlib.sha256()
            size = 0
            with path.open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    hasher.update(chunk)
                    size += len(chunk)
            actual = hasher.hexdigest()
            state = "verified" if expected in (None, actual) else "digest_mismatch"
        except OSError:
            state, size = "missing_or_unreadable", None
        artifacts[identifier] = {"path": path.relative_to(root).as_posix(), "role": item["role"],
            "required": item["required"], "expected_sha256": expected, "sha256": actual,
            "bytes": size, "status": state}
        if state != "verified" and item["required"]:
            issue("artifact_" + state, identifier + ": required artifact is not verified")
    for role in ("source", "configuration", "evidence"):
        if not any(a["role"] == role and a["required"] and a["status"] == "verified" for a in artifacts.values()):
            issue("missing_" + role, "At least one required, verified " + role + " artifact is needed.")
    for item in plan["attempts"] + plan["refusals"]:
        for reference in item["evidence_refs"]:
            if reference not in artifacts or artifacts[reference]["status"] != "verified":
                issue("attempt_evidence_missing", item["id"] + ": " + reference)
        if not item["evidence_refs"]:
            issue("attempt_evidence_missing", item["id"] + ": no evidence artifact linked")

    if not isinstance(plan["predicates"], dict):
        raise ValueError("predicates must be a named object")
    predicates = {}
    for identifier, item in plan["predicates"].items():
        _text(identifier, "predicate id")
        _object(item, {"status", "criterion", "evidence_refs", "reason", "disposition"}, label="predicate")
        if item["status"] not in {"passed", "failed", "unknown", "not_run"}:
            raise ValueError("invalid predicate status")
        _text(item["reason"], "predicate reason")
        _text(item["disposition"], "predicate disposition")
        _strings(item["evidence_refs"], "predicate evidence")
        if item["criterion"] is not None:
            _text(item["criterion"], "predicate criterion")
        else:
            issue("unknown_acceptance_bound", identifier + ": no predeclared criterion/bound")
        if item["status"] != "passed":
            issue("predicate_" + item["status"], identifier + ": " + item["reason"], failure=item["status"] == "failed")
        if item["status"] == "passed" and not item["evidence_refs"]:
            issue("unsupported_predicate_pass", identifier + ": pass has no linked evidence")
        for reference in item["evidence_refs"]:
            if reference not in artifacts or artifacts[reference]["status"] != "verified":
                issue("predicate_evidence_missing", identifier + ": " + reference)
        predicates[identifier] = dict(item)
    for identifier in REQUIRED_PREDICATES[stage]:
        if identifier not in predicates:
            predicates[identifier] = {"status": "unknown", "criterion": None, "evidence_refs": [],
                "reason": "Required stage predicate omitted.", "disposition": "No advancement; obtain the required evidence."}
            issue("missing_required_predicate", identifier)

    final = plan["final_state"]
    _object(final, {"motion_state", "task_outcome", "arm_state", "disposition"}, label="final state")
    if final["motion_state"] not in {"still", "moving", "unknown", "not_applicable"}:
        raise ValueError("invalid final motion state")
    if final["task_outcome"] not in {"succeeded", "failed", "canceled", "unknown", "not_applicable"}:
        raise ValueError("invalid final task outcome")
    if final["arm_state"] not in {"disarmed", "inhibited", "armed", "unknown", "not_applicable"}:
        raise ValueError("invalid final arm state")
    _text(final["disposition"], "final disposition")
    possible_actuation = any(a["possible_actuation"] for a in plan["attempts"])
    if possible_actuation:
        if final["motion_state"] != "still" or final["task_outcome"] == "unknown":
            issue("unresolved_actuation", "Retain external inhibition and obtain measured stop/reconciliation evidence.")
        if plan["scope"] in {"commanded_hardware", "human_participant"}:
            if not any(a["role"] == "permit" and a["required"] and a["status"] == "verified" for a in artifacts.values()):
                issue("missing_physical_permit", "A scoped physical permit artifact is required; this reporter cannot issue one.")
            if plan["permit_class"] not in {"bounded_commissioning", "integration_commissioning", "operational_release"}:
                issue("wrong_physical_permit_class", "Observation/software permits cannot cover physical actuation.")
    if not plan["run_started"]:
        status = "not_run"
    elif any(i["severity"] == "failed" for i in issues):
        status = "failed"
    else:
        status = "blocked" if issues else "passed"
    report = {"schema_version": 1, "stage": stage, "permit_class": plan["permit_class"],
        "scope": plan["scope"], "status": status, "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "planned_count": plan["planned_count"], "attempted_count": plan["attempted_count"],
        "refused_count": plan["refused_count"], "attempts": plan["attempts"], "refusals": plan["refusals"],
        "predicates": predicates, "artifacts": artifacts, "issues": issues, "final_state": final,
        "automatic_stage_advancement": False, "motion_authorized": False,
        "report_meaning": "Offline declared-criteria/artifact audit; not a commissioning approval or physical truth certification.",
        "plan_sha256": digest(plan), "notes": plan.get("notes", [])}
    for role in ("source", "configuration", "evidence", "permit"):
        report[role + "_digest"] = digest({k: a["sha256"] for k, a in artifacts.items() if a["role"] == role})
    report["report_sha256"] = digest(report)
    report = json.loads(json.dumps(report, allow_nan=False))
    return report


def write_report(report, output):
    """Exclusive creation: existing evidence is never overwritten."""
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(report, indent=2, allow_nan=False) + "\n")
