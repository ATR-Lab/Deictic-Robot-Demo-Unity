import copy
import hashlib
import json

import pytest

from transition_autonomy.stages import REQUIRED_PREDICATES, build_stage_report, load_plan, write_report
from transition_autonomy.journal import digest


@pytest.fixture
def evidence_plan(tmp_path):
    artifacts = {}
    for role in ("source", "configuration", "evidence"):
        path = tmp_path / (role + ".txt")
        path.write_text(role + " fixture; no hardware validation")
        artifacts[role] = {"path": path.name, "role": role, "required": True,
                           "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    return {"schema_version": 1, "stage": "S0", "permit_class": "offline", "scope": "software_test",
        "run_started": True, "planned_count": 1, "attempted_count": 1, "refused_count": 0,
        "attempts": [{"id": "test-run-1", "stratum": "mock-only", "outcome": "passed", "possible_actuation": False,
                      "evidence_refs": ["evidence"], "reason": "Recorded synthetic acceptance test completed."}],
        "refusals": [], "artifacts": artifacts,
        "predicates": {name: {"status": "passed", "criterion": "All declared synthetic assertions pass.",
            "evidence_refs": ["evidence"], "reason": "Synthetic fixture evidence exists.",
            "disposition": "No motion; report audit only."} for name in REQUIRED_PREDICATES["S0"]},
        "final_state": {"motion_state": "not_applicable", "task_outcome": "not_applicable",
                        "arm_state": "not_applicable", "disposition": "Offline test only."}}


def test_complete_report_hashes_artifacts_but_never_authorizes_or_advances(tmp_path, evidence_plan):
    report = build_stage_report(evidence_plan, tmp_path)
    assert report["status"] == "passed"
    assert not report["automatic_stage_advancement"] and not report["motion_authorized"]
    fingerprint = report.pop("report_sha256")
    assert fingerprint == digest(report)
    evidence_plan["attempts"][0]["reason"] = "mutated"
    assert report["attempts"][0]["reason"] != "mutated"


@pytest.mark.parametrize("change,code", [
    ("missing_file", "artifact_missing_or_unreadable"),
    ("mismatched_digest", "artifact_digest_mismatch"),
    ("unknown_bound", "unknown_acceptance_bound"),
    ("missing_predicate", "missing_required_predicate"),
    ("unsupported_pass", "unsupported_predicate_pass"),
    ("unknown_reference", "predicate_evidence_missing"),
    ("incomplete", "attempts_incomplete"),
])
def test_missing_or_unknown_evidence_blocks_pass(tmp_path, evidence_plan, change, code):
    predicate = evidence_plan["predicates"]["software_regressions"]
    if change == "missing_file":
        (tmp_path / "evidence.txt").unlink()
    elif change == "mismatched_digest":
        (tmp_path / "source.txt").write_text("changed after recorded validation")
    elif change == "unknown_bound":
        predicate["criterion"] = None
    elif change == "missing_predicate":
        del evidence_plan["predicates"]["ledger_recovery"]
    elif change == "unsupported_pass":
        predicate["evidence_refs"] = []
    elif change == "unknown_reference":
        predicate["evidence_refs"] = ["unretained-proof"]
    else:
        evidence_plan["planned_count"] = 2
    report = build_stage_report(evidence_plan, tmp_path)
    assert report["status"] == "blocked"
    assert code in {issue["code"] for issue in report["issues"]}


def test_overquota_and_failed_attempts_are_retained_not_dropped(tmp_path, evidence_plan):
    failed = copy.deepcopy(evidence_plan["attempts"][0])
    failed.update(id="attempt-2", outcome="failed", reason="Unexpected result in second attempt.")
    evidence_plan["attempts"].append(failed)
    evidence_plan["attempted_count"] = 2
    report = build_stage_report(evidence_plan, tmp_path)
    assert report["status"] == "failed" and report["attempted_count"] == 2
    assert "attempt_quota_exceeded" in {i["code"] for i in report["issues"]}


@pytest.mark.parametrize("change", ["bool_count", "mismatched_count", "duplicate_id", "outside_root", "read_only_actuation", "wrong_stage_scope"])
def test_invalid_accounting_or_scope_is_rejected(tmp_path, evidence_plan, change):
    if change == "bool_count":
        evidence_plan["planned_count"] = True
    elif change == "mismatched_count":
        evidence_plan["attempted_count"] = 2
    elif change == "duplicate_id":
        evidence_plan["attempted_count"] = 2
        evidence_plan["attempts"].append(copy.deepcopy(evidence_plan["attempts"][0]))
    elif change == "outside_root":
        evidence_plan["artifacts"]["source"]["path"] = "../outside"
    elif change == "read_only_actuation":
        evidence_plan["attempts"][0]["possible_actuation"] = True
    else:
        evidence_plan["stage"] = "S2"
    with pytest.raises(ValueError):
        build_stage_report(evidence_plan, tmp_path)


def test_not_run_does_not_turn_into_approval(tmp_path, evidence_plan):
    evidence_plan.update(run_started=False, attempted_count=0, attempts=[])
    report = build_stage_report(evidence_plan, tmp_path)
    assert report["status"] == "not_run" and not report["motion_authorized"]


def test_physical_attempt_without_permit_or_known_stop_is_blocked(tmp_path, evidence_plan):
    evidence_plan.update(stage="S4", permit_class="bounded_commissioning", scope="commanded_hardware")
    item = evidence_plan["predicates"]["software_regressions"]
    evidence_plan["predicates"] = {name: copy.deepcopy(item) for name in REQUIRED_PREDICATES["S4"]}
    evidence_plan["attempts"][0]["possible_actuation"] = True
    evidence_plan["final_state"].update(motion_state="unknown", task_outcome="unknown", arm_state="inhibited")
    report = build_stage_report(evidence_plan, tmp_path)
    assert report["status"] == "blocked"
    assert {"unresolved_actuation", "missing_physical_permit"} <= {i["code"] for i in report["issues"]}


def test_refusal_is_separate_from_possible_actuation(tmp_path, evidence_plan):
    refusal = evidence_plan["attempts"].pop()
    refusal.update(outcome="refused", reason="Missing commissioning record prevented admission.")
    evidence_plan.update(attempted_count=0, refused_count=1, refusals=[refusal])
    report = build_stage_report(evidence_plan, tmp_path)
    assert report["attempted_count"] == 0 and report["refused_count"] == 1
    assert report["status"] == "blocked"


@pytest.mark.parametrize("body", ['{"stage":"S0","stage":"S2"}', '{"x":NaN}', '{"x":1e999}'])
def test_plan_parser_rejects_ambiguous_json(tmp_path, body):
    path = tmp_path / "plan.json"
    path.write_text(body)
    with pytest.raises(ValueError):
        load_plan(path)


def test_existing_evidence_cannot_be_overwritten(tmp_path, evidence_plan):
    report = build_stage_report(evidence_plan, tmp_path)
    output = tmp_path / "report.json"
    write_report(report, output)
    with pytest.raises(FileExistsError):
        write_report(report, output)
    assert json.loads(output.read_text())["report_sha256"] == report["report_sha256"]
