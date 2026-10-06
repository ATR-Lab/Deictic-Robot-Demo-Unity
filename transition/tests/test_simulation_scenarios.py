"""Runner conformance and honest failure receipts; these are not physics tests."""
from dataclasses import replace
import importlib.util
import json
from pathlib import Path

import pytest

from transition_autonomy.backends.base import Observation, SkillCommand
from transition_autonomy.simulation_scenarios import (
    ExperimentBackend, SCENARIOS, Scenario, VirtualClock, logical_backend, run_suite,
)


ROOT = Path(__file__).resolve().parents[1]
TASK = ROOT / "examples/k1_pointing.json"


def run(tmp_path, factory=logical_backend, scenarios=SCENARIOS):
    clock = VirtualClock()
    return run_suite(factory, TASK, tmp_path / "run", scenarios=scenarios,
                     clock=clock, sleep=clock.sleep, clock_kind="accelerated_logical_fixture")


def test_logical_scenarios_record_faults_without_claiming_native_or_human_evidence(tmp_path):
    report = run(tmp_path)
    assert report["passed"] and not report["not_run"]
    assert [entry["scenario"] for entry in report["scenarios"]] == list(SCENARIOS)
    assert report["clock_kind"] == "accelerated_logical_fixture"
    for scenario in report["scenarios"]:
        assert scenario["journal_integrity"]["valid"]
        assert scenario["backend"] == "logical_simulation"
        assert scenario["human_data"] is False
        assert scenario["physical_robot_execution"] is False
        assert "no physics" in scenario["scope"]
        event_path = tmp_path / "run" / scenario["scenario"] / "events.jsonl"
        events = [json.loads(line) for line in event_path.read_text().splitlines()]
        assert events[0]["data"]["operator_mode"] == "scripted_simulation_experiment"
    retry, version, stale, revoke = report["scenarios"]
    assert len(retry["command_attempts"]) == 1
    assert version["final_state"]["lease"]["result"]["human_error"] is False
    assert not stale["command_attempts"]
    assert stale["injected_faults"][0]["physics_paused"] is False
    assert stale["injected_faults"][0]["network_failure_injected"] is False
    assert stale["final_state"]["fault"]
    assert revoke["revoke_terminal"]["status"] == "canceled"
    assert revoke["final_state"]["completed"] == ["point_a"]


def test_busy_worker_is_reported_without_starting_or_stopping_its_owner(tmp_path):
    backend = logical_backend()
    backend.start(SkillCommand("other-owner", "other-run", "point_a", "point",
        {"profile": "point_a"}, {}, "other-authority", 1, 0, 20), 0)
    def forbidden_stop(now):
        pytest.fail("preflight must not stop another owner")
    backend.request_stop = forbidden_stop
    report = run(tmp_path, lambda: backend)
    assert not report["passed"] and len(report["scenarios"]) == 1
    first = report["scenarios"][0]
    assert not first["command_attempts"] and not first["stop_requests"]
    assert first["initial_observation"]["active_command_id"] == "other-owner"
    assert first["status"] == "failed"
    assert report["not_run"] == list(SCENARIOS[1:])
    assert backend.starts == 1


@pytest.mark.parametrize("failure", ["disconnect", "stale", "boot_change", "foreign_owner"])
def test_stop_guard_refuses_unverified_ownership(failure):
    source = logical_backend()
    wrapped = ExperimentBackend(source)
    wrapped.boot_id = "initial-boot"
    wrapped.outstanding.add("ours")
    observation = Observation(True, False, 1, "initial-boot", active_command_id="ours")
    if failure == "disconnect":
        observation = replace(observation, connected=False)
    elif failure == "stale":
        observation = replace(observation, observed_at=0)
    elif failure == "boot_change":
        observation = replace(observation, boot_id="new-boot")
    else:
        observation = replace(observation, active_command_id="theirs")
    source.observe = lambda now: observation
    calls = []
    source.request_stop = lambda now: calls.append(now)
    wrapped.request_stop(1)
    assert calls == []
    assert wrapped.stop_requests[-1]["sent"] is False


def test_delivery_fault_keeps_original_source_timestamp_and_records_recovery(tmp_path):
    report = run(tmp_path, scenarios=("stale_snapshot_delivery",))
    scenario = report["scenarios"][0]
    assert report["passed"]
    saved = scenario["injected_faults"][0]["saved_source_sample"]
    restored = scenario["after_delivery_restored"]
    assert restored["observed_at"] - saved["observed_at"] >= .65
    assert saved["boot_id"] == restored["boot_id"]
    assert scenario["final_state"]["fault"]
    assert scenario["final_state"]["lease"]["result"]["human_error"] is False


def test_native_label_cannot_turn_logical_terminal_into_measured_physics(tmp_path):
    def dishonest_factory():
        backend = logical_backend()
        backend.name = "isaac_sim_k1"
        return backend
    report = run(tmp_path, dishonest_factory)
    assert not report["passed"]
    scenario = report["scenarios"][0]
    assert "advancing source-time settling" in scenario["error"]
    assert scenario["point_a_terminal"]["evidence"]["physics_validated"] is False
    assert scenario["journal_integrity"]["valid"]
    assert (tmp_path / "run" / "return_decision_retry" / "events.jsonl").is_file()


@pytest.mark.parametrize(("key", "value"), [
    ("settle_seconds", .29), ("settle_seconds", True),
    ("joint_error_rad", .036), ("max_velocity_rad_s", .041),
    ("reference_error_m", .013),
])
def test_native_receipt_threshold_violations_fail_the_experiment(tmp_path, key, value):
    backend = logical_backend()
    backend.name = "isaac_sim_k1"
    clock = VirtualClock()
    scenario = Scenario("return_decision_retry", backend, TASK, tmp_path,
                        clock=clock, sleep=clock.sleep)
    evidence = {"settle_clock": "advancing_physics_simulation_time",
                "velocity_source": "position_difference_per_sim_second",
                "settle_seconds": .3, "joint_error_rad": .01,
                "max_velocity_rad_s": .01, "reference_error_m": .005}
    evidence[key] = value
    with pytest.raises(AssertionError):
        scenario.check_native_evidence({"evidence": evidence}, completed=True)
    assert any(not check["passed"] for check in scenario.report["checks"])


def test_existing_evidence_directory_is_never_overwritten_or_dispatched(tmp_path):
    output = tmp_path / "run"
    output.mkdir()
    (output / "report.json").write_text("earlier failed experiment")
    calls = []
    with pytest.raises(FileExistsError):
        run(tmp_path, lambda: calls.append("constructed"))
    assert not calls
    assert (output / "report.json").read_text() == "earlier failed experiment"


@pytest.mark.parametrize("scenarios", [(), ("unknown",), (SCENARIOS[0], SCENARIOS[0])])
def test_invalid_scenario_selection_has_no_side_effects(tmp_path, scenarios):
    calls = []
    with pytest.raises(ValueError):
        run(tmp_path, lambda: calls.append("constructed"), scenarios=scenarios)
    assert not calls and not (tmp_path / "run").exists()


def test_physical_backend_is_not_supported_by_experiment_runner():
    backend = logical_backend()
    backend.name = "booster_k1_physical"
    with pytest.raises(ValueError, match="simulation backends only"):
        ExperimentBackend(backend)


@pytest.mark.parametrize("endpoint", [
    "http://192.168.10.102:8767", "https://127.0.0.1:8767",
    "http://user:password@127.0.0.1:8767", "http://127.0.0.1:8767/stop",
    "http://127.0.0.1:8767?target=robot",
])
def test_cli_rejects_nonloopback_or_ambiguous_endpoint(endpoint):
    import argparse
    spec = importlib.util.spec_from_file_location("scenario_cli", ROOT / "scripts/validate_transition_scenarios.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with pytest.raises(argparse.ArgumentTypeError):
        module.loopback_endpoint(endpoint)
