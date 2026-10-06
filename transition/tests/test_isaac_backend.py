import copy
import importlib.util
import json
from pathlib import Path

import pytest

from transition_autonomy.backends.base import SkillCommand
from transition_autonomy.backends.isaac_backend import IsaacBackend

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("isaac_state", ROOT / "deployment/isaac_state.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def command(identifier="command-1"):
    return SkillCommand(identifier, "run", "point_a", "point", {"profile": "point_a"}, {}, "operator", 1, 0, 20)


@pytest.fixture
def state(tmp_path):
    config = json.loads((ROOT / "config/isaac_k1.json").read_text())
    state = module.WorkerState(config, tmp_path / "ledger.jsonl", 0)
    state.reference_targets = {key: [1.,2.,3.] for key in config["profiles"]}
    for i in range(9):
        state.sample([0]*4, [0]*4, [0]*3, i*.05, i*.05)
    return state


def start(state):
    state.accept(command().to_dict(), 20, .5)
    state.begin([0]*4, .5)


def test_elapsed_time_does_not_prove_completion(state):
    start(state)
    state.desired([0]*4, 10)
    for now in [10.,10.5]:
        state.sample([0]*4, [0]*4, [1,2,3], now, now)
    assert state.active is not None
    assert state.ledger["command-1"]["event"]["status"] == "running"
    assert state.snapshot["facts"]["remote.pointed_target"]["status"] == "unknown"


def test_joint_match_without_independent_link_match_is_not_success(state):
    start(state)
    target = state.config["profiles"]["point_a"]["joints_rad"]
    for now in [3.,3.5]:
        state.sample(target, [0]*4, [9,2,3], now, now)
    assert state.active is not None


def test_dwell_success_and_current_measurements_renew_identity(state):
    start(state)
    target = state.config["profiles"]["point_a"]["joints_rad"]
    state.sample(target, [0]*4, [1,2,3], 3, 3)
    assert state.active is not None
    for i in range(1,9):
        state.sample(target, [0]*4, [1,2,3], 3+i*.05, 3+i*.05)
    event = state.ledger["command-1"]["event"]
    assert event["status"] == "succeeded"
    assert set(event["facts"]) == {"remote.pointed_target"}
    assert event["facts"]["remote.pointed_target"]["value"] == "A"
    state.sample(target, [0]*4, [1,2,3], 3.45, 3.45)
    assert state.snapshot["facts"]["remote.pointed_target"]["observed_at"] == 3.45
    state.sample(target, [0]*4, [1.1,2,3], 3.5, 3.5)
    assert state.snapshot["facts"]["remote.pointed_target"]["status"] == "unknown"


def test_stop_requires_measured_quiescence(state):
    start(state)
    state.stop_requested = True
    assert state.desired([.1]*4, 1) == [.1]*4
    state.sample([.1]*4, [.3]*4, [0]*3, 1, 1)
    assert not state.snapshot["quiescent"]
    for now in [2+i*.05 for i in range(9)]:
        state.sample([.1]*4, [0]*4, [0]*3, now, now)
    assert state.ledger["command-1"]["event"]["status"] == "canceled"
    assert state.snapshot["quiescent"]


def test_duplicate_does_not_dispatch_and_reboot_does_not_replay(state):
    payload = command().to_dict()
    first = state.accept(payload, 20, .5)
    assert state.accept(payload, 20, .6) == first
    changed = copy.deepcopy(payload)
    changed["parameters"]["profile"] = "point_b"
    with pytest.raises(ValueError, match="changed payload"):
        state.accept(changed, 20, .7)
    restarted = module.WorkerState(state.config, state.journal_path, 100)
    assert restarted.accept(payload, 20, 101)["status"] == "unknown"
    assert restarted.pending is None


def observation(server_time=50, observed=49.9):
    return {"server_now": server_time, "boot_id": "boot", "observation": {
        "observed_at": observed, "quiescent": True, "facts": {
            "remote.pointed_target": {"value": "A", "observed_at": observed, "lifetime": .5}}}}


def test_remote_clock_offset_is_converted_to_conservative_local_age():
    backend = IsaacBackend(transport=lambda *_: observation())
    result = backend.observe(1000)
    assert result.connected
    assert 999.89 < result.observed_at <= 999.9
    fact = result.facts["remote.pointed_target"]
    assert fact.valid_until == pytest.approx(fact.observed_at + .5)


@pytest.mark.parametrize("raw", [observation(observed=40), observation(observed=51)])
def test_stale_or_future_measurements_fail_closed(raw):
    result = IsaacBackend(transport=lambda *_: raw).observe(1000)
    assert not result.connected and not result.quiescent and not result.facts


def test_network_timeout_after_dispatch_is_unknown():
    def unavailable(*_):
        raise TimeoutError("transport unavailable")
    backend = IsaacBackend(transport=unavailable)
    assert backend.start(command(), 1).status == "unknown"
    assert not backend.observe(1).connected


def test_poll_is_non_destructive_but_adapter_deduplicates():
    raw = {"server_now": 10, "boot_id": "boot", "events": [
        {"command_id": "x", "status": "running", "at": 9}]}
    backend = IsaacBackend(transport=lambda *_: raw)
    backend._owned_commands.add("x")
    assert len(backend.poll(100)) == 1
    assert backend.poll(101) == []


def test_old_sessions_events_do_not_become_unsolicited_current_motion():
    raw = {"server_now": 10, "boot_id": "boot", "events": [
        {"command_id": "previous-validation", "status": "succeeded", "at": 9}]}
    backend = IsaacBackend(transport=lambda *_: raw)
    assert backend.poll(100) == []


def test_paused_source_cannot_accumulate_dwell_or_renew_freshness(state):
    start(state)
    target = state.config["profiles"]["point_a"]["joints_rad"]
    state.sample(target, [0]*4, [1,2,3], 3, 3)
    state.sample(target, [0]*4, [1,2,3], 10, 3)
    assert state.active is not None
    assert not state.snapshot["quiescent"]
    assert state.snapshot["observed_at"] == 3
    assert state.active["stable_since"] is None


@pytest.mark.parametrize("next_wall,next_source", [(3.5, 3.5), (5,3.05), (3.05,1)])
def test_source_gap_wall_gap_or_reset_restarts_dwell(state, next_wall, next_source):
    start(state)
    target = state.config["profiles"]["point_a"]["joints_rad"]
    state.sample(target, [0]*4, [1,2,3], 3, 3)
    state.sample(target, [0]*4, [1,2,3], next_wall, next_source)
    assert state.active is not None
    assert not state.snapshot["quiescent"]
