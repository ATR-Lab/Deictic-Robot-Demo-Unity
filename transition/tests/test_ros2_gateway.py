"""Pure protocol tests; live rclpy/DDS roundtrip has a separate receipt."""
import importlib.util
import json
from pathlib import Path

import pytest

from transition_autonomy.backends.logical import LogicalBackend

spec = importlib.util.spec_from_file_location("ros2_gateway", Path(__file__).parents[1] / "deployment/ros2_gateway.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
ROSSkillGateway, decode_request = module.ROSSkillGateway, module.decode_request


def envelope(request_id="r1", command_id="c1", operation="start", **changes):
    request = {"schema_version": 1, "request_id": request_id, "operation": operation, "clock_id": "test-clock"}
    if operation == "start":
        request["command"] = {"command_id": command_id, "run_id": "run1", "skill_id": "point-a", "kind": "point",
                              "parameters": {"profile": "point_a"}, "dependency_versions": {}, "authority_id": "test",
                              "authority_revision": 1, "issued_at": 1.0, "deadline": 10.0}
    else:
        request["command_id"] = command_id
    request.update(changes)
    return json.dumps(request)


def setup(tmp_path):
    backend = LogicalBackend({"point-a": {"duration": 0.2, "effects": {"remote.pointed_target": "A"}}})
    gateway = ROSSkillGateway(backend, tmp_path / "gateway.db", clock_id="test-clock")
    return gateway, backend


def test_correlated_result_and_state(tmp_path):
    gateway, backend = setup(tmp_path)
    response = gateway.handle(envelope(), 1.0)
    assert response["event"]["status"] == "accepted"
    events, state = gateway.tick(1.3)
    assert events[0]["request_id"] == "r1"
    assert events[0]["event"]["status"] == "succeeded"
    assert events[0]["event"]["evidence"]["physics_validated"] is False
    assert state["observation"]["facts"]["remote.pointed_target"]["value"] == "A"
    gateway.close()


def test_duplicates_never_forward_and_changed_payload_rejected(tmp_path):
    gateway, backend = setup(tmp_path)
    gateway.handle(envelope(), 1.0)
    gateway.handle(envelope(), 1.0)
    gateway.handle(envelope(request_id="r2"), 1.0)
    assert backend.starts == 1
    changed = json.loads(envelope(request_id="r3"))
    changed["command"]["authority_revision"] = 2
    assert gateway.handle(json.dumps(changed), 1.0)["message_type"] == "error"
    assert gateway.handle(envelope(command_id="other"), 1.0)["message_type"] == "error"
    gateway.close()


def test_restart_lost_backend_history_blocks_replay_and_new_work(tmp_path):
    gateway, backend = setup(tmp_path)
    gateway.handle(envelope(), 1.0)
    gateway.close()
    restarted, new_backend = setup(tmp_path)
    _, state = restarted.tick(1.1)
    assert state["observation"]["quiescent"] is False
    assert "reconciliation" in state["observation"]["fault"]
    response = restarted.handle(envelope(request_id="r2"), 1.1)
    assert response["event"]["status"] == "unknown"
    assert restarted.handle(envelope(request_id="r3", command_id="c2"), 1.1)["message_type"] == "error"
    assert new_backend.starts == 0
    restarted.close()


@pytest.mark.parametrize("mutation", [
    {"schema_version": True}, {"clock_id": "another-host"}, {"extra": 1}, {"operation": "reset"},
])
def test_strict_envelope(mutation):
    with pytest.raises(ValueError):
        decode_request(envelope(**mutation), clock_id="test-clock", now=1.0)


def test_duplicate_json_keys_and_unbounded_values_rejected():
    with pytest.raises(ValueError, match="Duplicate"):
        decode_request('{"a":1,"a":2}', clock_id="test-clock", now=1.0)
    bad = json.loads(envelope())
    bad["command"]["deadline"] = float("nan")
    with pytest.raises(ValueError):
        decode_request(json.dumps(bad), clock_id="test-clock", now=1.0)


def test_stop_target_and_terminal_status(tmp_path):
    gateway, backend = setup(tmp_path)
    gateway.handle(envelope(), 1.0)
    wrong = gateway.handle(envelope("wrong", "c2", "stop"), 1.0)
    assert wrong["message_type"] == "error"
    response = gateway.handle(envelope("stop", "c1", "stop"), 1.0)
    assert response["message_type"] == "stop_requested"
    status = gateway.handle(envelope("status", "c1", "status"), 1.1)
    assert status["event"]["status"] == "canceled"
    gateway.close()
