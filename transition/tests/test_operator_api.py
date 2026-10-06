"""Real loopback HTTP checks for Unity session bootstrap and strict intent."""
import json
import sqlite3
import time
import urllib.error
import urllib.request

import pytest

from test_cli_web import console, post
from transition_autonomy.web import Controller


def test_session_bootstrap_is_loopback_and_same_origin(console):
    runtime, url, token, _ = console
    session = json.load(urllib.request.urlopen(url + "/api/session"))
    assert session == {"schema_version": 1, "session_token": token,
                       "run_id": runtime.run_id, "backend": runtime.backend.name}
    for headers in ({"Origin": "https://foreign.invalid"}, {"Sec-Fetch-Site": "cross-site"},
                    {"Host": "foreign.invalid"}):
        request = urllib.request.Request(url + "/api/session", headers=headers)
        with pytest.raises(urllib.error.HTTPError) as error:
            urllib.request.urlopen(request)
        assert error.value.code == 403


def test_display_clock_endpoint_is_read_only_uncached_and_does_not_enter_owner(console, monkeypatch):
    runtime, url, _, _ = console
    def forbidden_owner_call(*args, **kwargs):
        raise AssertionError("display clock must not enter the runtime owner")
    monkeypatch.setattr(Controller, "call", forbidden_owner_call)
    before = time.time()
    with urllib.request.urlopen(url + "/api/time") as response:
        assert response.headers["Cache-Control"] == "no-store"
        sample = json.load(response)
    after = time.time()
    assert sample["schema_version"] == 1 and sample["run_id"] == runtime.run_id
    assert sample["scope"] == "display_only" and sample["clock_domain"] == "server_system_utc"
    assert before <= sample["server_receive_utc_seconds"] <= sample["server_send_utc_seconds"] <= after
    assert 0 <= sample["server_processing_seconds"] <= after - before
    assert "session_token" not in sample
    assert not any(event["kind"] == "operator_ingress" for event in runtime.journal.events())
    for headers in ({"Origin": "https://foreign.invalid"}, {"Sec-Fetch-Site": "cross-site"},
                    {"Host": "foreign.invalid"}):
        request = urllib.request.Request(url + "/api/time", headers=headers)
        with pytest.raises(urllib.error.HTTPError) as error:
            urllib.request.urlopen(request)
        assert error.value.code == 403


@pytest.mark.parametrize("body", [
    '{"operation":"grant","operation":"revoke"}',
    '{"operation":"grant","data":{"ttl":NaN}}',
    '{"operation":"grant","data":{"ttl":1e999}}',
    '{"operation":"grant","data":{"ttl":true}}',
    '{"operation":"grant","data":{"ttl":"600"}}',
    '{"operation":"grant","data":{"skills":"point_a"}}',
    '{"operation":"grant","data":{"skills":["point_a","point_a"]}}',
    '{"operation":"local","data":{"key":"local.ready","value":{"x":1,"x":2}}}',
    '{"operation":"run","data":{"enabled":1}}',
    '{"operation":"acknowledge","data":{"grant":true}}',
    '{"operation":"cue","data":{"window":15},"issued_at":123}',
    '{"operation":"grant","data":[]}',
    '[]',
])
def test_malformed_operator_request_never_enters_runtime(console, body):
    runtime, url, token, _ = console
    before = [e for e in runtime.journal.events() if e["kind"] == "operator_ingress"]
    request = urllib.request.Request(url + "/api/operator", data=body.encode(),
        headers={"X-Session-Token": token, "Content-Type": "application/json"})
    with pytest.raises(urllib.error.HTTPError) as error:
        urllib.request.urlopen(request)
    assert error.value.code == 400
    assert len(before) == len([e for e in runtime.journal.events() if e["kind"] == "operator_ingress"])


def test_lease_hash_and_stable_retry_through_http(console):
    runtime, url, token, _ = console
    post(url, token, "quiescent", {"confirmed": True})
    state = post(url, token, "cue", {"window": 10})
    lease = state["lease"]
    assert lease["snapshot"]["remote.cue"]["value"] == "pending"
    bound = {"lease_id": lease["id"], "snapshot_hash": lease["snapshot_hash"]}
    post(url, token, "displayed", bound)
    payload = {**bound, "decision_id": "one-click", "kind": "request_evidence", "target": "cue"}
    result = post(url, token, "respond", payload)
    assert result["correct"] and result["lease_id"] == lease["id"]
    assert post(url, token, "respond", payload) == {**result, "duplicate": True}
    closed = json.load(urllib.request.urlopen(url + "/api/state"))["lease"]
    assert closed["snapshot_hash"] == lease["snapshot_hash"]
    assert closed["snapshot"] == lease["snapshot"]


def test_definitive_response_rejection_echoes_exact_uncaptured_identity(console):
    runtime, url, token, _ = console
    post(url, token, "quiescent", {"confirmed": True})
    lease = post(url, token, "cue", {"window": 10})["lease"]
    # Deliberately omit the render acknowledgement; this choice cannot be captured.
    payload = {"lease_id": lease["id"], "snapshot_hash": lease["snapshot_hash"],
               "decision_id": "before-render", "kind": "request_evidence", "target": "cue"}
    with pytest.raises(urllib.error.HTTPError) as error:
        post(url, token, "respond", payload)
    result = json.load(error.value)
    assert result["response_captured"] is False
    assert all(result[key] == payload[key] for key in ("lease_id", "decision_id", "snapshot_hash"))
    assert not any(event["kind"] == "raw_response" for event in runtime.journal.events())


def test_postcapture_storage_failure_never_claims_response_was_rejected(console, monkeypatch):
    runtime, url, token, _ = console
    post(url, token, "quiescent", {"confirmed": True})
    lease = post(url, token, "cue", {"window": 10})["lease"]
    bound = {"lease_id": lease["id"], "snapshot_hash": lease["snapshot_hash"]}
    post(url, token, "displayed", bound)
    append = runtime.journal.append

    def unavailable_after_capture(kind, *args, **kwargs):
        if kind in {"semantic_score", "scoring_error"}:
            raise sqlite3.OperationalError("simulated disk write failure after capture")
        return append(kind, *args, **kwargs)

    monkeypatch.setattr(runtime.journal, "append", unavailable_after_capture)
    payload = {**bound, "decision_id": "durably-captured", "kind": "request_evidence", "target": "cue"}
    with pytest.raises(urllib.error.HTTPError) as error:
        post(url, token, "respond", payload)
    result = json.load(error.value)
    assert result["response_captured"] is True
    assert result["decision_id"] == payload["decision_id"]
    assert any(event["kind"] == "response_claimed" for event in runtime.journal.events())
    retry = post(url, token, "respond", payload)
    assert retry["label"] == "scoring_pending" and retry["duplicate"]
