import json
from pathlib import Path
import re
import threading
import urllib.error
import urllib.request

import pytest

from transition_autonomy.cli import demo, main
from transition_autonomy.web import make_server
from test_runtime import make


def test_demo_and_audit(tmp_path, capsys):
    output = tmp_path / "paired"
    assert demo(output) == 0
    report = json.loads((output / "report.json").read_text())
    assert report["choice_diverged"]
    assert all(run["return_result"]["outcome"] == "correct" for run in report["runs"])
    assert all(len(run["completed"]) == 2 for run in report["runs"])
    assert main(["audit", str(output / "ordinary.sqlite")]) == 0
    with pytest.raises(FileExistsError):
        demo(output)


@pytest.fixture
def console(tmp_path):
    runtime, _ = make(tmp_path)
    runtime.operator_mode = "human"
    server, controller = make_server(runtime, 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f"http://127.0.0.1:{server.server_port}"
    html = urllib.request.urlopen(url).read().decode()
    token = re.search("const token='([^']+)'", html)[1]
    yield runtime, url, token, html
    server.shutdown()
    thread.join(2)
    controller.close()
    server.server_close()
    runtime.journal.close()


def post(url, token, operation, data=None, **headers):
    request = urllib.request.Request(url + "/api/operator",
        data=json.dumps({"operation": operation, "data": data or {}}).encode(),
        headers={"X-Session-Token": token, "Content-Type": "application/json", **headers})
    return json.load(urllib.request.urlopen(request))


def test_console_requires_session_and_origin(console):
    _, url, token, html = console
    assert 'onclick="' not in html  # CSP permits nonce script, not inline attributes.
    for wrong_token, origin in (("wrong", url), (token, "https://untrusted.example")):
        with pytest.raises(urllib.error.HTTPError) as error:
            post(url, wrong_token, "grant", Origin=origin)
        assert error.value.code == 403


def test_http_response_bound_to_displayed_lease(console):
    runtime, url, token, _ = console
    post(url, token, "quiescent", {"confirmed": True})
    snapshot = post(url, token, "cue", {"window": 5, "checkpoint_id": "http-check"})
    lease = snapshot["lease"]["id"]
    with pytest.raises(urllib.error.HTTPError):
        post(url, token, "respond", {"kind": "request_evidence", "target": "cue"})
    post(url, token, "displayed", {"lease_id": lease})
    payload = {"kind": "request_evidence", "target": "cue", "lease_id": lease, "decision_id": "stable-human-id"}
    result = post(url, token, "respond", payload)
    assert result["correct"]
    assert post(url, token, "respond", payload) == {**result, "duplicate": True}
    payload["target"] = "different"
    with pytest.raises(urllib.error.HTTPError):
        post(url, token, "respond", payload)
    assert len([e for e in runtime.journal.events() if e["kind"] == "raw_response"]) == 1
