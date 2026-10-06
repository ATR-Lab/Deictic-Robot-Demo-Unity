"""Independent ingress/timer/startup regressions for the operator console."""
from collections import deque
from types import SimpleNamespace
import threading

import pytest

from transition_autonomy import web
from transition_autonomy import cli
from transition_autonomy.backends.logical import LogicalBackend
from transition_autonomy.journal import Journal
from transition_autonomy.model import TaskSpec
from transition_autonomy.runtime import Runtime


def test_queued_reads_do_not_starve_periodic_runtime_checks():
    """Logical time advances while the queue stays nonempty; no sleeps needed."""
    current = [0.0]
    ticks = []
    reads = []
    controller = web.Controller.__new__(web.Controller)

    class ReadOnlyRuntime:
        def public_state(self):
            reads.append(current[0])
            current[0] += 0.01
            if len(reads) == 40:
                controller.running = False
            else:
                controller.jobs.append({"op": "state", "data": {}, "ingress": current[0],
                                        "done": threading.Event()})
            return {}

        def tick(self, now, *, auto_dispatch, deadline_at=None):
            ticks.append(now)

    controller.runtime = ReadOnlyRuntime()
    controller.clock = lambda: current[0]
    controller.condition = threading.Condition()
    controller.jobs = deque([{"op": "state", "data": {}, "ingress": 0.0,
                             "done": threading.Event()}])
    controller.running = True
    controller.next_tick = 0.05
    controller.auto_dispatch = False
    controller._loop()
    assert len(reads) == 40
    assert ticks, "A continuously occupied HTTP queue suppressed all periodic checks for 0.4 seconds"


def test_predeadline_display_ingress_survives_late_queue_processing():
    task = TaskSpec("web-review", {}, {"remote.target": "revision"},
                    {"remote.target": "A"}, {"remote.target"},
                    [{"when": {"remote.target": "A"},
                      "responses": [{"kind": "defer", "target": "complete"}]}])
    journal = Journal(":memory:")
    runtime = Runtime(task, LogicalBackend({}), journal)
    runtime.grant(set(), 100.0, 0.0)
    runtime.acknowledge(0.0)
    runtime.set_local_quiescent(True, 0.0)
    lease_id = runtime.cue_return("return", 1.0, 0.0)
    controller = web.Controller.__new__(web.Controller)
    controller.runtime = runtime
    controller.auto_dispatch = False
    try:
        # Both messages arrived within the window; a preceding owner operation
        # delayed processing. Display and response preserve their admission order.
        controller._apply({"op": "displayed", "data": {"lease_id": lease_id},
                           "ingress": 0.8}, 1.1)
        result = controller._apply({"op": "respond", "data": {"lease_id": lease_id,
                    "decision_id": "first", "kind": "defer", "target": "complete"},
                    "ingress": 0.9}, 1.2)
        assert result["correct"] is True
        assert runtime.lease.result["cue_to_response"] == 0.9
    finally:
        journal.close()


def test_server_bind_failure_does_not_leak_controller(monkeypatch):
    created = []

    class SpyController:
        def __init__(self, runtime):
            self.closed = False
            created.append(self)

        def close(self):
            self.closed = True

    def fail_bind(*args, **kwargs):
        raise OSError("address already in use")

    monkeypatch.setattr(web, "Controller", SpyController)
    monkeypatch.setattr(web, "ThreadingHTTPServer", fail_bind)
    with pytest.raises(OSError, match="address already in use"):
        web.make_server(object(), 12345)
    assert all(controller.closed for controller in created), "Unreachable runtime owner remained running after bind failure"


def test_controller_rejects_calls_after_close_without_queueing(monkeypatch):
    class ImmediateEvent:
        def wait(self, timeout):
            return False

    controller = web.Controller.__new__(web.Controller)
    controller.running = False
    controller.condition = threading.Condition()
    controller.jobs = deque()
    controller.clock = lambda: 0.0
    monkeypatch.setattr(web.threading, "Event", ImmediateEvent)
    with pytest.raises(RuntimeError, match="clos|stopp|shut"):
        controller.call("state")
    assert not controller.jobs


def test_cli_startup_failure_closes_created_journal(tmp_path, monkeypatch):
    created = []

    class TrackingJournal(Journal):
        def __init__(self, path):
            super().__init__(path)
            self.closed = False
            created.append(self)

        def close(self):
            self.closed = True
            super().close()

    def fail_bind(*args, **kwargs):
        raise OSError("address already in use")

    monkeypatch.setattr(cli, "Journal", TrackingJournal)
    monkeypatch.setattr(web, "make_server", fail_bind)
    args = SimpleNamespace(task=None, world=None, backend="logical", endpoint=None,
                           run_dir=tmp_path / "run", policy="consequence", display="summary", port=12345)
    try:
        with pytest.raises(OSError, match="address already in use"):
            cli.serve(args)
        assert created and all(journal.closed for journal in created)
    finally:
        for journal in created:
            if not journal.closed:
                journal.close()


def test_close_does_not_claim_live_runtime_worker_has_stopped():
    class StillRunningThread:
        def join(self, timeout):
            pass

        def is_alive(self):
            return True

    controller = web.Controller.__new__(web.Controller)
    controller.running = True
    controller.auto_dispatch = True
    controller.condition = threading.Condition()
    controller.jobs = deque()
    controller.thread = StillRunningThread()
    with pytest.raises(TimeoutError, match="journal must remain open"):
        controller.close()


def test_cli_revoke_error_still_cleans_up_after_worker_stops(tmp_path, monkeypatch):
    journals = []

    class TrackingJournal(Journal):
        closed = False

        def __init__(self, path):
            super().__init__(path)
            journals.append(self)

        def close(self):
            self.closed = True
            super().close()

    class FailingRevoke:
        closed = False

        def call(self, operation):
            raise RuntimeError("revoke recording failed")

        def close(self):
            self.closed = True

    class Server:
        server_port = 12345
        closed = False

        def serve_forever(self, **kwargs):
            raise KeyboardInterrupt

        def server_close(self):
            self.closed = True

    server, controller = Server(), FailingRevoke()
    monkeypatch.setattr(cli, "Journal", TrackingJournal)
    monkeypatch.setattr(web, "make_server", lambda *args: (server, controller))
    args = SimpleNamespace(task=None, world=None, backend="logical", endpoint=None,
                           run_dir=tmp_path / "run", policy="consequence", display="summary", port=12345)
    try:
        with pytest.raises(RuntimeError, match="revoke recording failed"):
            cli.serve(args)
        assert controller.closed
        assert server.closed and journals[0].closed
        assert (args.run_dir / "events.jsonl").exists()
    finally:
        for journal in journals:
            if not journal.closed:
                journal.close()


def test_controller_preserves_explicit_absolute_runtime_clock(monkeypatch):
    class NoopThread:
        def __init__(self, **kwargs):
            pass

        def start(self):
            pass

        def join(self, timeout):
            pass

        def is_alive(self):
            return False

    absolute_clock = lambda: 1000.5
    runtime = SimpleNamespace(last_now=999.0, clock=absolute_clock)
    monkeypatch.setattr(web.threading, "Thread", NoopThread)
    monkeypatch.setattr(web.time, "monotonic", lambda: 1005.0)
    controller = web.Controller(runtime)
    try:
        assert controller.clock() == 1000.5
        assert runtime.clock() == 1000.5
    finally:
        controller.close()
