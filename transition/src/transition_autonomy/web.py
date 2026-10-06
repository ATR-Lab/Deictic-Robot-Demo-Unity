"""Loopback operator console with one serialized runtime owner.

Input admission and timer claims share a condition lock. Runtime processing is
serial; request admission time and queue delay are recorded separately. The
console is not an emergency stop or a hard real-time controller.
"""
from __future__ import annotations

from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import secrets
import threading
import time
import uuid

from .runtime import Runtime


class OperatorResponseError(Exception):
    """A correlated failure must distinguish rejection from durable participation."""
    def __init__(self, error, data, captured):
        super().__init__(str(error))
        self.status = 400 if isinstance(error, ValueError) else 409
        self.payload = {"error": str(error), "response_captured": captured,
                        "lease_id": data["lease_id"], "decision_id": data["decision_id"],
                        "snapshot_hash": data.get("snapshot_hash")}


def _strict_json(raw: bytes) -> dict:
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError("duplicate JSON key: " + key)
            result[key] = value
        return result

    def nonfinite(value):
        raise ValueError("nonfinite JSON value")

    value = json.loads(raw.decode("utf-8"), object_pairs_hook=pairs, parse_constant=nonfinite)
    if not isinstance(value, dict) or set(value) - {"operation", "data"} or "operation" not in value:
        raise ValueError("expected operator envelope with operation and data")
    validate_operation(value["operation"], value.get("data", {}))
    return value


def validate_operation(operation, data):
    """Reject malformed intent before admitting it to the serialized owner."""
    shapes = {
        "grant": (set(), {"ttl", "skills"}), "acknowledge": (set(), set()),
        "attention": ({"domain"}, {"domain"}), "quiescent": ({"confirmed"}, {"confirmed"}),
        "local": ({"key", "value"}, {"key", "value"}),
        "cue": (set(), {"window", "checkpoint_id"}),
        "displayed": ({"lease_id"}, {"lease_id", "snapshot_hash"}),
        "respond": ({"lease_id", "decision_id", "kind", "target"},
                    {"lease_id", "decision_id", "kind", "target", "snapshot_hash"}),
        "run": ({"enabled"}, {"enabled"}), "revoke": (set(), set()),
        "reconcile": (set(), set()),
    }
    if not isinstance(operation, str) or operation not in shapes or not isinstance(data, dict):
        raise ValueError("unknown operation or non-object data")
    required, allowed = shapes[operation]
    if not required <= set(data) or set(data) - allowed:
        raise ValueError("missing or unexpected operation fields")
    for key in ("lease_id", "decision_id", "checkpoint_id", "kind", "target", "key", "domain"):
        if key in data and (not isinstance(data[key], str) or not 1 <= len(data[key]) <= 256
                            or any(ord(c) < 32 for c in data[key])):
            raise ValueError("invalid " + key)
    if "snapshot_hash" in data and (not isinstance(data["snapshot_hash"], str)
            or len(data["snapshot_hash"]) != 64
            or any(c not in "0123456789abcdef" for c in data["snapshot_hash"])):
        raise ValueError("invalid snapshot_hash")
    for key in ("confirmed", "enabled"):
        if key in data and type(data[key]) is not bool:
            raise ValueError(key + " must be boolean")
    for key, maximum in (("ttl", 3600), ("window", 120)):
        if key in data and (type(data[key]) not in (int, float) or not math.isfinite(data[key])
                            or not 0 < data[key] <= maximum):
            raise ValueError("invalid " + key)
    if "skills" in data and (not isinstance(data["skills"], list) or len(data["skills"]) > 256
            or any(not isinstance(s, str) or not 1 <= len(s) <= 256 for s in data["skills"])
            or len(set(data["skills"])) != len(data["skills"])):
        raise ValueError("skills must be distinct identifiers")
    if operation == "attention" and data["domain"] not in {"local", "remote"}:
        raise ValueError("invalid attention domain")
    if operation == "respond" and data["kind"] not in {"execute", "defer", "request_evidence"}:
        raise ValueError("invalid response kind")
    # This also rejects exponent overflow such as 1e999 and nested NaN values.
    json.dumps(data, allow_nan=False)


class Controller:
    def __init__(self, runtime: Runtime):
        self.runtime = runtime
        self.source_clock = runtime.clock
        self.origin = time.monotonic() - runtime.last_now
        self.runtime.clock = self.clock
        self.condition = threading.Condition()
        self.jobs = deque()
        self.running = True
        self.auto_dispatch = False
        self.next_tick = self.clock() + 0.05
        self.thread = threading.Thread(target=self._loop, name="task-runtime", daemon=True)
        self.thread.start()

    def clock(self):
        return self.source_clock() if self.source_clock is not None else time.monotonic() - self.origin

    def call(self, operation: str, data: dict | None = None):
        done = threading.Event()
        with self.condition:
            if not self.running:
                raise RuntimeError("operator controller is closing")
            job = {"op": operation, "data": data or {}, "ingress": self.clock(), "done": done}
            self.jobs.append(job)
            self.condition.notify()
        if not done.wait(15):
            raise TimeoutError("operator request queued; inspect its outcome before retrying")
        if "error" in job:
            raise job["error"]
        return job["result"]

    def _loop(self):
        while self.running:
            with self.condition:
                admitted_at = self.clock()
                if admitted_at >= self.next_tick:
                    # Timers enter the same FIFO as input, even under sustained traffic.
                    self.jobs.append({"op": "_tick", "data": {}, "ingress": admitted_at,
                                      "done": threading.Event()})
                    self.next_tick = admitted_at + 0.05
                if not self.jobs:
                    self.condition.wait(max(0.0, self.next_tick - admitted_at))
                    continue
                job = self.jobs.popleft()
                claimed_at = self.clock()
            try:
                if job["op"] == "_tick":
                    self.runtime.tick(claimed_at, auto_dispatch=self.auto_dispatch,
                                      deadline_at=job["ingress"])
                else:
                    job["result"] = self._apply(job, claimed_at)
            except Exception as exc:
                if job["op"] != "_tick":
                    job["error"] = exc
                else:
                    # No exception may silently leave the operator thinking a run is healthy.
                    with self.runtime._lock:
                        self.runtime._latch("runtime processing error: " + type(exc).__name__, claimed_at)
            finally:
                if job:
                    job["done"].set()

    def _apply(self, job, now):
        runtime, data, op = self.runtime, job["data"], job["op"]
        if op == "state":
            return {**runtime.public_state(), "schema_version": 1,
                    "auto_dispatch": self.auto_dispatch, "server_time": now}
        if op == "events":
            return runtime.journal.events()
        validate_operation(op, data)
        runtime._record("operator_ingress", now, {"operation": op, "ingress_at": job["ingress"],
            "queue_delay": now - job["ingress"], "clock_domain": "server_monotonic_run"})
        if op == "grant":
            ttl = float(data.get("ttl", 600))
            if not 0 < ttl <= 3600:
                raise ValueError("authority duration must be in (0,3600] seconds")
            runtime.grant(set(data.get("skills", runtime.task.skills)), now + ttl, now)
        elif op == "acknowledge":
            runtime.acknowledge(now)
        elif op == "attention":
            runtime.set_attention(data["domain"], now)
        elif op == "quiescent":
            if not isinstance(data.get("confirmed"), bool):
                raise ValueError("confirmed must be boolean")
            runtime.set_local_quiescent(data["confirmed"], now)
        elif op == "local":
            runtime.update_local(data["key"], data["value"], now)
        elif op == "cue":
            window = float(data.get("window", 15))
            if not 0 < window <= 120:
                raise ValueError("decision window must be in (0,120] seconds")
            runtime.cue_return(data.get("checkpoint_id", str(uuid.uuid4())), window, now)
        elif op == "displayed":
            runtime.mark_displayed(data["lease_id"], now, ingress_at=job["ingress"],
                                   snapshot_hash=data.get("snapshot_hash"))
        elif op == "respond":
            if not data.get("lease_id") or not data.get("decision_id"):
                raise ValueError("response requires displayed lease_id and stable decision_id")
            try:
                return runtime.respond(data["kind"], data["target"], now, lease_id=data["lease_id"],
                    decision_id=data["decision_id"], ingress_at=job["ingress"],
                    snapshot_hash=data.get("snapshot_hash"))
            except Exception as error:
                # This lookup runs on the serialized owner after respond returns.
                # Storage/internal failures are uncertain unless a committed ID is
                # already known. HTTP status alone is never a no-capture receipt.
                captured = (True if data["decision_id"] in runtime._operator_event_ids
                            else False if isinstance(error, ValueError) else None)
                raise OperatorResponseError(error, data, captured) from error
        elif op == "run":
            if not isinstance(data.get("enabled"), bool):
                raise ValueError("enabled must be boolean")
            self.auto_dispatch = data["enabled"]
        elif op == "revoke":
            self.auto_dispatch = False
            runtime.revoke(now)
        elif op == "reconcile":
            return runtime.reconcile(now)
        else:
            raise ValueError("unknown operation")
        return runtime.public_state()

    def close(self):
        self.auto_dispatch = False
        self.running = False
        with self.condition:
            self.condition.notify_all()
        self.thread.join(10)
        if self.thread.is_alive():
            raise TimeoutError("runtime worker has not stopped; journal must remain open")
        with self.condition:
            while self.jobs:
                job = self.jobs.popleft()
                job["error"] = RuntimeError("operator controller closed before processing request")
                job["done"].set()


def make_server(runtime: Runtime, port: int = 8766):
    token = secrets.token_urlsafe(32)
    html = (Path(__file__).parent / "ui" / "index.html").read_text().replace("__TOKEN__", token)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, data, content_type="application/json"):
            body = data.encode() if isinstance(data, str) else json.dumps(data, allow_nan=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", content_type + "; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self' 'nonce-" + token + "'; style-src 'self' 'unsafe-inline'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(body)

        def _valid_host(self):
            return self.headers.get("Host") in {f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}

        def _valid_origin(self):
            return (self.headers.get("Origin") in {None, "http://" + self.headers.get("Host", "")}
                    and self.headers.get("Sec-Fetch-Site") in {None, "none", "same-origin"})

        def do_GET(self):
            received_monotonic = time.monotonic()
            received_utc = time.time()
            if not self._valid_host():
                return self._send(403, {"error": "loopback host required"})
            try:
                if self.path == "/":
                    self._send(200, html, "text/html")
                elif self.path == "/api/session":
                    if not self._valid_origin():
                        return self._send(403, {"error": "same-origin or local client required"})
                    self._send(200, {"schema_version": 1, "session_token": token,
                                    "run_id": runtime.run_id, "backend": runtime.backend.name})
                elif self.path == "/api/time":
                    if not self._valid_origin():
                        return self._send(403, {"error": "same-origin or local client required"})
                    # Camera age display only. Do not enqueue an operator event,
                    # sample a backend, or translate this clock into authority.
                    self._send(200, {"schema_version": 1, "run_id": runtime.run_id,
                                    "clock_domain": "server_system_utc", "scope": "display_only",
                                    "server_receive_utc_seconds": received_utc,
                                    "server_send_utc_seconds": time.time(),
                                    "server_processing_seconds": time.monotonic() - received_monotonic})
                elif self.path == "/api/state":
                    self._send(200, controller.call("state"))
                elif self.path == "/api/events":
                    self._send(200, controller.call("events"))
                else:
                    self._send(404, {"error": "not found"})
            except Exception as exc:
                self._send(503, {"error": str(exc)})

        def do_POST(self):
            if not self._valid_host() or self.headers.get("X-Session-Token") != token or not self._valid_origin():
                return self._send(403, {"error": "invalid session/origin"})
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if (not 0 < length <= 65536 or self.path != "/api/operator"
                        or self.headers.get_content_type() != "application/json"
                        or self.headers.get("Transfer-Encoding") is not None
                        or len(self.headers.get_all("Content-Length", [])) != 1):
                    raise ValueError("invalid request")
                data = _strict_json(self.rfile.read(length))
                result = controller.call(data["operation"], data.get("data", {}))
                self._send(200, result)
            except OperatorResponseError as exc:
                self._send(exc.status, exc.payload)
            except (ValueError, KeyError, TypeError) as exc:
                self._send(400, {"error": str(exc)})
            except Exception as exc:
                self._send(409, {"error": str(exc)})

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    try:
        controller = Controller(runtime)
    except Exception:
        server.server_close()
        raise
    return server, controller
