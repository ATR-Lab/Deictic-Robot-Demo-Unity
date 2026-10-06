#!/usr/bin/env python3
"""Localhost-only ROS 2 skill gateway; this entry point cannot select hardware.

std_msgs/String carries strictly validated JSON v1 envelopes. The task runtime
remains the authority/scheduler; this node wraps a Backend, not a servo loop.
Imports are ROS-independent, permitting protocol tests without rclpy installed.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import fcntl
import json
import math
import os
from pathlib import Path
import sqlite3
import time
import uuid

from transition_autonomy.backends.base import BackendEvent, SkillCommand
from transition_autonomy.journal import canonical

SCHEMA_VERSION = 1
TERMINAL = {"succeeded", "failed", "canceled"}
MAX_MESSAGE_BYTES = 16_384


def host_clock_id() -> str:
    """Kernel boot identity: v1 deliberately rejects cross-host monotonic clocks."""
    path = Path("/proc/sys/kernel/random/boot_id")
    return path.read_text().strip() if path.exists() else "local-" + str(uuid.getnode())


def _string(value, field: str) -> str:
    if not isinstance(value, str) or not 1 <= len(value) <= 160 or any(ord(c) < 32 for c in value):
        raise ValueError(f"{field} must be a nonempty bounded string")
    return value


def _number(value, field: str) -> float:
    if type(value) not in {int, float} or not math.isfinite(value):
        raise ValueError(f"{field} must be finite")
    return float(value)


def decode_request(raw: str, *, clock_id: str, now: float) -> dict:
    if not isinstance(raw, str) or len(raw.encode()) > MAX_MESSAGE_BYTES:
        raise ValueError("Envelope exceeds size limit")

    def no_duplicates(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("Duplicate JSON key")
            result[key] = value
        return result

    value = json.loads(raw, object_pairs_hook=no_duplicates,
                       parse_constant=lambda _: (_ for _ in ()).throw(ValueError("Nonfinite JSON constant")))
    if not isinstance(value, dict):
        raise ValueError("Envelope must be an object")
    base = {"schema_version", "request_id", "operation", "clock_id"}
    op = value.get("operation")
    if op not in {"start", "status", "stop"}:
        raise ValueError("Unsupported operation")
    if set(value) != base | ({"command"} if op == "start" else {"command_id"}):
        raise ValueError("Envelope fields do not match operation")
    if type(value["schema_version"]) is not int or value["schema_version"] != SCHEMA_VERSION:
        raise ValueError("Unsupported schema_version")
    _string(value["request_id"], "request_id")
    if value["clock_id"] != clock_id:
        raise ValueError("Cross-host/restarted clock domain rejected")
    if op != "start":
        _string(value["command_id"], "command_id")
        return value
    c = value["command"]
    fields = set(SkillCommand.__dataclass_fields__)
    if not isinstance(c, dict) or set(c) != fields:
        raise ValueError("SkillCommand fields do not match schema")
    for field in ("command_id", "run_id", "skill_id", "kind", "authority_id"):
        _string(c[field], field)
    if c["kind"] != "point" or not isinstance(c["parameters"], dict) or set(c["parameters"]) != {"profile"}:
        raise ValueError("Only named pointing profiles are exposed")
    if c["parameters"]["profile"] not in ("point_a", "point_b", "home"):
        raise ValueError("Unapproved profile")
    if type(c["authority_revision"]) is not int or c["authority_revision"] < 0:
        raise ValueError("Invalid authority_revision")
    deps = c["dependency_versions"]
    if not isinstance(deps, dict) or len(deps) > 128:
        raise ValueError("Invalid dependency map")
    for key, version in deps.items():
        _string(key, "dependency")
        if type(version) is not int or version < 0:
            raise ValueError("Invalid dependency version")
    issued, deadline = _number(c["issued_at"], "issued_at"), _number(c["deadline"], "deadline")
    if issued > now + 0.05 or now - issued > 120 or deadline <= issued or deadline - issued > 120:
        raise ValueError("Command clock/deadline outside v1 bounds")
    # An expired exact duplicate is still queryable; new dispatch rejects below.
    return value


class ROSSkillGateway:
    """ROS-independent handler with durable intent and correlation receipts."""

    def __init__(self, backend, journal: str | Path, *, clock_id: str | None = None):
        self.backend = backend
        self.clock_id = clock_id or host_clock_id()
        self.gateway_boot_id = str(uuid.uuid4())
        path = Path(journal)
        path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = open(str(path) + ".lock", "a+")
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self._lock.close()
            raise RuntimeError("Gateway ledger already owned") from None
        self.db = sqlite3.connect(path)
        self.db.execute("PRAGMA synchronous=FULL")
        self.db.executescript("""
        CREATE TABLE IF NOT EXISTS gateway_commands (
            command_id TEXT PRIMARY KEY, payload TEXT NOT NULL,
            request_id TEXT NOT NULL, event TEXT NOT NULL);
        CREATE TABLE IF NOT EXISTS gateway_requests (
            request_id TEXT PRIMARY KEY, payload TEXT NOT NULL, response TEXT NOT NULL);
        CREATE TABLE IF NOT EXISTS gateway_metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL);
        """)
        prior = self.db.execute("SELECT value FROM gateway_metadata WHERE key='backend'").fetchone()
        if prior and prior[0] != backend.name:
            self.db.close()
            self._lock.close()
            raise ValueError("Ledger belongs to a different backend")
        self.db.execute("INSERT OR IGNORE INTO gateway_metadata VALUES('backend',?)", (backend.name,))
        self.db.commit()

    def close(self):
        self.db.close()
        self._lock.close()

    def _envelope(self, message_type: str, now: float, request_id: str | None, **payload) -> dict:
        return {"schema_version": SCHEMA_VERSION, "message_type": message_type,
                "request_id": request_id, "gateway_boot_id": self.gateway_boot_id,
                "clock_id": self.clock_id, "published_at": now, "backend": self.backend.name,
                **payload}

    def _save_event(self, event: BackendEvent):
        self.db.execute("UPDATE gateway_commands SET event=? WHERE command_id=?",
                        (canonical(asdict(event)), event.command_id))
        self.db.commit()

    def _event_reply(self, event: BackendEvent | dict, now: float, request_id: str) -> dict:
        return self._envelope("event", now, request_id,
                              event=asdict(event) if isinstance(event, BackendEvent) else event)

    def _reconcile(self, command_id: str, now: float) -> dict:
        row = self.db.execute("SELECT event FROM gateway_commands WHERE command_id=?", (command_id,)).fetchone()
        if not row:
            raise ValueError("Unknown gateway command")
        event = json.loads(row[0])
        if event["status"] not in TERMINAL:
            try:
                current = self.backend.command_status(command_id, now)
                if current is not None:
                    self._save_event(current)
                    return asdict(current)
            except Exception:
                pass
            # Missing backend history is never grounds to resend.
            event = asdict(BackendEvent(command_id, "unknown", now,
                                       detail="Backend outcome unavailable; retained intent requires reconciliation"))
            self._save_event(BackendEvent(**event))
        return event

    def handle(self, raw: str, now: float) -> dict:
        request_id = None
        try:
            request = decode_request(raw, clock_id=self.clock_id, now=now)
            request_id = request["request_id"]
            canonical_request = canonical(request)
            existing = self.db.execute("SELECT payload,response FROM gateway_requests WHERE request_id=?", (request_id,)).fetchone()
            if existing:
                if existing[0] != canonical_request:
                    raise ValueError("request_id reused with another payload")
                cached = json.loads(existing[1])
                # Fresh envelope marks the restart; backend receipt time stays original.
                cached.update(gateway_boot_id=self.gateway_boot_id, published_at=now)
                return cached
            op = request["operation"]
            if op == "start":
                c = request["command"]
                command_id = c["command_id"]
                row = self.db.execute("SELECT payload FROM gateway_commands WHERE command_id=?", (command_id,)).fetchone()
                if row:
                    if row[0] != canonical(c):
                        raise ValueError("command_id reused with another payload")
                    response = self._event_reply(self._reconcile(command_id, now), now, request_id)
                else:
                    if c["deadline"] <= now:
                        raise ValueError("New command deadline expired")
                    for old in self.db.execute("SELECT command_id,event FROM gateway_commands").fetchall():
                        if json.loads(old[1])["status"] not in TERMINAL:
                            if self._reconcile(old[0], now)["status"] not in TERMINAL:
                                raise ValueError("Unresolved or occupied prior command")
                    observation = self.backend.observe(now)
                    if not observation.connected or not observation.quiescent or observation.fault:
                        raise ValueError("Backend not ready/quiescent")
                    if not math.isfinite(observation.observed_at) or not 0 <= now - observation.observed_at <= 0.5:
                        raise ValueError("Backend readiness observation is stale")
                    intent = BackendEvent(command_id, "unknown", now, detail="Gateway dispatch intent committed")
                    self.db.execute("INSERT INTO gateway_commands VALUES(?,?,?,?)",
                                    (command_id, canonical(c), request_id, canonical(asdict(intent))))
                    self.db.commit()
                    try:
                        result = self.backend.start(SkillCommand(**c), now)
                    except Exception as exc:
                        result = BackendEvent(command_id, "unknown", now,
                                              detail="Dispatch outcome uncertain: " + str(exc))
                    self._save_event(result)
                    response = self._event_reply(result, now, request_id)
            elif op == "status":
                response = self._event_reply(self._reconcile(request["command_id"], now), now, request_id)
            else:
                command_id = request["command_id"]
                event = self._reconcile(command_id, now)
                observation = self.backend.observe(now)
                if event["status"] not in TERMINAL:
                    if observation.active_command_id != command_id:
                        raise ValueError("Stop target is not the observed active command")
                    self.backend.request_stop(now)
                response = self._envelope("stop_requested", now, request_id, command_id=command_id,
                                          detail="Request processed; poll/status must establish cancellation")
            self.db.execute("INSERT INTO gateway_requests VALUES(?,?,?)",
                            (request_id, canonical_request, canonical(response)))
            self.db.commit()
            return response
        except Exception as exc:
            return self._envelope("error", now, request_id, error=str(exc))

    def tick(self, now: float) -> tuple[list[dict], dict]:
        replies = []
        try:
            for event in self.backend.poll(now):
                row = self.db.execute("SELECT request_id FROM gateway_commands WHERE command_id=?", (event.command_id,)).fetchone()
                if row:
                    self._save_event(event)
                    replies.append(self._event_reply(event, now, row[0]))
            observation = asdict(self.backend.observe(now))
            unresolved = [row[0] for row in self.db.execute("SELECT command_id,event FROM gateway_commands")
                          if json.loads(row[1])["status"] not in TERMINAL]
            if any(command_id != observation["active_command_id"] for command_id in unresolved):
                observation["quiescent"] = False
                observation["fault"] = observation["fault"] or "Gateway journal requires reconciliation"
            state = self._envelope("state", now, None, observation=observation)
        except Exception as exc:
            state = self._envelope("state", now, None,
                                  observation={"connected": False, "quiescent": False,
                                               "observed_at": now, "boot_id": "unavailable",
                                               "facts": {}, "active_command_id": None, "fault": str(exc)})
        return replies, state


def make_node(gateway: ROSSkillGateway):
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String

    class GatewayNode(Node):
        def __init__(self):
            super().__init__("transition_skill_gateway")
            command_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.VOLATILE)
            event_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
            state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.events = self.create_publisher(String, "/transition/event", event_qos)
            self.states = self.create_publisher(String, "/transition/state", state_qos)
            self.commands = self.create_subscription(String, "/transition/command", self._command, command_qos)
            self.timer = self.create_timer(0.05, self._tick)

        def _publish(self, publisher, envelope):
            msg = String()
            msg.data = canonical(envelope)
            publisher.publish(msg)

        def _command(self, msg):
            self._publish(self.events, gateway.handle(msg.data, time.monotonic()))

        def _tick(self):
            events, state = gateway.tick(time.monotonic())
            for event in events:
                self._publish(self.events, event)
            self._publish(self.states, state)

    return GatewayNode()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("logical", "isaac"), default="logical")
    parser.add_argument("--journal", required=True)
    parser.add_argument("--isaac-endpoint", default="http://127.0.0.1:8767")
    args = parser.parse_args()
    if os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST" or os.environ.get("ROS_STATIC_PEERS"):
        parser.error("Set ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST and clear ROS_STATIC_PEERS")
    if os.environ.get("ROS_DOMAIN_ID") in {None, "", "0"}:
        parser.error("Choose an explicit isolated nonzero ROS_DOMAIN_ID")
    if os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp") != "rmw_fastrtps_cpp":
        parser.error("This v1 deployment is verified with rmw_fastrtps_cpp only")
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    for variable in ("ROS_DISCOVERY_SERVER", "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"):
        if os.environ.get(variable):
            parser.error(f"Clear {variable} for isolated default DDS configuration")
    if args.backend == "logical":
        from transition_autonomy.backends.logical import LogicalBackend
        backend = LogicalBackend({"point-a": {"duration": 0.2, "effects": {"remote.pointed_target": "A"}},
                                  "point-b": {"duration": 0.2, "effects": {"remote.pointed_target": "B"}},
                                  "home": {"duration": 0.2, "effects": {"remote.pointed_target": None}}})
    else:
        from urllib.parse import urlparse
        from transition_autonomy.backends.isaac_backend import IsaacBackend
        endpoint = urlparse(args.isaac_endpoint)
        if endpoint.scheme != "http" or endpoint.hostname not in {"127.0.0.1", "localhost", "::1"}:
            parser.error("Isaac endpoint must be localhost HTTP")
        backend = IsaacBackend(args.isaac_endpoint)
    import rclpy
    rclpy.init()
    gateway = ROSSkillGateway(backend, args.journal)
    node = make_node(gateway)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        gateway.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
