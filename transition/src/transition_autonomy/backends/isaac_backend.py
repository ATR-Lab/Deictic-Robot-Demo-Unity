"""Clock-safe HTTP adapter for the native Isaac Sim K1 worker.

No Isaac imports are needed on the experiment/controller computer. The endpoint
must be loopback or an SSH-forwarded loopback port; this is not a public service.
"""
from __future__ import annotations

import json
import math
import time
import urllib.parse
import urllib.request
from typing import Callable

from .base import BackendEvent, FactUpdate, Observation, SkillCommand


class IsaacBackend:
    name = "isaac_sim_k1"

    def __init__(self, endpoint: str = "http://127.0.0.1:8767", *, timeout: float = 2.0,
                 max_round_trip: float = 0.5, max_observation_age: float = 0.5,
                 transport: Callable | None = None):
        self.endpoint = endpoint.rstrip("/")
        self.timeout = timeout
        self.max_round_trip = max_round_trip
        self.max_observation_age = max_observation_age
        self._transport = transport or self._http
        self._boot_id = "unconnected"
        self._seen_events: set[tuple] = set()
        self._owned_commands: set[str] = set()

    def _http(self, route: str, payload: dict | None) -> dict:
        data = None if payload is None else json.dumps(payload, allow_nan=False).encode()
        request = urllib.request.Request(self.endpoint + route, data=data,
                                         headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            return json.load(response)

    def _call(self, route: str, payload: dict | None = None) -> tuple[dict, float]:
        started = time.monotonic()
        response = self._transport(route, payload)
        elapsed = time.monotonic() - started
        stamp = response.get("server_now")
        if not isinstance(stamp, (float, int)) or not math.isfinite(stamp):
            raise ValueError("worker response has no finite clock sample")
        if elapsed > self.max_round_trip:
            raise TimeoutError("worker round trip exceeds freshness budget")
        self._boot_id = str(response["boot_id"])
        return response, elapsed

    @staticmethod
    def _time(stamp: float, response: dict, now: float, elapsed: float) -> float:
        # Conservative: subtract the full RTT. Never assign future freshness.
        if not isinstance(stamp, (float, int)) or not math.isfinite(stamp) or stamp > response["server_now"] + 1e-6:
            raise ValueError("worker reported invalid or future observation")
        return now - max(0.0, response["server_now"] - stamp) - elapsed

    def _facts(self, raw: dict, response: dict, now: float, elapsed: float) -> dict:
        result = {}
        for key, value in raw.items():
            if key not in {"remote.pointed_target", "robot.joints", "robot.hand_reference"}:
                raise ValueError("worker supplied an unsupported fact")
            observed = self._time(value["observed_at"], response, now, elapsed)
            lifetime = value.get("lifetime", self.max_observation_age)
            if not isinstance(lifetime, (float, int)) or not math.isfinite(lifetime) or lifetime <= 0:
                raise ValueError("invalid observation lifetime")
            status = value.get("status", "known")
            if status not in {"known", "unknown", "conflict"}:
                raise ValueError("invalid fact status")
            result[key] = FactUpdate(value.get("value"), observed, observed + lifetime,
                                     status, "isaac_measured")
        return result

    def _event(self, value: dict, response: dict, now: float, elapsed: float) -> BackendEvent:
        if value["status"] not in {"accepted", "running", "succeeded", "failed", "canceled", "unknown"}:
            raise ValueError("invalid worker command status")
        return BackendEvent(value["command_id"], value["status"],
                            self._time(value["at"], response, now, elapsed),
                            self._facts(value.get("facts", {}), response, now, elapsed),
                            value.get("detail", ""), value.get("evidence", {}))

    def start(self, command: SkillCommand, now: float) -> BackendEvent:
        if command.kind != "point" or set(command.parameters) != {"profile"}:
            raise ValueError("Isaac requires kind='point' and one named profile")
        if command.parameters["profile"] not in {"point_a", "point_b", "home"}:
            raise ValueError("unapproved Isaac profile")
        if command.deadline <= now:
            raise ValueError("command deadline elapsed")
        self._owned_commands.add(command.command_id)
        try:
            response, elapsed = self._call("/commands/start", {
                "command": command.to_dict(), "remaining_seconds": command.deadline - now})
            return self._event(response["event"], response, now, elapsed)
        except (OSError, ValueError, KeyError, TimeoutError) as exc:
            # A network failure can occur after motion is dispatched.
            return BackendEvent(command.command_id, "unknown", now,
                                detail=f"dispatch outcome uncertain: {exc}")

    def poll(self, now: float) -> list[BackendEvent]:
        response, elapsed = self._call("/events")
        events = []
        for event in response["events"]:
            if event["command_id"] not in self._owned_commands:
                continue
            key = (response["boot_id"], event["command_id"], event["status"], event["at"])
            if key not in self._seen_events:
                events.append(self._event(event, response, now, elapsed))
                self._seen_events.add(key)
        return events

    def observe(self, now: float) -> Observation:
        try:
            response, elapsed = self._call("/observation")
            raw = response["observation"]
            if type(raw["quiescent"]) is not bool:
                raise ValueError("worker quiescence must be a measured boolean")
            observed = self._time(raw["observed_at"], response, now, elapsed)
            if now - observed > self.max_observation_age:
                raise TimeoutError("stale physics observation")
            return Observation(True, bool(raw["quiescent"]), observed, self._boot_id,
                               self._facts(raw.get("facts", {}), response, now, elapsed),
                               raw.get("active_command_id"), raw.get("fault"))
        except (OSError, ValueError, KeyError, TimeoutError) as exc:
            return Observation(False, False, now, self._boot_id, fault=str(exc))

    def request_stop(self, now: float) -> None:
        self._call("/stop", {})

    def command_status(self, command_id: str, now: float) -> BackendEvent | None:
        self._owned_commands.add(command_id)
        response, elapsed = self._call("/commands/status?id=" + urllib.parse.quote(command_id, safe=""))
        event = response.get("event")
        return None if event is None else self._event(event, response, now, elapsed)
