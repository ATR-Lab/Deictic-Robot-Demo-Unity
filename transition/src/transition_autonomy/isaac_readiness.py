"""Wait for measured startup settling before creating an Isaac task runtime.

An open worker HTTP port is not evidence that its initial physics samples have
settled. This gate is read-only; it neither clears faults nor requests motion.
"""
from __future__ import annotations

import argparse
import ipaddress
import json
import math
import time
import urllib.parse
import urllib.request
from collections.abc import Callable


def _finite(value) -> bool:
    return type(value) in (float, int) and math.isfinite(value)


class ReadinessGate:
    """Require consecutive fresh, distinct, idle samples from the same boot."""

    def __init__(self, *, stable_seconds: float = .3, max_age: float = .5):
        if not _finite(stable_seconds) or stable_seconds <= 0:
            raise ValueError("stable_seconds must be finite and positive")
        if not _finite(max_age) or max_age <= 0:
            raise ValueError("max_age must be finite and positive")
        self.stable_seconds = stable_seconds
        self.max_age = max_age
        self.reason = "no worker observation"
        self._reset()

    def _reset(self):
        self._boot = None
        self._first_stamp = self._last_stamp = None
        self._first_received = self._last_received = None

    def reject(self, reason: str) -> bool:
        self.reason = reason
        self._reset()
        return False

    def sample(self, payload, received_at: float, round_trip: float = 0.0) -> bool:
        if not _finite(received_at) or not _finite(round_trip) or round_trip < 0:
            return self.reject("invalid local observation timing")
        if not isinstance(payload, dict):
            return self.reject("no worker observation")
        observation = payload.get("observation")
        if not isinstance(observation, dict):
            return self.reject("no worker observation")
        boot = payload.get("boot_id")
        stamp, server_now = observation.get("observed_at"), payload.get("server_now")
        if not isinstance(boot, str) or not boot.strip():
            return self.reject("missing worker boot identity")
        if not _finite(stamp) or not _finite(server_now):
            return self.reject("invalid worker observation clock")
        # Include the full request round trip, as the backend adapter does.
        age = server_now - stamp
        if age < 0 or age + round_trip > self.max_age:
            return self.reject("worker observation is stale or future-dated")
        if "fault" not in observation or observation["fault"] is not None:
            return self.reject("worker reports a fault")
        if observation.get("quiescent") is not True:
            return self.reject("worker has not measured quiescence")
        if "active_command_id" not in observation or observation["active_command_id"] is not None:
            return self.reject("worker has an active command")

        if self._boot is not None and (
            boot != self._boot
            or stamp <= self._last_stamp
            or stamp - self._last_stamp > self.max_age
            or received_at < self._last_received
            or received_at - self._last_received > self.max_age
        ):
            # A cached sample, restart or acquisition gap is not a continuous
            # stable window. The current valid sample begins a new window.
            self._reset()
        if self._boot is None:
            self._boot = boot
            self._first_stamp = stamp
            self._first_received = received_at
        self._last_stamp, self._last_received = stamp, received_at
        ready = (stamp - self._first_stamp >= self.stable_seconds
                 and received_at - self._first_received >= self.stable_seconds)
        self.reason = "ready" if ready else "waiting for consecutive settled physics samples"
        return ready


def observation_url(endpoint: str) -> str:
    parsed = urllib.parse.urlsplit(endpoint)
    try:
        loopback = ipaddress.ip_address(parsed.hostname or "").is_loopback
        valid_port = parsed.port is None or 0 < parsed.port <= 65535
    except ValueError:
        loopback = valid_port = False
    if (parsed.scheme != "http" or not loopback or not valid_port
            or parsed.username is not None or parsed.password is not None
            or parsed.path not in ("", "/") or parsed.query or parsed.fragment):
        raise ValueError("Isaac readiness requires an HTTP loopback endpoint")
    return endpoint.rstrip("/") + "/observation"


def _fetch(url: str, timeout: float):
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.load(response)


def wait_until_ready(endpoint: str = "http://127.0.0.1:8767", *, timeout: float = 900,
                     poll_interval: float = .1, fetch: Callable = _fetch,
                     clock: Callable = time.monotonic, sleep: Callable = time.sleep) -> dict:
    url = observation_url(endpoint)
    if not _finite(timeout) or timeout <= 0 or not _finite(poll_interval) or poll_interval <= 0:
        raise ValueError("timeout and poll_interval must be finite and positive")
    gate = ReadinessGate()
    deadline = clock() + timeout
    while clock() < deadline:
        started = clock()
        if started >= deadline:
            break
        try:
            payload = fetch(url, min(2.0, deadline - started))
            received = clock()
            if received >= deadline:
                break
            if gate.sample(payload, received, received - started):
                return payload
        except (OSError, ValueError, TimeoutError):
            gate.reject("worker observation unavailable or invalid")
        remaining = deadline - clock()
        if remaining > 0:
            sleep(min(poll_interval, remaining))
    raise TimeoutError(f"Isaac did not become measured ready within {timeout:g} seconds: {gate.reason}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="http://127.0.0.1:8767")
    parser.add_argument("--timeout", type=float, default=900)
    args = parser.parse_args(argv)
    try:
        payload = wait_until_ready(args.endpoint, timeout=args.timeout)
    except (ValueError, TimeoutError) as exc:
        parser.exit(1, str(exc) + "\n")
    print(f"Isaac measured ready: boot={payload['boot_id']}; consecutive settled observations verified.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
