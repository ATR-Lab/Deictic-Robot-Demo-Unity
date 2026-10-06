"""Independent *software* stop requester, not a certified device watchdog.

The stop-only factory is trusted local deployment code. It may open its own SDK
client, but must expose only stop/close. No motion command is sent through this
process. A blocked dispatcher, expired heartbeat, or closed parent pipe causes a
stop request. SDK acknowledgement is never represented as measured containment.
"""
from __future__ import annotations

import multiprocessing
import threading
import time

from .manifest import finite


def _serve(connection, factory, maximum_lease_s):
    stopper = None
    deadline = None
    tripped = False
    try:
        stopper = factory()
        connection.send({"state": "ready", "independent_process": True,
                         "pending_work_contained": False})
        while True:
            wait = .05 if deadline is None else max(0., min(.05, deadline-time.monotonic()))
            reason = None
            if connection.poll(wait):
                try:
                    operation, value = connection.recv()
                except EOFError:
                    operation, value = "close", "parent_pipe_closed"
                if operation == "renew" and not tripped:
                    now = time.monotonic()
                    if deadline is not None and now >= deadline:
                        reason = "dispatcher_heartbeat_expired"
                    elif type(value) not in (int, float) or not now < value <= now + maximum_lease_s:
                        reason = "invalid_or_expired_heartbeat"
                    else:
                        deadline = value
                elif operation in ("stop", "close"):
                    reason = str(value)
                else:
                    operation = "invalid"
                    reason = "invalid_or_rearmed_stop_channel"
            elif deadline is not None and time.monotonic() >= deadline:
                operation, reason = "expired", "dispatcher_heartbeat_expired"
            else:
                continue
            if reason and not tripped:
                tripped = True
                deadline = None
                receipt = {"state": "stop_requested", "reason": reason,
                           "pending_work_contained": False, "measured_stillness": False}
                try:
                    connection.send(receipt)
                except (OSError, EOFError):
                    pass
                # This call is independent of the command owner. It can still
                # block/fail in a stock SDK; external protection is required.
                try:
                    stopper.stop()
                    receipt = {**receipt, "state": "stop_rpc_returned"}
                except Exception as error:
                    receipt = {**receipt, "state": "stop_outcome_unknown",
                               "error": type(error).__name__ + ": " + str(error)}
                try:
                    connection.send(receipt)
                except (OSError, EOFError):
                    pass
            if operation == "close":
                return
    except (EOFError, BrokenPipeError):
        # Startup IPC failure still requests a stop if the client exists.
        if stopper is not None and not tripped:
            stopper.stop()
    finally:
        if stopper is not None and callable(getattr(stopper, "close", None)):
            stopper.close()
        connection.close()


class IndependentStopWorker:
    """Explicit construction starts the stop-only client in a spawned process.

    `renew` must be called by the progressing serialized dispatcher, never by an
    unrelated UI timer. A tripped instance cannot be rearmed or auto-restarted.
    """
    independent_process = True
    device_fencing_proven = False

    def __init__(self, factory, *, maximum_lease_s=.5, startup_timeout_s=3.):
        self.maximum_lease_s = finite(maximum_lease_s, "heartbeat limit", positive=True)
        if not .05 <= self.maximum_lease_s <= 2:
            raise ValueError("Software heartbeat limit must be between .05 and 2 seconds")
        finite(startup_timeout_s, "startup timeout", positive=True)
        if startup_timeout_s > 10:
            raise ValueError("Stop worker startup timeout exceeds 10 seconds")
        context = multiprocessing.get_context("spawn")
        self._connection, child = context.Pipe()
        self._lock = threading.Lock()
        self._tripped = False
        self._closed = False
        self._latest = {"state": "starting", "pending_work_contained": False}
        self._process = context.Process(target=_serve, args=(child, factory, self.maximum_lease_s),
                                        name="k1-independent-stop", daemon=False)
        self._process.start()
        child.close()
        if not self._connection.poll(startup_timeout_s):
            self._process.terminate()
            self._process.join(.2)
            self._connection.close()
            self._closed = True
            raise RuntimeError("Stop-only worker failed to become ready; motion must remain disabled")
        self._latest = self._connection.recv()
        if self._latest.get("state") != "ready":
            raise RuntimeError("Stop-only worker startup failed")

    def renew(self, expires_at):
        finite(expires_at, "heartbeat expiry")
        with self._lock:
            self._read_locked()
            if self._closed or self._tripped or not self._process.is_alive():
                raise RuntimeError("Stop channel tripped or unavailable")
            self._connection.send(("renew", expires_at))

    def request_stop(self, reason="operator_stop"):
        with self._lock:
            if self._tripped or self._closed:
                return dict(self._latest)
            self._tripped = True
            self._latest = {"state": "stop_requested", "reason": str(reason),
                            "pending_work_contained": False, "measured_stillness": False}
            try:
                self._connection.send(("stop", str(reason)))
            except (OSError, EOFError):
                self._latest["state"] = "stop_outcome_unknown"
            return dict(self._latest)

    def _read_locked(self):
        if self._closed:
            return
        try:
            while self._connection.poll():
                self._latest = self._connection.recv()
                if self._latest.get("state") != "ready":
                    self._tripped = True
        except (EOFError, OSError):
            self._tripped = True
            self._latest = {"state": "stop_channel_disconnected", "pending_work_contained": False}

    def status(self):
        with self._lock:
            self._read_locked()
            return dict(self._latest)

    def close(self, timeout_s=.5):
        """Do not kill a blocked stop RPC and then claim that the robot stopped."""
        with self._lock:
            if self._closed:
                return True
            self._tripped = True
            try:
                self._connection.send(("close", "gateway_shutdown"))
            except (OSError, EOFError):
                pass
        self._process.join(timeout_s)
        if self._process.is_alive():
            return False
        with self._lock:
            self._read_locked()
            self._connection.close()
            self._closed = True
        return True
