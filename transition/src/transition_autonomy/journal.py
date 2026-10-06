"""Durable intent/outcome journal. A journal cannot guarantee exactly-once motion."""
from __future__ import annotations

import hashlib
import json
import sqlite3
import threading
from contextlib import contextmanager, nullcontext
from pathlib import Path
from typing import Any


def canonical(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def digest(value: Any) -> str:
    return hashlib.sha256(canonical(value).encode()).hexdigest()


class Journal:
    def __init__(self, path: str | Path):
        self.path = str(path)
        if self.path != ":memory:":
            Path(self.path).parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._in_transaction = False
        self.db = sqlite3.connect(self.path, check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA synchronous=FULL")
        self.db.executescript("""
        CREATE TABLE IF NOT EXISTS events (
            seq INTEGER PRIMARY KEY, at REAL NOT NULL, kind TEXT NOT NULL,
            data TEXT NOT NULL, previous_hash TEXT NOT NULL, hash TEXT NOT NULL);
        CREATE TABLE IF NOT EXISTS commands (
            command_id TEXT PRIMARY KEY, payload TEXT NOT NULL,
            payload_hash TEXT NOT NULL, status TEXT NOT NULL);
        """)
        self.db.commit()

    def append(self, kind: str, at: float, data: dict, *, command: dict | None = None,
               command_status: tuple[str, str] | None = None) -> dict:
        """Event and intent/status update commit atomically before caller does I/O."""
        with self._lock, (nullcontext() if self._in_transaction else self.db):
            row = self.db.execute("SELECT seq,hash FROM events ORDER BY seq DESC LIMIT 1").fetchone()
            seq, previous = (row["seq"] + 1, row["hash"]) if row else (1, "0" * 64)
            entry = {"seq": seq, "at": float(at), "kind": kind, "data": data, "previous_hash": previous}
            entry["hash"] = digest(entry)
            if command is not None:
                existing = self.db.execute("SELECT payload_hash FROM commands WHERE command_id=?", (command["command_id"],)).fetchone()
                if existing is not None:
                    if existing[0] != digest(command):
                        raise ValueError("command ID was already used for a different payload")
                    raise ValueError("intent already committed; reconcile instead of dispatching again")
                self.db.execute("INSERT INTO commands VALUES(?,?,?,?)", (command["command_id"], canonical(command), digest(command), "intent"))
            if command_status:
                changed = self.db.execute("UPDATE commands SET status=? WHERE command_id=?", (command_status[1], command_status[0])).rowcount
                if changed != 1:
                    raise ValueError("cannot update an unknown command")
            self.db.execute("INSERT INTO events VALUES(?,?,?,?,?,?)", (seq, at, kind, canonical(data), previous, entry["hash"]))
            return entry

    @contextmanager
    def transaction(self):
        """Group reducer events, including raw response and lease close, atomically."""
        with self._lock:
            if self._in_transaction:
                raise RuntimeError("nested journal transaction")
            with self.db:
                self.db.execute("BEGIN IMMEDIATE")
                self._in_transaction = True
                try:
                    yield
                finally:
                    self._in_transaction = False

    def command(self, command_id: str) -> dict | None:
        with self._lock:
            row = self.db.execute("SELECT * FROM commands WHERE command_id=?", (command_id,)).fetchone()
            return {"payload": json.loads(row["payload"]), "payload_hash": row["payload_hash"], "status": row["status"]} if row else None

    def events(self) -> list[dict]:
        with self._lock:
            return [{**dict(r), "data": json.loads(r["data"])} for r in self.db.execute("SELECT * FROM events ORDER BY seq")]

    def verify(self) -> dict:
        previous = "0" * 64
        events = self.events()
        for expected_seq, event in enumerate(events, 1):
            actual_hash = event.pop("hash")
            if event["seq"] != expected_seq or event["previous_hash"] != previous or digest(event) != actual_hash:
                raise ValueError(f"journal integrity failure at event {expected_seq}")
            previous = actual_hash
        intents = {e["data"]["command"]["command_id"]: e["data"]["command"] for e in events if e["kind"] == "dispatch_intent"}
        with self._lock:
            commands = list(self.db.execute("SELECT * FROM commands"))
        if {row["command_id"] for row in commands} != set(intents):
            raise ValueError("command ledger does not match durable intents")
        for row in commands:
            payload = json.loads(row["payload"])
            if digest(payload) != row["payload_hash"] or payload != intents[row["command_id"]]:
                raise ValueError("command ledger payload was changed")
        return {"events": len(events), "commands": len(commands), "last_hash": previous, "valid": True}

    def export(self, path: str | Path) -> None:
        self.verify()
        Path(path).write_text("".join(canonical(e) + "\n" for e in self.events()))

    def close(self) -> None:
        self.db.close()
