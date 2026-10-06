"""Fast event simulator, explicitly not a robot-physics validation backend."""
from __future__ import annotations

from .base import BackendEvent, FactUpdate, Observation, SkillCommand
from ..journal import digest


class LogicalBackend:
    name = "logical_simulation"

    def __init__(self, private_realizations: dict[str, dict]):
        # Scheduler receives neither this map nor an outcome oracle.
        self._world = private_realizations
        self._active: tuple[SkillCommand, float] | None = None
        self._ledger: dict[str, tuple[str, BackendEvent]] = {}
        self._facts: dict[str, FactUpdate] = {}
        self.connected = True
        self.starts = 0

    def start(self, command: SkillCommand, now: float) -> BackendEvent:
        key = digest(command.to_dict())
        if command.command_id in self._ledger:
            old_key, event = self._ledger[command.command_id]
            if old_key != key:
                raise ValueError("duplicate ID with changed payload")
            return event
        if not self.connected or self._active:
            raise RuntimeError("backend unavailable or already moving")
        realization = self._world[command.skill_id]
        self._active = (command, now + float(realization["duration"]))
        self.starts += 1
        event = BackendEvent(command.command_id, "accepted", now, detail="logical simulation only")
        self._ledger[command.command_id] = (key, event)
        return event

    def poll(self, now: float) -> list[BackendEvent]:
        if not self.connected or self._active is None or now < self._active[1]:
            return []
        command, finished = self._active
        realization = self._world[command.skill_id]
        status = realization.get("status", "succeeded")
        facts = {k: FactUpdate(v, now, source="logical_fixture") for k, v in realization.get("effects", {}).items()}
        self._facts.update(facts)
        event = BackendEvent(command.command_id, status, now, facts, "logical fixture result", {"physics_validated": False, "planned_finish": finished})
        self._ledger[command.command_id] = (self._ledger[command.command_id][0], event)
        self._active = None
        return [event]

    def observe(self, now: float) -> Observation:
        # Observation freshness renews evidence without changing semantic versions.
        facts = {k: FactUpdate(v.value, now, status=v.status, source=v.source) for k, v in self._facts.items()}
        return Observation(self.connected, self.connected and self._active is None, now,
                           "logical-process", facts, self._active[0].command_id if self._active else None)

    def request_stop(self, now: float) -> None:
        if self.connected and self._active:
            command, _ = self._active
            event = BackendEvent(command.command_id, "canceled", now, detail="logical cancel; no physical guarantee")
            self._ledger[command.command_id] = (self._ledger[command.command_id][0], event)
            self._active = None

    def command_status(self, command_id: str, now: float) -> BackendEvent | None:
        return self._ledger.get(command_id, (None, None))[1] if self.connected else None
