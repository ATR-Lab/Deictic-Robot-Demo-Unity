"""Hardware-neutral skill transport.

Adapters report measurements, never a scheduler reward or a predicted outcome.
All times are seconds in the runtime's monotonic clock domain. Remote timestamps
must be converted by the gateway; unbounded clock uncertainty is a stale reading.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Protocol


@dataclass(frozen=True)
class SkillCommand:
    command_id: str
    run_id: str
    skill_id: str
    kind: str
    parameters: dict[str, Any]
    dependency_versions: dict[str, int]
    authority_id: str
    authority_revision: int
    issued_at: float
    deadline: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class FactUpdate:
    value: Any
    observed_at: float
    valid_until: float | None = None
    status: str = "known"  # known, unknown, conflict
    source: str = "backend"


@dataclass(frozen=True)
class BackendEvent:
    command_id: str
    status: str  # accepted, running, succeeded, failed, canceled, unknown
    at: float
    facts: dict[str, FactUpdate] = field(default_factory=dict)
    detail: str = ""
    evidence: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Observation:
    connected: bool
    quiescent: bool
    observed_at: float
    boot_id: str
    facts: dict[str, FactUpdate] = field(default_factory=dict)
    active_command_id: str | None = None
    fault: str | None = None


class Backend(Protocol):
    """One motion owner. Exceptions after dispatch imply an uncertain outcome.

    `start` may return accepted/running/unknown, never invent successful effects.
    `poll` must return succeeded only with measured task postconditions/evidence.
    `request_stop` is a request: quiescence must subsequently be observed.
    Duplicate commands require a durable adapter ledger on real hardware.
    """

    name: str

    def start(self, command: SkillCommand, now: float) -> BackendEvent: ...
    def poll(self, now: float) -> list[BackendEvent]: ...
    def observe(self, now: float) -> Observation: ...
    def request_stop(self, now: float) -> None: ...
    def command_status(self, command_id: str, now: float) -> BackendEvent | None: ...
