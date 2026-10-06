"""Single-writer task runtime. All motion remains inside a configured backend.

The lock serializes raw decisions, semantic updates, and return-lease transitions.
An existing journal is never treated as permission to replay physical commands.
"""
from __future__ import annotations

import copy
import math
import threading
import uuid
from dataclasses import asdict, dataclass, field
from typing import Any, Callable

from .backends.base import Backend, BackendEvent, FactUpdate, Observation, SkillCommand
from .journal import Journal, canonical, digest
from .model import Authority, Commitment, Fact, TaskSpec, semantic_equal
from .scheduler import select
from .scoring import score


@dataclass
class ReturnLease:
    id: str
    checkpoint_id: str
    cue_at: float
    deadline: float
    locked_facts: set[str]
    status: str = "draining"
    exposed_at: float | None = None
    displayed_at: float | None = None
    closed_at: float | None = None
    response_at: float | None = None
    snapshot: dict[str, dict] = field(default_factory=dict)
    snapshot_hash: str | None = None
    decision_summary: list[dict] = field(default_factory=list)
    commitment_id: str | None = None
    result: dict | None = None
    coherence_flags: list[dict] = field(default_factory=list)


class RecoveryRequired(RuntimeError):
    pass


class Runtime:
    def __init__(self, task: TaskSpec, backend: Backend, journal: Journal, *,
                 policy: str = "consequence", display: str = "summary",
                 now: float = 0.0, run_id: str | None = None,
                 observation_max_age: float = 0.5, operator_mode: str = "human",
                 configuration_hash: str = "unspecified", clock: Callable[[], float] | None = None):
        if journal.events():
            raise RecoveryRequired("existing run journal: audit/reconcile in-flight commands; automatic restart is disabled")
        if display not in {"summary", "history"}:
            raise ValueError("unknown display condition")
        self.task, self.backend, self.journal = task, backend, journal
        self.policy, self.display = policy, display
        self.operator_mode = operator_mode
        self.clock = clock
        self.run_id = run_id or str(uuid.uuid4())
        self.state = task.initial_state(now)
        self.authority: Authority | None = None
        self.commitment: Commitment | None = None
        self.attention = "remote"
        self.local_quiescent = False
        self.active: SkillCommand | None = None
        self.lease: ReturnLease | None = None
        self.leases: list[ReturnLease] = []
        self.fault: str | None = None
        self.observation_max_age = observation_max_age
        self.last_observation: Observation | None = None
        self.last_observation_valid = False
        self.boot_id: str | None = None
        self.last_now = now
        self._lock = threading.RLock()
        self._terminal_seen: set[str] = set()
        self._operator_event_ids: dict[str, tuple[str, dict]] = {}
        self.journal.append("run_started", now, {"run_id": self.run_id, "task_id": task.id,
            "backend": backend.name, "policy": policy, "display": display,
            "operator_mode": operator_mode, "configuration_hash": configuration_hash,
            "initial_facts": self._fact_view(), "validation_scope": "backend results require separately inspected evidence"})

    def _time(self, now: float) -> None:
        if now < self.last_now or not (-float("inf") < now < float("inf")):
            raise ValueError("runtime time must be finite and monotonic")
        self.last_now = now

    def _refresh_time(self, now: float) -> float:
        """Refresh a live run clock after blocking work; synthetic time is explicit."""
        refreshed = now if self.clock is None else self.clock()
        self._time(refreshed)
        return refreshed

    def _fact_view(self) -> dict[str, dict]:
        return {k: asdict(v) for k, v in self.state.facts.items()}

    def _record(self, kind: str, now: float, data: dict, **kw) -> None:
        self.journal.append(kind, now, data, **kw)

    def _latch(self, reason: str, now: float, *, request_stop: bool = True) -> None:
        if self.fault != reason:
            self.fault = reason
            self._record("runtime_fault", now, {"reason": reason})
        if request_stop:
            try:
                self.backend.request_stop(now)
            except Exception as exc:
                self._record("stop_request_failed", now, {"error": type(exc).__name__})
        if self.lease and self.lease.status in {"draining", "exposed"}:
            self._close_lease("technical_failure", now, {"reason": reason, "human_error": False})

    def _apply_updates(self, updates: dict[str, FactUpdate], now: float) -> None:
        for key, update in updates.items():
            if update.status not in {"known", "unknown", "conflict"}:
                raise ValueError("invalid evidence status")
            canonical(update.value)  # Reject non-JSON/nonfinite telemetry at boundary.
            if (not math.isfinite(update.observed_at)
                    or update.valid_until is not None and not math.isfinite(update.valid_until)):
                raise ValueError("nonfinite observation timestamp")
            if update.observed_at > now + 1e-6:
                raise ValueError("future observation timestamp")
            self._check_retrospective_evidence(key, update, now)
            old = self.state.facts.get(key)
            if old and update.observed_at < old.observed_at:
                self._record("observation_ignored", now, {"fact": key, "reason": "older than current evidence"})
                continue
            changed = old is None or old.status != update.status or not semantic_equal(old.value, update.value)
            version = 1 if old is None else old.version + int(changed)
            new = Fact(value=copy.deepcopy(update.value), version=version,
                       observed_at=update.observed_at, valid_until=update.valid_until,
                       status=update.status, source=update.source)
            self.state.facts[key] = new
            # Semantic change and evidence refresh are separate event classes.
            self._record("fact_changed" if changed else "fact_refreshed", now, {"fact": key, "value": asdict(new)})
            if changed and self.lease and self.lease.status == "exposed" and key in self.lease.locked_facts:
                self._close_lease("technical_failure", now, {"reason": "scored fact changed during lease", "fact": key, "human_error": False})

    def _check_retrospective_evidence(self, key: str, update: FactUpdate, now: float) -> None:
        """Flag point-timestamp contradictions even if newer evidence supersedes them.

        This cannot detect an unobserved change or reconstruct an acquisition interval.
        Scores remain immutable rubric results; coherence flags control attribution.
        """
        for lease in self.leases:
            if lease.exposed_at is None or key not in lease.locked_facts:
                continue
            end = lease.response_at if lease.response_at is not None else lease.closed_at
            end = now if end is None else end
            if not lease.exposed_at <= update.observed_at <= end:
                continue
            snapshot = lease.snapshot[key]
            if snapshot["status"] == update.status and semantic_equal(snapshot["value"], update.value):
                continue
            signature = digest({"fact": key, "observed_at": update.observed_at,
                                "status": update.status, "value": update.value})
            if any(flag["evidence_hash"] == signature for flag in lease.coherence_flags):
                continue
            flag = {"lease_id": lease.id, "fact": key, "observed_at": update.observed_at,
                    "received_at": now, "evidence_hash": signature,
                    "reason": "contradictory evidence timestamp overlaps decision interval",
                    "retrospective": lease.status == "closed", "human_error_attributable": False}
            self._record("snapshot_coherence_flag", now, flag)
            lease.coherence_flags.append(flag)
            if lease is self.lease and lease.status == "exposed":
                self._close_lease("technical_failure", now, {"reason": flag["reason"],
                    "fact": key, "human_error": False, "coherence_status": "uncertain"})
            elif lease.result is not None:
                lease.result = {**lease.result, "coherence_status": "uncertain", "human_error": False,
                    "human_error_attribution": "withheld_after_coherence_flag"}

    def _check_invariants(self, now: float) -> None:
        if self.active:
            skill = self.task.skills[self.active.skill_id]
            for key, expected in getattr(skill, "invariants", {}).items():
                fact = self.state.facts.get(key)
                if fact is None or not fact.fresh(now, skill.max_age) or not semantic_equal(fact.value, expected):
                    self._latch("continuous skill invariant violated: " + key, now)

    def observe(self, now: float) -> Observation:
        try:
            obs = self.backend.observe(now)
        except Exception as exc:
            obs = Observation(False, False, now, self.boot_id or "unknown", fault=type(exc).__name__)
        now = self._refresh_time(now)
        self.last_observation_valid = self._accept_observation(obs, now)
        return obs

    def _accept_observation(self, obs: Observation, now: float) -> bool:
        """One acceptance path for routine, dispatch and terminal evidence."""
        self.last_observation = obs
        fresh = 0 <= now - obs.observed_at <= self.observation_max_age
        valid = True
        if self.boot_id is not None and obs.boot_id != self.boot_id:
            self._latch("backend process changed; reconcile before further dispatch", now)
            valid = False
        self.boot_id = obs.boot_id
        if not obs.connected or not fresh or obs.fault:
            self._latch("backend unavailable, stale, or faulted", now)
            return False
        self._apply_updates(obs.facts, now)
        if obs.active_command_id is not None and (self.active is None or obs.active_command_id != self.active.command_id):
            self._latch("backend reports another motion owner", now)
            valid = False
        if self.active is None and not obs.quiescent:
            self._latch("unowned or unreconciled backend motion", now)
            valid = False
        return valid

    def grant(self, skill_ids: set[str], expires_at: float, now: float) -> None:
        with self._lock:
            self._time(now)
            if not skill_ids <= set(self.task.skills) or not math.isfinite(expires_at) or expires_at <= now:
                raise ValueError("invalid authority scope or expiration")
            if self.lease and self.lease.status == "exposed":
                self._close_lease("technical_failure", now, {"reason": "authority revised during snapshot", "human_error": False})
            revision = self.authority.revision + 1 if self.authority else 1
            self.authority = Authority(id=self.authority.id if self.authority else str(uuid.uuid4()),
                revision=revision, skill_ids=set(skill_ids), expires_at=expires_at)
            self._record("authority_granted", now, {"id": self.authority.id, "revision": revision,
                "skill_ids": sorted(skill_ids), "expires_at": expires_at})

    def revoke(self, now: float) -> None:
        with self._lock:
            self._time(now)
            if self.authority:
                self.authority = Authority(id=self.authority.id, revision=self.authority.revision + 1,
                    skill_ids=set(), expires_at=now, revoked=True)
            self._record("authority_revoked", now, {})
            if self.active:
                self._latch("authority revoked during execution", now)
            elif self.lease and self.lease.status == "exposed":
                self._close_lease("technical_failure", now, {"reason": "authority changed", "human_error": False})

    def acknowledge(self, now: float) -> str:
        with self._lock:
            self._time(now)
            self.commitment = Commitment(id=str(uuid.uuid4()),
                values={k: {"value": copy.deepcopy(f.value), "status": f.status} for k, f in self.state.facts.items()}, issued_at=now)
            self._record("acknowledgement", now, self.commitment.to_dict())
            return self.commitment.id

    def set_attention(self, domain: str, now: float) -> None:
        with self._lock:
            self._time(now)
            if domain not in {"local", "remote"}:
                raise ValueError("attention must be local or remote")
            self.attention = domain
            self._record("attention", now, {"domain": domain})

    def set_local_quiescent(self, confirmed: bool, now: float) -> None:
        with self._lock:
            self._time(now)
            self.local_quiescent = bool(confirmed)
            self._record("local_quiescence_attested", now, {"confirmed": bool(confirmed), "source": "operator"})
            if not confirmed and self.lease and self.lease.status == "exposed":
                self._close_lease("technical_failure", now, {"reason": "local stability withdrawn", "human_error": False})

    def update_local(self, key: str, value: Any, now: float) -> None:
        with self._lock:
            self._time(now)
            if key not in self.task.initial_facts or not key.startswith("local."):
                raise ValueError("operator cannot overwrite robot evidence")
            self._apply_updates({key: FactUpdate(value, now, source="operator_local_commit")}, now)
            self._check_invariants(now)

    def _selection(self, now: float):
        if self.authority is None or self.commitment is None:
            return None
        locked = self.lease.locked_facts if self.lease and self.lease.status in {"draining", "exposed"} else set()
        return select(self.task, self.state, self.commitment, self.authority, now,
                      self.policy, self.attention, locked_facts=locked)

    def _dispatch_guard(self, skill_id: str, obs: Observation, now: float) -> None:
        if self.fault or self.active:
            raise RecoveryRequired(self.fault or "a skill is already active")
        selection = self._selection(now)
        if selection is None or skill_id not in selection.candidates:
            reason = "missing authority or commitment" if selection is None else selection.rejections.get(skill_id, "urgency")
            raise ValueError("skill does not pass common eligibility/urgency checks: " + reason)
        if not obs.connected or not obs.quiescent or not 0 <= now - obs.observed_at <= self.observation_max_age:
            raise RecoveryRequired("backend observation is not fresh and quiescent at dispatch")

    def dispatch(self, skill_id: str, now: float, command_id: str | None = None) -> dict:
        with self._lock:
            self._time(now)
            command_id = command_id or str(uuid.uuid4())
            existing = self.journal.command(command_id)
            if existing:
                if existing["payload"]["skill_id"] != skill_id or existing["payload"]["run_id"] != self.run_id:
                    raise ValueError("command ID reused for another request")
                return {"duplicate": True, "command_id": command_id, "status": existing["status"]}
            if self.fault or self.active:
                raise RecoveryRequired(self.fault or "a skill is already active")
            obs = self.observe(now)
            now = self._refresh_time(now)
            self._dispatch_guard(skill_id, obs, now)
            skill = self.task.skills[skill_id]
            assert self.authority is not None
            command = SkillCommand(command_id, self.run_id, skill_id, skill.kind, copy.deepcopy(skill.parameters),
                {k: self.state.facts[k].version for k in skill.read_set if k in self.state.facts},
                self.authority.id, self.authority.revision, now, now + skill.duration_max)
            # Persist intent + support before the first possible physical effect.
            support = [asdict(o) for o in skill.outcomes]
            inflight = {key: Fact(value=None, version=self.state.facts[key].version + 1 if key in self.state.facts else 1,
                                 observed_at=now, status="unknown", source="inflight:" + command_id)
                        for key in skill.write_set}
            self._record("dispatch_intent", now, {"command": command.to_dict(), "support": support,
                "commitment_id": self.commitment.id, "reserved_resources": sorted(skill.resources),
                "inflight_facts": {k: asdict(v) for k, v in inflight.items()}}, command=command.to_dict())
            try:
                # Intent persistence may block. Recheck time-sensitive guards before I/O,
                # while the original observed facts still represent the pre-send state.
                now = self._refresh_time(now)
                self._dispatch_guard(skill_id, obs, now)
                if now >= command.deadline:
                    raise ValueError("dispatch overhead exhausted the operation duration allowance")
            except Exception as exc:
                self._record("dispatch_withheld", self.last_now, {"command_id": command_id,
                    "reason": str(exc), "sent_to_backend": False, "inflight_applied": False},
                    command_status=(command_id, "canceled"))
                raise
            self.state.facts.update(inflight)
            self.active = command
            try:
                receipt = self.backend.start(command, now)
            except Exception as exc:
                receipt = BackendEvent(command_id, "unknown", now, detail=type(exc).__name__)
            now = self._refresh_time(now)
            self._backend_event(receipt, now)
            return {"duplicate": False, "command_id": command_id, "status": receipt.status}

    @staticmethod
    def _expected_value(value: Any) -> tuple[Any, str]:
        if isinstance(value, dict) and "status" in value and "value" in value:
            return value["value"], value["status"]
        return value, "known"

    def _backend_event(self, event: BackendEvent, now: float) -> None:
        if not self.active or event.command_id != self.active.command_id:
            if event.command_id in self._terminal_seen:
                return
            self._latch("unsolicited command event", now)
            return
        if event.status not in {"accepted", "running", "succeeded", "failed", "canceled", "unknown"}:
            self._latch("invalid backend status", now)
            return
        self._record("backend_event", now, asdict(event), command_status=(event.command_id, event.status))
        if event.status in {"accepted", "running"}:
            return
        if event.status == "unknown":
            self._latch("unknown command outcome; reconciliation required", now)
            return
        skill = self.task.skills[self.active.skill_id]
        # Preserve the terminal report, then ingest current evidence through exactly
        # the same boot/owner/fault/fact checks used for ordinary observations.
        # A late success can never undo a stop/fault admitted before verification.
        fault_before_verification = self.fault
        prior_facts = copy.deepcopy(self.state.facts)
        self._apply_updates(event.facts, now)
        obs = self.observe(now)
        now = self.last_now
        self._check_invariants(now)
        grant = self.authority
        if (grant is None or not grant.valid(now) or grant.id != self.active.authority_id
                or grant.revision != self.active.authority_revision
                or skill.id not in grant.skill_ids):
            self._latch("authority expired or changed during terminal verification", now)
        if now > self.active.deadline:
            self._latch("skill duration bound exceeded during terminal verification", now)
        # Terminal motion alone is insufficient: require independently reported effects.
        def corroborated(key, expected):
            if key not in event.facts:
                return False
            measured = event.facts[key]
            value, status = self._expected_value(expected)
            if not semantic_equal(measured.value, value) or measured.status != status or not self.active.issued_at <= measured.observed_at <= now:
                return False
            current = self.state.facts.get(key)
            previous = prior_facts.get(key)
            if previous and previous.observed_at >= measured.observed_at:
                if not semantic_equal(previous.value, value) or previous.status != status:
                    return False
            if current and current.observed_at >= measured.observed_at:
                if not semantic_equal(current.value, value) or current.status != status:
                    return False
                measured = current  # Later agreeing observation may renew evidence.
            return now - measured.observed_at <= self.observation_max_age and self._evidence_current(measured, now)

        supported = any(all(corroborated(k, v) for k, v in outcome.effects.items())
                        and set(event.facts) == set(outcome.effects)
                        and event.status == getattr(outcome, "terminal_status", "succeeded") for outcome in skill.outcomes)
        succeeded = event.status == "succeeded" and supported and self.fault is None
        if not succeeded:
            for key in skill.write_set - set(event.facts):
                self._apply_updates({key: FactUpdate(None, now, status="unknown", source="unverified_terminal_effect")}, now)
            if self.fault is None:
                self._latch("out-of-model effects" if event.status == "succeeded" else "skill failed or canceled", now, request_stop=False)
        if (self.last_observation_valid and obs.quiescent
                and (event.status != "succeeded" or succeeded and not fault_before_verification)):
            if succeeded:
                self.state.completed.add(skill.id)
                self._record("skill_completed", now, {"skill_id": skill.id, "command_id": event.command_id})
            self._terminal_seen.add(event.command_id)
            self.active = None
        else:
            if self.fault is None or not obs.quiescent:
                self._latch("terminal command without observed quiescence", now)

    def cue_return(self, checkpoint_id: str, window: float, now: float) -> str:
        with self._lock:
            self._time(now)
            if not math.isfinite(window) or window <= 0 or self.lease and self.lease.status in {"draining", "exposed"}:
                raise ValueError("invalid or overlapping return opportunity")
            self.lease = ReturnLease(str(uuid.uuid4()), checkpoint_id, now, now + window, set(self.task.scored_facts))
            self.leases.append(self.lease)
            self._record("return_cue", now, {"lease_id": self.lease.id, "checkpoint_id": checkpoint_id,
                "deadline": self.lease.deadline, "locked_facts": sorted(self.lease.locked_facts)})
            self._advance_lease(now)
            return self.lease.id

    def _evidence_current(self, fact: Fact, now: float) -> bool:
        return fact.observed_at <= now and (fact.valid_until is None or now < fact.valid_until)

    def mark_displayed(self, lease_id: str, now: float, *, ingress_at: float | None = None,
                       snapshot_hash: str | None = None) -> None:
        """Browser acknowledgement after DOM update; server receipt, not eye tracking."""
        with self._lock:
            self._time(now)
            if self.lease is None or self.lease.id != lease_id or self.lease.status != "exposed":
                raise ValueError("display receipt for inactive snapshot")
            if snapshot_hash is not None and snapshot_hash != self.lease.snapshot_hash:
                raise ValueError("display receipt snapshot hash mismatch")
            ingress_at = now if ingress_at is None else ingress_at
            if not (self.lease.exposed_at <= ingress_at <= now and ingress_at < self.lease.deadline):
                raise ValueError("display receipt timestamp is outside the exposed interval")
            self._advance_lease(now, enforce_deadline=False)
            if self.lease.status != "exposed":
                raise ValueError("display receipt for invalidated snapshot")
            if self.lease.displayed_at is None:
                self._record("snapshot_display_ack", now, {"lease_id": lease_id,
                    "snapshot_hash": self.lease.snapshot_hash,
                    "ingress_at": ingress_at, "processed_at": now,
                    "timing_meaning": "server receipt of client post-render acknowledgement"})
                self.lease.displayed_at = ingress_at

    def _advance_lease(self, now: float, *, enforce_deadline: bool = True,
                       deadline_at: float | None = None) -> None:
        cutoff = now if deadline_at is None else deadline_at
        if not (-float("inf") < cutoff <= now):
            raise ValueError("deadline admission timestamp must be finite and no later than processing")
        lease = self.lease
        if lease is None or lease.status not in {"draining", "exposed"}:
            return
        if enforce_deadline and cutoff >= lease.deadline:
            self._close_lease("no_response" if lease.status == "exposed" else "system_unavailable", now,
                {"human_error": False, "reason": "deadline", "snapshot_was_exposed": lease.status == "exposed"})
            return
        if self.fault:
            self._close_lease("technical_failure", now, {"reason": self.fault, "human_error": False})
            return
        if self.authority is None or not self.authority.valid(now):
            self._close_lease("technical_failure", now, {"reason": "authority expired or absent", "human_error": False})
            return
        if lease.status == "exposed":
            if any(k not in self.state.facts or not self._evidence_current(self.state.facts[k], now) for k in lease.locked_facts):
                self._close_lease("technical_failure", now, {"reason": "scored evidence expired", "human_error": False})
            return
        obs = self.observe(now)
        now = self.last_now
        if self.fault or lease.status != "draining":
            return
        if self.authority is None or not self.authority.valid(now):
            self._close_lease("technical_failure", now, {"reason": "authority expired during snapshot acquisition", "human_error": False})
            return
        ready = self.active is None and obs.connected and obs.quiescent and self.local_quiescent
        ready &= all(k in self.state.facts and self._evidence_current(self.state.facts[k], now) for k in lease.locked_facts)
        # A delayed predeadline timer may retain the lease, but cannot manufacture
        # an on-time exposure once the real response window has already ended.
        if ready and now < lease.deadline:
            lease.snapshot = copy.deepcopy(self._fact_view())
            lease.snapshot_hash = digest(lease.snapshot)
            lease.decision_summary = self._decision_summary(lease.snapshot)
            lease.commitment_id = self.commitment.id if self.commitment else None
            lease.exposed_at, lease.status = now, "exposed"
            self._record("snapshot_exposed", now, {"lease_id": lease.id, "facts": lease.snapshot,
                "snapshot_hash": lease.snapshot_hash,
                "decision_summary": lease.decision_summary, "commitment_id": lease.commitment_id,
                "authority_revision": self.authority.revision if self.authority else None})

    def _close_lease(self, outcome: str, now: float, details: dict) -> None:
        assert self.lease is not None
        self.lease.status = "closed"
        self.lease.closed_at = now
        self.lease.result = {"outcome": outcome, "cue_to_close": now - self.lease.cue_at,
            "cue_to_snapshot": None if self.lease.exposed_at is None else self.lease.exposed_at - self.lease.cue_at,
            **details}
        self._record("return_closed", now, {"lease_id": self.lease.id, **self.lease.result})

    def respond(self, kind: str, target: str, now: float, *, decision_id: str | None = None,
                lease_id: str | None = None, ingress_at: float | None = None,
                snapshot_hash: str | None = None) -> dict:
        with self._lock:
            self._time(now)
            if kind not in {"execute", "defer", "request_evidence"}:
                raise ValueError("unknown response kind")
            decision_id = decision_id or str(uuid.uuid4())
            fingerprint = digest({"lease_id": lease_id, "snapshot_hash": snapshot_hash, "kind": kind, "target": target})
            if decision_id in self._operator_event_ids:
                old_hash, old_result = self._operator_event_ids[decision_id]
                if old_hash != fingerprint:
                    raise ValueError("decision ID reused with another response")
                return {**old_result, "duplicate": True}
            ingress_at = now if ingress_at is None else ingress_at
            if not (-float("inf") < ingress_at <= now):
                raise ValueError("invalid ingress timestamp")
            if self.lease is None or not self.lease.cue_at <= ingress_at < self.lease.deadline:
                self._advance_lease(now)
                raise ValueError("response ingress is outside the decision window")
            self.observe(now)
            now = self.last_now
            self._advance_lease(now, enforce_deadline=False)
            lease = self.lease
            if lease is None or lease.status != "exposed":
                raise ValueError("no open confirmed snapshot")
            if lease_id is not None and lease_id != lease.id:
                raise ValueError("response belongs to another snapshot lease")
            if snapshot_hash is not None and snapshot_hash != lease.snapshot_hash:
                raise ValueError("response snapshot hash mismatch")
            if lease.exposed_at is None or ingress_at < lease.exposed_at:
                raise ValueError("response ingress precedes snapshot exposure")
            if self.operator_mode == "human" and lease.displayed_at is None:
                raise ValueError("snapshot has no client display acknowledgement")
            if self.operator_mode == "human" and ingress_at < lease.displayed_at:
                raise ValueError("response ingress precedes client display acknowledgement")
            raw = {"decision_id": decision_id, "lease_id": lease.id, "kind": kind, "target": target,
                   "snapshot_hash": lease.snapshot_hash,
                   "issued_at": ingress_at, "processed_at": now, "queue_delay": now - ingress_at,
                   "dependency_versions": {k: lease.snapshot[k]["version"] for k in lease.locked_facts}}
            # Commit the first choice and all scoring inputs BEFORE calling the rubric.
            # A crash after this commit leaves an auditable pending score, not a retry.
            scoring_inputs = {"rules": copy.deepcopy(self.task.scoring_rules),
                "snapshot": copy.deepcopy(lease.snapshot), "kind": kind, "target": target}
            inputs_hash = digest(scoring_inputs)
            correlation = {"decision_id": decision_id, "lease_id": lease.id, "snapshot_hash": lease.snapshot_hash}
            pending = {"correct": None, "label": "scoring_pending", **correlation}
            close_details = {"decision_id": decision_id, "response_kind": kind, "target": target,
                "human_error": False, "scoring_status": "pending",
                "cue_to_response": ingress_at - lease.cue_at,
                "snapshot_to_response": ingress_at - lease.exposed_at,
                "display_ack_to_response": None if lease.displayed_at is None else ingress_at - lease.displayed_at}
            old_status, old_result = lease.status, lease.result
            old_closed_at, old_response_at = lease.closed_at, lease.response_at
            try:
                with self.journal.transaction():
                    self._record("raw_response", now, raw)
                    self._record("response_claimed", now, {"decision_id": decision_id, "lease_id": lease.id,
                        "response_fingerprint": fingerprint, "scoring_inputs": scoring_inputs,
                        "scoring_inputs_hash": inputs_hash})
                    lease.response_at = ingress_at
                    self._close_lease("response_captured", now, close_details)
            except BaseException:
                lease.status, lease.result = old_status, old_result
                lease.closed_at, lease.response_at = old_closed_at, old_response_at
                raise
            self._operator_event_ids[decision_id] = (fingerprint, pending)
            try:
                # Detached copies prevent a defective scorer from mutating the captured inputs.
                result = score(copy.deepcopy(scoring_inputs["rules"]),
                    copy.deepcopy(scoring_inputs["snapshot"]), kind, target)
                self._record("semantic_score", now, {"decision_id": decision_id, "lease_id": lease.id,
                    "scoring_inputs_hash": inputs_hash, **result})
            except Exception as exc:
                result = {"correct": None, "label": "scoring_error", "reason": type(exc).__name__}
                self._record("scoring_error", now, {"decision_id": decision_id, "lease_id": lease.id,
                    "scoring_inputs_hash": inputs_hash, **result})
            lease.result = {**lease.result, "outcome": result["label"],
                "scoring_status": "error" if result["label"] == "scoring_error" else "scored",
                "human_error": result["correct"] is False and not lease.coherence_flags}
            result = {**result, **correlation}
            self._operator_event_ids[decision_id] = (fingerprint, result)
            # Execution is a separate deliberate command; scoring never grants authority.
            return result

    def tick(self, now: float, *, auto_dispatch: bool = True, deadline_at: float | None = None) -> None:
        with self._lock:
            self._time(now)
            if self.active and self.authority and (self.authority.revoked or now >= self.authority.expires_at):
                self._latch("authority expired during execution", now)
            self._check_invariants(now)
            try:
                events = self.backend.poll(now)
                now = self._refresh_time(now)
                for event in events:
                    self._backend_event(event, now)
                    now = self.last_now
            except Exception as exc:
                self._latch("backend poll failed: " + type(exc).__name__, now)
            self.observe(now)
            now = self.last_now
            self._check_invariants(now)
            if self.active and self.authority and (self.authority.revoked or now >= self.authority.expires_at):
                self._latch("authority expired during execution", now)
            if self.active and now > self.active.deadline:
                self._latch("skill duration bound exceeded; outcome unresolved", now)
            self._advance_lease(now, deadline_at=deadline_at)
            if auto_dispatch and not self.fault and self.active is None:
                selection = self._selection(now)
                if selection is not None:
                    self._record("policy_decision", now, {"candidates": selection.candidates,
                        "chosen": selection.chosen.id if selection.chosen else None,
                        "burdens": selection.burdens, "rejections": selection.rejections,
                        "urgent": selection.urgent, "aggregation": self.task.aggregation})
                    if selection.chosen is not None:
                        self.dispatch(selection.chosen.id, now)

    def reconcile(self, now: float) -> dict:
        """Read-only reconciliation: no automatic retry or fault clearance."""
        with self._lock:
            self._time(now)
            obs = self.backend.observe(now)
            event = self.backend.command_status(self.active.command_id, now) if self.active else None
            result = {"observation": asdict(obs), "command": asdict(event) if event else None,
                "fault_remains_latched": self.fault, "automatic_retry": False}
            self._record("reconciliation_observed", now, result)
            return result

    def _decision_summary(self, facts: dict) -> list[dict]:
        summary = []
        if self.commitment:
            before = self.commitment.to_dict()["values"]
            for key in self.task.decision_fields:
                current, old = facts.get(key), before.get(key)
                if current and (old is None or not semantic_equal(old.get("value"), current["value"])
                                or old.get("status") != current["status"]):
                    summary.append({"fact": key, "before": copy.deepcopy(old), "current": copy.deepcopy(current)})
        return summary

    def public_state(self) -> dict:
        with self._lock:
            facts = self._fact_view()
            summary = self._decision_summary(facts)
            lease = None if self.lease is None else {"id": self.lease.id, "checkpoint_id": self.lease.checkpoint_id,
                "status": self.lease.status, "cue_at": self.lease.cue_at, "deadline": self.lease.deadline,
                "exposed_at": self.lease.exposed_at, "displayed_at": self.lease.displayed_at,
                "closed_at": self.lease.closed_at, "response_at": self.lease.response_at,
                "snapshot": copy.deepcopy(self.lease.snapshot), "snapshot_hash": self.lease.snapshot_hash,
                "decision_summary": copy.deepcopy(self.lease.decision_summary) if self.display == "summary" else [],
                "commitment_id": self.lease.commitment_id,
                "coherence_flags": copy.deepcopy(self.lease.coherence_flags), "result": copy.deepcopy(self.lease.result)}
            return {"run_id": self.run_id, "backend": self.backend.name, "policy": self.policy,
                "display": self.display, "attention": self.attention, "facts": facts,
                "completed": sorted(self.state.completed), "active": self.active.to_dict() if self.active else None,
                "fault": self.fault, "local_quiescent": self.local_quiescent, "lease": lease,
                "authority": None if self.authority is None else {"id": self.authority.id,
                    "revision": self.authority.revision, "expires_at": self.authority.expires_at,
                    "skills": sorted(self.authority.skill_ids), "revoked": self.authority.revoked},
                "commitment_id": self.commitment.id if self.commitment else None,
                "decision_summary": summary if self.display == "summary" else [],
                "skills": [{"id": s.id, "kind": s.kind} for s in self.task.skills.values()]}
