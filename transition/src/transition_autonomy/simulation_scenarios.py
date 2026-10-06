"""Bounded scripted return/authority experiments for simulation backends only.

These exercise the shared runtime against either native Isaac measurements or
an explicitly labeled logical fixture. Injected delivery faults are never
presented as a paused physics engine, a real network failure, or human data.
The caller must provide exclusive use of the simulator worker for the run.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import asdict
import math
from pathlib import Path
import time
from typing import Callable
import uuid

from .backends.base import Observation
from .backends.logical import LogicalBackend
from .cli import file_hash, manifest, write_json
from .journal import Journal, digest
from .model import load_task
from .runtime import Runtime


SCENARIOS = (
    "return_decision_retry",
    "snapshot_version_change",
    "stale_snapshot_delivery",
    "revoke_during_motion",
)
TERMINAL = {"succeeded", "failed", "canceled", "unknown"}


class VirtualClock:
    """Accelerated logical-fixture time, never used for a native run."""

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def logical_backend():
    return LogicalBackend({
        "point_a": {"duration": .4, "effects": {"remote.pointed_target": "A"}},
        "point_b": {"duration": 2, "effects": {"remote.pointed_target": "B"}},
        "home": {"duration": .4, "effects": {"remote.pointed_target": None}},
    })


def fresh_idle(observation: Observation, now: float) -> bool:
    return (observation.connected and observation.quiescent and not observation.fault
            and observation.active_command_id is None
            and math.isfinite(observation.observed_at)
            and 0 <= now - observation.observed_at <= .5)


class ExperimentBackend:
    """Record dispatches and optionally withhold delivery of a saved sample.

Stop is forwarded only while a fresh source observation names a command started
by this scenario. Preflight rejection never sends a stop to another owner.
This is an experiment guard, not fencing against concurrent command producers.
"""

    def __init__(self, backend):
        if backend.name not in {"logical_simulation", "isaac_sim_k1"}:
            raise ValueError("scenario runner accepts simulation backends only")
        self.source = backend
        self.name = backend.name
        self.commands = []
        self.events = []
        self.stop_requests = []
        self.outstanding = set()
        self.boot_id = None
        self.frozen = None

    def _event(self, event):
        if event is not None:
            key = (event.command_id, event.status, event.at)
            if not any((e["command_id"], e["status"], e["at"]) == key for e in self.events):
                self.events.append(asdict(event))
            if event.status in TERMINAL - {"unknown"}:
                self.outstanding.discard(event.command_id)
        return event

    def start(self, command, now):
        self.commands.append(command.to_dict())
        self.outstanding.add(command.command_id)  # An exception may follow dispatch.
        return self._event(self.source.start(command, now))

    def poll(self, now):
        return [self._event(event) for event in self.source.poll(now)]

    def observe(self, now):
        return self.frozen if self.frozen is not None else self.source.observe(now)

    def command_status(self, command_id, now):
        return self._event(self.source.command_status(command_id, now))

    def request_stop(self, now):
        attempt = {"at": now, "sent": False}
        self.stop_requests.append(attempt)
        if not self.outstanding:
            attempt["reason"] = "no outstanding command owned by this scenario"
            return
        obs = self.source.observe(now)
        attempt["observation"] = asdict(obs)
        if (not obs.connected or obs.fault or obs.boot_id != self.boot_id
                or not 0 <= now - obs.observed_at <= .5
                or obs.active_command_id not in self.outstanding):
            attempt["reason"] = "source does not freshly identify this scenario as active owner"
            return
        self.source.request_stop(now)
        attempt["sent"] = True


class Scenario:
    def __init__(self, name, backend, task_path, output, *, clock, sleep):
        if name not in SCENARIOS:
            raise ValueError("unknown simulation scenario")
        self.name = name
        self.backend = ExperimentBackend(backend)
        self.task_path = Path(task_path)
        self.output = Path(output)
        self.clock, self.sleep = clock, sleep
        self.runtime = None
        self.journal = None
        self.report = {
            "scenario": name, "passed": False, "checks": [], "injected_faults": [],
            "backend": backend.name,
            "scope": ("scripted native Isaac runtime experiment" if backend.name == "isaac_sim_k1"
                      else "logical fixture; no physics validation"),
            "human_data": False, "physical_robot_execution": False,
            "requires_exclusive_worker": True,
        }

    def check(self, name, condition, evidence=None):
        self.report["checks"].append({"name": name, "passed": bool(condition), "evidence": evidence})
        if not condition:
            raise AssertionError(name)

    def initialize(self):
        observation = self.backend.source.observe(self.clock())
        self.report["initial_observation"] = asdict(observation)
        self.check("worker is fresh, idle, and has no command owner",
                   fresh_idle(observation, self.clock()))
        self.backend.boot_id = observation.boot_id
        self.journal = Journal(self.output / "events.sqlite")
        now = self.clock()
        self.runtime = Runtime(load_task(self.task_path), self.backend, self.journal,
            now=now, clock=self.clock, operator_mode="scripted_simulation_experiment",
            configuration_hash=file_hash(self.task_path))
        self.runtime.observe(self.clock())
        self.check("runtime accepted initial simulator observation", self.runtime.fault is None)
        self.runtime.grant(set(self.runtime.task.skills), self.clock() + 120, self.clock())
        self.runtime.update_local("local.ready", True, self.clock())
        self.runtime.acknowledge(self.clock())
        self.runtime.set_attention("local", self.clock())
        self.runtime.set_local_quiescent(True, self.clock())

    def wait(self, predicate, *, timeout=25, tick=True):
        until = self.clock() + timeout
        while self.clock() < until:
            if tick:
                self.runtime.tick(self.clock(), auto_dispatch=False)
            if predicate():
                return
            if tick and self.runtime.fault:
                raise RuntimeError(self.runtime.fault)
            self.sleep(.04)
        raise TimeoutError("scenario condition did not arrive within its bounded wait")

    def reach_a(self):
        self.runtime.dispatch("point_a", self.clock(), str(uuid.uuid4()))
        self.wait(lambda: "point_a" in self.runtime.state.completed)
        fact = self.runtime.state.facts["remote.pointed_target"]
        self.check("A completion has known A evidence", fact.status == "known" and fact.value == "A")
        terminal = next(e for e in reversed(self.backend.events) if e["status"] == "succeeded")
        self.report["point_a_terminal"] = terminal
        self.check_native_evidence(terminal, completed=True)

    def check_native_evidence(self, event, *, completed):
        if self.backend.name != "isaac_sim_k1":
            self.check("logical terminal is explicitly nonphysical",
                       not event["evidence"].get("physics_validated", False))
            return
        evidence = event["evidence"]
        finite = lambda key: (type(evidence.get(key)) in (int, float)
                              and math.isfinite(evidence[key]))
        self.check("native terminal retains advancing source-time settling",
                   evidence.get("settle_clock") == "advancing_physics_simulation_time"
                   and evidence.get("velocity_source") == "position_difference_per_sim_second"
                   and finite("settle_seconds") and evidence["settle_seconds"] >= .3,
                   evidence)
        self.check("native terminal meets joint and speed limits",
                   finite("joint_error_rad") and 0 <= evidence["joint_error_rad"] <= .035
                   and finite("max_velocity_rad_s") and 0 <= evidence["max_velocity_rad_s"] <= .04)
        if completed:
            self.check("native completed point meets reference distance limit",
                       finite("reference_error_m") and 0 <= evidence["reference_error_m"] <= .012)

    def expose(self):
        lease_id = self.runtime.cue_return(self.name, 8, self.clock())
        self.check("return snapshot exposed", self.runtime.lease.status == "exposed")
        self.check("snapshot hash matches frozen contents",
                   self.runtime.lease.snapshot_hash == digest(self.runtime.lease.snapshot))
        return lease_id, self.runtime.lease.snapshot_hash

    def rejects(self, name, operation):
        try:
            operation()
        except ValueError as exc:
            self.check(name, True, str(exc))
        else:
            self.check(name, False)

    def return_decision_retry(self):
        self.reach_a()
        runtime = self.runtime
        authority = runtime.authority
        runtime.set_local_quiescent(False, self.clock())
        lease_id = runtime.cue_return(self.name, 8, self.clock())
        self.check("unconfirmed local boundary keeps return draining", runtime.lease.status == "draining")
        runtime.set_attention("remote", self.clock())
        self.check("attention return preserves authority", runtime.authority == authority)
        runtime.set_local_quiescent(True, self.clock())
        runtime.tick(self.clock(), auto_dispatch=False)
        self.check("both quiescence conditions expose snapshot", runtime.lease.status == "exposed")
        frozen_hash = runtime.lease.snapshot_hash
        self.rejects("wrong rendered hash cannot acknowledge snapshot",
            lambda: runtime.mark_displayed(lease_id, self.clock(), snapshot_hash="0" * 64))
        self.check("rejected display leaves receipt unset", runtime.lease.displayed_at is None)
        runtime.mark_displayed(lease_id, self.clock(), snapshot_hash=frozen_hash)
        decision_id = str(uuid.uuid4())
        response = runtime.respond("execute", "select_A", self.clock(), lease_id=lease_id,
                                   snapshot_hash=frozen_hash, decision_id=decision_id)
        self.check("predeclared A response matches rubric", response["correct"] is True, response)
        # Open another opportunity before the lost-reply retry. The old decision
        # must still resolve to its original immutable lease and result.
        new_lease, _ = self.expose()
        retry = runtime.respond("execute", "select_A", self.clock(), lease_id=lease_id,
                                snapshot_hash=frozen_hash, decision_id=decision_id)
        self.check("identical retry returns original result", retry == {**response, "duplicate": True}, retry)
        self.check("retry does not consume next return", runtime.lease.id == new_lease and runtime.lease.status == "exposed")
        self.rejects("changed payload cannot reuse decision ID", lambda: runtime.respond(
            "execute", "select_B", self.clock(), lease_id=lease_id,
            snapshot_hash=frozen_hash, decision_id=decision_id))
        counts = Counter(e["kind"] for e in self.journal.events())
        self.check("one durable first response and one semantic score",
                   counts["raw_response"] == counts["semantic_score"] == 1, dict(counts))
        self.check("decisions never dispatch another arm command", len(self.backend.commands) == 1)
        runtime.revoke(self.clock())  # Close remaining opportunity without motion.

    def snapshot_version_change(self):
        runtime = self.runtime
        lease_id, frozen_hash = self.expose()
        runtime.mark_displayed(lease_id, self.clock(), snapshot_hash=frozen_hash)
        before = runtime.state.facts["local.ready"].version
        runtime.update_local("local.ready", False, self.clock())
        self.check("explicit local semantic change increments version",
                   runtime.state.facts["local.ready"].version == before + 1)
        self.check("frozen snapshot retains original local value and hash",
                   runtime.lease.snapshot["local.ready"]["value"] is True
                   and digest(runtime.lease.snapshot) == frozen_hash)
        self.check("changed dependency closes return without human blame",
                   runtime.lease.status == "closed"
                   and runtime.lease.result["outcome"] == "technical_failure"
                   and runtime.lease.result["human_error"] is False, runtime.lease.result)
        self.rejects("late old-view decision is rejected", lambda: runtime.respond(
            "request_evidence", "pointing_state", self.clock(), lease_id=lease_id,
            snapshot_hash=frozen_hash, decision_id=str(uuid.uuid4())))
        self.check("invalidated opportunity creates no score or motion",
                   not self.backend.commands and not any(e["kind"] in {"raw_response", "semantic_score"}
                                                         for e in self.journal.events()))

    def stale_snapshot_delivery(self):
        runtime = self.runtime
        lease_id, frozen_hash = self.expose()
        runtime.mark_displayed(lease_id, self.clock(), snapshot_hash=frozen_hash)
        self.backend.frozen = self.backend.source.observe(self.clock())
        self.check("sample is fresh before delivery is withheld",
                   fresh_idle(self.backend.frozen, self.clock()))
        self.report["injected_faults"].append({
            "kind": "cached observation delivery held past freshness budget",
            "injection_layer": "scenario adapter", "physics_paused": False,
            "network_failure_injected": False, "hold_seconds": .65,
            "saved_source_sample": asdict(self.backend.frozen),
        })
        self.sleep(.65)
        runtime.tick(self.clock(), auto_dispatch=False)
        self.check("stale delivery latches fault and invalidates snapshot",
                   runtime.fault is not None and runtime.lease.status == "closed"
                   and runtime.lease.result["human_error"] is False, runtime.lease.result)
        self.rejects("stale-view decision cannot be scored", lambda: runtime.respond(
            "request_evidence", "pointing_state", self.clock(), lease_id=lease_id,
            snapshot_hash=frozen_hash, decision_id=str(uuid.uuid4())))
        self.backend.frozen = None
        self.sleep(.04)
        runtime.tick(self.clock(), auto_dispatch=False)
        fresh = self.backend.source.observe(self.clock())
        self.report["after_delivery_restored"] = asdict(fresh)
        self.check("source remains fresh and idle after delivery restoration", fresh_idle(fresh, self.clock()))
        self.check("delivery restoration does not clear fault or redispatch",
                   runtime.fault is not None and not self.backend.commands)
        self.check("stale opportunity has no human score", not any(
            e["kind"] in {"raw_response", "semantic_score"} for e in self.journal.events()))

    def revoke_during_motion(self):
        runtime = self.runtime
        self.reach_a()
        runtime.update_local("local.selected", "A", self.clock())
        runtime.acknowledge(self.clock())
        before = self.backend.source.observe(self.clock())
        command_id = str(uuid.uuid4())
        runtime.dispatch("point_b", self.clock(), command_id)
        self.check("B command accepted before revoke", runtime.active is not None and runtime.fault is None)
        if self.backend.name == "isaac_sim_k1":
            baseline = before.facts["robot.joints"].value
            def moved():
                sample = runtime.last_observation
                joints = sample.facts.get("robot.joints")
                return (sample.active_command_id == command_id and joints is not None
                        and max(abs(joints.value[k] - baseline[k]) for k in baseline) >= .002)
            self.wait(moved, timeout=3)
            self.report["pre_revoke_observation"] = asdict(runtime.last_observation)
            self.check("native measured arm position changed before revoke", moved())
        else:
            self.sleep(.1)
            runtime.tick(self.clock(), auto_dispatch=False)
            self.report["pre_revoke_observation"] = asdict(runtime.last_observation)
        runtime.revoke(self.clock())
        self.check("revoke latches runtime fault", runtime.fault is not None)
        self.check("stop request reached owned simulator command",
                   any(attempt["sent"] for attempt in self.backend.stop_requests))
        terminal = None
        stopped = None
        def settled():
            nonlocal terminal, stopped
            # Read source independently of runtime, preserving the revocation.
            self.backend.source.poll(self.clock()) if self.backend.name == "logical_simulation" else None
            terminal = self.backend.command_status(command_id, self.clock())
            stopped = self.backend.source.observe(self.clock())
            return terminal is not None and terminal.status in TERMINAL and fresh_idle(stopped, self.clock())
        self.wait(settled, timeout=10, tick=False)
        self.report["revoke_terminal"] = asdict(terminal)
        self.report["settled_stop_observation"] = asdict(stopped)
        self.check("revoked motion ends in canceled terminal", terminal.status == "canceled", asdict(terminal))
        self.check_native_evidence(asdict(terminal), completed=False)
        runtime.tick(self.clock(), auto_dispatch=False)
        self.check("revoked B never receives completion credit", runtime.state.completed == {"point_a"})
        self.check("revoke remains latched after measured hold", runtime.fault is not None and runtime.authority.revoked)
        self.check("no automatic replay or next motion", len(self.backend.commands) == 2)

    def cleanup(self):
        # Never clean up by sending a stop to an unowned or restarted worker.
        self.backend.frozen = None
        if not self.backend.outstanding:
            return
        self.backend.request_stop(self.clock())
        until = self.clock() + 10
        while self.clock() < until:
            for command_id in list(self.backend.outstanding):
                self.backend.command_status(command_id, self.clock())
            observation = self.backend.source.observe(self.clock())
            if not self.backend.outstanding and fresh_idle(observation, self.clock()):
                self.report["cleanup_observation"] = asdict(observation)
                return
            self.sleep(.04)
        raise TimeoutError("scenario cleanup did not establish terminal owned command and fresh idle source")

    def run(self):
        self.output.mkdir(parents=True, exist_ok=False)
        started = self.clock()
        write_json(self.output / "report.json", {**self.report, "status": "started"})
        try:
            self.initialize()
            getattr(self, self.name)()
            self.report["passed"] = True
        except Exception as exc:
            self.report["error"] = f"{type(exc).__name__}: {exc}"
        finally:
            try:
                self.cleanup()
            except Exception as exc:
                self.report.update(passed=False, cleanup_error=f"{type(exc).__name__}: {exc}")
            self.report.update(elapsed_clock_seconds=self.clock() - started,
                               command_attempts=self.backend.commands, backend_events=self.backend.events,
                               stop_requests=self.backend.stop_requests)
            if self.journal is not None:
                self.report["final_state"] = self.runtime.public_state()
                try:
                    self.report["journal_integrity"] = self.journal.verify()
                    self.journal.export(self.output / "events.jsonl")
                except Exception as exc:
                    self.report.update(passed=False, journal_error=f"{type(exc).__name__}: {exc}")
                finally:
                    self.journal.close()
            self.report["status"] = "completed" if self.report["passed"] else "failed"
            write_json(self.output / "report.json", self.report)
        return self.report


def run_suite(backend_factory: Callable, task_path, output, *, scenarios=SCENARIOS,
              clock=time.monotonic, sleep=time.sleep, clock_kind="wall_monotonic", provenance=None):
    if not scenarios or len(scenarios) != len(set(scenarios)) or set(scenarios) - set(SCENARIOS):
        raise ValueError("choose unique known scenarios")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    report = {"scope": "scripted simulation engineering experiments; no physical robot or human validation",
              "clock_kind": clock_kind, "manifest": manifest(task_path), "provenance": provenance,
              "planned_scenarios": list(scenarios), "scenarios": [], "passed": False}
    write_json(output / "report.json", report)
    for name in scenarios:
        result = Scenario(name, backend_factory(), task_path, output / name,
                          clock=clock, sleep=sleep).run()
        report["scenarios"].append(result)
        write_json(output / "report.json", report)
        if not result["passed"]:
            break  # Retain the first failure; do not continue unrelated motion.
    report["not_run"] = list(scenarios[len(report["scenarios"]):])
    report["passed"] = not report["not_run"] and all(r["passed"] for r in report["scenarios"])
    write_json(output / "report.json", report)
    return report
