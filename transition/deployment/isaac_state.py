"""Simulator-independent command ledger and measured completion gate.

The worker owns this state under one lock. Simulator values enter via sample();
there is deliberately no predicted task outcome or scheduler reward here.
"""
from __future__ import annotations
import hashlib
import json
import math
from pathlib import Path
import uuid


class WorkerState:
    def __init__(self, config, journal_path, now):
        self.config = config
        self.boot_id = str(uuid.uuid4())
        self.journal_path = Path(journal_path)
        self.journal_path.parent.mkdir(parents=True, exist_ok=True)
        self.ledger = {}
        self.events = []
        self.active = None
        self.pending = None
        self.stop_requested = False
        self.snapshot = {"observed_at": now, "quiescent": False, "facts": {}, "fault": "starting"}
        self.stable_since = None
        self.last_source_time = None
        self.last_sample_time = None
        self.target_id = None
        self.target_known = False
        self.reference_targets = {}
        if self.journal_path.exists():
            for line in self.journal_path.read_text().splitlines():
                entry = json.loads(line)
                self.ledger[entry["command_id"]] = entry
            for entry in self.ledger.values():
                # Even terminal evidence belongs to the previous process clock.
                # Preserve its status/evidence in the receipt, but do not renew facts.
                previous = entry["event"]
                entry["event"] = {"command_id": entry["command_id"], "status": "unknown", "at": now,
                                  "facts": {}, "detail": "previous worker boot; reconcile explicitly",
                                  "evidence": {"previous_status": previous["status"], "previous_event": previous}}

    def record(self, command_id, status, now, detail="", facts=None, evidence=None):
        event = {"command_id": command_id, "status": status, "at": now,
                 "facts": facts or {}, "detail": detail, "evidence": evidence or {}}
        self.ledger[command_id]["event"] = event
        with self.journal_path.open("a") as stream:
            stream.write(json.dumps(self.ledger[command_id], allow_nan=False) + "\n")
            stream.flush()
            import os
            os.fsync(stream.fileno())
        self.events.append(event)
        return event

    def accept(self, command, remaining, now):
        digest = hashlib.sha256(json.dumps(command, sort_keys=True, allow_nan=False).encode()).hexdigest()
        command_id = command["command_id"]
        if command_id in self.ledger:
            if digest != self.ledger[command_id]["digest"]:
                raise ValueError("command ID reused with changed payload")
            return self.ledger[command_id]["event"]
        if command.get("kind") != "point" or set(command.get("parameters", {})) != {"profile"}:
            raise ValueError("only a named point profile is supported")
        profile = command["parameters"]["profile"]
        if profile not in self.config["profiles"]:
            raise ValueError("unknown profile")
        if not isinstance(remaining, (float, int)) or not math.isfinite(remaining) or not 0 < remaining <= 120:
            raise ValueError("deadline must have 0..120 seconds remaining")
        if self.active or self.pending or not self.snapshot["quiescent"] or self.snapshot.get("fault"):
            raise ValueError("worker is not measured quiescent")
        if now - self.snapshot["observed_at"] > .5:
            raise ValueError("physics observation is stale")
        self.ledger[command_id] = {"command_id": command_id, "digest": digest, "command": command, "boot_id": self.boot_id}
        self.pending = {"command_id": command_id, "profile": profile, "deadline": now + remaining}
        self.snapshot["quiescent"] = False
        self.snapshot["active_command_id"] = command_id
        self.target_known = False
        self.snapshot["facts"]["remote.pointed_target"] = {"value": None, "status": "unknown", "observed_at": now, "lifetime": .5}
        self.stop_requested = False
        return self.record(command_id, "accepted", now, "queued for the physics owner")

    def begin(self, joints, now):
        if not self.pending:
            return
        self.active = self.pending
        self.pending = None
        self.active.update(start=now, initial=list(joints), stable_since=None)
        self.record(self.active["command_id"], "running", now, "articulation drive started")

    def desired(self, joints, now):
        if self.stop_requested and self.active:
            if "stop_target" not in self.active:
                self.active["stop_target"] = list(joints)
                self.active["stable_since"] = None
                self.active["stopping_status"] = "canceled"
            return self.active["stop_target"]
        if not self.active:
            return None
        if now > self.active["deadline"]:
            self.active["stop_target"] = list(joints)
            self.active["stable_since"] = None
            self.active["stopping_status"] = "failed"
            self.stop_requested = True
            return list(joints)
        fraction = min(1.0, max(0.0, (now - self.active["start"]) / self.config["motion_seconds"]))
        smooth = fraction * fraction * (3 - 2 * fraction)
        target = self.config["profiles"][self.active["profile"]]["joints_rad"]
        return [a + (b-a)*smooth for a, b in zip(self.active["initial"], target)]

    def sample(self, joints, velocities, reference, now, sim_time, *, velocity_source="native", native_velocities=None):
        if not math.isfinite(now) or not math.isfinite(sim_time) or sim_time < 0:
            self.stable_since = None
            if self.active:
                self.active["stable_since"] = None
            self.target_known = False
            self.snapshot.update(quiescent=False, fault="invalid physics source clock")
            return
        advancing = self.last_source_time is None or sim_time > self.last_source_time
        gap = (self.last_source_time is not None and (
            not advancing or sim_time - self.last_source_time > self.config.get("max_source_gap_seconds", .1)
            or now - self.last_sample_time > self.config.get("max_wall_gap_seconds", .5)
            or now < self.last_sample_time))
        if gap:
            self.stable_since = None
            if self.active:
                self.active["stable_since"] = None
        self.last_source_time = sim_time
        self.last_sample_time = now
        if not advancing:
            self.target_known = False
            self.snapshot.update(quiescent=False, fault="physics source clock is not advancing")
            return
        values = list(joints) + list(velocities) + list(reference)
        if len(joints) != len(self.config["joint_names"]) or len(velocities) != len(joints) or len(reference) != 3 or not all(math.isfinite(float(x)) for x in values):
            self.snapshot.update(observed_at=now, quiescent=False, fault="invalid physics measurements")
            self.target_known = False
            self.stable_since = None
            if self.active:
                self.active["stable_since"] = None
            return
        speed = max(abs(x) for x in velocities)
        settled = speed <= self.config["velocity_tolerance_rad_s"]
        if not settled:
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = sim_time
        stable = self.stable_since is not None and sim_time - self.stable_since >= self.config["settle_seconds"]
        facts = {
            "robot.joints": {"value": dict(zip(self.config["joint_names"], joints)), "observed_at": now, "lifetime": .5},
            "robot.hand_reference": {"value": list(reference), "observed_at": now, "lifetime": .5},
            "remote.pointed_target": {"value": self.target_id if self.target_known else None, "status": "known" if self.target_known else "unknown", "observed_at": now, "lifetime": .5},
        }
        if self.active:
            active = self.active
            profile = self.config["profiles"][active["profile"]]
            target = active.get("stop_target", profile["joints_rad"])
            joint_error = max(abs(a-b) for a,b in zip(joints,target))
            expected = self.reference_targets[active["profile"]]
            distance = math.sqrt(sum((a-b)**2 for a,b in zip(reference,expected)))
            matches = settled and joint_error <= self.config["joint_tolerance_rad"] and ("stop_target" in active or distance <= self.config["reference_tolerance_m"])
            if not matches:
                active["stable_since"] = None
            elif active["stable_since"] is None:
                active["stable_since"] = sim_time
            dwell = 0 if active["stable_since"] is None else sim_time - active["stable_since"]
            if matches and dwell >= self.config["settle_seconds"]:
                status = active.get("stopping_status", "succeeded")
                self.target_known = status == "succeeded"
                self.target_id = profile["target_id"] if self.target_known else None
                facts["remote.pointed_target"].update(value=self.target_id, status="known" if self.target_known else "unknown")
                self.record(active["command_id"], status, now, "measured settled postcondition" if status == "succeeded" else "measured settled hold after stop/deadline",
                            {"remote.pointed_target": facts["remote.pointed_target"]}, {"joint_error_rad": joint_error, "reference_error_m": distance,
                                    "max_velocity_rad_s": speed, "settle_seconds": dwell,
                                    "settle_clock": "advancing_physics_simulation_time",
                                    "velocity_source": velocity_source,
                                    "native_joint_velocities_rad_s": native_velocities,
                                    "measured_joints_rad": dict(zip(self.config["joint_names"], joints)),
                                    "measured_reference_m": list(reference), "target_reference_m": list(expected),
                                    "sim_time_seconds": sim_time, "fixed_base": True})
                self.active = None
                self.stop_requested = False
        elif self.target_known:
            # Renew target identity only while current measurements still support it.
            profile_name = next(k for k,v in self.config["profiles"].items() if v["target_id"] == self.target_id)
            profile = self.config["profiles"][profile_name]
            if not stable or max(abs(a-b) for a,b in zip(joints,profile["joints_rad"])) > self.config["joint_tolerance_rad"] or math.dist(reference,self.reference_targets[profile_name]) > self.config["reference_tolerance_m"]:
                self.target_known = False
                facts["remote.pointed_target"].update(value=None,status="unknown")
        self.snapshot = {"observed_at": now, "quiescent": stable and self.active is None and self.pending is None,
                         "facts": facts, "active_command_id": self.active["command_id"] if self.active else None, "fault": None,
                         "measurements": {"joint_velocities_rad_s": list(velocities), "max_velocity_rad_s": speed,
                                          "velocity_source": velocity_source, "native_joint_velocities_rad_s": native_velocities,
                                          "stable_since": self.stable_since, "sim_time_seconds": sim_time}}
