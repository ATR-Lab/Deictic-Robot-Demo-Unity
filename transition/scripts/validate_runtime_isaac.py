#!/usr/bin/env python3
"""Shared runtime + native Isaac scenario. Run only with exclusive worker use."""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from transition_autonomy.backends.isaac_backend import IsaacBackend
from transition_autonomy.cli import manifest, write_json
from transition_autonomy.journal import Journal
from transition_autonomy.model import load_task
from transition_autonomy.runtime import Runtime


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--endpoint", default="http://127.0.0.1:8767")
    p.add_argument("--output", required=True)
    p.add_argument("--policy", choices=("ordinary", "consequence"), default="consequence")
    args = p.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    backend = IsaacBackend(args.endpoint)
    task_path = ROOT / "examples/k1_pointing.json"
    task = load_task(task_path)
    journal = Journal(output / "events.sqlite")
    start = time.monotonic()
    clock = lambda: time.monotonic() - start
    runtime = Runtime(task, backend, journal, policy=args.policy, operator_mode="scripted_integration",
        observation_max_age=0.5, clock=clock)
    report = {"scope": "scripted runtime/native Isaac integration; no human or real K1 validation",
        "manifest": manifest(task_path), "policy": args.policy, "stages": [], "passed": False}
    try:
        runtime.observe(clock())
        if runtime.fault:
            raise RuntimeError(runtime.fault)
        runtime.grant(set(task.skills), clock() + 180, clock())
        runtime.update_local("local.ready", True, clock())
        runtime.acknowledge(clock())
        runtime.set_attention("local", clock())
        runtime.set_local_quiescent(True, clock())
        for index, (expected, selection) in enumerate((("A", "select_A"), ("B", "select_B"), (None, "sequence_complete"))):
            before = len(runtime.state.completed)
            deadline = clock() + 25
            while len(runtime.state.completed) == before and clock() < deadline:
                runtime.tick(clock())
                if runtime.fault:
                    raise RuntimeError(runtime.fault)
                time.sleep(0.04)
            if len(runtime.state.completed) == before:
                raise TimeoutError("runtime stage did not complete")
            fact = runtime.state.facts["remote.pointed_target"]
            if fact.status != "known" or fact.value != expected:
                raise AssertionError("unexpected measured point identity")
            runtime.cue_return(f"scripted-stage-{index}", 5, clock())
            if runtime.lease.status != "exposed":
                raise AssertionError("return snapshot was not exposed")
            runtime.mark_displayed(runtime.lease.id, clock())
            result = runtime.respond("defer" if expected is None else "execute", selection,
                clock(), lease_id=runtime.lease.id, decision_id=f"integration-decision-{index}")
            if not result["correct"]:
                raise AssertionError("scripted response did not match independent rubric")
            report["stages"].append({"measured_target": expected, "response": result,
                "completed": sorted(runtime.state.completed), "return": runtime.lease.result})
            if expected is not None:
                runtime.update_local("local.selected", expected, clock())
                runtime.acknowledge(clock())
        report["passed"] = len(runtime.state.completed) == len(task.skills) and not runtime.fault
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        backend.request_stop(clock())
    finally:
        report["elapsed_seconds"] = clock()
        report["final_state"] = runtime.public_state()
        report["journal_integrity"] = journal.verify()
        journal.export(output / "events.jsonl")
        journal.close()
        write_json(output / "report.json", report)
    print(json.dumps({k: report[k] for k in ("passed", "stages", "elapsed_seconds")}))
    if "error" in report:
        print(report["error"], file=sys.stderr)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
