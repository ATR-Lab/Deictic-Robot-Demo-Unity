"""Developer entry points; physical actuation requires separate commissioning."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

from .backends.logical import LogicalBackend
from .journal import Journal
from .model import load_task
from .runtime import Runtime


def example(name):
    bundled = Path(__file__).parent / "examples" / name
    return bundled if bundled.exists() else Path(__file__).resolve().parents[2] / "examples" / name


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def new_run(path):
    path = Path(path)
    path.mkdir(parents=True, exist_ok=False)
    return path


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def manifest(task, world=None):
    return {"created_at_utc": datetime.now(timezone.utc).isoformat(),
        "python": sys.version, "task_sha256": file_hash(task),
        "world_sha256": file_hash(world) if world else None,
        "source_sha256": {str(p.relative_to(Path(__file__).parent)): file_hash(p)
            for p in sorted(Path(__file__).parent.rglob("*.py"))}}


def demo(output):
    """Paired deterministic fixture; decisions are scripts, never human data."""
    output = new_run(output)
    task_path, world_path = example("logical_cowork.json"), example("logical_world.json")
    runs = []
    for policy in ("ordinary", "consequence"):
        journal = Journal(output / f"{policy}.sqlite")
        runtime = Runtime(load_task(task_path), LogicalBackend(json.loads(world_path.read_text())),
            journal, policy=policy, operator_mode="scripted_fixture", configuration_hash=file_hash(task_path))
        runtime.grant(set(runtime.task.skills), 30, 0)
        runtime.acknowledge(0)
        runtime.set_attention("local", 0)
        runtime.set_local_quiescent(True, 0)
        runtime.tick(0)
        first = runtime.active.skill_id
        runtime.cue_return("scripted-return", 5, 0.2)
        for i in range(3, 61):
            now = i / 10
            runtime.tick(now, auto_dispatch=False)
            if runtime.lease.status == "exposed":
                # Deliberately independent predeclared script, not rubric oracle.
                if policy == "ordinary":
                    response = ("execute", "select_blue")
                else:
                    response = ("request_evidence", "cue")
                runtime.mark_displayed(runtime.lease.id, now)
                runtime.respond(*response, now, decision_id=f"{policy}-decision", lease_id=runtime.lease.id)
                break
        for i in range(int(now * 10) + 1, 101):
            runtime.tick(i / 10)
            if len(runtime.state.completed) == len(runtime.task.skills):
                break
        journal.export(output / f"{policy}.jsonl")
        runs.append({"policy": policy, "first_skill": first,
            "completed": sorted(runtime.state.completed), "fault": runtime.fault,
            "return_result": runtime.lease.result, "journal": journal.verify()})
        journal.close()
    report = {"scope": "deterministic logical fixture; no physics, hardware, or human validation",
        "manifest": manifest(task_path, world_path), "runs": runs,
        "choice_diverged": runs[0]["first_skill"] != runs[1]["first_skill"]}
    write_json(output / "report.json", report)
    print(json.dumps({"output": str(output.resolve()), "choice_diverged": report["choice_diverged"],
        "faults": [r["fault"] for r in runs]}, indent=2))
    return 0 if report["choice_diverged"] and all(not r["fault"] for r in runs) else 1


def serve(args):
    from .web import make_server
    task_path = Path(args.task or example("logical_cowork.json" if args.backend == "logical" else "k1_pointing.json"))
    world = Path(args.world or example("logical_world.json")) if args.backend == "logical" else None
    if args.backend == "logical":
        backend = LogicalBackend(json.loads(world.read_text()))
    else:
        from .backends.isaac_backend import IsaacBackend
        backend = IsaacBackend(endpoint=args.endpoint)
    output = new_run(args.run_dir)
    write_json(output / "manifest.json", manifest(task_path, world))
    journal = Journal(output / "events.sqlite")
    runtime = server = controller = None
    try:
        runtime = Runtime(load_task(task_path), backend, journal, policy=args.policy, display=args.display,
            configuration_hash=file_hash(task_path))
        server, controller = make_server(runtime, args.port)
        print(f"Operator console: http://127.0.0.1:{server.server_port}", flush=True)
        print(f"Run evidence: {output.resolve()}", flush=True)
        print("Sequencing starts only after task authority, acknowledgement, local attention, and Start sequencing.", flush=True)
        server.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_error = None
        if controller is not None:
            try:
                controller.call("revoke")
            except Exception as exc:
                shutdown_error = exc
            # If the worker cannot stop, close() raises before the journal closes.
            controller.close()
        if server is not None:
            server.server_close()
        try:
            journal.export(output / "events.jsonl")
            if runtime is not None:
                write_json(output / "final_state.json", runtime.public_state())
        finally:
            journal.close()
        if shutdown_error is not None:
            raise shutdown_error
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    d = commands.add_parser("demo", help="run paired logical fixture and save audited results")
    d.add_argument("--output", required=True)
    s = commands.add_parser("serve", help="loopback operator console for logical or Isaac backend")
    s.add_argument("--backend", choices=("logical", "isaac"), default="logical")
    s.add_argument("--task")
    s.add_argument("--world")
    s.add_argument("--endpoint", default="http://127.0.0.1:8767")
    s.add_argument("--policy", choices=("ordinary", "consequence"), default="consequence")
    s.add_argument("--display", choices=("summary", "history"), default="summary")
    s.add_argument("--port", type=int, default=8766)
    s.add_argument("--run-dir", required=True)
    a = commands.add_parser("audit", help="verify journal integrity; never replay motion")
    a.add_argument("journal")
    a.add_argument("--export")
    args = parser.parse_args(argv)
    try:
        if args.command == "demo":
            return demo(args.output)
        if args.command == "serve":
            return serve(args)
        if not Path(args.journal).is_file():
            raise ValueError("journal does not exist")
        journal = Journal(args.journal)
        try:
            result = journal.verify()
            if args.export:
                journal.export(args.export)
            print(json.dumps(result, indent=2))
        finally:
            journal.close()
        return 0
    except (ValueError, FileExistsError, FileNotFoundError) as exc:
        parser.exit(2, f"error: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
