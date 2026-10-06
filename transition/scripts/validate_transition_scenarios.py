#!/usr/bin/env python3
"""Run bounded scripted transition experiments with exclusive simulator use."""
import argparse
import json
from pathlib import Path
import sys
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from transition_autonomy.backends.isaac_backend import IsaacBackend
from transition_autonomy.simulation_scenarios import SCENARIOS, VirtualClock, logical_backend, run_suite


def loopback_endpoint(value):
    parsed = urlsplit(value)
    if (parsed.scheme != "http" or parsed.hostname not in {"127.0.0.1", "localhost", "::1"}
            or parsed.username or parsed.password or parsed.query or parsed.fragment
            or parsed.path not in {"", "/"}):
        raise argparse.ArgumentTypeError("use a loopback HTTP Isaac worker or SSH-forwarded loopback endpoint")
    return value.rstrip("/")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("logical", "isaac"), default="logical")
    parser.add_argument("--endpoint", type=loopback_endpoint, default="http://127.0.0.1:8767")
    parser.add_argument("--output", required=True, help="new evidence directory; existing directories are refused")
    parser.add_argument("--scenario", choices=SCENARIOS, action="append", help="default: all four scenarios")
    args = parser.parse_args(argv)
    options = {"scenarios": tuple(args.scenario or SCENARIOS)}
    if args.backend == "logical":
        clock = VirtualClock()
        factory = logical_backend
        options.update(clock=clock, sleep=clock.sleep, clock_kind="accelerated_logical_fixture")
    else:
        # Read-only provenance before any start request, so this cannot silently
        # turn into an experiment against an arbitrary motion gateway.
        check = IsaacBackend(args.endpoint)
        try:
            payload, _ = check._call("/provenance")
            provenance = payload["provenance"]
            config = json.loads((ROOT / "config/isaac_k1.json").read_text())
            if (provenance.get("fixed_base") is not True
                    or provenance.get("all_non_right_arm_joints_fixed") is not True
                    or provenance.get("model_revision") != config["model_revision"]
                    or provenance.get("urdf_sha256") != config["urdf_sha256"]
                    or provenance.get("joint_names") != config["joint_names"]):
                raise ValueError("worker provenance does not match the pinned fixed-support K1 scene")
        except Exception as exc:
            parser.exit(1, f"Isaac preflight failed before any motion: {exc}\n")
        factory = lambda: IsaacBackend(args.endpoint)
        options["provenance"] = provenance
    try:
        report = run_suite(factory, ROOT / "examples/k1_pointing.json", args.output, **options)
    except (ValueError, FileExistsError) as exc:
        parser.exit(2, f"error: {exc}\n")
    print(json.dumps({"passed": report["passed"], "output": str(Path(args.output).resolve()),
                      "scenarios": [{"name": r["scenario"], "passed": r["passed"], "error": r.get("error")}
                                    for r in report["scenarios"]], "not_run": report["not_run"]}))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
