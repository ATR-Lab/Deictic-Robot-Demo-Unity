#!/usr/bin/env python3
"""Audit declared stage evidence offline. Never connects to or authorizes a robot."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from transition_autonomy.stages import build_stage_report, load_plan, write_report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True, help="Strict schema1 JSON evidence plan")
    parser.add_argument("--artifact-root", required=True, help="Allowed root for every evidence path")
    parser.add_argument("--output", required=True, help="New JSON path; existing files are never overwritten")
    args = parser.parse_args()
    report = build_stage_report(load_plan(args.plan), args.artifact_root)
    write_report(report, args.output)
    print(f"{report['stage']}: {report['status']} ({len(report['issues'])} unresolved findings). No motion authorized.")
    return 0 if report["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
