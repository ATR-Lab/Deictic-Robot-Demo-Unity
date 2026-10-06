# Offline stage evidence reports

`scripts/write_stage_report.py` implements the reporting/accounting portion of
hardware specification §13. It never imports an SDK, connects to ROS, issues a
commissioning permit or authorizes motion. The physical approval registry and
admission boundary are separate components.

From the monorepo root:

```sh
python3 transition/scripts/write_stage_report.py \
  --plan transition/examples/stages/s2-observation-preflight.json \
  --artifact-root "$PWD" \
  --output output/s2-preflight-new.json
```

Exit0 means the **declared coverage/artifact audit** passed; exit2 means a retained
`blocked`, `failed` or `not_run` report. Malformed input or an existing output path
raises an error. This utility does not certify an author's observations merely
because their file exists. Validate source hashes against the actual run's
manifest before interpreting a report as evidence for a release candidate.

The strict schema includes stage S0–S7, evidence scope, permit class, run-started
flag, planned/attempted/refused counts, every attempt/refusal, predicates,
artifact paths and final motion/task/arm disposition. Artifact paths must resolve
inside the explicit root. Source, configuration, evidence and optional permit
files receive SHA-256 hashes; supplied expected hashes must match. The report
also hashes each category, the input plan and the complete report content
excluding its own final `report_sha256` field.

Each predicate has `status` (`passed`, `failed`, `unknown`, `not_run`), a
predeclared `criterion` (or null if unresolved), evidence references, reason and
disposition. Required predicates omitted for a stage are materialized as unknown.
A claimed pass without linked verified evidence, unknown criterion, missing
required artifact or incomplete attempt count blocks stage acceptance. An
observed failed attempt or exceeded attempt quota produces a failed report.
All attempted failures/interruptions/invalid observations/artifacts remain in the
denominator; pre-actuation refusals are counted separately. Never rewrite the
planned count after execution to make a rerun campaign appear predeclared.

`automatic_stage_advancement` and `motion_authorized` are always false, including
for passed reports. A physical attempt with unknown stopping/outcome or without
a permit artifact cannot pass. The reporter does not verify permit authenticity;
only the separate trusted admission service can do that. Stage/scope validation
prevents a simulation receipt from being labelled read-only physical observation.

The supplied S1/S2 examples preserve the September 24 engineering outcomes and
missing evidence. Keep historical reports immutable and author a new plan for
a new, explicitly scoped campaign.

## September 24 native simulation result

`examples/stages/s1-final-candidate-prospective-20260924.json` is the unchanged
declaration for one combined trial: native lifecycle, ordinary runtime,
consequence runtime and receive-only ROS delivery. Its separately authored
`s1-final-candidate-result-20260924.json` records one planned trial, one attempted,
one passed and zero refused. The generated report is at the monorepo path
`output/transition-validation/stages/s1-final-candidate-20260924.json`.

Both runtime policies completed A/B/home with three scripted responses and 526
hashed journal events each. The independent offline audit recomputed both event
chains and matched all 13 frozen source/configuration hashes against the pre-run
manifest. The 25-second ROS probe received 407 measured joint samples and 199
stereo frame pairs with matching stamps. Full evidence and the independent audit
are under `transition/.runtime/reports-final-20260924/`.

This is an S1 pass for the fixed-support native simulator and scripted operator
inputs. It does not establish headset rendering, human performance, a policy
advantage or physical K1 commissioning. The earlier diagnostic campaign retains
its failed status and both attempts, including the report-write failure; S2
physical observation acceptance remains blocked by missing commissioning data.
