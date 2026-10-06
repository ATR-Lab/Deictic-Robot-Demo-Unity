# Audit for a K1 simulation-to-hardware acceptance specification

Read-only audit of the current checkout, 23 September 2026. No server, ROS process, SDK connection, simulator or robot was executed for this audit. This is input for the parent specification, not authorization to commission hardware.

## 1. What the retained evidence currently supports

| Layer | Existing executable / evidence | Established result | Transfer boundary |
|---|---|---|---|
| Current software | `.venv/bin/python -m pytest -q`; `artifacts/paper-audit-runtime.json` | Latest recorded local suite: **133 passed**; regression prevents completion credit before terminal quiescence | This final correction was not rerun on the remote/native pipeline. The older local and remote JUnit files both contain **132** tests; do not present them as 133-test receipts. |
| Isaac backend conformance | `scripts/validate_isaac.py`; `artifacts/isaac/validation_receipt.json`, `runtime_manifest.json`, `RUN_REPORT.md` | Actual native A→B→home, duplicate ID, mid-motion stop, and dropped accepted-response recovery passed | Fixed trunk; only right arm moves; self-collision disabled; project-defined hand reference; no free-standing, collision, physical K1, or directional-pointing proof. |
| Native measurements | Same final receipt and `config/isaac_k1.json` | A/B/home reference errors 4.240/5.784/2.042 mm; joint errors 0.01150/0.01912/0.00767 rad; at least 0.3 advancing physics seconds of settling | These are simulator conformance figures, **not physical acceptance tolerances or hardware repeatability estimates**. Native `qdot` remained inconsistent; the simulator explicitly uses position-derived velocity and retains raw `qdot`. |
| Actual ROS transport | `deployment/test_ros2_roundtrip.py`; `artifacts/ros2-logical-roundtrip.json`, `artifacts/ros2-isaac-roundtrip.json` | Separate Jazzy client/gateway processes; measured native result; status and identical-command retry preserve terminal receipt | Current executable selects only logical/Isaac; local kernel/boot clock; localhost discovery. It does not connect to the K1 SDK or establish a physical ROS driver. |
| Full runtime/native integration | `scripts/validate_runtime_isaac.py`; `artifacts/runtime-isaac-release/report.json` and journal | Three native skills and three explicit scripted local/return decisions; journal verifies | **Historical baseline:** package-relative source hashes differ from current `runtime.py` after the terminal-quiescence fix. Task file still matches. Rerun the current source before calling this the current end-to-end baseline. |
| Browser interaction | `artifacts/operator-console-qa.json` | Actual UI interaction, display acknowledgement and durable first response, with logical backend | Assistant QA, not a participant, not a physical sensor, and not evidence of situation awareness or a scheduler benefit. |
| K1 SDK adapter | `src/transition_autonomy/backends/booster_backend.py`; 13 fake-transport tests in `tests/test_booster_backend.py`; `docs/booster_sdk_evidence.json` | Guarded API, durable intent/receipt, duplicate handling, stale-state and restart behavior tested; SDK 1.6.3 binding inspected | No robot connection or physical motion. SDK inspection did not initialize its client/DDS factory. Physical dispatch defaults disabled. |
| Policy logic | `tests/test_model_scheduler.py`, `tests/test_runtime.py`; paired logical demo receipts | Same candidate set can produce different first actions without optional idle; supported-outcome and independent scoring logic tested | `examples/k1_pointing.json` is a sequential integration task. It cannot establish policy efficacy because it lacks meaningful competing ready operations. |

Hash inspection during this audit: all seven entries in the native runtime manifest and all six entries in the final native ROS receipt match current local files. The two hashes in the 133-test correction receipt also match. In the full-runtime native manifest, paths are relative to `src/transition_autonomy`, not the repository root; `runtime.py` is the substantive mismatch. Its two `._*` entries are archive metadata, not executable modules.

Retain the negative/preliminary evidence: initial quiescence failure, native velocity diagnostics, `pre_source_dwell_validation_receipt.json`, and `ros2-isaac-roundtrip-before-source-dwell.json`. The latter two predate simulation-time dwell and must not substitute for the final receipts.

## 2. Contract that transfers, and measurements that do not

| Contract item | Existing simulation behavior | Required hardware interpretation / acceptance evidence |
|---|---|---|
| Action | `kind="point"`, exactly `parameters={"profile": "point_a"}`; only `point_a`, `point_b`, `home` | Preserve named profiles and command ID/run ID/authority/dependency version fields. Do not accept network-supplied poses, joint arrays, gains, durations or mode changes. Hardware `PointProfile` must be independently commissioned. |
| Task semantics | A→`"A"`, B→`"B"`, home→known `None`; startup/motion→unknown | Assert these exact labels, including distinction between known `None` and unknown. `K1Config.validate()` currently permits subsets and does not enforce label-to-name consistency; configuration acceptance must check all profiles needed by this task and their labels explicitly. |
| Endpoint | URDF joint target plus virtual tool offset `(0,-0.10,0)` on terminal right-arm link | SDK V2 uses a **torso-frame Cartesian request**. Do not copy the simulator joint vector or virtual offset into physical profiles. Verify physical torso/hand conventions, units, axis signs, endpoint calibration and target registration. |
| “Point” meaning | Reference-point reach; no finger-ray or orientation check | Current hardware receipt explicitly says `orientation_verified:false`. Standard K1 has four arm DOF. Success means the commissioned endpoint reached its region, **not** independent 6D pose control, object selection by a ray, or visual recognition. Add a measured directional/target-identification postcondition before claiming those tasks. |
| World vs torso target | Fixed torso makes simulator/world registration constant | If A/B denote world-fixed fixtures, torso motion can invalidate that meaning even when torso-relative endpoint is correct. Require continuously qualified support/base pose plus registered targets, or add measured body-to-world registration. Arm stillness alone does not prove body/target stability. |
| Completion | Joint error + independently read USD reference position + pose-derived velocity + advancing simulation-time dwell | Hardware checks both arms' SDK velocities and requested hand's torso transform over `sampled_at` dwell. Qualify the physical velocity signal, endpoint accuracy and sampling continuity separately; do not inherit Isaac's velocity-estimator assumption or numerical tolerances. |
| Clock/freshness | HTTP converts worker age and full RTT into caller clock; stale/future data fails closed; 0.5 s fact lifetime | Keep SDK runtime in the robot host's absolute monotonic domain. Current SDK sample time is **callback receipt time**, not acquisition time. Establish source age, delay/queue bound, sequence/epoch and packet ordering, or extend the transport; refreshing callback time is not proof that old data is fresh. |
| Sample coherence | One simulation owner reads post-step articulation/reference state | Hardware `read()` snapshots joint data, then queries identity/status/left transform/right transform sequentially. This is not an atomic sample. Commission or represent acquisition intervals and maximum cross-sensor/RPC skew; a single `sampled_at` cannot prove coherence by itself. |
| Stillness | At least 0.3 advancing physics seconds; gap/reset/contradiction resets dwell | Hardware completion dwell uses fresh advancing sample times and resets on gap/regression. `arm()` and idle `observe()` use an instantaneous speed check, not an initial stillness dwell; require the site monitor/commissioning procedure to establish sustained readiness or strengthen this before deployment. |
| Stop | A request first; canceled only after measured hold; canceled target is unknown | Verify vendor endpoint stop scope, response, residual travel, both-arm state and body/support behavior. A planner stop/canceled status is not whole-body emergency stop. Require an independent robot-local intervention/watchdog path. |
| Ambiguous command | Native worker retains per-command history; proxy loss can reconcile the same ID | The verified physical V2 RPC has no project command ID/history. After uncertain delivery, the physical adapter retains ambiguity and blocks motion; it **cannot promise the simulator's successful historical reconciliation**. Physical recovery requires recorded operator/supervisor reconciliation, never automatic replay or ledger deletion. |
| Ownership | One native producer; local journal lock | Local lock is not DDS-wide ownership. A robot-local monitor must exclude competing teleoperation, gestures, vendor apps and raw publishers, and detect ownership loss. Current `site_ready` callable supplies this evidence; its implementation is not provided. |
| Local task facts | Runtime only advances local selections after explicit commits; example begins `local.ready=false` | `local.ready`/`local.selected` are deliberate operator records, not independent fixture sensors. Define attestation vs measured state in the hardware task; do not infer physical readiness from clicking a UI control. |

## 3. Proposed acceptance-test matrix for the parent specification

“Existing” below names demonstrated software evidence; “physical pending” identifies a test/procedure not implemented or run. Numerical physical bounds must be fixed in the commissioning profile before trials, justified by the actual task and instruments. They must not be copied from the simulator merely to obtain a pass.

| ID / stage | Test and required observation | Acceptance condition | Present coverage / missing deliverable |
|---|---|---|---|
| S01 — offline source baseline | Freeze current worker/state/adapter/runtime/task/profile/source, asset/license and SDK artifact hashes | Receipt names exact source/configuration and dependency versions; current local suite passes; current end-to-end native run repeated after runtime fix | Hash manifests exist; final native runtime rerun remains outstanding. |
| S02 — offline contract rejection | Wrong profile, arbitrary joint/pose input, malformed/nonfinite/stale/future fields, changed payload under duplicate ID | No new dispatch; clear rejection/unknown state; unchanged motion counter and retained receipt | Fake backend/protocol tests exist. Do not send malformed commands to a physical robot to test this. |
| S03 — simulated measured lifecycle | Distinct A/B/home starts, identical ID retry, measured terminal pose and dwell, unknown while moving | Exactly one accepted dispatch per ID; endpoint/joint/velocity/dwell gates; exact facts; no timer-only success | Native conformance receipt passes; reuse harness with fresh output names. |
| S04 — source clock / stale data | Pause/repeat/reset timestamps; source/wall gaps; contradictory samples; stale/future observation | No accumulated dwell from rereads; stale target loses validity; unknown/fault blocks further work | Isaac source-clock and Booster fake-sample regressions exist. Actual hardware timing qualification is H03. |
| S05 — transport ambiguity | Native TCP proxy drops accepted response and reads; process restart with unresolved ledger; identical retry | Unknown/disconnected/non-quiescent during outage; zero automatic replay; source-aware recovery | Native loopback loss injection and offline restart tests exist. Do not equate this with physical device history or network-wide exactly-once actuation. |
| S06 — isolated middleware | Actual local Jazzy logical and native roundtrips; status/duplicate queries; wrong clock ID | IDs preserved; terminal measured evidence; unchanged terminal timestamp on duplicate; isolation checked | Existing ROS harness passes. Physical ROS selection is not implemented. |
| H01 — nonactuating physical inventory | Robot serial/edition, firmware, host/architecture/Python, exact SDK wheel/hash, explicit interface/domain/name; model/joint names | Configuration matches the actual robot and approved environment; mismatches disallow arming; no mode changes used to force a pass | SDK/API inspected only. Requires a separately authorized read-only robot session and receipt. |
| H02 — support, ownership and intervention | Observe commissioned vendor stance/support/workcell, all relevant command producers, stop/watchdog state and ownership loss | Positive robot-local evidence with bounded freshness; independent intervention remains effective if UI/runtime/RPC process stalls | `site_ready` is required but monitor, ownership lease and commissioning evidence are absent. A UI Boolean is not acceptance. |
| H03 — timing/coherence at rest first | Timestamp/sequence/epoch and receive traces; source vs receipt delay under realistic load; RPC duration/skew; independent observation of motion/quiet | Predeclared age/skew bounds hold; repeated/out-of-order/delayed samples cannot renew evidence; synchronous calls have a supervised bound | Callback receipt timestamps and sequential transforms are current limitations. No device-source timestamp is exposed in the inspected LowState binding. Need a source-age solution or a justified restricted operating envelope, not an assumed clock. |
| H04 — calibrated endpoint/profile qualification | External or otherwise independently calibrated endpoint reference, torso/world registration, joint layout/sign/units, clearance and reachable region for every profile | Predeclared endpoint accuracy/repeatability and geometry bounds pass from distinct approved starting poses; labels correspond to registered targets; orientation explicitly unverified unless measured | No hardware profiles/default poses or calibration artifact supplied. SDK-computed FK alone is internal conformance, not an independent physical accuracy measurement. |
| H05 — velocity and stillness qualification | Compare native `dq` and position-derived motion against timed observations, including both arms and relevant body/support motion | Chosen velocity estimator and acquisition interval are defensible; sustained entry and terminal stillness established; disagreement yields unknown | Sim native-velocity anomaly is retained; physical velocity is unqualified. Hardware currently uses SDK `dq`; do not silently substitute receive-time differences. |
| H06 — supported named reach | Under separately approved supported conditions, execute each approved profile with one owner; record complete motion and postcondition samples | Correct known target only after fresh coherent endpoint + stillness dwell; no unexplained trajectory/limit/body/clearance violations; all unexpected effects retained | Physical pending. Requires a new bounded commissioning harness/deployment, not either existing simulator validation command. |
| H07 — measured stop | First exercise stop logic offline; then approved reduced-risk mid-motion planner cancellation with independent local stop available | Record request/response, residual travel, time to measured both-arm stillness, body/support state and status; canceled never implies target reached or emergency-stop certification | Native stop and fake hardware transport pass. Physical stop latency/behavior and watchdog guarantees remain unmeasured. |
| H08 — stale/lost feedback and process failure | Offline replay/injected transport faults first; any physical fault trial only after H02/H07 independently work | Local monitor handles loss without depending on failed remote channel; adapter marks unknown/disarms; operator reconciles without replay; retained intent/receipts survive restart | Do not unplug network, kill controllers, change modes or introduce competing writers during free-standing motion merely to reproduce a software test. Those are separate commissioning procedures, not existing accepted scripts. |
| I01 — physical runtime integration | Current runtime plus commissioned adapter; explicit local ready/selection, grant/acknowledgement, A/B/home and return records | Effects observed, completion only with quiescence, local changes explicit, raw decision persisted before scoring, stale/revised snapshot invalidates correctly; journal verifies | Assembly API documented but no hardware CLI/server/harness exists. Sequential success is integration evidence only. |
| R01 — policy experiment readiness | Construct common pool with at least two competing useful ready operations, bidirectional dependencies and measured outcomes; pilot both policies under common safety/return/display rules | Show an actual eligible choice can differ while information/authority/execution gates are equal; preregister progress, resumption and disruption measures; separate trials/participants from developer scripts | Logical choice divergence exists; current K1 sequential task cannot test the proposed scheduling benefit. Human study and physical efficacy remain unperformed. |

For H04/H06, log per-trial starting state, target/profile hash, repeat identity, command/authority ID, measured endpoint trajectory, joint/body state, sample acquisition/receipt times, stop interventions, deviations and failure reasons. Report dispersion, worst deviations and failure count from the **declared commissioning repetitions**; do not turn the three simulator endpoints into a physical success rate. The specification still needs to choose and justify those repetitions and numerical bounds.

## 4. Commands that actually exist (reproduction instructions, not run by this audit)

Local software and protocol checks:

```sh
.venv/bin/python -m pytest -q
.venv/bin/python -m pytest -q tests/test_isaac_backend.py tests/test_booster_backend.py tests/test_ros2_gateway.py tests/test_runtime_review.py
.venv/bin/transition-autonomy demo --output runs/transfer-logical-001
.venv/bin/transition-autonomy audit runs/transfer-logical-001/ordinary.sqlite
```

Native Linux GPU host, in its isolated project copy:

```sh
python3 scripts/fetch_k1_assets.py
bash scripts/run_isaac_worker.sh
```

After native startup, reserve one command producer and use another terminal:

```sh
test ! -e runs/transfer-native-001.json
python3 scripts/validate_isaac.py --output runs/transfer-native-001.json
python3 scripts/validate_runtime_isaac.py --output runs/transfer-runtime-001
PYTHONPATH=src python3 -m transition_autonomy.cli serve --backend isaac --task examples/k1_pointing.json --run-dir runs/transfer-console-001
```

Run these producers sequentially, not together. `validate_runtime_isaac.py` requires a new output directory. **Evidence preservation detail:** `validate_isaac.py` permits overwriting its output JSON and saves frame copies under the fixed `artifacts/isaac/` directory, regardless of `--output`; its default output overwrites the named validation receipt. Archive the current worker frames/ledger/provenance with each new receipt and use a new output path. The CLI's “new run directory” protection does not automatically protect all native artifacts.

Actual ROS2 harness, on the same Jazzy host/kernel as its gateway:

```sh
source /opt/ros/jazzy/setup.bash
python3 deployment/test_ros2_roundtrip.py --backend logical --output runs/transfer-ros-logical-001.json
python3 deployment/test_ros2_roundtrip.py --backend isaac --profile point_a --output runs/transfer-ros-native-001.json
```

The harness sets localhost-only discovery, nonzero domain 173 by default, empty peers and FastDDS itself; it starts and terminates only its own gateway. It defaults to the loopback Isaac worker on port 8767 and has **no endpoint CLI argument**. The standalone gateway does have `--isaac-endpoint`; use `docs/ROS2_GATEWAY.md`'s environment setup if deploying it directly.

**No existing command selects physical hardware.** Neither `transition-autonomy serve --backend booster` nor `ros2_gateway.py --backend hardware` exists. `BoosterK1Backend`, `K1Config`, `PointProfile` and `BoosterSDKTransport` are Python assembly interfaces. The example factory in `docs/HARDWARE_INTEGRATION.md` is documentation, not an executable hardware launcher. A physical harness/monitor/configuration/recovery procedure must be implemented and reviewed separately before H06/I01. `BoosterSDKTransport(permit_connection=True, ...)` creates a real SDK/DDS connection; it is not a harmless import or dry run.

## 5. Acceptance receipt requirements and open items for parent integration

Use one evidence ledger per layer: unit/fake transport, actual logical DDS, native physics, physical commissioning, then participant/policy evaluation. Every new acceptance cohort needs fresh source/configuration/asset/SDK hashes, host/device identity, scope, clock/estimator definitions, observed samples, command IDs/status transitions, failures, interventions and a statement of what was not tested. Preserve actor identity for return decisions: scripted integration, assistant QA and participant data are distinct.

Open deliverables before a real run: (1) robot-local site/ownership/stop monitor, (2) bounded SDK supervision and source-age/coherence solution, (3) registered and calibrated physical profile library with semantic mapping, (4) commissioned stillness/endpoint/stop bounds and repetition protocol, (5) explicit recovery procedure for unqueryable physical dispatch, and (6) a bounded physical assembly/harness. The current documentation correctly leaves all six unclaimed. Keep them as pending acceptance items rather than marking hardware readiness from the simulator's passing receipt.
