# Validation and release boundaries

**Engineering validation, September 23, 2026.** The integration baseline passed **132 tests locally and on the Linux host**. A subsequent manuscript audit corrected completion credit occurring before terminal quiescence; the current local suite passes **133 tests**, including the new regression. The [audit receipt](../artifacts/paper-audit-runtime.json) records that change and source hashes. Remote/native/ROS2 checks below predate this correction and were not repeated. This file separates actual native execution, protocol/unit tests, and untested deployment requirements. No participant study or physical K1 motion occurred.

## What passed

| Layer | Evidence | What it establishes |
|---|---|---|
| Shared model/runtime and adapters | [Local JUnit receipt](../artifacts/unit-tests.xml), [release manifest](../artifacts/release-validation.json) | Exact suite counts, source/configuration hashes, and failure-path regressions; fake transports are labeled in tests |
| Packaged installation | [Wheel demo report](../artifacts/wheel-release-demo/report.json) | A clean environment can install the wheel and run its bundled logical examples |
| Paired logical policies | [Paired report](../artifacts/logical-demo/report.json) | Same public task produces different first choices; both complete required abstract steps; scripted return responses are independently scored |
| Native K1 arm lifecycle | [Run report](../artifacts/isaac/RUN_REPORT.md), [receipt](../artifacts/isaac/validation_receipt.json), [provenance](../artifacts/isaac/runtime_manifest.json) | Actual Isaac import/render, A/B/home motion, measured settling, duplicate suppression, stop, and lost-response reconciliation |
| ROS2 logical transport | [Logical DDS receipt](../artifacts/ros2-logical-roundtrip.json) | Separate gateway/client processes exchange real Jazzy DDS messages and preserve IDs |
| ROS2 native motion | [Native DDS receipt](../artifacts/ros2-isaac-roundtrip.json) | Actual ROS2 request reaches Isaac and returns measured completion with duplicate/status consistency |
| Full task runtime plus Isaac | [Release scenario](../artifacts/runtime-isaac-release/report.json), [journal](../artifacts/runtime-isaac-release/events.jsonl) | A → B → home with three explicit scripted local/return decisions, independent scores, completion state and verified journal |
| Browser operator workflow | [Console QA receipt](../artifacts/operator-console-qa.json) | Actual browser grant/acknowledgement, attention, sequencing, display acknowledgement, first response and pause; this is agent QA, not participant data |

The suite covers lexicographic/envelope counterexamples, hidden-realization isolation, branch-duration/failure support, known/unknown/Boolean semantics, authority and invariants, stale/future evidence, restart ambiguity, duplicate IDs, partial/out-of-support effects, raw-response durability under scorer/storage failures, delayed ingress/timers, display binding, retrospective contradictory observations, HTTP session/origin checks, timer fairness, shutdown, and backend source-clock gaps. Native receipts are separate from those synthetic fault tests.

## Native measurements and limitations

The conformance sequence measured endpoint errors of **4.240 mm (A), 5.784 mm (B), and 2.042 mm (home)**, under the configured 12 mm threshold. Joint errors were 0.01150, 0.01912 and 0.00767 rad, under 0.035 rad. Each completed at least 0.3 s of advancing physics-time settling. These figures describe this reference scene and observer, not physical K1 accuracy or scientific latency benchmarks.

The ROS2 native check measured 4.240 mm endpoint error and 0.300000016 physics seconds of settling. Receipt wall time is an engineering observation on a shared GPU machine, not a controlled performance comparison.

The native image is the host's existing `k1-isaac-sim:5.0.0`, frozen by image hash; its internal version file reports `5.0.0-rc.45+release.23960.184afb15.gl`. The host is Ubuntu 24.04 with ROS2 Jazzy, Python 3.12 and a 24 GiB Quadro RTX 6000. No claim is made about all Isaac versions or vendor-recommended hardware combinations.

The robot trunk and all non-right-arm joints are constrained. Self-collision is disabled, targets are registered profiles, and the reference point is project-defined. There is no balance, walking, obstacle avoidance, grasping, calibrated physical fingertip, vision/occlusion or directional-pointing validation.

The native generalized-velocity output remained inconsistent with the observed pose trajectory. Completion uses an explicitly labeled finite-difference velocity from consecutive measured positions over advancing simulation time. Native velocity is retained in receipts. This establishes bounded measured pose stability, not correctness of native velocity telemetry. The stopping test proves a measured simulator hold under this model, not an emergency-stop or human-safety guarantee.

## Failures retained

Initial tests correctly refused motion because the imported zero-damping articulation did not satisfy readiness. Subsequent diagnostics retained the native-velocity mismatch after adding damping and disabling sleeping. The earlier successful receipt used wall-time dwell; it is retained as `pre_source_dwell_validation_receipt.json` and superseded by the source-time final run. The preliminary ROS receipt is similarly named `ros2-isaac-roundtrip-before-source-dwell.json`.

The final lost-response check deliberately discarded an accepted command's TCP reply and subsequent reads. The adapter reported unknown/disconnected/non-quiescent, then queried the same ID after reconnection without sending another motion. That is controlled transport fault injection, not a network reliability estimate.

## Physical and research gates remaining

The K1 SDK adapter is disabled for physical dispatch by default. Tests use a fake transport; SDK API inspection did not initialize a robot client or DDS connection. Actual serial/firmware/mode, frame calibration, endpoint profiles, support/workcell monitoring, effective ownership across every command channel, source timing, watchdog and stopping behavior still require commissioning. Local file locks and JSON receipts cannot establish those physical conditions. [Hardware integration](HARDWARE_INTEGRATION.md) describes the exact required inputs and assembly interface.

Snapshots are timestamped decision records. The runtime detects reported dependency changes and selected late contradictions, but cannot prove continuous physical stability, unseen sample coverage, multisensor skew or acquisition-interval coherence. Local operator attestation is not an independent fixture sensor. Scoring success is not evidence of understanding, situation awareness or benefit from the scheduler.

The current physical integration task is sequential. A richer no-grasp task with competing useful operations, bidirectional dependencies and calibrated observations is needed before a valid policy comparison or human study. Model critique from the requested 6 Pro conversation improved the implementation but does not certify novelty, hardware readiness or experimental results.

## Reproduction and service lifecycle

Use [README](../README.md) for installation and operator commands, [simulation deployment](SIMULATION_DESIGN.md) for native startup, and [ROS2 deployment](ROS2_GATEWAY.md) for isolated discovery. Use new run directories and preserve journals. Run only one command producer against the Isaac worker at a time.

The authorized server copy is `/home/marnett5/Developer/transition-deictic-implementation`. Validation affected only this project and its named container. The final delivery stops the task's native container and local QA console to release resources; the scripts restart them. Other projects and containers are left untouched. No authentication secret is stored in the project.
