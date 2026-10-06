# Hardware specification — test-workflow review

Reviewed `docs/HARDWARE_IMPLEMENTATION_SPEC.md` revision 0.1 against the current launcher, native/ROS validators, hardware adapter, tests and retained receipts. Read-only review; no server/robot execution and no edits to the specification. Findings below exclude the parent's incoming 6 Pro changes for transition/start-region qualification, blocked-call stopping, late success after abort, replayed telemetry, disk-failure shutdown and actual device enforcement.

## Material findings

### R1 — Stage-specific readiness is needed to break the commissioning bootstrap cycle (high)

**Where:** Sections 3, 8.1–8.2, 9 and 13; particularly “`null` required values must fail validation before connection/arming,” H01's requirement for complete commissioning records, and S4's prerequisite that H01–H11/site requirements have already passed.

**Problem:** The final manifest requires calibration, observed timing/stop bounds, profile commissioning records and watchdog evidence that the read-only and bounded commissioning stages are intended to obtain. A strict implementation of the current universal preflight cannot enter those stages; a permissive implementation may quietly weaken the operational gate. The same issue exists before first diagnostic connection if missing final profile/calibration evidence fails all connection validation.

**Required resolution:** Define distinct, explicit authorization records and prerequisite sets:

- **Observation permit:** identity/network binding, approved diagnostic channels and protected site access; the process cannot expose movement/mode-changing methods. Missing final profile-performance records must not be misrepresented as established evidence.
- **Bounded commissioning permit:** separate approval for a listed candidate profile/transition, starting envelope, trial limit, expiration, supervision, independent support/stop prerequisites and allowed evidence collection. It is more narrowly scoped than operational use and cannot start an automatic framework sequence.
- **Operational release:** issued only after the commissioned measurements and applicable tests pass; links to the evidence that the commissioning permit generated.

This is a staged gate, not `--force` or an exception hidden in a Boolean. For any prerequisite that genuinely cannot be established without an independently approved vendor procedure, say so and remain observation-only until that procedure supplies the evidence.

**Acceptance test / artifacts:** Test missing/expired/mismatched records at each stage; demonstrate that an observation permit cannot reach `move`, a commissioning permit cannot execute an unlisted profile or exceed its attempt allowance, and neither permit can authorize S5. Preserve the permit ID/scope, candidate digest, individual prerequisites, trial outcome and resulting release decision separately. On failure, do not turn the incomplete commissioning receipt into operational readiness.

### R2 — “Ten valid commanded trials” does not specify a bounded or unbiased trial plan (medium)

**Where:** Section 13, paragraph following the stage table.

**Problem:** A “fixed number such as ten valid commanded trials” conflicts with counting failed, interrupted and invalid-observation attempts in the denominator. An implementation can keep trying until ten successes or technically valid traces exist, making the exposure count and stopping rule variable. Retaining the extra attempts helps reporting but does not define the authorized trial budget or prevent collecting until a desired result appears.

**Required resolution:** State a fixed **attempted-trial** plan per approved transition/start-state stratum, with a maximum authorized number of attempts and explicit stop rules. If a minimum usable metrology count is also needed, specify it separately with a predeclared replacement policy/cap; an invalid measurement must not disappear from the physical-attempt record. Distinguish a pre-dispatch refusal from an attempt for which actuation may have begun. Any unexpected motion, envelope violation or unresolved command must stop the campaign irrespective of the remaining trial quota.

Ten can remain an explicitly illustrative exploratory sample size, not a reliability claim. Allocate repetitions to the qualified transitions/start regions, including the other-arm/body configuration relevant to the trial, rather than merely to three endpoint labels. The parent's new transition-specific requirements should provide the exact strata.

**Acceptance test / artifacts:** The planned attempt table is frozen before S4. The harness refuses attempt N+1 without a new approval; reports all admitted, refused, failed, aborted, observation-invalid and completed attempts; applies declared replacement rules without rewriting original trial IDs; and records why the campaign was stopped or released. Summary statistics must identify their denominator and any conditional subset.

### R3 — Spatial measurement uncertainty is recorded but absent from the pass predicate (high)

**Where:** Sections 6.1–6.3, S2/S4 and T04/T06/T14.

**Problem:** Calibration uncertainty and acquisition intervals are required records, but terminal acceptance remains `endpoint_error <= position_tolerance`. A measured error just inside the tolerance can pass even when frame/calibration/metrology uncertainty places the physical endpoint outside it. For example, an estimate of 11 mm with a justified 5 mm uncertainty cannot establish a 12 mm error bound. The registration-valid Boolean does not resolve this numerical acceptance ambiguity.

**Required resolution:** Define what the tolerance certifies and an uncertainty-aware decision rule. One suitable bounded-error form is `endpoint_error_upper = estimated_error + validated_uncertainty_bound`, requiring `endpoint_error_upper <= tolerance`; a set-based equivalent is also valid. Specify which registration, kinematic/metrology and temporal-alignment terms are included and avoid double-counting correlated terms. If using probabilistic metrology, specify its coverage level and operational scope rather than calling the estimate an absolute bound. Unknown or excessive uncertainty must produce inconclusive/unknown evidence, not a certified endpoint success. Apply an equally explicit rule to any derived velocity threshold if that estimator is commissioned.

Do not copy the simulator's 12 mm/0.04 rad/s values or assume its position-difference observer solves physical metrology. The existing text already rejects that transfer; the missing piece is the executable acceptance predicate.

**Acceptance test / artifacts:** Include boundary cases below, exactly at and above the tolerance after uncertainty propagation; missing calibration, invalid registration and excessive acquisition skew must not pass. Each result records raw estimate, uncertainty components/model/version, conservative or coverage-qualified result, chosen threshold and final disposition. Retain nominally close but inconclusive trials as such.

### R4 — Native reproduction needs separate foreground-worker and validation terminals, plus fresh evidence binding (medium)

**Where:** Section 12's Isaac command block and S1's exact-release-candidate requirement.

**Problem:** `bash scripts/run_isaac_worker.sh` executes a foreground `docker run`; the following validation commands in the same pasted shell block do not execute while the worker is running. Stopping it to advance the shell leaves no worker for validation. In addition, `validate_isaac.py` can overwrite its output receipt and fixed `artifacts/isaac/{point_a,point_b,home}.png`; it does not generate a fresh `runtime_manifest.json`. The current archive warning is correct, but the instructions do not yet provide a concrete per-attempt capture boundary for S1's source/configuration claim.

**Required resolution:** Show **terminal A** starting the worker and **terminal B**, after readiness, running one validator/producer at a time. Give the native validator a new `--output` file and explicitly archive that attempt's worker provenance/frames/ledger before another attempt overwrites fixed paths. Record the running worker boot, source/configuration/model/image identifiers for the new cohort rather than reusing the historical manifest. `validate_runtime_isaac.py --output <new-directory>` already creates a fresh directory and records current core/task hashes; its record does not replace the native worker/configuration provenance.

**Acceptance test / artifacts:** A clean operator should be able to follow the two-terminal instructions without backgrounding guesswork, obtain fresh measured readiness, run the current native/runtime checks sequentially, and identify the matching worker boot and source/configuration in each result bundle. A rerun must preserve the previous attempt. Keep the current-native rerun requirement after the 133rd-test runtime correction; historical native success is not evidence that the changed runtime was rerun.

## Checks that do not require a new finding

- Proposed `k1_preflight.py`, `k1_monitor.py`, `k1_observer.py`, `k1_runtime.py` and `validate_k1_hardware.py` are clearly labeled unimplemented work items. Section 12 correctly avoids presenting them as current commands.
- The shown current CLI, audit, Isaac and logical ROS harness entry points/arguments exist. There is correctly no physical `--backend booster` option.
- The specimen assembly uses existing classes, a persistent device-ledger parameter, absolute monotonic time and an explicit disarmed guard. It is correctly described as illustrative rather than a production launcher.
- Fixed-root/right-arm scope, disabled simulator self-collision, uncalibrated virtual tool offset, four-DOF orientation limits, callback receipt timing, non-atomic physical transforms, and lack of a physical command-history query are distinguished adequately. Endpoint arrival is not asserted to verify finger-ray pointing or visual object identity.
- Sequential A/B/home is correctly separated from a meaningful policy comparison. Scripted or operator integration answers are not presented as participant-effectiveness evidence.

No additional finding is made here for the failure paths the parent is already adding from 6 Pro. Their eventual tests should preserve independent test-assertion outcome, task-command outcome, measured motion state and deployment-readiness disposition; passing an expected-fault test must not silently imply that a real device is ready for another motion.
