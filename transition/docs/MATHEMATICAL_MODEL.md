# Mathematical model and operational semantics

**Status:** Proposed implementation contract, September 23, 2026. This document refines the current [project proposal](../research/PROJECT_PROPOSAL.md). It specifies a minimal finite-state model for simulation and a shared coordination layer for physical execution. The consequence ordering is an engineered heuristic, not a validated cognitive model. Conditional properties below are not claims of verified hardware safety, general liveness, or improved human performance.

## 1. Scope of the first model

Version 0 has one remote robot executing one task-level atomic skill at a time, a separately instrumented local human task, a finite dependency graph, finite public outcome supports, explicit authority, and fixed assigned return opportunities. The current physical target is a standard Booster K1, with Isaac Sim as the simulation target; first skills are reaching, pointing, presentation, and inspection, not grasping. The local configuration determines a required remote target or inspection, and the observed remote result determines a necessary local placement/selection. The mathematical contract does not presume object transport or gripper capability.

Robot skills have common execution guards and adapter/controller safeguards. Productive local work can overlap a remote skill; resource conflicts still prevent incompatible operations. Atomicity means no ordinary policy-specific preemption; a safety stop or controller failure may always interrupt. This task coordinator does not establish K1 balance, whole-body stability, collision safety, or safe joint limits. Those remain independently validated controller and hardware-adapter obligations.

The intervention changes the order of commonly admissible robot skills during local attention. It does not add authorization, invent task steps, remove execution guards, or choose discretionary idle. A common return protocol temporarily holds scored state in every study condition. The resulting claim is **work conservation outside that common hold and common execution blocks**, not continuous robot motion at every instant.

The minimal runtime consists of a deterministic reducer over an append-only event stream, a shared eligibility function, two rankers, an executor adapter, an immutable acknowledgement reference, a return-lease manager, and an independent scorer. A simulator may know the latent world; the policy receives only the public state derived from observations.

## 2. State, knowledge, and time

Let the unobserved physical or simulated world be \(x_t\). Let \(S_t\) be the coordinator's information state after processing events through sequence number \(n\). They are different objects. A scheduler reads \(S_t\), never \(x_t\) or future scripted events.

A fact record is

\[
f=(id,job,object,value,version,status,t_{obs},t_{valid},source).
\]

`status` is `known`, `unknown`, or `conflict`; `stale` is derived when a known fact no longer meets its configured freshness requirement. Unknown is not false. A known `false` predicate and an unknown predicate have different semantics. `version` is a scalar semantic version, monotonically increasing for changes to that fact's content/status. Refreshing an identical observation updates its observation time, validity, and evidence source without increasing the semantic version. The runtime must still reevaluate time-sensitive guards and leases on freshness updates or expiry; a matching semantic version alone does not prove fresh evidence.

At snapshot time \(t\), evaluate a prerequisite \(p\) in three-valued logic:

\[
\operatorname{eval}(p,S_t)\in\{\top,\bot,?\}.
\]

An execution prerequisite passes only at \(\top\). A sensing action may intentionally read an unknown attribute: that fact need not be a positive execution prerequisite. For example, the target identity and permitted camera/pointing configuration must be established before an inspection, while the inspected attribute may remain unknown.

Use the coordinator's monotonic clock for cue, snapshot, dispatch, response, and deadline comparisons. Preserve device timestamps, observation age, and synchronization uncertainty separately. A total event sequence gives reproducible software ordering; it does not prove that distributed physical events occurred in that order. Set freshness limits conservatively when sensor-clock uncertainty matters.

The state also contains task-step statuses, current resource reservations, authority grants, job commitments, acknowledgement records, actor attention, and active leases. A global state hash is useful for receipts. It is not a reason to reject every action whenever any unrelated fact changes.

## 3. Core data contract

These are semantic fields for a typed implementation; serialization and library choices can vary. Keep spec records immutable and runtime instances distinct.

| Record | Required fields and meaning |
|---|---|
| `Fact` | `fact_id`, `job_id`, optional `object_id`, `value`, `version`, `status`, `observed_at`, optional `valid_until`, `source_event_id`. |
| `Predicate` | `fact_id`, a finite supported operator and operands, freshness requirement, and check phase (`start`, `throughout`, or `completion`). Version 0 must explicitly declare which phases its adapters implement. |
| `ActionSpec` | `action_id`, `job_id`, `task_step_id`, object/role bindings, start preconditions, continuously checked invariants, `read_set`, `write_set`, `resources`, duration bounds, `required_work`, `outcome_support`, authority requirements, predecessor steps, optional due time. |
| `ActionInstance` | Unique `instance_id`, spec/model revision, job/object/task-step identities, bound dependency versions, authority grant/revision, commitment ID, dispatch status and timestamps. |
| `Outcome` | `outcome_id`, semantic effects, optional branch `duration_max`, and `terminal_status` (`succeeded` or `failed`). Omitted branch duration uses the skill bound; specified duration cannot exceed it. Version 0 uses no probabilities. |
| `OutcomeReceipt` | Instance ID, observed outcome or `unclassified`, measured evidence IDs, start/end timestamps, support receipt ID, and any deviation. It is never filled from a policy's prediction. |
| `Acknowledgement` | `ack_id`, operator/job/commitment IDs, decision-schema ID, explicitly acknowledged fact bindings with values/status/versions, event sequence, and acknowledgement time. |
| `AbsenceEpoch` | `absence_id`, operator/job IDs, frozen acknowledgement/commitment/schema IDs, start time, end/revision reason. |
| `PolicyReceipt` | Public-state/evidence reference, epoch ID, common eligible set and exclusions, each candidate's support and rank, selected instance, policy revision, and selection time. |
| `ReturnLease` | Lease/checkpoint IDs, scored dependency closure, snapshot ID, cue/exposure/deadline times, relevant authority revision, status, and invalidation reason. |
| `Decision` | Decision/lease/checkpoint IDs, `execute`, `defer`, or `request_evidence`, target or reason/evidence item, coordinator receipt time, and raw payload before assistance/guards. |
| `ScoreReceipt` | Checkpoint/rule/snapshot IDs, independent admissible-response set, raw decision, correctness/timeout/invalidation result, and separate execution-guard outcome. |

The scorer's truth fields and hidden simulation variables are not present in `PolicyReceipt` inputs. For auditability, support and rank receipts must exist before dispatch, and before the realized outcome is revealed.

## 4. Task graph, resources, and meaningful progress

Represent a job as a finite directed acyclic graph \(\mathcal T=(V,E)\) with task-step actors, predicates, outcome branches, and required outputs. An edge expresses a completion/evidence prerequisite, not merely a preferred order. Branches activate through observed outcomes; a conditional step cannot execute merely because it appears somewhere in the graph.

Each step has a status in `inactive`, `pending`, `ready`, `running`, `succeeded`, `failed`, `cancelled`, or `invalidated`. Define legal status transitions explicitly. A successful step is not automatically ready again. A retry is a new bounded attempt with its own instance ID. Unbounded retries, dynamically generated tasks, and cyclic repair plans are outside the first liveness argument.

A step is meaningful required work when it advances an active prerequisite or required output of the current committed job. No-op, cosmetic, or filler actions cannot keep the proposed policy “busy” while postponing useful work. Inspection is required work when its evidence is needed by an active job dependency, even though its outcome is unknown.

For each resource \(r\), define capacity \(c_r\) and use \(u_{ar}\). Concurrent running actions must satisfy

\[
\sum_{a\in Running(t)}u_{ar}\le c_r.
\]

Examples are the remote arm motion channel, camera, whole-body controller mode, local jig, and a target presentation region. Resources encode physical exclusivity; fact read/write dependencies encode semantic interference. Two actions can use different machines yet conflict through the same committed recipe or object state. Reserving a whole-body controller mode is a software coordination rule, not proof that a reaching motion preserves physical balance.

For timing, \(s_a\) and \(e_a\) are start/end times, with modeled duration bounds \(\underline d_a\le e_a-s_a\le\overline d_a\). Precedence gives \(e_a\le s_b\). A deadline gives \(e_a\le D_a\) only as an assumed/model-checked condition, not a consequence of being work conserving. Human activity and sensor latency can violate planning duration assumptions; record that deviation.

### Minimal common urgency rule

Version 0 should implement an explicit **nonempty urgency-tier filter**, not an unspecified “keep progress acceptable” test. Assign each required step a frozen critical-path priority and, where justified, a latest-start estimate \(L_a\) derived from the task's due times and conservative duration assumptions. Let \(E_t\) be the nonempty ready/authorized/resource-compatible set below, and let \(h_t=\max_{b\in E_t}\overline d_b\). Mark a candidate urgent when \(L_a-t\le h_t\). If any candidate is urgent, set \(G_t\) to the urgent candidates; otherwise set \(G_t=E_t\). No configured due time means no deadline-derived urgency for that step.

This filter never empties a nonempty ready set. It is identical across policies and uses no consequence costs. Its purpose is a bounded first implementation of shared urgency, not a schedulability proof. Two incompatible urgent tasks can both miss deadlines despite this rule. A stricter resource-constrained feasibility solver is a later common component and must be tested independently.

For true multi-robot execution, selecting independent minima is insufficient: actions can compete for the same resource or have joint effects. One must select a feasible action set and evaluate its joint outcome support. Version 0 reserves one remote action at a time and models local work separately, so it does not silently claim a multi-resource optimal scheduler.

## 5. Shared eligibility and dependency-level validity

For ready action \(a\), define common eligibility

\[
a\in E_t\iff Ready(a,S_t)\land Required(a,S_t)\land
Authorized(a,S_t)\land Preconditions(a,S_t)=\top\land
ResourcesAvailable(a,S_t)\land ModelAvailable(a,S_t)\land
CommitmentCompatible(a,S_t)\land LeaseCompatible(a,S_t).
\]

Every predicate is implemented identically for both policies. Ranking cannot promote an action into \(E_t\). A preauthorized change of remote pointing/inspection target must preserve the local committed requirement. If an observed result requires a different local placement or configuration, dependent steps become invalid under the shared task engine until an explicit commitment revision. Autonomy is not permitted to silently make already completed local work incompatible; switching targets does not imply physically moving them.

An instance captures versions of its named dependencies, object binding, authority grant, and commitment. At dispatch and declared subsequent check phases, compare and semantically revalidate those dependencies. If a relevant version changed but the intended referent, prerequisite, and authority remain valid, refresh the binding with a receipt. If they changed meaning or became unknown/stale, apply the common rejection, cancellation, or recovery rule. An unrelated object's version does not invalidate the instance.

The initial selector also requires a finite authority expiry to cover `now + duration_max`; an already expired grant is excluded. The configured `duration_max` must be a **whole-operation allowance**: dispatch and transport, execution, endpoint verification/dwell, and a declared termination/reconciliation margin. A nominal trajectory duration alone is insufficient. The compact schema currently stores this aggregate bound rather than validating a breakdown. Expiration exactly at that aggregate bound is accepted by the selector; this is arithmetic eligibility, not measured timing headroom. Document and commission the component bounds and clock uncertainty before hardware use.

Check authority again at the last practical dispatch boundary; time spent journaling, queuing, or transporting cannot be ignored by reusing an old timestamp. `Runtime(clock=...)` accepts a monotonic run clock sharing the same origin as operation timestamps. It refreshes after backend observation and after durable intent persistence, repeating common eligibility and observation-freshness checks before send. If the final check fails, a `dispatch_withheld` event marks the durable command canceled without applying its in-flight facts or calling the backend; that command ID cannot be reused to try again. With no clock configured, synthetic runs retain their explicit supplied time. This is a practical final check, not a hard real-time bound on the remaining instructions, transport, or stop completion. Revocation and overruns follow the common stop-and-reconcile path, and no retry is implied. This time-horizon check avoids knowingly choosing an operation whose declared allowance outlasts its grant; modeled bounds do not enforce actual completion, stopping, or reconciliation within that allowance.

Use a single atomic check-and-reserve/dispatch decision in the coordinator to avoid a software check/use race. This cannot freeze the physical world. Runtime monitoring and the executor adapter must enforce declared `throughout` conditions or stop/recover when they fail. If an adapter only checks at start, its guarantees must say so.

In the compact API, `Skill.preconditions` are start checks, while `Skill.invariants` must pass at start and remain freshly known and true throughout execution. Both contribute named read dependencies and use the skill's freshness constraint. The selector checks invariants initially; the runtime owns continuous rechecking and the common stop/recovery response. No sampled software monitor proves that a physical invariant held between samples. Sensor coverage, update bounds, controller response, and physical operating limits require separate validation.

Separate three outcomes:

1. **Wrong at issuance:** the raw operator decision was outside the correct set for its held snapshot.
2. **Valid but overtaken:** correct when issued, then a relevant dependency or authority changed before execution.
3. **Guard result:** accepted, refreshed, or rejected according to the current execution contract.

A rejected command is not automatically a human mistake. Idempotent instance processing prevents duplicate software dispatch; exactly-once physical effects require executor acknowledgement and reconciliation after transport failures. Do not claim physical exactly-once execution from an identifier alone.

## 6. Fixed acknowledgement reference and consequence counts

At departure, freeze an absence reference

\[
A_e=(ack\_id,commitment\_id,\theta,\{k\mapsto \bar z_k\}_{k\in K_\theta}).
\]

\(\theta\) is a versioned decision schema; \(K_\theta\) is a fixed set of canonical semantic slots relevant to the assigned return decision and committed task. Example slots are the bound component role, required destination, completion state of a named step, and the availability of evidence needed by a named dependency. They are not raw sensor channels or event-list rows.

Let \(z_k(S)\) project public state onto slot \(k\). A fixed classifier assigns the unresolved difference between \(z_k(S)\) and its acknowledged reference to `none`, `planned_context_advance`, `contingent_revision`, or `evidence_change`. The implemented category key for planned context advance remains `advance`. Each slot contributes at most once, with a declared precedence when several labels apply. A suitable first precedence is evidence change, then contingent revision, then planned context advance. The manifest must specify examples and boundary cases, not infer a category from the participant's eventual error.

\[
C_j(S,A_e)=\sum_{k\in K_\theta}\mathbf 1[\gamma_\theta(k,S,A_e)=j],\qquad
C=(C_{evidence},C_{revision},C_{advance}).
\]

Planned progress that leaves the relevant decision unchanged contributes zero. Completing a pointing/inspection step that makes its repetition obsolete can contribute planned context advance. This terminology describes task structure, not a prediction about what the operator expects or a probabilistic expected value. A permitted selection of another compatible target can contribute a contingent revision. Missing, conflicting, or expired evidence can contribute an evidence change even if no packet announces it. Freshness must therefore be evaluated at the projected completion time as well as at selection where relevant.

The scheduler cannot replace \(K_\theta\), rename the goal, select an easier checkpoint, or reset \(A_e\) to lower its count. An explicit human revision closes the old epoch and creates a new immutable reference with a recorded reason. It never rewrites prior receipts. Rendering a summary, obtaining gaze, progressing to another step, or regaining the old physical arrangement does not acknowledge anything automatically.

### Count limitations and counterexamples

- Repeating the same fact in ten event rows is one semantic difference, not ten. Alias normalization and fixed slot keys prevent inflation.
- Changing a contextual value A→B→A can restore its current value while leaving an important episode in the history. Its difference count may become zero; the count is not an event-memory or surprise measure. A revoked/consumed authority grant, invalidated dependency permit, or old instance token never becomes valid merely because the contextual value returned to A. Those validity rules remain in the common authority/dependency engine. If the episode matters to the next decision, give that obligation an explicit persistent slot.
- A single critical change may matter more than several minor changes. Lexicographic category order is a chosen heuristic, not a universal severity scale.
- An unknown quality becoming known can enable a new response. The scheme can count planned context advance while reducing missing evidence; it must not treat every sensor acquisition as harmful merely because a value changed.
- Exchanging one revised fact for another can preserve the numeric vector while changing their identities. Keep the slot-level record; aggregate counts alone are inadequate for explaining decisions.
- Acknowledgement is a protocol record, not proof of comprehension. A lower vector therefore does not establish lower cognitive load or better situation awareness.

Freeze the schema and ordering before efficacy collection. Pilot trace checks validate implementation and interpretability; human benefits require the planned study.

## 7. Finite support, time, and the two rankers

The public outcome model returns a nonempty finite **timed** support

\[
F(S_t,a)=\{(S^{(o)}_{t+\overline d_o},\overline d_o,\tau_o):o\in O_a\},
\quad 0<\overline d_o\le\overline d_a,
\quad \tau_o\in\{succeeded,failed\}.
\]

Each branch preserves the association between its semantic effects, terminal status, and duration bound. If no branch bound is supplied, use the skill bound. Evaluate that branch's freshness and consequence count at its own modeled completion time. Combining a quick revision branch's effects with a different slow branch's duration can invent an unsupported stale-plus-revision state. A single state predicted at “now” is also insufficient when evidence expires before completion.

The compact effect model timestamps declared updates at branch completion. If a sensor reads earlier during a skill, model an appropriately conservative evidence lifetime or extend the timing model; do not claim arbitrary observation-time correlations are already represented. Finite support remains a declared abstraction, not all possible physical outcomes.

An inspection with unknown quality but known pass/fail/inconclusive branches has available support. An inspection whose effect semantics are unspecified has no model and is excluded by both policies. Retain explicitly modeled failure or unknown-result branches when ranking. An inconclusive inspection can still be a successfully executed sensing action with unknown evidence; a failed motion has `terminal_status='failed'`. Only a succeeded branch adds the task step to `State.completed`. A failed branch may update observed facts, but never pretends the required step completed or permits an automatic retry.

Support may not omit a plausible modeled branch merely because its consequence is inconvenient. Physical termination outside the model is an unmodeled failure and common reconciliation case, not evidence that the finite-support objective was robust to all physics. If failure should change the burden vector, the scenario must include an explicit canonical failure/evidence slot: the ranker does not invent an unreviewed universal failure penalty from the terminal label alone.

Version 0 assigns no probabilities. A three-element support does not mean equal probabilities, and frequency in a scripted benchmark does not establish a calibrated real-world distribution. A probabilistic extension would require separately justified estimates and a changed objective.

For the one-skill horizon, define each supported branch vector \(c_o=C(S^{(o)}_{t+\overline d_o},A_e)\). The **primary aggregation** selects the lexicographically worst supported vector:

\[
b^{lex}(a)=\operatorname{max}_{lex}\{c_o:o\in O_a\}.
\]

This vector is one of the supported branch vectors. `TaskSpec.aggregation='lex_worst'` is the default. Its conservatism is relative only to the declared support and category ordering; it is neither a physical-risk guarantee nor a model of human cognition. The explicit comparison option `upper_envelope` instead computes \(b^{env}_j(a)=\max_o c_{o,j}\). That componentwise envelope can combine extrema from different outcomes and need not represent one possible successor. Neither aggregation is an expected cost or a most-likely outcome.

For example, support vectors `{(1,0,0), (0,10,0)}` give `lex_worst=(1,0,0)` and `upper_envelope=(1,10,0)`. Against another action with vector `(1,5,0)`, minimizing lexicographic worst cost chooses the first action, while minimizing its upper envelope chooses the second. These are different objectives and must be named, configured, and logged distinctly. Do not silently swap them under a general “worst case” label.

Let \(r_0(a)=(priority_a,\widehat d_a,action\_id)\), with lower values preferred and frozen priority/duration conventions. Both cells form the same \(G_t\) from common eligibility and urgency. Then

\[
\pi_0(S_t)=\operatorname{argmin}_{a\in G_t}r_0(a),
\]

\[
\pi_1(S_t,A_e)=\operatorname{argmin}_{a\in G_t}(b_{evidence}(a),b_{revision}(a),b_{advance}(a),r_0(a))
\]

during local attention. During remote attention use \(\pi_0\). With equal vectors or one candidate, the ordinary order resolves the selection. If \(G_t\ne\varnothing\), an action is selected regardless of the vector's magnitude. There is no budget threshold or discretionary-wait branch.

Recompute at skill boundaries and on relevant acknowledgement, attention, authority, dependency, observation, or expiry events. Events during execution affect the next selection and shared monitors; they do not grant policy-specific preemption. Rank/eligibility data affected by an event must be recomputed before dispatch.

## 8. What work conservation does and does not establish

Given identical public input and common functions, \(E_t\) and \(G_t\) are identical across policies at that state. Because the urgency filter preserves at least one action when \(E_t\) is nonempty and ranking is total, either ranker selects an action. It never repairs an empty eligible set by admitting unauthorized, conflicting, or unsupported work; all blocked ready work retains its rejection reason. The precise claim is **work-conserving among commonly admissible skills, outside common holds**. Real trajectories can reach different states, so their later eligible sets need not match.

Finite completion is conditional. In a finite active acyclic task graph with finite successful skill durations, finite return holds, bounded retries, eventual required observations/authorizations, no endless invalidation, and a required ready action whenever work remains, repeated work-conserving selection eventually exhausts required work. Those assumptions do substantial work. This document does not establish them for hardware.

Counterexamples include infinite arrivals of low-burden work starving an inspection, repeated failed retries, an authority grant that never returns, a missing predecessor that never completes, perpetual stale measurements, and a local actor holding a needed resource indefinitely. Avoid infinite arrivals and unbounded retries in version 0; expose other blockers instead of labelling them policy failure or success. Common watchdogs may identify lack of progress but cannot create missing authorization or evidence.

Work conservation does not imply minimal makespan, deadlines, information acquisition, resource fairness, or human benefit. Sequencing may delay a necessary inspection while executing other required work. Measure discovery latency and residual unknowns as well as busy time. A policy cannot claim improvement merely because its robot never appears idle.

## 9. Correct-decision set independent of the ranker

For each assigned checkpoint \(q\), define a separately authored, versioned response rule

\[
R_q(Z_q)\subseteq\{execute(a),defer(reason),request(evidence)\},
\]

where \(Z_q\) is the held scoring snapshot and independent scenario truth needed to validate it. The scorer does not call \(\pi_1\), compare the human with the selected robot action, or derive correctness from a lower consequence vector. Several responses may be correct. A valid authorized robot operation is not necessarily the correct human response at this checkpoint.

Rules use the task objective, commitments, sufficient available evidence, and justified reasons to wait or investigate. Hidden truth can validate the apparatus and outcome, but cannot make an unsupported guess the required human response: if necessary evidence is unavailable, requesting it can be correct even when a private simulator variable already contains the answer.

The first deliberate raw response is recorded before command assistance or guards. Inspection/hovering/history viewing and acknowledgement are not response choices. A reasoned deferral is correct only when the task justifies it; merely not choosing yet does not make waiting correct. Timeout, correct non-action, incorrect action, valid action overtaken later, and technical invalidation remain distinct.

Recording order alone is insufficient. The runtime commits `raw_response`, a first-response `response_claimed` event containing detached scoring rules and snapshot plus their hash, and lease closure together **before** invoking the independent scorer. A second operation records `semantic_score` or `scoring_error`. A process interruption between them leaves a captured response with a pending score; it must be retained and audited, never replaced by a second first choice. Capture-transaction failure is not acknowledged as persistence and leaves the lease open. Scoring does not dispatch, acknowledge a new commitment, or grant authority. Commands after a response still pass current execution guards.

In the journal, `return_closed` records `outcome='response_captured'` and pending scoring for this path. Analysis joins that event to `semantic_score` or `scoring_error` by decision ID and then applies any `snapshot_coherence_flag`; it must not mistake pending capture for correct, incorrect, or missing participation. The in-memory/public lease result is updated after the separate score record commits. A scoring exception is a technical scoring outcome with no attributed human error. Restart with an existing journal still requires audit/reconciliation; automated pending-score recovery is not implemented.

The primary denominator is the predetermined assigned opportunities. Achieved progress, return state, and observed changes are policy mediators, not baseline covariates. Score the first response within the common cue deadline; do not remove checkpoints because the policy made the correct response “wait” or “no further work.”

## 10. Operator return leases and stable scoring snapshots

A return lease freezes a **decision record** and establishes a temporary protocol over semantic facts and affected physical resources. It is not proof that the physical world remained frozen. For checkpoint \(q\), compute the scored read set \(K_q\) and its dependency closure \(K_q^*\): every fact whose change could alter the response set, target binding, evidence validity, or authority at that checkpoint.

At the scheduled cue \(t_c\), start the primary clock immediately and establish a reservation intent for \(K_q^*\). Prevent new actions with possible writes intersecting that set or with incompatible physical-resource effects. Existing atomic skills finish or follow their common failure procedure. The local actor must suspend relevant manipulations and the apparatus must verify the committed local configuration. A software bit cannot guarantee that a human stopped touching a fixture.

Acquire the snapshot only after relevant in-flight actions drain, dependencies are known and fresh enough for the window, and authority and resource bindings are confirmed. Capture versions and evidence IDs; expose the corresponding scene at \(t_e\). A fact's normal expiry before the deadline must be handled explicitly: use a shorter valid window, a live validating observation that preserves semantic content, or invalidate when freshness is lost. Do not retain a `known` flag merely because writes are locked.

During the lease, allow another operation only when its modeled read/write and physical dependencies are verified not to affect \(K_q^*\). The minimal implementation can pause all task-level remote operations during this common hold, while low-level stabilization and safety monitoring continue. All conditions use the same rule.

The first of a deliberate response, the absolute deadline \(t_c+\Delta_q\), or invalidation closes the scoring window. A response is eligible for scoring only when bound to the active lease/snapshot and received before its deadline. A late response cannot cause execution under an expired snapshot; require fresh validation or a new deliberate decision according to the common protocol.

Use a trusted server ingress timestamp for the complete authenticated, lease-bound request, then serialize ingress and deadline handling with an explicit tie rule. The runtime requires `exposed_at <= ingress_at < deadline` and `ingress_at <= processed_at`; human mode additionally requires a recorded post-render acknowledgement no later than response ingress. The acknowledgement's timestamp must itself fall within the exposed interval and cannot be in the future. It measures reported rendering received by the server, not gaze or understanding. Cue-to-ingress is an operational response measure; publication-to-render acknowledgement and ingress-to-processing delay remain separate. Runtime timestamp checks alone cannot repair an upstream queue that processes a timeout ahead of an already admitted response.

`tick(now, deadline_at=admitted_at)` permits a serialized controller to apply a timer's admission timestamp only to the deadline cutoff while checking authority, evidence, and faults at actual processing time. A timer admitted strictly before the deadline must not expire the lease merely because it was processed late. A timely display acknowledgement and response may likewise be processed after the deadline without becoming nonresponse. Admission exactly at the deadline is ineligible. Defaults use processing time, preserving explicit-time synthetic behavior. The caller remains responsible for fair, deterministic FIFO admission; timestamps alone do not enforce ordering or validate a client-provided clock.

Unexpected physical change, authority revocation, evidence conflict/expiry, or an unverifiable fixture state invalidates the lease. Stop the scoring window and log a technical invalidation. Safety and authority revocation always override experimental holds. The all-started endpoint records that resumption was not achieved, while human-error counts do not falsely blame the participant. Preserve the valid-snapshot sensitivity analysis separately.

Late telemetry can also challenge an already closed snapshot. The implementation checks even superseded observations against each lease: a contradictory value/status with acquisition point timestamp inside the exposed-to-response interval creates an auditable `snapshot_coherence_flag`, deduplicated by evidence hash. For an open lease it closes as technical failure. For a captured response it preserves the raw response and original rubric result, marks coherence uncertain, and withholds human-error attribution. A later change with a timestamp strictly after response is an ordinary subsequent state change. Analysis must retain and inspect flags rather than silently rewriting the original semantic score.

This is a bounded detector. `FactUpdate` currently supplies a point timestamp, not an acquisition interval, source sequence/epoch, or clock-uncertainty bound. The runtime therefore cannot establish maximum cross-sensor skew, detect all missed source samples, prove physical stability between samples, or identify every delayed contradiction whose uncertain acquisition interval overlapped the decision. The overall observation heartbeat age and fact validity checks are insufficient for those claims. Before physical snapshot interpretation, specify source observation age, acquisition intervals, clock uncertainty, maximum source-sample gap, and invalidation-detection latency, then retain coverage failures as technical uncertainty. Local quiescence attestation establishes only the stated local protocol; it is not independent verification of configuration correctness.

Record \(t_c\), atomic-boundary time, \(t_e\), first response time, close time, scored versions, authority revision, and release/invalidation reason. Primary time is cue-to-decision, so delaying exposure cannot improve the score. Boundary timeout is distinct from a participant who had the scene but did not respond. The lease ends by response, deadline, cancellation, or invalidation; it must never remain held indefinitely after client disconnect.

## 11. Event semantics and minimum implementation checks

Use a reducer `S[n+1] = reduce(S[n], event[n+1])` with validated event transitions. Replaying an identical ordered stream and configuration must reproduce public state, rank receipts, and scores. A simulation seed belongs to the environment adapter; policy inputs contain only observations already emitted. Keep the scoring oracle in a separate module/interface.

Before integrating hardware, verify these properties with small synthetic traces:

| Check | Required behavior |
|---|---|
| Same state, different policy | Identical common candidate sets and exclusions; only ranking can differ. |
| Nonempty candidate set | Both rankers choose an action; proposed policy never chooses optional idle. |
| Unknown world quality, known support | Inspection remains possible if physical prerequisites and authority hold. |
| Unknown model | Common exclusion with a named reason. |
| Hidden outcome changed | Pre-observation support/rank unchanged when public observations are identical. |
| Duplicate or aliased events | Fixed slot differences are not inflated. |
| Acknowledgement/commitment changes | Explicit new reference epoch; old receipts unchanged. |
| Unrelated fact revision | No spurious command rejection or lease invalidation. |
| Relevant stale/unknown dependency | Common revalidation/block; no assumption that unknown means true. |
| Lease race or expiry | No stale snapshot execution; raw decision and execution outcome remain separate. |
| Correct wait/evidence response | Scored correct without requiring robot motion. |
| Scorer exception or post-capture interruption | Raw response, immutable inputs, and first-response claim survive; score is error or pending. |
| Capture storage failure | No partial raw/claim/closure commit, no score, no persistence acknowledgement. |
| Response/render timestamp ordering | No response before snapshot or required render receipt, no future receipt, no late eligibility. |
| Delayed contradiction during a prior decision | Original participation and score retained, coherence flag added, human-error attribution withheld. |
| Cyclic or unbounded task input | Rejected from the bounded profile or marked outside its completion assumptions. |
| Outcome outside support | Model violation retained, never silently rewritten as a predicted branch. |

These checks substantiate software semantics, not physical correctness. Hardware acceptance separately requires measured skill behavior, freshness and stop response, fixture/lease stability, and adapter failure recovery. Simulation and physical runs should share contracts and receipts while keeping their evidence claims distinct.

## 12. Claims that remain open

The model supports an auditable intervention and an unambiguous scoring protocol under stated assumptions. It does not establish that the selected categories capture human resumption burden, that worst-supported ranking is preferable to an expected-cost policy, that the urgency heuristic protects all deadlines, or that the physical controller satisfies every modeled invariant. Phase 0, engineering validation, and the planned same-summary sequencing comparison address different portions of that gap; none substitutes for the others.

## 13. Implemented core profile and deliberate limitations

The initial [model module](../src/transition_autonomy/model.py) and [scheduler](../src/transition_autonomy/scheduler.py) implement the agreed compact API: `Fact`, `State`, `Outcome`, `Skill`, `TaskSpec`, `Commitment`, `Authority`, and `select(...) -> Selection`. `TaskSpec.from_dict`, `load_task`/`load_task_spec`, `task.initial_state`, and pure `apply_outcome` support deterministic logical fixtures. Applying an outcome in this module updates declared logical facts; it is not independent confirmation that physical effects happened.

- `Skill.id` is the one-shot task-step identifier; `TaskSpec.id` is the job/template context. Runtime command envelopes carry instance and authority identity. Reusable/cyclic skills are outside the finite profile.
- `decision_fields` maps each fixed canonical fact key to one of `evidence`, `revision`, or `advance` (planned context advance). This implements a **static category assignment**, a simpler special case of the transition-sensitive classifier above. The code does not infer severity, semantic aliases, or decision relevance. Scenario authors must make those keys and assignments explicit.
- `Commitment.values` accepts raw JSON values or explicit `{value, status}` snapshots, detaches and deep-freezes them, and provides `to_dict()` for serialization. Missing fixed reference fields cause an error instead of an implicit reference reset.
- Effects and initial facts accept raw JSON values, or `{value, status, valid_for}` descriptors. `status` distinguishes the descriptor form. An unknown observation must be explicitly marked; a known null value is not automatically unknown. Semantic version changes compare JSON types, so `true` and `1` are different.
- `Skill.max_age` applies to start preconditions and invariants. An unknown field in `read_set` can still be sensed when it is not a required known precondition/invariant. Consequence projection uses `Fact.valid_until` at each branch's own completion bound; specify observation lifetimes when expiry affects the decision.
- `TaskSpec.aggregation` defaults to `lex_worst`; `upper_envelope` is an explicit comparison. `Outcome.duration_max` is optional and bounded by the skill duration. `Outcome.terminal_status` defaults to `succeeded` (input alias `success` is normalized); `failed` remains in ranking support but does not mark the task step complete.
- The common urgency filter currently uses `deadline - duration_max` as the skill's latest-start estimate. It does not calculate a complete downstream resource-constrained schedule. `priority` is supplied by the task configuration, not learned by either policy.
- The loader validates finite acyclic predecessor structure, effect write footprints, configured fact identities, and that every authored rubric `when` dependency is in `scored_facts`. It does not infer all physical/transitive dependencies; those remain explicit configuration and adapter obligations.

The accompanying [core tests](../tests/test_model_scheduler.py) exercise candidate parity, work conservation, distinct aggregation objectives, branch-duration correlations, modeled failures, invariant entry checks, uncertainty and freshness, immutable references, authority horizon, footprints, urgency, and schema validation. They establish these bounded software behaviors only. Continuous invariant enforcement, runtime leases, event journals, and physical adapters have separate tests and evidence.
