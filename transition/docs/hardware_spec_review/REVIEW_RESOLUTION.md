# Hardware specification review and resolution

September 23, 2026. Reviewed artifact: [HARDWARE_IMPLEMENTATION_SPEC.md](../HARDWARE_IMPLEMENTATION_SPEC.md). This is a documentation and implementation-gap review. No physical robot or remote simulator was contacted for this task, and no production source was changed. One reviewer reproduced the repeated-stop issue with fake transport only.

## Review inputs

- [Adapter/code audit](adapter_audit.md): exact existing interfaces, guards and hardware gaps.
- [Simulation/test transfer audit](test_transfer_audit.md): current command paths, retained evidence and acceptance matrix.
- [Vendor sources](vendor_sources.md): pinned primary sources and downloaded source hashes; [vendor claim review](vendor_claim_review.md).
- [Adapter specification review](adapter_spec_review.md) and [test procedure review](test_spec_review.md): independent critique of the initial specification. These retain historical findings so that the reasoning remains inspectable.
- Requested 6 Pro conversation `6ab2d7bf-7278-83ea-aaca-f9835c1d0acf`, response `7540117c-8a7b-43f9-840d-ade654899f86`: [request](../../architecture_feedback/hardware_specs_request.json) and [original response](../../architecture_feedback/hardware_specs_response.json). Its architectural feedback was reviewed against the code. Embedded citation markers in that response are not used as independent vendor-source evidence.

## Resolved specification findings

| Finding | Where addressed | Resolution |
|---|---|---|
| Startup target unknown is not home or historical task completion; approve starting regions and transitions | Sections 6.2–6.3, T22 | Separate pose, target and command state; recovery motion requires its own qualified transition. |
| Receipt/sample IDs and RPC intervals do not prove acquisition progression/coherence | Section 6.1, H06, T03–T04 | Require source progression or justified acquisition/buffering bounds, retain unknown uncertainty, refresh evaluation times without relabeling samples. |
| Repeated stops reset dwell; faults suppress observation | Section 7.3, H02–H03, T08–T09 | Specify one stop episode, continuing read-only reconciliation and separate outcome/motion/fault states. These remain unimplemented fixes. |
| Late success, queued RPCs and blocked I/O can defeat a superficial stop | Sections 2, 6.4, 7.3; T20–T21 | Define stop/success ordering, account for buffered work, preserve external inhibition, and use an independent stop/watchdog path. |
| Owner/authority metadata does not exist at the current profile-only transport boundary | Section 4.1, H04/H07 | Proposed trusted admission envelope/resolver carries owner, task-grant and configuration evidence to the actual enforcing gateway. Stock SDK support is not invented. |
| Terminal observation bypasses the normal runtime reducer | H13, T19 | Explicit release-blocking conformance change; fault/boot/owner/contradictory facts cannot pass as terminal success. |
| Hardware approval and profile commissioning form a circular prerequisite | Section 8.3 and S2–S5 | Separate discovery, bounded profile commissioning, bounded sequence integration and operational release. Independent protective prerequisites remain mandatory. |
| Manifest digest includes itself | Section 8.1 | Hash a defined validated configuration payload excluding the commissioning-reference block; verify detached approval and canonicalization version. |
| Uncertainty recorded but absent from pass predicate | Section 6.3 | Distinguish current raw-error gate from required uncertainty-aware commissioned acceptance, with unknown bounds blocking passage. |
| Ten valid trials can mean collecting until success | Section 13 | Fixed attempted-trial counts/strata, capped replacement rules and mandatory abort dispositions; refusals and possibly actuated attempts separated. |
| Foreground worker and stale/overwritten native artifacts | Section 12 | Two-terminal workflow, fresh cohort paths, current worker/image/source provenance, post-run archival and explicit no historical-manifest reuse. |
| Suspend/reboot, stop-authority expiry and ambiguous shutdown | Sections 2, 6.1, 7 and T23 | Clock/owner epochs, revalidation after resume, continuing stop authority and retained external inhibition. |
| Vendor example/mode/network assumptions | Section 5 and vendor audit | Pin known SDK, record network-selector ambiguity, reject movement-bearing demo initialization, distinguish project gateway from vendor interfaces. |

Resolving a finding in this document does **not** implement or validate the proposed hardware change. H01–H13 and site-specific fields remain an explicit delivery/commissioning backlog. There are no supplied physical profile coordinates, safe gains or fabricated timing bounds.

The final independent rechecks are recorded in [adapter resolution](adapter_spec_resolution.md) and [test-workflow resolution](test_spec_resolution.md). The source command examples also passed a syntax check; the API signatures are explicitly presented as interface descriptions rather than executable Python statements.

## Document verification

The parent checks local links, fenced Python/shell syntax, requirement/test identifiers, existing API names and the source state used by the specification. Results are stored in `spec_validation.json`. These are document checks, not acceptance of a physical deployment. The existing 133-test result is cited as prior local evidence; the new hardware requirements require additional regressions and measured commissioning.
