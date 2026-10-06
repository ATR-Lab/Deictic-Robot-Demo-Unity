# Adapter-contract specification resolution

Rechecked the latest [hardware implementation specification](../HARDWARE_IMPLEMENTATION_SPEC.md) against the five findings in `tmp/hardware_spec_adapter_review.md`. This was a document review only: no code changes, test execution, SDK initialization, robot contact, or physical commissioning.

| Finding | Resolution in the specification |
|---|---|
| Terminal observation bypass | **Resolved as a required implementation change.** H13 requires one consistent terminal-observation acceptance path, including fault, boot, ownership, current evidence, and authority checks before completion credit. T19 defines the corresponding negative test. |
| Authority at the actual admission boundary | **Resolved as a proposed contract.** Section 4.1 introduces `PhysicalAdmission`, a trusted grant resolver, command/configuration/owner binding, revocation/timing rechecks, and correlated gateway receipts. It explicitly distinguishes these project records from stock SDK fields. |
| Observation processing time after blocking work | **Resolved as a required timing rule.** Section 6.1 requires a refreshed evaluation clock after bounded I/O or a consistently defined as-of cache, including terminal verification. Sensor provenance cannot be rewritten to bypass future-time checks. |
| Self-referential manifest digest | **Resolved.** Section 8.1 excludes the entire commissioning block from a versioned canonical configuration payload and binds its digest to a separate trusted approval record. Loaded configuration must be immutable. |
| Circular commissioning/release prerequisite | **Resolved.** Section 8.3 separates discovery, bounded commissioning, and operational release. The backlog distinguishes pre-motion software checks from physical evidence obtained during commissioning. S5 requires its own bounded integration permit after transition commissioning, without requiring completed operational-release evidence to run that integration test. |

**No material contradiction from this bounded adapter-contract review remains unresolved in the specification.** These resolutions make the requirements explicit; they do not mark the corresponding prototype code or physical deployment complete. H01–H13, applicable tests, device enforcement, observation bounds, and physical commissioning remain outstanding implementation/evidence gates as the specification states.

The original review findings remain useful historical records. This note does not revalidate existing native, remote, or hardware results.
