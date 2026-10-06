# Vendor-claim review of the hardware specification

Reviewed September 23, 2026 against the [pinned primary-source audit](vendor_sources.md) and the [preserved evidence](vendor_sources/source_receipts.json). The inspected specification hash is in [vendor_claim_review.json](vendor_claim_review.json).

**No material vendor-claim errors found** in `docs/HARDWARE_IMPLEMENTATION_SPEC.md`. The specification correctly bounds K1 V2 endpoint support, deprecated orientation behavior, Python acknowledgment versus measured completion, network-selector ambiguity, computed transforms, separate state-acquisition times, arm indices, and the absence of demonstrated controller-side durable command recovery. It does not claim that ROS integration or simulator success commissions physical K1 operation.

Site-specific firmware/planner compatibility, support, calibration, ownership enforcement, watchdog/stop behavior, motor-health interpretation and timing remain explicit requirements. This conclusion verifies the specification's vendor attributions; it does not certify those missing components or the device.

The durable evidence retains 21 inspected source files byte-for-byte, their primary URLs and SHA-256 values, repository revisions, the failed-path receipt, and the SDK/asset repository license files. Broad recursive repository trees were omitted. No main-specification changes, hardware commands, robot connections or simulation-server access were made during this review.
