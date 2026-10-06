# Inspected Booster vendor evidence

This folder preserves the 21 source files actually downloaded and inspected, plus the SDK and asset repositories' license files during the September 23, 2026 vendor audit. `source_receipts.json` retains exact primary URLs and SHA-256 values; its `local` paths are relative to the project root. `revisions.json` records the inspected official repository heads and source-discovery paths. Large recursive repository trees were omitted because they are not needed to verify these artifacts.

The files are reference evidence, not an installable SDK or runnable deployment package. In particular, the archived vendor RPC example issues mode and movement commands and must not be used as a preflight script. A recorded HTTP 404 has no local artifact; the later successful motion-name-file receipt is separate.

The authoritative explanation and source boundaries are in `../vendor_sources.md`. The original downloaded files are byte-for-byte preserved; the audit and receipt paths were adjusted for this durable location. No robot or simulation-server access was performed.
