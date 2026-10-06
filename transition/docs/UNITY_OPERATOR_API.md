# Unity operator API (schema 1)

The workstation owns task state, journals, authority, monotonic time and backend
commands. Unity is an operator display/input client. This HTTP API does not accept
raw joint poses, native SDK commands or client timestamps.

## Connection and session

Run the task console on workstation loopback port 8766, and forward the same port:

```sh
ssh -N -L 8766:127.0.0.1:8766 marnett5@mlworkstation.atr.cs.kent.edu
```

Unity uses `http://127.0.0.1:8766`. `GET /api/session` returns:

```json
{"schema_version":1,"session_token":"session-secret","run_id":"run-id","backend":"logical_simulation"}
```

Use the token only in `X-Session-Token` on POST requests. Do not log, journal or
persist it. The endpoint validates its loopback Host, rejects foreign Origin and
cross-site fetch metadata, sends no CORS permission and returns `Cache-Control:
no-store`. This is local/tunneled access, not authentication against hostile local
processes. Changing `run_id` or session requires explicit fresh interaction; never
replay a pending operation into a new run. HTTP host validation requires forwarding
8766 to 8766 rather than changing the local port.

`GET /api/state` returns the existing runtime public state plus `schema_version`,
`auto_dispatch` and `server_time`. `GET /api/events` returns journal events. Reads
do not acknowledge context, grant authority or dispatch commands.

### Camera display clock

`GET /api/time` is a read-only, uncached clock sample with the same Host/Origin
checks as session bootstrap. It does not require the token or enter the runtime
owner queue. Its schema is:

```json
{"schema_version":1,"run_id":"run-id","clock_domain":"server_system_utc","scope":"display_only","server_receive_utc_seconds":1790264000.0,"server_send_utc_seconds":1790264000.0001,"server_processing_seconds":0.0001}
```

For client UTC send/receive times `t0`, `t3`, estimate server-minus-client offset
as `((server_receive_utc_seconds - t0) + (server_send_utc_seconds - t3))/2`.
Use client monotonic round-trip time, subtract server processing time, and retain
at least half the remaining round trip as uncertainty; asymmetric transport can
consume that whole uncertainty interval. Reject nonfinite or backward samples,
clock jumps relative to monotonic elapsed time, excessive round trips, and run-ID
changes. Expire the estimate and retain an unknown camera-age state when no valid
sample exists. The returned send time is sampled immediately before response
serialization; its serialization/transmission delay remains in the uncertainty.

This estimate supports the camera display's age indicator only. It is not a
device clock calibration, a physical observation freshness certificate, an
authority deadline, or evidence that a camera image was rendered in the headset.

## Explicit operator operations

POST `/api/operator`, `Content-Type: application/json`, with:

```json
{"operation":"attention","data":{"domain":"local"}}
```

Only the following fields are accepted; unknown keys, duplicate JSON keys,
nonfinite values, Boolean numeric values and incorrectly typed fields fail with
HTTP 400 before owner admission. Body limit is 65,536 bytes. Identifiers/targets
are nonempty strings of at most 256 characters without control characters.

| Operation | Data |
|---|---|
| `grant` | Optional `skills` (distinct string array), `ttl` (seconds, >0 and <=3600; default600) |
| `acknowledge` | Empty object |
| `attention` | `domain`: `local` or `remote` |
| `quiescent` | `confirmed`: Boolean local-stability attestation |
| `local` | `key`: declared `local.*` fact; `value`: JSON value |
| `run` | `enabled`: Boolean; false pauses future skill starts |
| `revoke` | Empty object; revoke task authority and request task stop |
| `cue` | Optional `checkpoint_id`, `window` (>0 and <=120 seconds; default15) |
| `displayed` | `lease_id`, optional `snapshot_hash` |
| `respond` | `lease_id`, `decision_id`, `kind`, `target`, optional `snapshot_hash` |
| `reconcile` | Empty object; read-only reconciliation, no automatic retry |

Runtime guard failures normally return HTTP 409. A 15-second owner wait timeout
means processing may still occur: it is not proof that an operation was rejected.
Only `respond` has the stable-ID retry guarantee below. Inspect state/journal
before retrying other uncertain state-changing operations. Pause/revoke are task
operations, not the robot's independent emergency stop. No hardware-arm operation
is exposed by this prototype API.

## Display and first-response binding

`state.lease` is null when absent. Otherwise it contains `id`, `checkpoint_id`,
`status` (`draining`, `exposed`, `closed`), `cue_at`, `deadline`, `exposed_at`,
`displayed_at`, `closed_at`, `response_at`, `snapshot`, `snapshot_hash`,
`decision_summary`, `commitment_id`, `coherence_flags` and `result`. The lease's
summary and acknowledgement identity are frozen at exposure; show that summary
beside the immutable snapshot rather than the current state's live summary.
History display returns an empty summary. Absent times/hashes are null; a draining snapshot is
empty. Snapshot and hash remain available after closure.

Each snapshot entry is the fact's detached `value`, `version`, `status`,
`observed_at`, `valid_until`, and `source`. Render this immutable snapshot for an
exposed return opportunity, not a later live `state.facts` map. The hash is SHA-256
of UTF-8 Python canonical JSON (sorted keys, compact separators, default ASCII
escaping, finite values only). Clients should retain and echo the supplied hash;
they need not reproduce Python number formatting.

1. Observe an exposed lease and render its snapshot.
2. After an actual visible frame, POST `displayed` with its `lease_id` and
   `snapshot_hash`. Receiving or parsing data is not a render acknowledgement.
3. Enable first-response controls only after the server accepts that display
   receipt. Allow `execute`, `defer`, and `request_evidence` with an explicit target
   or reason. Do not preselect a response from the independent scoring rubric.
4. Generate one stable `decision_id` for that deliberate choice. Preserve the
   complete payload through a lost reply. The first admitted response wins.

```json
{"operation":"respond","data":{"lease_id":"lease-id","snapshot_hash":"64-lowercase-hex-digits","decision_id":"one-choice-id","kind":"execute","target":"select_A"}}
```

Result fields include `correct` (Boolean or null), `label`, `lease_id`,
`decision_id`, and `snapshot_hash`. An identical retry returns the original result
plus `duplicate:true`, including after closure or a later lease; a changed payload
with the same decision ID fails. Closed leases cannot accept a new first choice.
Scorer failure/pending score retains the captured choice. Snapshot hash validation
is optional only for legacy clients; Unity must send it for both operations.

HTTP 400/409 alone does not prove rejection before capture: a later storage error
can occur after the raw response has committed. A serialized response failure
echoes `lease_id`, `decision_id`, and `snapshot_hash`, plus `response_captured`:
false means a definitive validation rejection before that ID was captured; true
means the ID is already captured; null means the result is uncertain. Release a
pending choice only for Boolean false with all three identities matching the
exact pending payload. Missing/null flags, lost replies and generic HTTP errors
must retain the original ID for reconciliation/retry.

Deadlines use trusted server ingress and start at cue, including drain/exposure
time. Timely ingress can be processed later. Input at the exact deadline is late.
Client countdowns are presentation only; do not send Windows monotonic values.
If evidence/authority changes, the server may invalidate the lease. Keep raw
participation distinct from semantic correctness and technical unavailability.

## K1 reference workflow

Grant this task and acknowledge the presented context separately. Commit
`local.ready=true`, select local attention and start sequencing. After measured A,
confirm local stability, cue, render/acknowledge and record `execute/select_A`.
Then separately commit `local.selected="A"`. Repeat for B with `select_B` and
`"B"`. After measured home record `defer/sequence_complete`. Unknown target
evidence supports `request_evidence/pointing_state`.

A response never executes motion or commits local facts. Changing camera view
never changes task authority, acknowledgement, attention or command frames.
Manual teleoperation and task sequencing must not be concurrent command owners.
This reference sequence tests integration; it cannot establish scheduler efficacy.
