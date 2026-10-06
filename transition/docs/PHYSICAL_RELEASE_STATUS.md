# Physical release implementation, 24 September 2026

**Hardware diagnostics and physical release are held after a confirmed K1 memory
exhaustion/service-reset incident.** See [incident status](K1_RESET_INCIDENT.md).
Passing offline guards does not resolve that incident or authorize reconnecting.

K1 observation and physical command release are separate. Observation/shadow
gateways and a supervised named-profile physical dispatcher are implemented.
The legacy `BoosterSDKTransport.move()` remains disabled; the new reviewed V2
boundary lives in `physical/vendor_sdk.py` and `physical/gateway.py`. Real command
release still requires commissioned protection, site approval and calibrated
observation evidence. No CLI option converts raw telemetry into that evidence.

## Implemented software

`physical/manifest.py` strictly parses the complete version-1 JSON commissioning
structure. It rejects unknown/duplicate fields, nonfinite values, booleans used
as numbers, incomplete release profiles, incorrect target labels and inconsistent
hashes. Parsed data is detached and immutable. Observation manifests may retain
unknowns. Canonicalization v1 uses Python finite JSON numeric serialization,
sorted keys, compact separators and ASCII escapes, excluding the complete
`commissioning` block from the hash.

`physical/admission.py` separates manifests from trusted approval records.
Matching a hash or copying a receipt string cannot approve a deployment. A
registry record needs an allowed issuer, nonrevoked/unexpired permit, reviewed
starting-region/profile transitions and operation margin. Bounded commissioning
also names exact command IDs; the persistent ledger prevents replay. Site and
task-grant resolvers belong to trusted local composition, never UI/ROS payloads.
The library does not supply a trusted issuer or certify caller-provided monitors;
deployment must integrate the real local approval and authority services.

`physical/sampling.py` provides immutable component acquisition intervals,
boot/owner epochs, sequence, uncertainty and calibration provenance. Unknown
acquisition progression or transport/measurement uncertainty fails release.
Age uses the earliest plausible acquisition; skew spans the full bundle. A
bounded cache cannot refresh a dead sampler's timestamps and latches boot changes.
`physical/observer_worker.py` isolates read-only calls in a subprocess. A stuck
read times out, terminates that observation process, latches and is not retried.
It accepts plain serializable diagnostics; it never infers calibrated evidence.
This is not a motion worker, device fence or independent stop watchdog.

`physical/deployment.py` assembles a disarmed backend with a persistent
serial-derived ledger under a site-configured absolute registry root. It does
not take a per-run ledger path, connect an SDK or arm. The low-level constructor
remains available to tests/trusted composition; production must use the persistent
assembly and protect its root.

The backend rechecks state, stillness, owner, task authority and full operation
budget after durable intent and immediately before `move_admitted`. That transport
must independently re-resolve expiry/ownership near the robot and return a
correlated receipt. `ProtectedProfileTransport` now implements the software
boundary: it persists its own intent, resolves immutable named profiles, and
checks live authority/site/protection again after the vendor identity/status
queries. It repeats complete measured starting-state and stillness checks at that
final boundary. A vendor acknowledgement does not prove movement or completion.

`physical/vendor_sdk.py` exposes capability-limited query, stop-only, and named
profile factories using the pinned workstation SDK. The motion port invokes only
the documented V2 endpoint and requires a final admission callback; arbitrary
poses, controller wrappers, mode switches and generic API calls are not exposed.
`assemble_supervised(...)` composes these ports with the existing measured backend
and persistent per-device ledgers. Missing trusted protection prevents SDK port
construction, rather than supplying a fabricated enforcement receipt.

`physical/stop_worker.py` requests stop in a separately spawned process when its
dispatcher heartbeat expires or the parent pipe closes. An expired lease cannot
be revived by a delayed renewal. Stop intake bypasses the command queue and disk
lock; queued work is inhibited. This is a software stop requester: the stock SDK
cannot attest hardware fencing, host-death protection, a bounded stop response,
or containment of previously delivered RPCs. Those facts must come from a real
commissioned protection monitor before physical arm. See [the gateway deployment
guide](PHYSICAL_GATEWAY.md) for observe/shadow commands and composition details.

Stop requests are idempotent: repeated ticks neither resend stop RPCs nor reset
settling. Stop admission before an acknowledgment prevents later success. Faults
permit valid continued observation; measured stillness and unknown task outcome
remain separate. Dwell uses conservative acquisition intervals, not receipt time.
Endpoint/velocity checks include explicit uncertainty bounds. An ambiguous
dispatch cannot become successful merely because the arm is subsequently still.

Closing an active/ambiguous adapter is refused. `shutdown(now)` performs one
orchestration step while callers continue polling. Ambiguous work requires both
measured stopping and an external enforcing-gateway receipt containing pending
work before resources close. Unresolved history survives and blocks the next
run. The interface does not implement an independent hardware stop/watchdog.

From the `transition` directory:

```bash
python3 deployment/k1_preflight.py config/k1-observation-only.json
python3 -m pytest tests/test_booster_backend.py tests/test_physical_release.py \
  tests/test_physical_gateway.py tests/test_vendor_sdk.py -q
```

Preflight imports no SDK and opens no connection. Even a complete manifest
reports `physical_ready=false`; actual release requires trusted commissioning
and the commissioned enforcing transport.

## Remaining hardware evidence

H02/H03 stop/fault paths, H04/H05 late admission, H09 strict configuration, and
H10 persistent identity/history have offline regressions. H01/H06/H08 have
concrete gate/sampling/uncertainty/assembly interfaces. H07/H11 still require an
independent commissioned owner/expiry/watchdog and stop path that contains queued
or in-flight work. H12 has a strict ROS gateway wrapper and an optional commissioned
physical composition; actual K1 actuation remains unvalidated and disabled without
the required live evidence. Receive-only telemetry is not a physical backend.
Common-runtime H13 changes are documented
separately. Software regressions do not complete the physical acceptance stages.

Still needed: trusted local approval/monitor services; physical support and workcell
controls; independent stop validation; source progression and acquisition delay
evidence; compatible SDK/firmware; world/body/hand calibration; approved endpoint
and transition envelopes; timing budgets; actual stale-owner, stop and failure
tests. Do not turn fictional test fixtures into site records or erase ledgers.

The integration task's read-only inspection on 24 September found a different
onboard artifact: CPython 3.10/aarch64, shared-object SHA-256
`ca6c042ff8de9851ee2d7a0f5985f92a9a2230516be09f249c73348847de314fd`.
It lacks distribution metadata and the inspected `GetRobotInfo`/`GetStatus`
methods; its `GetFrameTransform` uses an output argument and integer return.
This is incompatible with the previously reviewed 1.6.3 command transport.
No SDK or firmware upgrade is performed. Existing ROS telemetry can be observed
independently of that incompatible command interface.

An isolated, hash-verified SDK 1.6.3 client can be installed on MLWorkstation
without replacing the onboard SDK. The integration task subsequently verified
its 40 installed wheel payload files and successfully queried both `GetRobotInfo`
and `GetStatus`. The preserved `sdk-readonly-identity-3.json` receipt identifies
model `K1` and reported firmware `v1.6.1.1-release-01967-2026-04-27`; the status
query reported mode 1, body-control 2 and an empty current-actions list. This
establishes read-only identity/status connectivity with the isolated newer client.
It does not approve mode 1/body-control 2 for motion, establish compatibility of
motion RPCs, or commission the firmware/control combination. The robot's installed
SDK and firmware remain unchanged.

## Future isolated workstation SDK diagnostic

This procedure is currently held after the memory/reset incident. The entry point
refuses to start while the hold exists. Keep these commands as reference for a
future bounded validation; do not resume robot traffic now.

`deployment/k1_sdk_diagnostics.py` permits exactly `GetRobotInfo` and `GetStatus`.
It verifies all installed wheel payloads, including package wrappers and native
extensions, against the previously inspected CPython 3.12/x86_64 wheel before
importing the SDK. Installer-rewritten `RECORD` is excluded; import paths must
resolve to the verified files. Each query runs in its own
subprocess with a configurable 1–15 second deadline; failures are retained and
never retried automatically. It constructs only `ChannelFactory` and an
uninitialized `B1LocoClient`, initializes the explicit channel, then performs
the two allowed queries. This creates the client's request/reply RPC channels
and sends query requests. It does not instantiate controller wrappers or invoke
movement, stop, mode changes, raw API IDs or low-level command publishers.

Use a separate **workstation** venv, preserving the robot's existing SDK:

```bash
/usr/bin/python3.12 -m venv transition/.runtime/sdk-diagnostics-venv
transition/.runtime/sdk-diagnostics-venv/bin/python -m pip download \
  --no-deps --require-hashes -r transition/config/sdk-diagnostics-requirements.txt \
  --dest transition/.runtime/sdk-diagnostics-wheels
transition/.runtime/sdk-diagnostics-venv/bin/python -m pip install \
  --no-deps --no-index transition/.runtime/sdk-diagnostics-wheels/booster_robotics_sdk_python-1.6.3-cp312-cp312-manylinux_2_34_x86_64.whl
```

After resolving the incident hold and verifying the intended SDK network binding, domain and robot-name suffix,
run the following with those explicit values. The network binding is the control
host's demonstrated interface/address, not automatically the robot's IP.

```bash
env -u PYTHONPATH transition/.runtime/sdk-diagnostics-venv/bin/python \
  transition/deployment/k1_sdk_diagnostics.py \
  --wheel transition/.runtime/sdk-diagnostics-wheels/booster_robotics_sdk_python-1.6.3-cp312-cp312-manylinux_2_34_x86_64.whl \
  --network-binding VERIFIED_CONTROL_HOST_BINDING \
  --domain-id VERIFIED_VENDOR_DOMAIN --robot-name VERIFIED_ROBOT_SUFFIX \
  --output transition/.runtime/sdk-diagnostic-unique.json
```

An explicitly empty `--robot-name ''` selects the documented default vendor RPC
channel. Do not reuse an earlier output filename: diagnostic receipts are created
exclusively. The report includes package/wheel/extension hashes, identity, status,
RPC timing and per-query errors. Old firmware may reject or omit these APIs;
a timeout does not establish which cause applies. `physical_ready` always
remains false. A newer isolated client responding does not commission its motion
API or upgrade the robot firmware.
