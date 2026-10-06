# K1 memory exhaustion and service resets — 24 September 2026

**Hardware diagnostics are held. Do not restart the hardware observation stack
or camera service while this incident is unresolved.** Simulation and offline
tests remain separate from the robot. This is not a completed physical release.

## Observed facts

- The operator reported repeated torque loss and vendor-service resets.
- Kernel logs show repeated system-wide out-of-memory kills, beginning at least
  at **15:13:30 UTC / 11:13:30 EDT**. Victims include vendor RPC services,
  robot-state publication, perception/agent processes and our camera observer.
- At **16:32:37 UTC / 12:32:37 EDT**, the kernel explicitly killed `motion`
  (PID 87846). Vendor daemons subsequently restarted. This is direct evidence of
  memory exhaustion affecting the motion stack, not an intentional mode change.
- The latest motion process inspected started at 16:36:03 UTC. The robot's OS
  uptime continued: these were process/service resets, not necessarily reboots.
- Our MLWorkstation diagnostics finished/stopped; a process check found no
  observer, gateway, ROS probe, SDK probe or receive-only bridge still running.
  The K1 camera observer was stopped and disabled at approximately 16:38–16:40 UTC.
  Its unit reported `MainPID=0`, `UnitFileState=disabled`, with six prior restarts.
- The operator then reported **no further resets so far**. A subsequent SSH-only
  memory reading showed 5,057,628 KiB available out of 7,803,520 KiB total.
- At 16:45:06 UTC, memory available was still 5,017,560 KiB and the kernel log
  contained no new OOM event since 16:38:10 UTC. This is a short recovery window,
  not proof that the underlying issue is fixed.

Only read-only identity, status and hand-transform queries were sent during the
diagnostics; no movement, stop, torque or mode-change API was intentionally
invoked. Read-only traffic can still consume resources. The association between
our diagnostics and the incident is being treated seriously; the component
responsible for the memory growth has **not** been isolated.

## Investigation limits

Direct SDK queries and standalone ROS queries worked. The combined ROS observer
developed query timeouts while telemetry continued. Disabling periodic file I/O
and reducing callback work did not resolve that symptom. A short diagnostic with
only joint/motor subscriptions answered all four query types, while the full
subscription set did not. These observations do not establish that either
`/robot_states` or `/fall_down` caused the memory exhaustion.

The earlier 30-second successful collector run and later failed runs are retained
as separate evidence. The live observe/shadow smoke test did **not** pass its
complete query criterion. No physical task trial or stop commissioning occurred.

The kernel log, successful/failed collector summaries, failed gateway logs and
query receipts were copied to the local ignored directory
`output/k1-reset-investigation-20260924/`. The original kernel capture is also
preserved on the K1 under
`~/transition-observation/.runtime/incident-20260924-kernel.log`.

## Containment

- Our camera service remains disabled; its candidate unit now uses `Restart=no`.
- `transition/.runtime/k1-diagnostics-hold.json` on Windows and MLWorkstation,
  and the matching `.runtime` file under the robot's observer installation,
  prevent routine hardware launchers and diagnostic entry points from starting.
- No vendor service, firmware, controller mode, firewall or system memory policy
  was changed. Other users' workstation jobs were untouched.
- Subsequent robot inspection uses SSH for OS logs/process/memory readings only,
  without creating ROS/DDS participants or SDK clients.
- Hold checks passed for the Windows launcher (without changing Unity startup
  settings), Linux hardware launch scripts, and Python diagnostic entry points.

## Work required before reconnecting

An offline camera candidate now rejects oversized raw payloads, invalid strides,
dimensions, encoding, stamps and frame IDs before caching or decoding. It defaults
to a single mono source; `--stereo` explicitly enables both raw eyes. Camera
endpoints use best-effort, volatile, depth-one QoS. Parameter services and rosout
are disabled, and OpenCV is limited to one thread. Mocked ROS tests exercise source
selection, QoS, invalid-frame retention and the hold before native imports.
These changes have **not been deployed to the K1**. They limit application work
and retained frames, not middleware allocations before callbacks or memory in
vendor processes. They do not establish the cause or resolution of the OOMs.

The final offline regression run passed **385 tests in 28.45 seconds** under
Ubuntu/WSL, including the camera and incident-hold checks. Receipt:
`output/transition-physical-continuation-tests.xml`. An earlier run had one fixture
failure because the real hold stopped its missing-wheel preflight test; that test
now uses an isolated temporary installation, with native workers forbidden. The
production hold remains in place. These are software checks, not a sustained
memory or slow-receiver validation.

Continue auditing DDS QoS/resource limits, discovery/transport configuration and
inherited RMW settings. Validate candidate guards with synthetic traffic and a slow receiver in an isolated
environment, recording memory and thread growth. A limit on our own process
cannot protect against allocations inside vendor processes.

After that, prepare a short supervised test with a memory budget, abort criteria
and a person beside the disarmed/supported robot. Leave the hold in place until
that test is concrete and agreed. Do not treat removing a file, restarting a
service or a passing unit test as resolution of the incident.

The separate Unity execution-policy investigation is recorded in
`output/transition-unity-policy-diagnostics-20260924.md` in the local repository.
Codex rejected the Unity launch before execution; Windows Smart App Control
separately blocked a generated .NET test harness. Neither explains these Linux
kernel OOM events.
