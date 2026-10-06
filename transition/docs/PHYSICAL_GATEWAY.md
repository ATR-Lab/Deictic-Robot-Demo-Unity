# Supervised physical gateway

**Physical connections are currently held after the K1 memory/reset incident.**
See [the incident report](K1_RESET_INCIDENT.md). Observe/shadow can inspect saved
files without SDK access; they do not clear the hold or certify hardware recovery.

The gateway can inspect saved files in observation or shadow mode using the raw
snapshot written by `deployment/k1_ros_observer.py`. Neither mode constructs SDK
ports, arms the robot, grants task authority, or fabricates measured task effects.
The file must come from the same kernel clock, and its diagnostic publication
must be recent. Stopped or expired snapshots report unavailable/stale observations;
saved files do not substitute for a live validation. This check does not certify
acquisition freshness. Keep the hardware observer stopped during the incident hold.

From the workstation's `transition` directory, in a ROS Jazzy Python environment:

```sh
unset PYTHONHOME PYTHONPATH
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src:${PYTHONPATH:-}"
/usr/bin/python3 deployment/physical_gateway.py --mode observe --domain 175 \
  --observation-file "$PWD/.runtime/physical-observation/latest.json" \
  --ledger "$PWD/.runtime/physical-observation/observe-gateway.sqlite"
```

Use `--mode shadow` and a different persistent ledger path to inspect proposals.
Shadow records the proposed named profile, a distinct shadow ID, the exact raw
observation snapshot and its hash, and the reason physical admission remains
blocked. A duplicate command ID retrieves that record; it never creates a new
proposal or a predicted successful outcome.

Domain 175 is separate from the vendor domain 0 and visualization domain 174.
The wrapper publishes `/transition/physical/state` and
`/transition/physical/event`. Shadow and physical modes subscribe to
`/transition/physical/command` (`std_msgs/String` containing strict JSON).
Observation mode has no command subscription. Physical mode additionally has a
stop-only `/transition/physical/stop` subscription. No wire request can arm,
change modes, grant authority, edit profiles, or send raw joint/end-effector poses.

Command envelopes use `schema_version: 1`, `clock_id` matching the state envelope,
`operation: "start"`, and a complete named `SkillCommand` in `command`. Its only
parameters are `{"profile":"point_a"}`, `point_b`, or `home`. Status requests use
`operation: "status"` and `command_id`. Deadlines belong to the same control-host
monotonic clock. The task runtime remains responsible for its grant and scheduler.

## Command execution and stop behavior

Physical mode uses three distinct capabilities: a bounded observation cache, a
named-profile vendor client, and a stop-only SDK client in a spawned process.
The serialized dispatcher commits intent before invoking the backend. The
protected transport also persists intent before the final vendor boundary.
Duplicate IDs do not replay, changed payloads are rejected, one local file owner
holds each ledger, and any unresolved intent inhibits restart.

After live vendor identity/status checks, the final callback re-resolves task
authority, owner, site evidence, profile, deadline and protection receipt. A stop
that arrives during these checks prevents dispatch. A stop after dispatch leaves
the outcome uncertain until the measured backend resolves it. Local queue
inhibition is not evidence that a request already delivered to the robot was
removed from its queue.

Protection is checked before polling/reducing completion, again after a slow
poll, and before renewing the owner heartbeat. A success that races inhibition or
protection loss is retained as an unknown outcome with the original measurement
evidence, empty task effects and an unresolved intent. Genuine stop/cancellation
and fault observations remain available; they cannot silently become success.

The separate software stop requester watches an expiring dispatcher heartbeat
and parent-pipe closure. A blocked command, observation call, or disk operation
cannot renew that heartbeat. Stop intake bypasses the command queue and ledger
lock. A returned `StopHandEndEffector` call is reported as an RPC return only:
`pending_work_contained`, `measured_stillness` and
`physical_containment_verified` remain false until independent evidence exists.
The software worker cannot prove an emergency-stop bound, survive loss of the
whole machine, or fence old firmware work. Those are commissioning requirements.

## Physical composition

Physical startup requires a complete manifest, persistent device registry and
trusted installed site factory. The shared `assemble_supervised(...)` function
constructs the reviewed SDK factories and backend, so site code supplies only
its actual approval/grant/site services, bounded observation cache, and verified
protection monitor. It must not manufacture those records from UI input, raw
receipt timestamps, or a successful SDK call.

`ProtectionEvidence` must match the manifest, current owner and trusted issuer,
remain fresh and unexpired, and attest commissioned exclusive command ownership,
host-death inhibition and old-RPC containment. Stock SDK capability discovery
does not provide those attestations. Missing evidence blocks physical arming;
the raw observation/shadow modes remain usable.

```sh
/usr/bin/python3 deployment/physical_gateway.py --mode physical --domain 175 \
  --manifest /etc/transition/commissioned-k1.json \
  --registry-root /var/lib/transition/devices \
  --site-factory site_transition:assemble --arm
```

This command is a deployment template, not a claim that the listed site files or
commissioning evidence currently exist. `--arm` is explicit local startup only.
Without it, physical mode starts disarmed. The site factory returns exactly
`backend`, `transport`, and `stop_worker` from `assemble_supervised(...)`; the
gateway and both backend/transport ledgers use the stable device registry.

Offline tests exercise duplicate suppression, restart ambiguity, owner locking,
missing protection, late revocation/stop, queue inhibition, blocked dispatch,
heartbeat expiry, parent-pipe closure, and shadow observation provenance. They
use fictional evidence and do not establish hardware readiness or H11 completion.
