# Scripted transition failure and return experiments

`scripts/validate_transition_scenarios.py` exercises the shared transition runtime
with the existing fixed-support K1 scene. It covers return boundaries, immutable
decisions, stale delivery and authority revocation. These are engineering checks;
they produce no participant data, scheduler-efficacy estimate, physical K1
validation, or XR rendering evidence.

The native runner needs **exclusive use of the Isaac worker**. Stop this
project's operator server and other command producers first. An idle observation
checks current state; it does not exclude a competing client that starts later.
Ownership verification and a subsequent stop are separate operations, so this
check cannot provide fencing against a concurrent producer.
The runner checks the worker's pinned URDF/model, joint order and fixed-support
provenance before sending any named profile. It has no physical SDK path and
accepts only a loopback HTTP endpoint, including an SSH-forwarded loopback port.

From the repository root, with the native worker already ready:

```sh
/usr/bin/python3 transition/scripts/validate_transition_scenarios.py \
  --backend isaac --endpoint http://127.0.0.1:8767 \
  --output output/transition-scenarios-native-$(date -u +%Y%m%dT%H%M%SZ)
```

For an accelerated logical run without Isaac, ROS or hardware traffic:

```sh
/usr/bin/python3 transition/scripts/validate_transition_scenarios.py \
  --backend logical \
  --output output/transition-scenarios-logical-$(date -u +%Y%m%dT%H%M%SZ)
```

An existing output directory is refused. Use `--scenario NAME` to choose a
single scenario, or repeat that option for an ordered subset. Stop on the first
failure; inspect its retained evidence before trying again in a new directory.

| Scenario | Stimulus and acceptance condition | Evidence boundary |
|---|---|---|
| `return_decision_retry` | Reach A; return stays draining until local quiescence is confirmed. Attention change preserves authority. Reject a wrong display hash, capture one predeclared A choice, then retry the identical ID after another return opens. The original result is returned without another score or motion. | The client display receipt and decision are direct scripted runtime calls. No HTTP reply is dropped by this scenario, and it does not establish that Unity rendered a frame or a person saw it. |
| `snapshot_version_change` | Change an explicit local fact while a snapshot is exposed. Its version increases, the frozen record stays intact, and an old-view response is rejected without a human-error score. | The change is an explicit local commit, not a sensor or scene perturbation. |
| `stale_snapshot_delivery` | Save one observation and deliver that same sample after 0.65 seconds. Runtime freshness expires, the return closes, and recovery does not automatically clear the fault or dispatch. | The adapter withholds a saved sample without rewriting its timestamp. Isaac continues running; this is not a physics pause or actual packet-loss injection. |
| `revoke_during_motion` | Reach A, start B, observe native joint movement of at least 0.002 rad, and revoke authority. Require a canceled terminal plus fresh idle observation; B receives no completion credit. | Native receipt retains advancing physics-time settling, measured joint/reference data and the velocity-estimator label. A measured simulated hold is not a physical emergency-stop guarantee. |

Each scenario records a separate SQLite journal, hash-verified JSONL export and
JSON report. The suite report includes source hashes, task hash, worker
provenance, the planned order, failed checks, and scenarios not attempted after a
failure. Native completions must satisfy the existing 0.035 rad joint error,
0.012 m reference distance, 0.04 rad/s speed and 0.3 physics-second dwell limits.
Canceled holds use the same joint, speed and dwell gates but do not require the
abandoned B endpoint to have been reached. Logical terminal evidence is labeled
separately and cannot satisfy native receipt checks.

The runner forwards a stop only while a fresh source observation identifies an
outstanding command from that scenario. A busy preflight sends no stop or motion.
If cleanup cannot establish an owned terminal and fresh idle source, the run
fails and preserves that uncertainty. Successful revocation leaves the arm held
at its measured intermediate position; it does not automatically return home.

These checks complement [native command validation](SIMULATION_DESIGN.md) and
the separate Unity/Meta XR Simulator checks. They preserve the physical-hardware
diagnostic hold and do not require, connect to, or restart the charging K1.

## Native result — 24 September 2026

All four planned scenarios passed on the dedicated native Isaac 5.0.0 worker:
**54 checks, four valid journal chains, 312 journal events, three arm commands
and one forwarded stop**. No scenario was skipped after a failure. The
[suite report](../../output/transition-sim-experiments-20260924/native-scenarios-report.json)
records source/task hashes and boot identity
`9f2621b6-2c17-4fa5-950b-ce5b64c96f8d`. Individual SQLite journals, JSONL exports
and reports are retained under
`output/transition-sim-experiments-20260924/native-scenarios/` relative to the
repository root. Generated output is ignored by Git and must be copied separately
when sharing the evidence.

| Native scenario | Observed result |
|---|---|
| Return and decision retry | A completed at 4.240 mm reference error and 0.01150 rad joint error, with 0.300000016 physics seconds of settling. One first response and one semantic score survived the repeated decision ID and a newly opened return. No second arm command was sent. |
| Snapshot version change | Local version increased while the frozen value/hash remained intact. The opportunity closed as a technical failure; its old-view decision produced no score or motion. |
| Stale delivery | Holding the original sample for 0.65 seconds invalidated the return. Restoring fresh delivery did not clear the latched fault or dispatch. No arm command or stop was forwarded. |
| Revocation during motion | After A completed at 7.896 mm reference error, measured joint movement was observed during B before revocation. One stop produced a canceled terminal and fresh idle source, with 0.01176 rad hold error, zero finite-difference speed at the terminal sample and 0.300000016 physics seconds of settling. B remained incomplete. |

The stopped reference point was 67.348 mm from B. That value is retained in the
receipt and is expected for an interrupted command; it is not a completed B
reach. Native velocity values remain in the same receipt and differ from the
finite-difference estimate used by the gate. These results do not resolve that
known simulator telemetry limitation or establish physical stopping behavior.

A separate [25-second ROS receive-only baseline](../../output/transition-sim-experiments-20260924/ros-baseline.json)
received 412 joint states, 412 telemetry samples, 202 images per eye and 202
matched stereo timestamps, with nonblank pixels and zero motion commands.
The [scene provenance](../../output/transition-sim-experiments-20260924/scene-provenance.json)
retains the fixed trunk/left arm/head/legs and the assumed synthetic stereo
calibration. These checks do not establish a calibrated hardware stereo camera
or head-following articulation in this bounded transition scene.

The subsequent [Windows receive-only wire check](../../output/transition-sim-experiments-20260924/windows-sim-wire.json)
received **411 joint samples and 203 unique 640×240 stereo frames in 25 seconds**,
with 203 distinct source stamps and image hashes and zero protocol errors.
This verifies the simulation transport to Windows. It does not establish
presentation in an XR eye buffer or certified acquisition freshness.

The complete Python regression suite passed **408 tests** with zero failures or
skips; [JUnit receipt](../../output/transition-sim-experiments-20260924/python-tests.xml).
The final enlarged five-fact Meta XR Simulator **205.0** fixture passed **two
tests**, with zero failures or skips; its [scope receipt](../../output/transition-xr-20260924T173121614Z-fixture/scope.json)
and [test results](../../output/transition-xr-20260924T173121614Z-fixture/results.xml)
are separate from these native/backend passes. The subsequent Meta XR/native
Isaac A → B → home workflow passed **one integration test** in 11.182 seconds,
using actual HTTP exchanges and measured native completions; [results](../../output/transition-xr-20260924T173223115Z-isaac/results.xml)
and [scope](../../output/transition-xr-20260924T173223115Z-isaac/scope.json).
The fixture and workflow cover mono panel rendering, synthetic ray interaction,
hash-bound display acknowledgement and scripted decisions. They do not establish
per-eye rendering, actual controller input or participant behavior. The elapsed
workflow time is not a controlled latency benchmark.

The earlier workflow failed at A because the full frozen snapshot did not fit
the visible panel area; [failed results](../../output/transition-xr-20260924T172507550Z-isaac/results.xml)
are preserved. The snapshot area was enlarged to 390 pixels, the five-fact
regression was added, and the retake passed. Initial native simulator crashes
were intermittent and are also retained separately. The main Unity editor and
Play mode have now been operated under the user's subsequent authorization.

In the actual `DeicticDemo` main scene, a separate native Meta Quest 3S simulator
visual check observed the world-fixed task panel and full-screen live Isaac
camera in the left-eye and right-eye views separately. Desktop shortcut/fallback
activation worked. These observations add simulator eye-view evidence; they do
not turn the scripted fixture into an actual controller-input test or establish
behavior in a physical headset.

Native simulator point-and-click selection remains unresolved. The
[passive input trace](../../output/transition-input-20260924T174902438Z/input.jsonl)
shows both trigger axes at 1 during the right-controller action, even with
**Use same input for both sides** off. The application's simultaneous-trigger
guard therefore blocks selection as intended. Investigation continues without
claiming a native input pass or weakening that guard. A mouse-routing guard was
compiled and source-reviewed so desktop clicks require a mouse-derived ray, but
the post-change trace did not capture a Unity mouse press. Event-level
verification of that change remains open; earlier desktop activation evidence
predates it.

At **17:54:47 UTC**, leaving Unity Play mode produced another native crash in
OpenXR message-pump/deinitialization code; its [crash log](../../output/transition-xr-play-exit-20260924/Editor-crash.log)
is retained. The cause remains unresolved. Recovery through Unity Hub completed
at **17:58 UTC**: `DeicticDemo` is open in Edit mode, Play is stopped, the original
hierarchy is restored, and project backups are retained. This failure is
separate from, and does not erase, the passing scripted workflow receipts.

At **17:55 UTC**, the operator service shut down successfully through SIGINT.
Its [final journal audit](../../output/transition-xr-20260924T173223115Z-isaac/journal-audit.json)
verified **96,221 events, three commands and three correct scores**, with A/B/home
complete, authority revoked, no active command and no runtime fault. The archive
`output/transition-xr-20260924T173223115Z-isaac/operator-evidence.tar.gz` and its
extracted `xr-operator-20260924-layout/` directory preserve the full journal,
manifest and final state. This continuing operator journal is separate from the
four-scenario suite's 312 events.

Owned simulator and visualization services were stopped through SIGTERM; their
systemd units classify exit 143 as failed, as expected for that termination
path. Listeners on loopback ports 8766, 8767 and 10000 were absent, the owned
Isaac container was removed, and the Windows SSH forward was stopped. The
pre-existing `rosenv-1005` container and unrelated workstation jobs were left
untouched. Native input and OpenXR exit reliability remain unresolved. No
headset test is claimed, and this engineering run does not advance physical
commissioning stages or remove the hardware diagnostic hold.
