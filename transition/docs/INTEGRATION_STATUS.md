# Integration status — 24 September 2026

The supplied transition-aware framework is integrated into this monorepo and the
Quest Link application. Native Isaac sequencing, return/fault experiments and
the historical physical receive-only ROS path are verified. The scripted Meta XR
Simulator/Isaac A → B → home operator workflow now passes. Physical task motion
and native simulator controller selection remain incomplete. A separate visual
check of the main application observed its world-fixed task panel and full-screen
Isaac camera in both the simulator's left-eye and right-eye views.

**Current hardware state:** diagnostics are held following repeated K1 kernel
out-of-memory kills and vendor-service resets. All owned diagnostic traffic is
stopped; the camera service is disabled with restart off. The operator reported
no further resets after shutdown. See [incident evidence and containment](K1_RESET_INCIDENT.md).

## Implemented

- A separate world-fixed Unity task panel for grants, local/remote attention,
  version acknowledgement, local selections, sequencing, return cues and decisions.
- Frozen, hash-bound return snapshots; display acknowledgement requires an actual
  visible camera render. Response recording is separate from actuation and local
  fact updates. Uncertain retries retain the decision identity.
- Ordinary and consequence-aware schedulers sharing the same admission guards,
  measured completion reducer, event journal and independent response scoring.
- Native Isaac A → B → home task with a fixed root and four right-arm joints,
  synthetic head stereo, idempotent commands, stop and lost-response handling.
- A simulation-only experiment runner for return boundaries, immutable decision
  retries, local dependency changes, stale observation delivery and revocation
  during measured native arm movement. Each scenario retains its own journal,
  injected-fault description and measurement receipt; failures stop the suite.
- Startup modes separating manual simulation, transition simulation and physical
  observation. Transition ROS connections cannot register publications/services.
  Hardware mode cannot submit operator HTTP mutations.
- Read-only HTTP clock estimation for simulated camera display, with bounded
  exchange uncertainty and expiry; it never adjusts the robot command clock or
  certifies physical freshness. Normal Unity domain/scene reload ensures the next
  Play session applies its selected startup mode to a fresh ROS connection.
- Strict physical configuration, trusted admission interfaces, immutable acquisition
  evidence, bounded observation subprocesses, persistent device history and
  conservative stop/completion handling. Added pinned SDK query/stop/profile
  ports, a serialized named-profile gateway, durable intent records, and a separate
  software stop process. These do not supply commissioned device ownership,
  hardware fencing or validated stop bounds.
- Runnable observation/shadow gateway on isolated domain 175, backed by a
  read-only collector for vendor ROS messages and identity/status/transform RPCs.
  Neither mode grants task authority or reports successful physical effects.
- One-command Windows/MLWorkstation launchers with isolated DDS domains, SSH
  forwarding, scoped child cleanup, and no automatic Unity/Play operation.

The original manual bimanual IK/head-following demo remains a separate simulation
mode. It is not connected to physical K1 control by this integration.

## New verification evidence

The source archive's older validation documents are historical. The following
checks were performed for this integration; generated receipts remain in ignored
local output/runtime directories, rather than being presented as participant data.

| Check | Result and limits |
|---|---|
| Python regression suite | **408 passed**, zero failed/skipped, under Ubuntu/WSL; [current receipt](../../output/transition-sim-experiments-20260924/python-tests.xml). Includes the 23 new simulation-runner tests alongside scheduler/runtime, physical guards, pinned SDK ports, gateway/watchdog, ROS protocol, camera bounds, hardware-hold and process-cleanup regressions. Earlier post-incident receipt: 385 passed. This does not establish hardware memory stability. |
| Native S1 candidate campaign | Passed its prospective declaration: one planned/attempted campaign, zero refusals. Lifecycle, ordinary and consequence runtimes, and ROS visualization passed. |
| Native task outcomes | A/B/home effects measured in Isaac; three scripted correct decisions under each policy; both 526-event chains independently verified. This sequential task does not distinguish scheduler efficacy. |
| Native transition experiments, 24 September | All four planned scenarios passed: return/decision retry, snapshot version change, stale delivery, and revoke during motion. All 54 checks and four journal chains passed (312 events, three arm commands, one forwarded stop). Fault cases intentionally retain technical-failure state without human blame or automatic replay. [Native report](../../output/transition-sim-experiments-20260924/native-scenarios-report.json); [scope and commands](SIMULATION_EXPERIMENTS.md). |
| Native ROS visualization, 24 September | Latest 25-second receive-only check: **412 joint-state and 412 telemetry samples, 202 frames per eye, 202 matched stereo timestamps**; nonblank 320×240 RGB pixels checked. Zero motion commands from this probe. [Receipt](../../output/transition-sim-experiments-20260924/ros-baseline.json). Earlier integration check: 407 joint/telemetry samples and 199 stereo pairs. |
| Windows simulation wire path, 24 September | Receive-only 25-second check: **411 joint samples and 203 unique 640×240 stereo frames**, with 203 distinct stamps and image hashes, zero protocol errors. [Receipt](../../output/transition-sim-experiments-20260924/windows-sim-wire.json). This verifies transport to Windows, not frame presentation in the headset or certified acquisition freshness. |
| Unity / Meta XR Simulator fixture | Final enlarged five-fact fixture with Meta XR Simulator **205.0: two passed, zero failed/skipped**; [results](../../output/transition-xr-20260924T173121614Z-fixture/results.xml) and [scope](../../output/transition-xr-20260924T173121614Z-fixture/scope.json). This follows an earlier normal-fixture pass and intermittent initial native crashes. Scope: scripted mono panel rendering and synthetic ray interaction, not per-eye or actual controller verification. Earlier headless run: 106 project tests passed, eight GPU cases skipped. |
| Unity / native Isaac workflow | **One integration test passed** in 11.182 seconds with Meta XR Simulator 205.0, actual HTTP exchanges and measured A → B → home completions; [results](../../output/transition-xr-20260924T173223115Z-isaac/results.xml), [scope](../../output/transition-xr-20260924T173223115Z-isaac/scope.json), and three retained snapshot images/JSON records. Scripted mono panel rendering, hash-bound display acknowledgement and decisions passed. See [Unity verification notes](../../docs/TRANSITION_UNITY.md). |
| Final operator journal audit | After graceful operator shutdown: **96,221 events, three commands, three correct scores**, A/B/home complete, authority revoked, no active command and no runtime fault; [audit](../../output/transition-xr-20260924T173223115Z-isaac/journal-audit.json). The complete operator evidence archive and extracted journal/state/manifest are retained beside it. This is separate from the four-scenario runner's 312-event evidence. |
| Main application, simulator visual check | Actual `DeicticDemo` Play mode with native Meta Quest 3S simulation displayed the world-fixed task panel and full-screen live Isaac camera in separately inspected left-eye and right-eye views. Desktop shortcut/fallback activation worked. This is simulator visual evidence, not a physical headset check or a native controller-input pass. |
| Native simulator point-and-click | **Unresolved.** Passive traces show both left and right trigger axes at 1 for the right-controller action even with **Use same input for both sides** off. The task guard blocks this as simultaneous triggers. [Input trace](../../output/transition-input-20260924T174902438Z/input.jsonl); investigation continues separately from the passing synthetic-ray fixture and desktop fallback. |
| Desktop mouse routing candidate | Compiled and source-reviewed: mouse clicks require a mouse-derived ray, preventing clicks from activating an unrelated XR-ray target. The post-change passive trace did not capture a Unity mouse press, so its interactive event-level regression remains **unverified**. Earlier desktop activation observations predate this change. |
| Unity Play-mode exit | A later native crash occurred at **17:54:47 UTC** in OpenXR message-pump/deinitialization code; [retained log](../../output/transition-xr-play-exit-20260924/Editor-crash.log). Its cause remains unresolved and it is separate from the passing workflow test. Recovery completed at **17:58 UTC**: `DeicticDemo` is open in Edit mode, Play is stopped, the original hierarchy is restored, and backups are retained. |
| Windows hardware transport | Authenticated SSH to the receive-only ROS-TCP endpoint: 470 joint samples, 25 status samples and 20 head images in 25 seconds; no endpoint errors. This checks the wire path, not headset rendering. |
| Installed camera service | Historical 20-second check received 374 joints but only one new head image; the source was intermittent. The service later suffered OOM kills and is now stopped and disabled under the incident hold. |
| Complete Isaac launcher | Started native worker, camera relay, receive-only endpoint and operator API; session/state and the read-only display clock endpoint responded. Its ROS-TCP route delivered 402 joint samples and 196 stereo images in 25 seconds. Owned services/container stopped cleanly afterward. |
| Pinned workstation SDK | All 40 wheel payloads verified against SDK 1.6.3. Read-only `GetRobotInfo` and `GetStatus` succeeded. No motion/mode/stop RPC was sent. |
| Official ROS interfaces | Built `booster_interface` from pinned commit `5f9182e1a1b704f65e0065c83a3b6be2d066f3b3`, with a recorded syntax-only fix for upstream `Subtitle.msg`. LowState/MotorState/RobotStates/RPC definitions match the robot's installed definitions. Vendor examples were not executed. |
| Vendor ROS collector | 30 seconds: 3,104 LowState and 3,100 joint samples; 20 successful queries each for identity, status and both torso-to-hand transforms. Final error map empty. Acquisition bounds and exclusive command ownership remain unverified. |
| Later live gateway attempt | Joint/motor state reached the observation gateway, but all four query types timed out during the complete smoke check. Standalone SDK/ROS queries and a reduced-subscription diagnostic succeeded; the full live check is not passed. No task motion/effects were produced. |
| Physical release | Blocked. No physical A/B/home trial, calibration, support/stop test or command-owner commissioning occurred. |

The final native declaration and result input are in
`examples/stages/s1-final-candidate-prospective-20260924.json` and
`examples/stages/s1-final-candidate-result-20260924.json`. The generated report is
`output/transition-validation/stages/s1-final-candidate-20260924.json` relative to
the repository root. It includes source/artifact hashes and explicitly does not
authorize physical motion. Earlier diagnostic failures and retries are retained;
they were not relabeled as the prospective campaign.

The newer four-scenario experiment is a separate engineering run, not another
prospective S1 campaign or an S2–S7 advancement. Its two A completions measured
4.240 mm and 7.896 mm reference error. Revocation produced a canceled terminal,
0.01176 rad error relative to the measured hold target and at least 0.3 advancing
physics seconds of settling; B was not credited. Native generalized velocity
remains separately recorded, while the gate uses the existing finite-difference
estimate. Direct-runtime decision retry does not inject an HTTP response loss;
stale delivery is an explicit adapter fault and does not pause physics. Exclusive
worker use was provided by orchestration, not established by the idle preflight.

The first Unity/Isaac workflow attempt failed at A because the full frozen
snapshot did not fit its visible panel area. The failure is retained in
`output/transition-xr-20260924T172507550Z-isaac/`. After expanding the snapshot
area to 390 pixels and adding the five-fact fixture regression, both the fixture
and the native workflow retake passed in separate directories. This is a
retained failure followed by a verified fix, not a first-attempt success. The
11.182-second workflow duration is an engineering observation on this setup,
not a controlled latency measurement.

## Actual K1 connection

MLWorkstation reaches `192.168.10.102` through its control interface `enp2s0`.
The robot runs Ubuntu 22.04/aarch64/ROS Humble; MLWorkstation runs ROS Jazzy with
system Python 3.12. Vendor traffic remains on domain 0; diagnostic Unity traffic
uses domain 174. The workstation gateway uses domain 175. The manual simulation
uses domain 42.

The native 22-joint state is mapped explicitly to the supplied URDF names. The
vendor RPC service is visible, but no physical FollowJointTrajectory action was
discovered or fabricated. Physical command services are not exposed to Unity.
The isolated SDK reported model **Booster K1** and version
`v1.6.1.1-release-01967-2026-04-27`. Identity/status query success does not certify
motion compatibility. The onboard SDK, firmware and vendor controller were left
unchanged.

The installed robot user service `transition-k1-camera-observer.service` is
stopped and disabled under the incident hold. Its diagnostic display uses the raw
left head camera as a mono image in both eyes. The offline candidate defaults to
mono only; `--stereo` explicitly adds the second raw subscription and diagnostic
pair publication. The candidate has not been deployed or shown to resolve the
memory incident. Pair metadata retains both source stamps but does not certify
synchronization or rectification. Both raw publishers are
intermittent. The vendor `/booster_video_stream` also repeats identical old frames
(169 messages but nine distinct timestamps in a 20-second sample), so it is not
the default camera source. Stale frames are not relabeled or repeatedly forwarded.
Unity blanks its view when new images stop; a continuous physical POV is not yet
established.

## Remaining work

1. Investigate the memory/reset incident offline, including DDS allocation and
   slow-receiver behavior. Keep robot traffic held until a bounded supervised
   validation is prepared and agreed; see [the incident report](K1_RESET_INCIDENT.md).
2. Resolve and verify native Meta XR Simulator point-and-click delivery. The
   separate eye views and earlier desktop fallback were observed, but paired
   trigger values currently activate the intentional simultaneous-trigger guard.
   Keep this guard intact while investigating simulator input mapping. Verify
   mouse-event delivery for the compiled routing fix and investigate the native
   OpenXR crash on Play-mode exit. Arrange a user-visible simulator check before
   a headset test.
3. Stabilize/characterize the robot's camera publishers and commission source
   timing, registration, velocity and transform uncertainty. Receipt timing is
   insufficient for physical admission.
4. Obtain the actual support arrangement, independent stop, exclusive command
   ownership and calibrated A/B/home/transition records. Integrate and commission
   device enforcement and independent protection against those site controls;
   the new software dispatcher/stop process alone cannot establish them.
5. Advance through the supplied S2–S7 campaign with declared trials and retained
   evidence. No permission flag, profile fixture or successful simulation bypasses
   these gates. See [physical release status](PHYSICAL_RELEASE_STATUS.md).

Earlier in this integration, the Codex execution tool rejected both the native Meta XR Simulator launch and
the reduced graphics test launch with the stated reason “blocked by policy.” A
renewed request at 16:02:52Z on 24 September was also rejected before execution.
The specific rule is not exposed; local configuration and logs do not establish
that an Auto-review agent caused it. Separately, CodeIntegrity event 3077 confirms
Windows Smart App Control (`VerifiedAndReputableDesktop`) blocked the generated
.NET contract harness (error 4551). No Windows security event matched the renewed
Unity request. No security settings were changed or bypass attempted. This is
historical diagnostic evidence; newer Meta XR Simulator launches reached native
execution, with intermittent initial crashes followed by passing normal and
five-fact fixtures and the native Isaac workflow. The user subsequently
authorized operation of the main Unity editor and Play mode, and those have now
been operated for simulator validation. The main-application eye-view check has
now been observed, while native simulator trigger selection remains unresolved.
No headset test was conducted during these validation runs.

See [setup and launch commands](../README.md) and the
[Unity operator protocol](UNITY_OPERATOR_API.md). No passwords or commissioning
credentials are included in source control.

Cleanup completed at **17:55 UTC**. The operator service shut down successfully
through SIGINT and its final journal was audited. The owned simulation worker
and visualization services stopped through SIGTERM; their systemd units report
exit 143/failed because of that signal classification. Loopback listeners on
8766, 8767 and 10000 were absent, the owned Isaac container was removed, and the
Windows SSH forwarding process was stopped. The pre-existing `rosenv-1005`
container and other users' workstation jobs were untouched.

The K1 diagnostic camera service remains stopped and disabled; installed hardware
entry points remain held while the robot charges. The memory incident takes
precedence over historical hardware startup instructions. Native input and
OpenXR exit reliability remain open issues despite the passing simulator task
workflow and completed service cleanup.
