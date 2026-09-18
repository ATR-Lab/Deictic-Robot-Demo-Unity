# Validation record

This is a simulation-first engineering reproduction of the paper's architecture. It does not establish the paper's user-study results, physical-robot accuracy, or physical Quest camera performance.

Results are dated and preserve unsuccessful trials as well as later corrections. The named `output/` and `sim/artifacts/` files were retained in the development workspace; generated logs, images, APKs and test reports are excluded from the source repository. A fresh clone contains the test and capture tools, not those historical artifacts. Start with [the current setup](setup.md) and [teleoperation validation commands](arm-teleoperation.md#validation) to produce new reports.

## Bimanual pose correction, protocol v2 — 2026-09-18

Review against [Unitree xr_teleoperate, pinned at 817fb00](https://github.com/unitreerobotics/xr_teleoperate/tree/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b)
exposed weaknesses in the previous position servo. A reachable 5–10 cm inward
left target produced no movement from the straight-arm singularity; the right
arm could select a joint-limit branch despite a collision-free alternative.
The replacement uses current-head-relative full poses, measured-state nonlinear
IK, deterministic singularity escape, K1-derived translation scale, recoverable
target saturation and relay lease acknowledgement. See [the mapping and limits](arm-teleoperation.md).

The separate Unity validation copy passed **53/53 EditMode checks**
(`output/teleop-pose-v2-editmode-r2.xml`). New coverage includes common body
translation/yaw invariance, controller orientation and FLU direction, invalid
quaternions/heading, protocol mismatch and persistent fault diagnostics.
The first run passed 52/53: a test compared equivalent opposite-sign
quaternions as unequal. The corrected test checks rotation equivalence and
independently verifies transformed axes; runtime frame conversion was unchanged.

On Ubuntu Jazzy with Python 3.12, NumPy 1.26.4 and SciPy 1.11.4, the complete
controller plus changed relay/verifier selection passed **176/176**
(`teleop-pose-v2-native.xml`). After adding the final post-solve freshness
recheck and verifier callback recovery, **41/41 focused checks** passed
(`teleop-pose-v2-finalfix.xml`). The latter overlaps the full run and is not an
additional independent 41-test suite. The new freshness tests simulate expired
input, feedback, relay acknowledgement and computation deadlines before an
active command can receive a new timestamp.

A native 220-tick bilateral 10 cm inward benchmark measured mean **14.91 ms**,
p95 **25.09 ms**, maximum **46.20 ms**, and final setpoint FK errors
**0.126/0.102 mm** (`teleop-pose-v2-benchmark.log`). This bounded sample fits the
50 ms servo period; it is not a real-time scheduling or physical tracking
guarantee. The separate WSL independent benchmark passed all nine declared
cases, including six FK-generated targets, singular inward motion, stalled
feedback/recovery and an impossible 180° orientation request. Its artifact
`output/teleop-artifacts/pose_ik_independent_benchmark.json` includes source and
URDF hashes. Nominal acceleration stayed at or below 0.7 rad/s² for those
cases; emergency feedback/geometry holds can override that nominal limit.

The first live verifier attempt (`teleop-pose-v2-live-r1.json`) stopped before
motion because the offline geometry preflight blocked callback processing.
The verifier now drains callbacks until fresh status and joints are available,
then rechecks ownership and inactivity. Its 0.5-second observation freshness
limit and all production watchdogs remain unchanged.

The corrected live ROS-input/Isaac run passed all stages
(`teleop-pose-v2-live-r2.json`). Each arm requested 50 mm inward independently
from the default simulation posture. At the eight-second left phase endpoint,
left progress was 98.58% and measured 3D error was **8.12 mm**; it continued
settling while the right phase ran. Right progress was 99.84% with **0.272 mm**
error. The held left tool moved 6.65 mm while finishing its preceding request;
these are bounded tracking results, not instantaneous perfect following.

The same run then requested 0.15 rad orientation changes independently along
each measured arm's preflighted positional-null direction. Measured angular
errors were **0.00309/0.00241 rad** (about **0.177°/0.138°**), with position
drifts **0.326/0.078 mm**. This tests feasible small rotations, not arbitrary
six-dimensional control. Silence produced an actual inactive backend command
**0.271 s** after the last input timestamp; held input did not resume motion,
and explicit release/reclutch did. All phases ended with a measured hold.

The production Unity clutch/frame conversion/TCP path then passed its separate
PlayMode integration test at **18:41:16–18:41:37 UTC**, **20.916 s** test duration
(`output/teleop-pose-v2-playmode.xml`). Common body translation and 60° yaw each
retained fresh active command publication while all eight targets stayed
constant within 1e-6 rad. Independent inward controller movement produced
measured left/right joint changes of **0.638/0.483 rad**, followed by hold,
held-trigger rearm rejection and a no-jump new clutch. This wire test started
from the preceding live test's held pose; the ROS test above started near the
default straight-arm posture. A separate native offline repeat of the nine
independent benchmark cases also passed.

Final read-only stack health passed the 10 s idle / 25 s stereo demand / 5 s
idle lifecycle check (`teleop-pose-v2-final-health.json`): **400/400** commit
gates remained open, maximum registration age **0.668 s**, and **37** stereo
frames arrived during display demand (observed **1.48 Hz**, maximum ROS-header
age **0.339 s**). The requested display cap remains 5 Hz; that is not the
measured delivered rate. Both head-camera subscriptions were released after
display demand ended. WebRTC visually showed the running K1/table scene.

The user's main Unity editor and Play mode were not operated during this
correction. Scripted validation is distinct from native Meta trigger delivery;
the prior native bilateral-input limitation below still applies. No physical
headset or robot test was performed.

## Historical bimanual position IK — 2026-09-18

These results describe the original protocol-v1 position servo. Subsequent
inward-motion tests exposed a straight-arm singularity and alternate-branch
joint-limit failure that the small forward-motion checks below did not cover.
The replacement protocol-v2 pose implementation is documented in
[arm teleoperation](arm-teleoperation.md); these older passes must not be
interpreted as validation of that replacement or of unrestricted arm motion.

The simulator now unlocks both four-joint arms. A paired index-trigger clutch
captures each measured tool position and controller reference; subsequent
controller displacement drives bounded position IK. Release, input expiry,
tracking/focus loss and cancellation hold both arms and require release before
rearming. The earlier single-arm reaching results below describe the earlier
right-arm mode; they do not establish bimanual control performance.

The native Ubuntu Jazzy control suite passed **117/117** cases, including actual
GTSAM, IK, input freshness/replay guards, planning exclusion and lifecycle
checks (`teleop-control-jazzy-20260918.xml`). The simulator/relay suite passed
**49/49**, including actual controller JSON emission into the relay contract.
The Unity EditMode suite passed **45/45** (`teleop-final-editmode.xml`), covering
clutch/release, frozen yaw and frame conversion, trigger-intent exclusion,
recenter/focus/tracking faults, disabled-component cancellation, and existing
clock/preview/image regressions.

In the running **Meta XR Simulator**, both native LTouch and RTouch controllers
were connected and position/orientation tracked. Unity sent fresh paired idle
packets through the real TCP endpoint, and the backend reached
`teleop_ready=true`. The focused native input trace
`xr-input-trace-20260918-154038-250.json` contains 177 samples but no observed
trigger presses. Native two-trigger activation and controller-driven motion
remain unverified: installed v205 exposes a shared T trigger binding and
selects left or right action input, while the available UI automation does not
provide sustained key-down/up control. “Use same input for both sides” copies
the input type; it is not a bilateral trigger command. Scripted tests below
are distinct from native input verification. No physical headset or robot
was used for this implementation's tests.

The existing WebRTC client reconnected after the eight-joint Isaac restart and
displayed the live K1/table scene.

The scripted Unity PlayMode wire/physics check passed **1/1** at
15:51:00–15:51:11 UTC (`teleop-scripted-playmode.xml`, 10.43 s test case).
It used the production clutch/frame conversion and TCP bridge, checked left
and right input independently, release of either trigger, prevention of
reclutching before full release, and capture of new references. Backend joint
targets stayed constant to within 1e-6 rad during stationary zero-displacement
clutches. A read-only observer recorded 198 input packets, 146 backend
commands, two clutches/two releases, and maximum measured FK displacement
of 28.33 mm left / 24.09 mm right (`teleop-scripted-unity-20260918.json`).
These are scripted input results, not native trigger results or latency
measurements. The initial test's 0.015-rad measured hold allowance failed at
0.01736 rad; the final test separately checks exact command constancy and
allows 0.03 rad of physical drive settling. No production control limit changed.

The final synthetic ROS input test passed all movement, hold and recovery
stages (`bimanual_synthetic_ros_validation_r3.json`). From its declared current
pose, each arm requested a 20 mm forward displacement plus 3 mm lateral and
5 mm vertical displacement. Measured FK errors were **0.883 mm left** and
**0.718 mm right**, with the other tool held within 0.329 mm. Input silence
caused an inactive backend command and measured hold; held input did not
resume motion, and an explicit release/reclutch did. Seven verifier tests
also passed on Ubuntu (`teleop-verifier-tests-20260918.xml`).

The first verifier attempt stopped before motion while DDS endpoint names
were unresolved. The second completed both arm motions and release, but
its 0.30-second status assertion fired before the next status update despite
an observed inactive backend command 0.234 seconds after silence began.
The final verifier allows 0.50 seconds for observing status and separately
checks the actual timeout command's source/arrival time. These corrections
affect the test harness only; the 0.25-second controller lease is unchanged.

## World controls and head stereo — 2026-09-17

The camera toggle and monitor now have a fixed world pose. Controller-ray hover
and Trigger select the toggle through the existing input route. Moving the head
or demo owner does not move the canvas. Image and UI materials are separate:
the image shader selects the matching half of an atomic left/right head pair,
while labels and controls render uncropped in both eyes. The shader now uses
URP's stereo transforms and fragment eye index; scene depth is respected.

Two camera lifecycle/world-position PlayMode tests passed. The 36-test EditMode
suite passed across its initial run and focused GPU rerun. GPU readback checks
the actual two-layer single-pass-instanced render target: red source reaches
the left layer, green reaches the right, and ordinary UI is visible in both.
The mono preview/ROS-row test initially exposed the test harness's missing D3D
render-texture projection conversion; correcting its camera matrices left the
expected pixels unchanged and all three GPU tests passed. Evidence:
`output/head-stereo-playmode.xml`, `output/head-stereo-editmode.xml` and
`output/head-stereo-gpu.xml`. These are desktop D3D11 checks, not physical Quest
or Android multiview observations.

The ARM64/Vulkan Quest development APK built successfully at 17:45 UTC:
`output/DeicticK1.apk`, 124,627,064 bytes, SHA-256
`256D3B30F97DD9FF124BE7ACDAC41C0507D86BEE8B75C48323A1F963C8CD92FE`.
The APK is available but was not installed: the user chose Quest Link for the
physical check. In the running Unity demo with Meta's OpenXR runtime and XR
Simulator inactive, the user confirmed all three checks: the button stays fixed
while moving the head, the right-controller ray can hover/toggle it, and the
robot image is visible through both eyes. This is physical Quest Link validation;
the standalone Android APK and physical K1 cameras were not tested.

The simulated pair uses the vendor head optical orientation and a provisional
64 mm baseline with 20 mm forward standoff. This is an assumed simulation
configuration, not physical camera calibration. Wrist/headset registration
remains separate. Live checks are recorded below when completed; the first
launch exposed duplicate visual/collision link names (fixed by selecting the
rigid link), and the first image probe found no head frames despite healthy
registration. Keeping both render products enabled across deferred frame
callbacks resolved this. A bounded acquisition window, reference watermark and
first-observed renderer-reference clock prevent old frames being relabeled fresh.
The failed probes do not count as camera validation.

The subsequent 20-second live probe passed all checks: 33 exact left/right
640×480 source pairs with matching CameraInfo, and 27 matched 960×360 relay
frames. Received PNGs are actual Isaac pixels; each relay half exactly matched
the resized corresponding source. Source image age was at most 0.269 s and
relay age at most 0.408 s. The level, forward-facing head sees the floor/horizon;
the camera does not dynamically aim at the tabletop. See
`sim/artifacts/head_stereo_live_check_lifecycle_20260917/validation.json` and
the adjacent images/headers.

The viewer-only probe exposed dropped best-effort RGB fragments. Switching both
relay eye subscriptions to reliable delivery with one-frame queues improved
received rate from 0.72 to 1.52 frames/s in the measured 25-second display phase;
5 Hz remains a ceiling, not an achieved rate. The final 10/25/5-second
idle/display/idle observation retained `can_commit=true` in all 400 status
samples, with maximum registration age 0.747 s. All 38 received display frames
were at most 0.369 s old, and both source subscriptions disappeared after the
viewer closed. Evidence:
`sim/artifacts/head_stereo_health_reliable_20260917.json`.

The full Unity/SSH/ROS/Isaac integration passed with a 960×360 SBS frame, followed
by 15 new frames in ten seconds, maximum displayed age 1.183 s, and zero stale
samples (`output/head-stereo-sustained-unity.xml`). The inspected desktop image
is `output/unity-head-stereo.png`; a mono desktop camera displays the left eye.
WebRTC was simultaneously connected and its client logged success at
18:03:06 UTC. These checks do not establish physical two-eye presentation.

Regression checks passed: 35 relay cases, 27 simulator cases, and four real
Linux socket tests. The latter reproduce TCP TIME_WAIT after shutdown and
confirm the launch preflight can now restart through it while still rejecting
active TCP/UDP listeners. No unrelated workload was stopped and no physical
robot command was sent. The user clarified that the physical check uses Quest
Link; Unity was started with the simulator inactive and the Meta OpenXR runtime.
The demo remains in Play mode for the user; the owned remote stack is running.

## WebRTC startup recovery — 2026-09-17

The Windows client reached the signaling server but stayed black and reported
`No offer is received from remote`. Inspection found yesterday's K1 container
still running alongside a newly launched K1 container, with two TCP 49100
listeners. The ROS endpoint, controller, registration provider, and relays also
had duplicate instances; the new endpoint process survived its listener thread's
failed port bind.

A simulation cancel/hold was sent before clearing the identified demo processes.
Removing the stale simulator alone did not recover the affected stream, so the
demo was restarted once with one instance of each component. Other users' jobs
were left untouched. At **16:03:37 UTC**, the client logged `Stream started` with
`status=success`; native window inspection then showed the K1 and tabletop in the
Isaac application. ROS status showed markerless/GTSAM tracking, confidence
0.989, registration age 0.489 s, `can_commit=true`, and `executing=false`.
This verifies stream recovery and an idle tracking sample, not a new motion test.

Both launchers now reject occupied ports before starting services. Seven isolated
simulator-launcher checks and five bridge-launcher checks passed, including
occupied ports and the absence of child launches after rejection. The checks
detect existing listeners; they are not atomic locks for simultaneous launches.
The updated scripts were copied to the workstation and parsed there. Recovery
logs are under the workstation repository's `.codex/*-recovery.log`.
The recovered services were launched in the background; their verified process
IDs and log paths are recorded in `.codex/stream-recovery-state.json` on the
workstation. Check current process identity before using those recorded IDs.

## Environment

- Unity 6000.6.0f1 / Meta XR SDK 205.0.0, Windows host; original OVRCameraRig scene preserved.
- Supplied ROS-TCP-Connector 0.7.0-preview and URDF-Importer 0.5.2-preview embedded in the project. ROS 2 endpoint release 0.7.0.
- Ubuntu WSL / ROS Lyrical used for local checks. Remote Ubuntu 24.04 / ROS Jazzy, Isaac Sim 5.0 container, Quadro RTX 6000 24 GB, driver 570.211.01.
- GTSAM 4.2.2 runs in an isolated Python 3.12 / NumPy 1.26 environment. NumPy 2 is rejected by the control startup because the installed GTSAM binary was incompatible.
- Tests and generated screenshots are under ignored `.codex/`, `output/` and `sim/artifacts/`; commands and source files are kept separately from generated evidence. No Git commit has been created by this session.

## Checks completed

| Check | Result and scope |
|---|---|
| Unity asset generation | K1 hierarchy and meshes generated from pinned URDF, demo scene compiled in a separate validation copy without closing the user's editors. |
| Unity frame/input/wire/image/feedback gates | **26/26 EditMode tests pass** in `editmode-feedback-verified-results.xml` (0.252 s): the prior 23 frame/input/wire/lifecycle/binding/image tests plus three feedback cases. The new cases verify base-local path and exact committed-target reprojection under changed alignment, honestly labeled endpoint fallback, pending-goal retention, cancel clearing, no targeting colliders and resource cleanup. An earlier run's two cleanup-only failures came from manually starting an EditMode component without invoking its paired destruction callback; the test lifecycle was corrected. Production reprojection assertions passed in both runs. |
| Unity clock-quality regression suite | **32/32 EditMode tests pass**, `editmode-clock-quality-results.xml` (0.255 s): the existing 26 cases plus six clock cases covering asymmetric delay, rejected slow/invalid/uncorrelated replies, recent minimum-RTT selection without falsely refreshing sample age, clock steps and resets. Source now rejects RTT above 80 ms to keep modeled half-RTT uncertainty below the controller's unchanged 50 ms future-goal allowance. The subsequent five-target traversal passed, and the current 20:12:47 APK contains this change. |
| ROS core and lifecycle | **27/27 pass** in `control_preview_binding_review_tests.xml` (7.964 s): 11 core, 1 lifecycle and 15 execution-contract cases. These include actual GTSAM iSAM2, malformed/non-object JSON rejection, confidence/freezing, IK/collision checks, graph reset, exact execute tokens, preserved original trajectories, cumulative drift, rotation-induced target displacement, and runtime NaN/infinite tolerance rejection. The preceding preview-binding suite passed 25/25; the earlier shared joint-planner revision passed 20/20. |
| Nonblocking planner regression suite | **44/44 core/lifecycle/execution tests passed** in `control_async_planner_tests.xml` (12.12 s), after the reliable-camera retry exposed joint-feedback starvation during synchronous planning. Requests copy their inputs; one worker and one latest pending request bound the queue. Cancellation/deadlines invalidate obsolete work, and only the ROS executor accepts a result after current freshness, epoch, original-target mapping and start-pose checks. Finite positive `plan_timeout` is required before planning and execution. Existing quality/freshness gates are unchanged. The later partial live traversal is recorded below. |
| Camera display relay | 15/15 tests pass for format/stride validation, bounded aspect-preserving resize, unchanged headers and demand-driven source subscription. Five live Isaac frames were 480×360 RGB8 (518,400 bytes each), matched their original source stamps/frame IDs, and arrived 0.079–0.207 s after the preserved publication stamps. Source subscription was removed when the observer exited. This is display transport evidence, not physical-camera latency. |
| Unity camera-view lifecycle | **1/1 PlayMode pass** in `playmode-camera-lifecycle-results.xml` (16:41:57 UTC). The test invokes the button callback, checks ray/button intersection, missing-feed handling, return to User view, and disable/re-enable subscription cleanup. It is not a manually operated Quest-controller test. |
| Actual ROS image in Unity UI | **1/1 PlayMode pass** in `playmode-camera-live-results.xml` (16:32:51 UTC): actual wrist stream rendered at **480×360**, reported frame age **0.207 s**, row orientation checked, and feed hidden/released on toggle/disable. |
| Updated camera overlay | **2/2 PlayMode pass** in `playmode-camera-overlay-results.xml` (16:57:20 UTC), rerunning the lifecycle and actual-ROS image checks after the overlay-shader change. The actual frame was 480×360 with reported age 0.214 s; `output/unity-robot-camera-view.png` shows the readable image, labels and return button. Physical rendering remains untested. |
| Camera UI after feedback update | **2/2 PlayMode pass** in `playmode-camera-feedback-results.xml` (1.064 s), using the live matte-floor scene's wrist stream. The frame was **480×360**, reported age **0.110 s**; `output/unity-robot-camera-matte-view.png` was visually inspected. This reruns lifecycle/display behavior, independently of whether that fixture qualifies for markerless registration. |
| Camera UI with full live stack | **2/2 PlayMode pass**, `playmode-camera-full-stack-results.xml`, **19:40:29–19:40:30 UTC**, run **1.455 s**. Actual Isaac wrist RGB rendered at **480×360**, reported age **0.276 s**, while visual registration and WebRTC remained active. `output/unity-robot-camera-full-stack.png` was visually inspected. This verifies short toggle/feed integration, not sustained bandwidth endurance. |
| Registration geometry | 11 tests pass, including outliers, depth validation, planar/nonplanar PnP, padded RGB and endian handling. |
| Registration GPU resource guard | **8/8 tests pass** in `registration_resource_tests.xml` (0.757 s), covering invalid fractions, CUDA-device resolution, allocation ordering and unchanged CPU behavior. The actual PyTorch 2.7 CUDA constructor and all six subsequent development configurations also ran. The 25% cap constrains this process's PyTorch allocator, not all driver/context memory. |
| Learned features | Real pretrained SuperPoint/LightGlue on a generated calibrated image pair: 459 matches / 434 inliers at 512 keypoints, 0.834 px median residual, 1.05 mm translation and 0.000787 rad rotation error against generated truth; 1.17 s CPU at 640×480. This is not a physical or Isaac-camera result. |
| K1 model/trajectory | 6/6 pure-Python tests pass. All 35 recommended target center/surface points solved from both tested rest poses. |
| Isaac camera pairing | **10 total simulator source tests pass**, including the 6 model/trajectory cases and 4 camera cases. They check cached RGB/depth/capture pose, snapshot ownership, rejection of missing or mismatched render references, and native mixed headset/wrist resolutions with depth-shape validation. The revised pipeline also produced coherent live frames. |
| Bounded Isaac physics | 360 loop steps, 43 rendered wrist frames; maximum settled joint error 0.007732 rad for the commanded four-joint target. |
| Measured tool frame | 20 timestamp-matched Isaac joint/tool samples: maximum URDF FK versus measured tool position difference 1.85×10⁻⁷ m. |
| ROS TCP with mock controller | Real wire handshake, stale-goal rejection, exact preview stamp, execution success and cancel tested; final tool FK error 0.323 mm. Mock-controller results are separate from PhysX results. |
| Unity → ROS → Isaac | Full PlayMode test passed in 11.378 s with **2.42 mm** tool error, and the later exact-preview-token version passed in 10.325 s with **2.46 mm**, against `(0.12,-0.25,0.02)` m. Both checked correlated preview, execution, measured arm motion and measured completion under known synthetic alignment. Clock exchange RTTs were 32.2 and 66.4 ms; these are not motion latency measurements. |
| WebRTC | Actual Isaac K1 scene displayed in the installed Windows client. A campus NAT address change later interrupted the connection; both observed source addresses were added to scoped workstation firewall rules. |
| Meta XR Simulator | Launched from the generated Unity scene and rendered the robot/controller rays. **Meta Quest 3S** was observed active in Settings > Simulated Headset after restarting Play mode; the first-run firewall prompt no longer appeared. Manual controller-button reaching and physical Quest behavior remain unverified. |
| Stationary MR rig in native simulator | The inherited first-person locomotor drove the rig to Y=−1046.899 m. The generated scene now disables its `Locomotor` subtree, uses FloorLevel tracking and turns off the legacy headset emulator. Native render-phase probe `xr-runtime-probe-20260916-174104-632.json`, frame 2235, records rig `(0,0,0)`, head `(0,1.7,0)`, OpenXR Floor origin, and finite in-view projections of the table, target and all toggle corners in both eyes. This validates the corrected pose/projection, not an actual toggle input. |
| Native keyboard camera toggle | The user clicked the Unity Game view and pressed **V**, then confirmed that the robot-camera panel appeared; the assistant subsequently observed the live wrist feed and **Robot camera → User view** button in the native Unity display. This is a user-performed one-way native keyboard check. Earlier automated key attempts produced no V held/edge state in two 10-second traces despite enabled keyboard, Game focus and DynamicUpdate; no input-runtime change was made to accommodate that automation limit. A physical/native controller-pointer click and the reverse keyboard transition are not established by this check. |
| Quest APK | The current Android ARM64 IL2CPP development build succeeded at **20:12:47 UTC**, including the clock-quality correction, exact committed-target/base-local preview feedback and the prior lifecycle, overlay shader, Android User-view passthrough, geometric preview binding and stationary-rig fixes: **`edu.kent.atr.deictick1`**, **124,631,496 bytes (124.6 MB)**. SHA256 independently verified: `e37ba9f14f39a6fe22124709c1a652e7ab86a87aad9380182686d05c14d6c085`. AAPT verified version 0.1.0, `arm64-v8a`, INTERNET and optional `com.oculus.feature.PASSTHROUGH`. No physical installation/run is claimed. |
| Static learned-registration validation | After selecting filter 0.7 on a development frame, three newly acquired frames passed unchanged gates: confidence 0.870/0.785/0.783; worst five-target errors 3.84/3.46/1.04 mm. All use one deliberately textured static simulation fixture. |
| Live learned-registration candidates | **30/30 observations passed**, zero frontend failures over 60 s; minimum confidence 0.745568, worst independent target-mapping error **4.102 mm**, source-header-to-arrival age **2.012–2.264 s**, maximum result interval 2.100 s. Candidate outputs were isolated from the controller during this check. |
| Fixed-world markerless → GTSAM → Isaac | **1/1 PlayMode pass** in `playmode-markerless-results.xml` at 16:48:57–16:49:06 UTC: **1.70 mm grounding error**, **2.27 mm measured tool error**, test-case duration 8.983 s, clock RTT 30.1 ms. The test asserted `markerless`/`gtsam_isam2`, selected a fixed Unity-world target independently of estimated alignment, observed real joint motion, and required fresh post-execute tool feedback. This is one reach in the declared textured simulation fixture, not a paper task-time or physical accuracy measurement. |
| Visual loss and maintenance recovery | The subsequent monitor recorded zero-match failures and closed commits after the wrist view lost shared content. An explicit checked joint-space maintenance return restored the rest viewpoint and registration: planned 6.612 s/398 samples, measured maximum final joint error 0.00958 rad. This was an operator-invoked simulator maintenance action, not automatic markerless recovery. Three selected shared-planner tests passed in `maintenance_plan_tests.xml`. |
| Provider interruption/recovery | In a separate idle test, only the visual provider was paused for 6 s. Commit gates first closed at registration age **4.544 s** with `registration_stale`; the transform stayed unchanged and no preview/execution was active. Resuming the provider recovered a fresh accepted observation in approximately **2.70 s**, confidence **0.91268**, age **2.099 s**. Evidence: `registration_liveness.json`. |
| Optional reach-balanced mount, static stage | Filter 0.7 failed development confidence (0.5605). Stricter 0.8 passed development and was frozen before three new captures; 0.9 was not tried. All three passed unchanged gates: confidence **0.8241/0.9119/0.9108**, maximum five-target errors **7.96/1.36/1.89 mm**, CPU **1.88–1.91 s**. Evidence: `balanced_fixture_registration_validation08.json`. |
| First balanced-mount integration attempt | `playmode-markerless-balanced-results.xml` records a **pre-motion failure** at 17:12:05–17:12:08 UTC: grounding 1.31 mm, but a healthy optimizer version update discarded the preview before execute. The mapped target changed only approximately 0.0481 mm. This failed attempt is retained separately from successful runs. |
| Balanced reach with original-preview binding | **1/1 PlayMode pass**, `playmode-markerless-binding-results.xml`, 17:20:11–17:20:27 UTC: **0.91 mm grounding**, **2.78 mm measured tool error**, test duration **15.448 s**. The test deliberately waited 4.5 s for preview review while registration continued before executing the original approved trajectory. Sustained postreach registration is a separate check; the subsequent 30-second monitor had commits open in only 20/300 status samples and only one new accepted graph key. Forearm occlusion substantially reduced shared visible texture. |
| Bounded two-pose CUDA comparison | **No configuration passed both poses** in `balanced_gpu_development_selection.json`. All six combinations of 512/1,024/2,048 features and filters 0.8/0.9 failed postreach geometry or confidence, while five passed at rest. Synchronized warm processing took **69–98 ms**, PyTorch peak allocation/reservation **226,789,888/249,561,088 bytes**, and sampled whole-process GPU usage peaked at **438 MiB**. Faster processing did not fix the current view's visual-quality failure. No GPU profile was selected or deployed. |
| 10 cm bracket, fresh two-pose validation | A bracket position selected by an independent mesh-occlusion calculation was tested with otherwise unchanged camera/scene settings. Development-only selection chose CUDA/1,024 features/filter 0.9, then six new frames were acquired after freezing. **Rest 3/3 passed; postreach 0/3 passed** (`bracket10_gpu_frozen_validation.json`). Postreach confidence was **0.556/0.270/0.391**, below 0.7; maximum five-target errors were 9.42/24.19/8.21 mm. Warm processing took **76–85 ms**. The development success did not repeat at postreach, so no live profile was enabled. |
| Projection-oriented mount, six-frame development | A further declared virtual bracket was evaluated using **three rest and three end development captures** for every one of the six predefined CUDA settings. None passed all six (`projection_gpu_development_selection.json`): every setting passed the three rest captures, but at most one end capture passed. All 36 outcomes are retained; warm processing was **62–84 ms**. No configuration freeze, new held-out stage or live profile followed this comparison. Physical bracket feasibility is unverified. |
| Rotation and matte-floor diagnostics | On the same projection development data, fixed CUDA/1,024/filter 0.8 tested wrist rotations 0/90/180/270 degrees, mapping matches back to original pixels before unchanged PnP. The unrotated setting passed only 3 rest frames; every rotated case failed (24 outcomes). A separately declared matte-floor-only ablation then repeated the six-configuration/six-frame protocol: none passed all six, although two settings passed 5/6. Warm processing was **70–89 ms**. Both failures are retained; neither yielded a live profile. |
| Read-only depth-boundary diagnosis | Fixed CUDA/1,024/filter 0.8 on three matte-floor end development frames gave **31/32/33 fitted outliers**. Only **2/2/4** exceeded 3 px distance from the true epipolar line. At the same continuous headset-pixel ray, valid 3×3 neighboring depths plausibly explained **18/22/15** outliers; 5×5 explained **19/22/15**. These oracle diagnostics used simulator truth only after ordinary estimation and did not correct depth, select matches or create accepted observations. Evidence: `matte_depth_boundary_diagnostic.json`. |
| Native headset-resolution frozen validation | Actual headset RGB/depth rendered at **1280×960**, wrist at 640×480, with correspondingly calibrated K. The first predefined candidate, **CUDA/512/filter 0.8**, passed six development captures and then **all six new post-freeze captures**. Rest confidence **0.987/0.988/0.990**, maximum five-target errors **2.75/1.24/3.22 mm**; reached-pose confidence **0.956/0.956/0.955**, errors **0.97/0.95/0.73 mm**. Warm processing **95–110 ms**; development whole-process sampled GPU peak **1,196 MiB**. Nearest-depth sampling and all geometric/confidence gates were unchanged. Evidence: `head1280_gpu_frozen_validation.json`. |
| Native-resolution live rest health | Over **35 s**, **350/350 controller statuses** permitted commits and **66/66 registrations** passed, with zero failures. Minimum confidence **0.981509**, source-header delivery age **0.133–0.340 s**, maximum independent five-target error **5.944 mm**. This used the actual CUDA provider and GTSAM with the original **one-second** registration lifetime. No goal or motion was sent. Evidence: `head1280_live_rest_control.json`, `head1280_live_rest_registration.json`. |
| Native-resolution fixed-world markerless reach | **1/1 PlayMode pass**, `playmode-markerless-head1280-results.xml`, 18:58:35–18:58:49 UTC: **2.16 mm independent grounding**, **4.51 mm measured tool error**, Unity test-run duration **13.744 s**, clock RTT **81.2 ms**. The preview was reviewed for 4.5 s before execution. `output/unity-markerless-head1280-preview.png` was visually inspected for the exact committed-target marker, path, ghost and aligned state. |
| Native-resolution center postreach health | Dedicated **30 s** recording: **300/300 statuses** permitted commits, graph keys advanced **360→416**, no active execution or registration failures; **57/57 observations** passed. Minimum confidence **0.9161**, delivery age **0.173–0.471 s**, maximum independent five-target error **2.179 mm**. Controller registration age reached **0.972 s**, below the unchanged one-second lifetime. This confirms maintained registration during this reached-pose window; the subsequent five-target attempt failed before its first motion, as recorded below. Evidence: `head1280_center_postreach_control.json`, `head1280_center_postreach_registration.json`. |
| First native-resolution five-target attempt | **Failed before motion on target 1**, `playmode-markerless-five-targets-results.xml`, **19:01:22–19:01:29 UTC**, Unity run **6.834 s**. Independent grounding was **0.40 mm**, but the preview was invalidated during the 4.5-second review and Execute was blocked. No target was executed in this traversal. The preserved stream confirms registration_stale at observation ages 1.063835 and 1.164125 s; a fresh observation restored alignment but correctly left the preview cleared. Epoch 0 was unchanged, and there was no geometric-binding failure. This does not alter the separately passed center-reach and 30-second reached-pose results. |
| Scheduling and capture-cadence follow-ups | A provider-only 0.50→0.30 s schedule change still had **5/351 stale status samples**, although **63/63 estimates passed**. Under concurrent GPU load the source delivered approximately 1.86 camera pairs/s. Reading each actual render (`frequency=-1`, existing render cadence, 15 Hz wall cap) increased source throughput to approximately 3.2–3.8 Hz, but best-effort transport still had result gaps up to 1.285 s and **6 stale samples**, despite **101/101 estimates passing**. Geometry, matcher, quality gates and the one-second lifetime were unchanged. Evidence: `head1280_cadence03_*.json`, `head1280_rendercadence_*.json`, `capture_cadence_comparison.json`. |
| Reliable camera subscriptions | The explicit native-resolution profile uses `reliable_latest`: RELIABLE, KEEP_LAST depth 2, VOLATILE, on all seven camera/image-info/pose subscriptions. The default remains sensor-data best-effort. Seven policy cases plus four existing image-message cases passed on ROS Jazzy. Following deployment, **35 s passed: 109/109 visual estimates and 350/350 gate-open statuses**, maximum controller age **0.8992 s**, graph keys **3904→4012**. Delivery age was median **0.379 s**, p95 **0.552 s**, maximum **0.587 s**; maximum result interval **0.400 s**. WebRTC remained on and the unrelated user's GPU workload was untouched. Evidence: `head1280_reliable_control.json`, `head1280_reliable_registration.json`, `head1280_reliable_feedback_live.json`. This is a bounded idle window; the subsequent traversal encountered a separate planning issue below. |
| Five-target retry with reliable cameras | **Failed before motion/preview acceptance on target 1**, `playmode-markerless-five-targets-reliable-results.xml`, **19:37:38–19:37:51 UTC**, Unity run **13.3095 s**; first-target grounding **2.35 mm**. Synchronous planning blocked joint-message processing and triggered `joint_feedback_stale` at 19:37:40.219/40.220. Registration remained valid (age **0.879/0.880 s**, confidence **0.989**). Tracking recovered at 19:37:40.284, but the preview correctly stayed cleared. This is distinct from the earlier visual-registration timeout; no target was executed. The nonblocking correction passed the regression suite above and enabled the later two-target partial traversal without relaxing timeouts. |
| Partial traversal after nonblocking planning | `playmode-markerless-five-targets-async-results.xml`, **19:55:34–19:56:14 UTC**, **39.6577 s**, completed **targets 1 and 2**, with grounding **1.11/0.15 mm** and measured tool errors **2.16/4.45 mm**. Target 3 grounding was **0.22 mm**, but `goal_timestamp_out_of_range` rejected it before motion. The overall five-target test failed. Across the associated 150-second monitor, **1507/1507 gates stayed open and 457/457 estimates passed**, maximum controller age **0.8672 s**; joint-feedback starvation did not recur. The accepted clock sample had RTT 188.2 ms, permitting up to 94.1 ms offset uncertainty against the 50 ms future-goal allowance. The clock correction above was tested before the later successful traversal; this partial run remains a failed five-target attempt. |
| Target-2 reached-pose health | Following that partial traversal, a separate **30-second** window passed **301/301 gate-open statuses and 92/92 estimates**, with no failures, maximum controller age **0.848 s** and graph keys **726→817**. Evidence: `head1280_async_target2_post_control.json`, `head1280_async_target2_post_registration.json`. This supports continued registration at target 2, not completion of targets 3–5. |
| Complete five-target traversal after clock correction | **Passed 1/1**, `playmode-markerless-five-targets-clock-results.xml`, **20:03:28–20:04:07 UTC**, **39.1761614 s**. Targets 1→5 had independent grounding errors **0.17/0.78/0.21/0.37/0.66 mm** and measured tool errors **2.62/2.83/3.93/3.56/4.87 mm**. Every target had 4.5 seconds of preview review, measured arm movement and fresh post-execute feedback. The initial arm pose was the previous target-2 reached pose, not neutral rest; subsequent targets started from the prior measured pose. This stationary-headset traversal is not paper T2. |
| Full traversal and final-pose live health | The associated **150-second** observer recorded **1516/1516 gate-open statuses and 453/453 passing estimates**, zero failures, maximum controller age **0.928550 s** and graph keys **1599→2051**. The dedicated **30-second target-5** observer recorded **299/299 gate-open statuses and 92/92 passing estimates**, zero failures, maximum controller age **0.907099 s**, confidence at least **0.929153** and maximum independent diagnostic five-target error **3.947 mm**. Its graph keys advanced **2026→2116** while no execution was active. Evidence: `head1280_five_target_clock_{control,registration}.json` and `head1280_five_target_clock_post_{control,registration}.json`. These bounded windows overlap and are not summed; the separately retained `segment2` continuation also overlaps. The one-second lifetime and all quality gates remain unchanged. |

## Integration corrections

Native workstation DDS uses `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. The WSL `LARGE_DATA` workaround partitioned native discovery; default shared memory failed to deliver some messages across the container's private IPC namespace. UDPv4 delivered actual Isaac joint states.

The host clocks differed by approximately two seconds. Unity performs a four-timestamp exchange and applies the offset to outgoing timestamps; neither machine's clock was changed. The original 250 ms RTT acceptance allowed too much uncertainty for the backend's 50 ms future-goal bound, causing the target-3 rejection above. The tested source now accepts at most 80 ms RTT, chooses the minimum-RTT sample from a recent 10-second window, retains that sample's own freshness age, retries rejected samples promptly, and resets on pause/resume or a detected clock step. The corrected source passed the five-target traversal and is included in the current 20:12:47 APK.

Isaac RGB/depth/pose synchronization uses the same cached renderer reference, but its shared ROS wall-clock header is assigned when that pair is retrieved for publication. Reported image/registration ages start at that source header; rendering-to-publication delay or physical exposure latency was not independently measured. The clock-exchange RTT, display-frame age, registration age and whole test duration are distinct measurements.

The supplied connector's ROS 2 empty-message serialization omitted the dummy byte required by native ROS 2. This affected both execute and cancel and is patched in the embedded package. ROS 1 behavior remains unchanged.

The Android build exposed invalid Any-platform metadata in the supplied URDF importer's native Assimp plugins. Desktop binaries are now restricted to their actual OS/architecture and excluded from Android; Quest uses pre-generated K1 meshes. Opaque and transparent overlay materials are retained as Resources assets so their shaders and transparency variants survive player-build stripping.

The latest camera-view source adds a Resources-backed `ZTest Always` UI shader so nearby table geometry cannot occlude the head-locked toggle. Its two desktop PlayMode checks and robot-panel render passed. Android User view now enables passthrough even with a synthetic robot/workcell; editor synthetic mode keeps its rendered view. `DemoSetup` also sets `OVRProjectConfig.insightPassthroughSupport=Supported` instead of the previous `None`. These presentation changes and geometric preview binding are included in the current APK. Physical passthrough remains unvalidated and is separate from the disabled-by-default camera-publishing adapter.

Continuous healthy optimization exposed an overly strict preview-version check. The revised binding keeps the original preview transform, base target and exact approved trajectory, permits at most 5 mm cumulative remapping of that target within the same graph epoch, and rechecks the bound before execute. It never rebases the original comparison after small updates. Origin changes, excessive displacement, backend invalidation, stale feedback/registration, plan age and exact execute-token checks still apply. Unity now follows epoch/backend invalidation rather than invalidating on every optimizer version. The 23 Unity and latest 27 controller tests pass, including rejection of runtime nonfinite tolerance changes. The updated live reach also passed after a deliberate 4.5-second review interval with continuing registration; the current APK contains this binding.

MR preview feedback now stores the planned FK path in robot-base coordinates so later alignment updates reproject it together with the ghost robot. A separate collider-free marker retains the exact committed base point, rather than replacing it with the IK endpoint; a preview without a local committed point is labeled **Planned endpoint**. Empty old wire previews preserve the pending request, while cancel/execute and other preview invalidation clear the graphics and committed point. These focused changes passed the 26-test suite and remain included in the current APK; the later clock revision passed all 32 tests. Both camera-view PlayMode checks also passed again with the actual live wrist stream.

Native simulator inspection exposed an additional scene issue hidden by the non-XR integration test: the supplied comprehensive rig's `FirstPersonLocomotor` applies its own gravity, even though its Rigidbody is kinematic. It continuously copied the falling character-controller height to the tracking origin. `DemoSetup.ConfigureStationaryRig` now deactivates only the generated scene's `Locomotor` subtree, selects FloorLevel and disables legacy Ctrl/mouse headset emulation. The original sample scene and SDK prefabs remain unchanged. The subsequent native probe confirms stable rig/eye height and valid stereo projection. The user later confirmed the V shortcut opens Robot view; native controller-pointer activation remains unverified.

## Remaining validation boundaries

The two earlier Unity-to-robot motion checks used known synthetic camera relationships as GTSAM observations; the later fixed-world test used genuine learned registration and GTSAM. The generated Unity scene/APK still defaults to a synthetic workcell with physical camera publishing disabled; the default bridge-launcher mode is `mock`, and its explicit `synthetic` mode supplies known alignment. Unity's `syntheticScene` flag selects scene/input behavior; it does not itself switch the ROS estimator. Learned markerless control must be selected explicitly. The completed markerless check is one supervised simulation reach; changing headset viewpoints, dynamic targets and physical capture remain unvalidated.

Early learned-registration pairs failed because the renderer's lens-calibration image size differed from its output resolution and the wrist camera lay on the hand mesh. Both issues were corrected. On that earlier fixture, an independent geometry check found 2,399 shared tabletop samples with median cross-view color difference 1/255.

Actual GPU SuperPoint/LightGlue/PnP on the corrected rendered pair recovered alignment with **5.98 mm** translation and **0.006375 rad** rotation error against truth used only for scoring: 87 depth-valid matches, 41 RANSAC inliers and 1.490 px median residual. The cold call took 1.07 s. The default fusion confidence was about **0.138**, below the 0.7 commit threshold, so that observation did not qualify for live control. This successful geometric estimate is separate from the 2.42 mm robot reaching result under known synthetic alignment; the latter is not markerless targeting accuracy.

The revised fixture uses larger distinct printed features, clearer head-camera framing, a leveled rigid wrist mount, and matched cached RGB/depth/CameraParams render references. Filter 0.5 still failed confidence on three fresh frames; filter 0.7 was then selected only on the development frame and frozen before another three acquisitions. The later static and live candidate results are recorded above. Simulator truth enters only after estimation to score five fixed targets. These are fresh acquisitions in one scene, not independent tasks or evidence for arbitrary rooms.

With the mount used by the first successful markerless reach, the final wrist viewpoint lost usable shared scene content and registration failed; new commits closed. An explicit checked maintenance return to rest restored visual registration. A separate idle provider-pause test verified timeout, frozen alignment and recovery on fresh observations. These results demonstrate failure handling and controlled recovery, not automatic viewpoint recovery or continuous operation across the arm workspace. Further mount/view-coverage trials remain separate from this recorded result.

The optional `reach-balanced` fixed side-bracket mount is evaluated separately to improve view coverage during the arm path. Its exact extrinsic and capture provenance are stored with its datasets. Development-only matcher selection chose filter 0.8 after 0.7 failed, with truth reserved for later scoring. Three new rest captures and the subsequent bound-preview reach passed. However, a dedicated 30-second postreach recording had commits open in only 20/300 status samples, with one new accepted graph key; other observations failed unchanged quality/freshness gates. A separate captured postreach pair gave 15/35 inliers, 1.211 px median residual and confidence 0.1122 despite a 2.14 mm truth-scored target error. Geometric visibility, low independently scored error and a completed reach alone do not establish sustained registration. `isaac_balanced_cpu.yaml` preserves the original `isaac_cpu.yaml`/0.7 baseline and changes only the explicit match-filter setting; age, geometric and confidence gates are identical.

A subsequent bounded CUDA comparison declared the rest and postreach pairs as development data and tested feature budgets 512/1,024/2,048 at filter 0.8, followed by 0.9. Selection used only the unchanged geometry/confidence gates; truth was used afterward for error scoring. At postreach, 512/0.8 failed refinement with 11/33 inliers; 1,024/0.8 and 2,048/0.8 produced confidence 0.1306 and 0.1726; all three 0.9 settings had fewer than 12 RANSAC inliers. Thus no setting qualified, and no fresh-validation or live-GPU stage was run. Warm timing was synchronized and excluded model initialization; the 25% resource limit applied only to PyTorch's allocator. These results distinguish an occluded-view quality problem from the slower CPU profile's latency limitation.

The follow-up 10 cm side-bracket fixture changed only the camera's local X offset from 0.06 to 0.10 m, selected using mesh visibility before image matching. The same ordered CUDA comparison first passed both development poses at 1,024 features/filter 0.9 (rest/end confidence 0.895/0.740). Six unique acquisitions after that freeze tested the fixed profile: three rest frames passed with worst target error 8.02 mm, but all three end frames failed confidence. End-frame transform translation errors were 39.78/116.59/28.33 mm, distinct from their 9.42/24.19/8.21 mm target-mapping errors. No thresholds were relaxed or new setting selected using those validation frames, and no live GPU provider was started.

Earlier correspondence-region diagnostics used a projected tabletop-plane footprint and nearby blue pixels. Their labels identify **floor-adjacent image regions**, not proven grid-to-grid identity mistakes: elevated object silhouettes can fall outside that planar footprint and receive the same label. The matte-floor ablation did not by itself qualify a configuration. In the subsequent read-only depth diagnostic, a plausible boundary explanation required a valid local depth span above 2 cm, nearest-depth truth reprojection above 3 px, some 3×3 or 5×5 depth at the same continuous source ray yielding at most 3 px error, and true epipolar distance at most 3 px. Among fitted outliers whose two pixels lay inside the projected table footprints (15/14/16 points), the 3×3 criterion explained 5/7/3; 2/1/2 had epipolar distance above 3 px, which changing depth alone cannot remove. The regions can contain foreground occlusion, and oracle-neighborhood agreement does not prove a match is correct. No runtime depth substitution or relaxed gate resulted.

The subsequent native-resolution trial changed only headset renderer sampling to 1280×960, with aligned native optical-Z depth and fx=fy=640; wrist images and K remained 640×480/fx=fy=320. It retained the projection-balanced mount and matte fixture. The first predefined CUDA/512/filter-0.8 candidate passed all six declared development captures; six new captures after freezing also passed unchanged quality and independent transform/target checks. Separate 35-second live rest health, a fixed-world center reach and 30-second reached-pose health passed under the original one-second lifetime. Those checks used registration/recovery periods 0.5/0.25 s, preserved in `isaac_head1280_cuda_05.yaml`, with a 25% PyTorch allocator cap. These results establish the documented rest/reach/end observation windows; the longer five-target attempt was blocked before its first motion by a freshness timeout.

The 240-second observer recorded 45/2,402 closed status samples, all due to `registration_stale`, despite all 442 delivered observations individually passing. At 19:01:27.684 UTC, age 1.063835 s closed the gate; age reached 1.164125 s at 27.784. Recovery at 27.884 had age 0.418764 s, but a new target commit remained necessary. The preceding observation arrived already 0.616139 s old, and the next delivery came 0.550282 s later. A provider-only change to 0.30 s nominal attempts retained 0.25 s recovery, the existing GTSAM graph and all one-second age/quality gates, but still failed continuous idle health. The unrelated GPU workload that began between the center success and first traversal was left intact.

The subsequent simulator capture change reads every actual rendered camera frame while keeping physics, render cadence, calibration, scene and strict same-frame checks unchanged. This improved source cadence, but best-effort image delivery still produced stale intervals. The selected profile therefore explicitly requests reliable camera subscriptions with a depth-two queue. A 35-second live check then passed all 109 observations and 350 status samples, with maximum controller age 0.8992 s. Kernel UDP drops still increased during an interior sample window (provider +23 datagrams, host receive-buffer errors +45), so this is evidence that reliable delivery maintained freshness during that window, not evidence of lossless transport. No socket-buffer/sysctl change or unrelated-workload termination was used. The fresh five-target retry failed before any motion because synchronous planning blocked joint-feedback handling, while registration remained valid. The nonblocking planner correction passed 44 controller tests and the subsequent run completed two targets with no gate closures before a separate target-3 timestamp rejection. The tighter clock-quality rule passed unit tests and a later complete five-target traversal. Dedicated final-pose health is recorded separately.

At a nominal 3.33 accepted observations/s, the explicit 10,000-keyframe capacity would last about 50 minutes; actual acceptance rate determines this limit, and there is no automatic graph reset. The older 2 Hz example is about 83 minutes.

The opt-in `ros2/src/deictic_registration/config/isaac_cpu.yaml` keeps the geometric/confidence gates unchanged and explicitly uses 512 features, four CPU threads, frontend maximum age 3 s and controller registration lifetime 4.5 s. Four threads were fastest in the bounded same-frame 1/2/4-thread check (median 1.973 s versus 4.595/2.850 s). Core controller registration lifetime remains one second. This slow static simulation adaptation does not reproduce the paper's latency and needs separate validation for motion. Reproduction commands and full failed/successful frontend history are in the [registration README](../ros2/src/deictic_registration/README.md).

The paper's eye gaze and stylus are adapted to Quest 3S head direction and a tracked controller. Voice accepts a transcript hook; speech recognition is not configured. The installed XR Simulator exercises tracked input but cannot validate physical passthrough cameras or SLAM drift. Automated UI callback tests do not establish manual controller-button use. Physical camera acquisition, nonidentity rig transforms, calibration, synchronization and latency remain unverified because no calibrated wrist camera is available.

K1 motion is fixed-base: legacy reaching uses position-only IK for the right four-joint arm; protocol-v2 bimanual teleoperation uses both four-joint arms with position-prioritized pose IK. Conservative sampled capsule/table/trunk/obstacle checks are implemented; this is not full mesh collision checking, grasping, locomotion or a physical safety controller. No physical robot was commanded.

Read-only Windows inspection found enabled streaming-client rules allowing any TCP/UDP inbound ports on the active Public profile, and effective Public outbound action Allow. Thus the requested client traffic is covered by existing rules. Additional named peer/port rules are prepared in `scripts/Enable-WebRTC-Firewall.ps1` and its dry run was checked; those additional rules have not been applied because this Windows process is not elevated. Workstation inbound rules are scoped to the observed host addresses; workstation outbound policy accepts the requested traffic.

The four-condition study, participant results, task-success rates and paper timing claims remain to be measured using the [experiment plan](experiment-plan.md).
