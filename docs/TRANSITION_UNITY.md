# Transition task controls in Unity

The transition workflow has a separate world-fixed panel. The existing camera
button still switches the image filling both eyes. Switching that image does not
change task attention, acknowledge facts, or grant authority.

Start the workstation stack with `scripts/Start-Transition.ps1`, then enter Unity
Play mode yourself. The script writes a local startup configuration and forwards
the ROS visualizer and, for simulation, the operator HTTP endpoint. The panel
uses `http://127.0.0.1:8766`; its session token stays in memory. Only loopback
addresses are accepted, redirects are disabled, and SSH protects the remote hop.

## Modes and ownership

| Startup mode | Controls and transport |
|---|---|
| `transition_simulation` | Task panel sends explicit operator operations to the serialized runtime. Legacy head tracking, arm clutch, target goals and execution publishers are disabled. |
| `hardware_observation` | Receives joint states, camera images and `/transition/hardware/status`. No HTTP session, task mutation, motion or stop command is available. The panel shows commissioning blockers. |
| `manual_simulation` | Existing bimanual arm clutch, robot head tracking and deictic simulation controls; no transition task panel. |

The checked-in settings choose transition simulation. The existing `Start-Demo`
launcher chooses manual simulation. Mode is captured at startup and cannot switch
command ownership inside an active run. Stop Play mode before launching another
mode. Normal domain and scene reload are enabled so the next Play session reloads
the startup file and recreates ROS transport registrations. Malformed machine-local configuration disables command output and automatic
ROS connection.

Hardware mono diagnostic images use the dedicated
`/transition/hardware/head/image_raw/compressed` topic and
`head_color_optical_frame`. The same mono image fills each eye; this is explicitly
not a stereo stream. JPEG dimensions, format and payload remain bounded. The
display reports time since receipt, **not verified acquisition freshness**. This
diagnostic camera path does not produce task evidence or command authority.

Transition simulation aligns the camera display clock through read-only HTTP
`GET /api/time`. The client rejects exchanges with more than 80 ms residual round
trip or inconsistent wall/monotonic intervals and expires calibration after
12 seconds. It displays the estimated uncertainty and blanks the camera while
calibration is unavailable. This clock is private to image presentation: it does
not update the legacy ROS command clock, scheduler time, authority or physical
readiness. No ROS time-sync publisher is enabled in transition modes.

## Panel workflow

Point the right controller ray at a button, press and release its index trigger
over the same button. Pressing both triggers cancels UI selection until both are
released. In desktop simulation, mouse click activates the mouse-targeted
control; Space activates the hovered control. The camera shortcut remains V.

For Meta XR Simulator point-and-click investigation, **Compatibility Mode**
enabled controller hover. Passive traces recorded both index triggers together
even after **Use same input for both sides** was turned off and the right action
was selected. The application intentionally cancels UI selection while that
two-trigger chord is active; independent right-trigger delivery remains under
investigation. Desktop mouse clicks activate UI only when the desktop mouse supplies
the pointer ray; clicking elsewhere in Unity cannot activate a control hovered
by the XR controller. Space remains an explicit shortcut for the hovered control.

1. Grant the task and acknowledge the presented facts separately.
2. Complete the local setup, commit local ready, explicitly attend local work,
   and start sequencing.
3. After measured robot completion, confirm that local state is stable and cue a
   return opportunity.
4. Look at the frozen return snapshot. First-decision buttons become available
   only after the eye camera has rendered the complete snapshot and the server
   accepts its display receipt.
5. Record select A, select B, defer because the sequence is complete, or request
   pointing evidence. Then separately commit the local selection you actually
   completed. Recording a decision does not execute motion or commit that fact.

The snapshot retains its lease ID and hash across polls; newer live facts cannot
replace it. The changes summary is also the lease's frozen summary. A missing
response to a deliberate choice leaves its complete payload and decision ID
pending. “Retry same decision” sends that exact choice; it never invents a new
first response. A new runtime session discards pending input rather than replaying
it into another run.

An HTTP error alone never clears a pending choice: the runtime may have already
captured it. Only an explicit `response_captured: false` receipt bound to the same
lease, snapshot hash and decision ID clears that pending choice as rejected.

“Pause new skills” prevents future dispatch. “Revoke / request stop” (also B or
Escape) requests the task-level stop in simulation; it does not claim measured
stillness. A hardware observer cannot issue this request. Independent robot stop
equipment remains separate.

## Startup file

`Deictic-Robot-Demo/Assets/StreamingAssets/transition-runtime.json` is ignored by
Git and read once at scene startup. Example:

```json
{
  "schema_version": 1,
  "control_mode": "transition_simulation",
  "ros_host": "127.0.0.1",
  "ros_port": 10000,
  "operator_url": "http://127.0.0.1:8766",
  "robot_camera_topic": "/deictic/camera_view/stereo/image_raw/compressed",
  "robot_camera_stereo": true
}
```

Only `schema_version` and `control_mode` are required. Other omitted fields retain
the settings asset values. Unknown fields, duplicate keys and incorrect types
are rejected. Quest Link runs this Windows configuration; a standalone Android
build uses its built settings and requires a separate loopback transport setup.

## Simulator tests in the open Unity editor

With Play mode stopped, use **Tools → Deictic → Transition XR Tests → Fixture
only (Meta XR)**. This runs the frozen-snapshot render/acknowledgement case and
the disconnected hardware-observer UI case. The latter creates no robot
connection. No workstation service is required for these two tests.

For the complete simulated A → B → home workflow, start a **fresh disposable**
Isaac stack and wait for the operator endpoint to become available:

```powershell
.\scripts\Start-Transition.ps1 -Mode Isaac
```

Then choose **Tools → Deictic → Transition XR Tests → Disposable Isaac workflow
(Meta XR)**. This option uses only `http://127.0.0.1:8766` and requires the
`isaac_sim_k1` backend, no existing authority, no running/completed task, and no
return lease. It grants authority for the test, waits for each measured skill
completion, renders and acknowledges each frozen snapshot, records the scripted
decision, and separately commits the local selections. It revokes its authority
at completion. On assertion failure, teardown makes a bounded revoke attempt
only while the observed run and authority still match those captured by the test.
Stop the disposable stack with Ctrl+C when finished; teardown cannot confirm a
stop if transport is unavailable. Restart the stack before repeating the test.

The menu changes only process-scoped simulator environment variables and restores
their previous values after the test finishes or fails. It verifies the **active**
OpenXR runtime reports `Meta XR Simulator` before starting its display; selecting
a manifest does not establish the identity of an already initialized loader.
It does not change the Windows OpenXR runtime or security settings.

One 2026-09-24 run crashed inside `SIMULATOR.dll` during OVRPlugin temporary
instance preinitialization, before the panel executed. A later experiment to
isolate that hook was refused because Unity had already initialized the loader;
the experimental control was removed without changing project feature assets.
The original full-feature Meta XR fixture subsequently passed both tests.
The initial native crash remains an intermittent unresolved issue, with its
original crash and unsuccessful-isolation artifacts retained separately. A
second native crash occurred at 17:54:47 UTC while exiting the main application
Play session. Its managed stack reached the OpenXR message pump during loader
deinitialization; the exact native cause is unknown and has not been linked to
the initial simulator access violation. No native lifecycle fix is claimed.

Each run writes a timestamped `output/transition-xr-*` folder containing scope
metadata, NUnit XML, and a bounded editor log. Successful rendering also saves a
panel image; the Isaac workflow saves one image and a lease/hash snapshot JSON
for each return. These are scripted **mono panel-render and HTTP integration**
tests with the native simulator loaded. They do not establish controller input,
per-eye image correctness, participant performance, or physical robot behavior.
Run status is stored independently of NUnit results. If the editor exits without
a completion receipt, the next editor session marks the run interrupted rather
than treating it as an active or successful test.

For passive input diagnosis while the transition application is in Play mode,
use **Tools → Deictic → Transition XR Tests → Record simulator input (30 sec)**.
It requires the active Meta XR Simulator and transition simulation mode, and
records changed native trigger/mouse/controller state, focus, ray hover and
camera-view state in `output/transition-input-*/input.jsonl`. It stops after
30 seconds or 4,096 samples, and also stops on Play exit or assembly reload. It
does not inject input, send task commands, or read network endpoints.

The full K1 snapshot includes measured joints and hand reference as well as task
facts. The facts region accommodates that payload and its changes summary.
The display acknowledgement still requires every fact to fit and the entire
region to be inside the eye camera view. A clipped or hidden snapshot cannot
enable decisions. Failed workflow tests capture the panel and its actual text
height, visibility, lease and state before teardown, so a rendering failure can
be distinguished from an unexposed or closed return lease.

For an isolated editor launched normally, the equivalent test hooks are
`DEICTIC_NATIVE_SIMULATOR=1`, a process-scoped `XR_RUNTIME_JSON` pointing to
the installed `meta_openxr_simulator.json`, and the PlayMode filter
`TransitionPanelTests`. The real-runtime case additionally requires
`DEICTIC_TRANSITION_TEST_URL=http://127.0.0.1:8766` and
`DEICTIC_TRANSITION_TEST_BACKEND=isaac_sim_k1`. Its default expected backend is
`logical_simulation` for a disposable logical runtime configured with
`transition/examples/k1_pointing.json` and the matching logical world.

## Validation status, 2026-09-24

The revised tests ran in the open Unity 6000.6.0f1 editor with the active native
runtime verified as **Meta XR Simulator 205.0.0** and its OpenXR display running.
No production XR feature setting or Windows security policy was changed.

| Current check | Result and evidence |
|---|---|
| Five-fact frozen snapshot and observer isolation | **2 passed** in `output/transition-xr-20260924T173121614Z-fixture/results.xml`. The full-precision K1 fixture exceeds the old 205px region; an intentionally clipped region cannot acknowledge display, while the complete visible frame can. |
| Native Isaac → Unity task workflow | **1 passed**, 11.182 seconds, in `output/transition-xr-20260924T173223115Z-isaac/results.xml`. Measured A, B and home completion; three distinct frozen returns; accepted display acknowledgements; correctly scored select-A, select-B and defer decisions; separate local commits; final authority revoke. |
| Snapshot presentation evidence | The Isaac folder contains A/B/home PNGs and JSON with frozen lease/hash, full state and layout metrics. Measured text heights were 252/275/275px inside the 390px region, with each complete region visible. |
| Main application, simulator visual check | The Quest 3S simulator's separate left-eye and right-eye views both displayed the full-screen Isaac camera and world UI. The application also rendered its controller ray. Camera switching was activated through the Unity desktop input path before the mouse-routing change below; independent native trigger clicks remain unverified. |
| Native point-and-click input | Traces `transition-input-20260924T174700581Z` and `transition-input-20260924T174902438Z` recorded both trigger axes together and the intended two-trigger UI block. This persisted with the simulator's same-input setting off; its cause remains unresolved. Compatibility mode enabled hover. No successful independent right-trigger click was established. |
| Desktop click routing | The mouse gate compiled and passed source review: desktop clicks require a mouse-derived ray, preventing activation of an unrelated XR-ray target. After import, a blank Game-view click did not visibly toggle, but `transition-input-20260924T175303749Z` captured no Unity mouse press. Event delivery was therefore not established, so the interactive regression is **unverified**. |
| Earlier project EditMode checks | 106 passed, 8 GPU tests skipped under `-nographics`, zero project failures. The broader run included two unrelated vendor-package player-build failures; restrict project reruns to `Deictic.Tests`. |

The native panel tests use a scripted eye-camera render texture and synthetic
panel ray. They verify the rendered-snapshot/HTTP contract while the native Meta
runtime is active; they do not establish real controller input or per-eye image
correctness. The interactive application check is recorded separately. No
physical robot or headset test was performed in this simulation session.

Earlier failures remain separate evidence:

- `transition-xr-20260924T171128544Z-fixture`: native `SIMULATOR.dll` access
  violation during OVRPlugin preinitialization, before panel execution; no test
  completion XML. The original native cause remains unresolved.
- `transition-xr-20260924T172238875Z-fixture`: two setup failures in the temporary
  standard-OpenXR isolation experiment because Unity already initialized the
  loader. No feature mutation occurred; the experimental option was removed.
- `transition-xr-20260924T172356482Z-fixture`: the original full-feature native
  fixture then passed both tests, establishing that startup failure is
  intermittent.
- `transition-xr-20260924T172507550Z-isaac`: point A completed and a return
  snapshot was exposed, but no display acknowledgement was accepted. The complete
  five-fact payload exceeded the small facts area. Failure teardown recorded a
  revoke request. The enlarged layout and clipping regression were subsequently
  verified by the current passing runs above.
- `transition-xr-play-exit-20260924/Editor-crash.log`: a separate native crash
  occurred when exiting the main application Play session at 17:54:47 UTC. Mono
  reported an unknown native failure; the managed stack passed through
  `OpenXRLoaderBase.Internal_PumpMessageLoop`, `Deinitialize`, and
  `XRGeneralSettings.Quit` during the Play-mode state change. This identifies the
  shutdown phase, not the failing native component or cause. The editor closed;
  the earlier passing test receipts remain valid, but clean native Play shutdown
  was not established.
- Earlier tool-launched Unity requests were rejected before execution with only
  `blocked by policy` as the supplied reason. The existing-editor menu runs above
  succeeded through normal Unity UI. A separate generated .NET harness execution
  was blocked by Windows Smart App Control; this was not established as the cause
  of the Codex tool rejection. No security bypass or security setting change was
  used.
