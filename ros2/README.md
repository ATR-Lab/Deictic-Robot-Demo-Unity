# ROS 2 control and frame fusion

For a new installation, start with [complete setup](../docs/setup.md). For the current four-process native-Ubuntu profile, use [the startup runbook](../docs/start-demo.md). The examples below retain the tested lab paths and historical mock/synthetic profiles; substitute your own paths and SSH destination outside that installation.

This package supplies the paper's **SE(3) pose graph**, confidence gating, K1
position IK, preview/execute/cancel contract, and simulation test path. It does
not contain a physical Booster motor driver. K1's four arm joints cannot realize
arbitrary six-dimensional tool poses: legacy deictic reaching intentionally
reaches a position and reports `position_only=true`.

[Bimanual teleoperation](../docs/arm-teleoperation.md) adds relative controller
pose IK for both arms, prioritizing position with soft orientation, a two-trigger
clutch, a separate command lease and relay acknowledgement. Protocol v2 carries
current-head-relative positions and xyzw rotations. Update Unity, controller and
relay together. SciPy >=1.11,<2 is required (the supplied venv has 1.11.4).
This relative mode does not require learned camera alignment; deictic
goal selection retains its existing alignment gates. The simulation launcher
enables `allow_teleoperation`; direct node startup defaults it to false.

The GTSAM implementation uses headset VIO and camera FK chains, a headset-frame
anchor, FK priors, and Huber-robust ternary registration factors
`log(Z^-1 C^-1 X H)`. iSAM2 updates the graph incrementally. The published frame
correction includes the optimized current headset pose relative to its raw VIO
pose; otherwise drift absorbed into graph headset nodes would never correct
incoming targets. The node defaults to a configurable 10,000-keyframe capacity
and synthetic registration at 2 Hz (about 83 minutes). At capacity the graph
stops accepting updates and requires a session restart; it does not silently
discard uncertainty. Status includes `graph_keyframes` and `graph_capacity`.

No camera or hand-eye calibration is invented. `synthetic_test` generates known
camera relationships for simulation; this is always visible in the status.
`simulation_reference` is an explicit test-only fallback that evaluates the
known frame relationship and does **not** perform markerless estimation. Real
registration comes from the separate `deictic_registration` package and requires
calibrated RGB/depth/pose inputs and SuperPoint weights.

## Run

The startup scripts use source modules directly; a standard `colcon build
--base-paths ros2/src` also installs the ament packages. ROS distributions are not
interchangeable within one Python interpreter.

On the supplied WSL Ubuntu 26.04 / ROS Lyrical installation, use the lightweight
mock path. It checks the complete network/planning loop without Isaac:

```bash
cd /mnt/c/Users/ATR\ Lab/Documents/GitHub/Deictic-Robot-Demo-Unity
source /opt/ros/lyrical/setup.bash
bash ros2/scripts/setup.sh
bash ros2/scripts/run_bridge.sh mock
```

The TCP endpoint is pinned to Unity's `ROS2v0.7.0`, commit
`54c1a64b6d5ef6ffa0a0431570bb74329b79b15b`. The script binds localhost port 10000,
uses ROS domain 42, and applies the supplied Reachy workspace's verified WSL DDS
settings. Set `ROS_TCP_PORT`, `ROS_DOMAIN_ID`, or `ROS_BIND_IP` before launch to
override. Do not run mock feedback alongside Isaac on the same ROS domain.

GTSAM 4.2.2 requires NumPy <2; passing NumPy 2 arrays into its native bindings
can crash Python. The backend checks this before importing GTSAM. WSL's Python
3.14 / NumPy 2 is kept intact. On the remote Ubuntu 24.04 / ROS Jazzy machine:

```bash
cd ~/Developer/deictic-k1-reproduction
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m venv --system-site-packages .venv-control
.venv-control/bin/python -m pip install numpy==1.26.4 scipy==1.11.4 gtsam==4.2.2
bash ros2/scripts/setup.sh
DEICTIC_PYTHON="$PWD/.venv-control/bin/python" bash ros2/scripts/run_bridge.sh synthetic
```

Run the Isaac adapter and `sim/trajectory_relay.py` in domain 42 as described in
the simulation README. This mode uses the **real GTSAM optimizer with synthetic
camera observations**, and measured joint feedback from Isaac. Change
`synthetic` to `markerless` only after configuring the real registration node.
The markerless backend keeps commits disabled until fresh accepted observations
arrive. Scripts explicitly enable simulation execution; the node default is
`allow_execution=false`.

The native 1280×960 headset / 640×480 wrist Isaac fixture uses the separate
`isaac_head1280_cuda.yaml` profile: CUDA, 512 keypoints, match filter 0.8, four
CPU threads and a 0.25 PyTorch allocator fraction. It retains nearest-pixel
optical-Z depth and the original one-second frontend/controller age limits;
registration attempts run every 0.3 s, or 0.25 s during recovery. This profile is tied
to the projection-balanced rigid mount and matte-floor simulation scene.
Its explicit `camera_qos: reliable_latest` uses reliable, volatile subscriptions
with keep-last depth 2 for the seven camera inputs. Other deployments retain
the default `sensor_data` best-effort policy. Both policies keep only the latest
synchronized set for processing and reject expired results.
From a stopped bridge, start the endpoint/control/display services with:

```bash
DEICTIC_PYTHON="$PWD/.venv-control/bin/python" \
DEICTIC_CONTROL_PARAMS="$PWD/ros2/src/deictic_registration/config/isaac_head1280_cuda.yaml" \
  bash ros2/scripts/run_bridge.sh markerless
```

In another terminal with ROS Jazzy, domain 42, UDPv4 and the source package
paths configured, start the actual image estimator (see its package README for
the isolated dependencies and pretrained weights):

```bash
export ROS_DOMAIN_ID=42 FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export PYTHONPATH="$PWD/ros2/src/deictic_registration:$PWD/ros2/src/deictic_control:${PYTHONPATH:-}"
.venv-registration/bin/python -m deictic_registration.node --ros-args \
  --params-file ros2/src/deictic_registration/config/isaac_head1280_cuda.yaml
```

When retaining an already-running endpoint, replace only the controller with
the following command:

```bash
.venv-control/bin/python -m deictic_control.node --ros-args \
  --params-file ros2/src/deictic_registration/config/isaac_head1280_cuda.yaml \
  -p urdf:="$PWD/models/K1/K1_22dof.urdf" -p allow_execution:=true
```

Stop the previous controller and clear motion before replacing it. Do not mix
synthetic observations with visual observations in the same graph. Keep the
normal `/deictic/registration_failure` topic: Unity's tracking events also use it.

The historical coarse-pattern CPU profile `isaac_cpu.yaml` takes about two
seconds per estimate and explicitly uses a 3 s frontend age limit and 4.5 s
controller lifetime. Its successful single reach lost visual overlap afterward.
The separate historical reach-balanced rigid-camera fixture uses
`config/isaac_balanced_cpu.yaml` in the registration package. Use that filename
for **both** commands when running that specific mount: its match filter is
0.8, selected on development confidence and then frozen before three fresh
validation captures. The original `isaac_cpu.yaml` retains filter 0.7 and the
earlier verified fixture. Neither profile changes the geometric/confidence
gates or supplies a physical camera calibration.

On this native host, use `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` for the backend,
endpoint, relay, and independent ROS observers. The startup script applies this
by default outside WSL. Default shared-memory transport discovers the
host-networked/private-IPC Isaac container but fails to deliver its samples;
the WSL-specific `LARGE_DATA` setting also partitions discovery from Isaac.

Forward the endpoint to the Unity host when it runs remotely:

```powershell
ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 -L 10000:127.0.0.1:10000 marnett5@mlworkstation.atr.cs.kent.edu
```

Authenticate interactively. No credentials belong in config files. Configure
Unity's connector for ROS2 on **both Standalone and Android**. The editor uses
127.0.0.1:10000; an actual Quest requires a reachable LAN endpoint or forwarding
path, not the headset's localhost.

This campus network can rotate the Windows host's external address and reset
SSH connections. An optional reconnecting Windows helper keeps the local
listener alive and retries SSH. It prompts once, then passes the password
through an anonymous stdin pipe to a detached hidden process; the password is
retained only in worker memory. It verifies the existing OpenSSH known-host
key and stores only non-secret logs/PID under `.codex`:

Run this from the repository root using a Windows Python 3 installation; replace the host and user placeholders with your SSH destination.

```powershell
python -m pip install --target .codex/tunnel-deps paramiko==4.0.0
python ros2/scripts/ssh_tunnel.py --host 'YOUR_WORKSTATION_HOST' --user 'YOUR_SSH_USER'
Get-Content .codex/tunnel.log -Tail 10
# Stop the helper when finished:
Stop-Process -Id ([int](Get-Content .codex/tunnel.pid))
```

Choose either the OpenSSH command or the helper so only one process binds port
10000. After network loss, Unity must reconnect and complete a fresh clock-sync
round trip before enabling commits.

## Robot head-stereo display stream

`run_bridge.sh` also starts `camera_view_relay.py`. Unity's viewer subscribes to
`/deictic/camera_view/stereo/image_raw`: one atomic RGB8 side-by-side image,
left eye in the left half and right eye in the right half. Each eye is at most
480×360, so the combined frame is at most 960×360 at 5 Hz. Maximum pixel payload
is 5.184 MB/s (about 41.5 Mbit/s); TCP/SSH framing adds overhead. The estimator
continues using the untouched full-resolution `/k1/wrist_camera/image_raw`
together with the separate headset RGBD input. Head stereo is for display.

The source topics are `/k1/head_camera/left/image_raw` and
`/k1/head_camera/right/image_raw`: RGB8, equal dimensions no larger than
640×480, distinct optical frame IDs, and exactly equal integer acquisition
timestamps. Isaac reads both from the same cached renderer reference/time and
assigns the common ROS time first observed for that renderer reference. Deferred
callbacks cannot relabel an old capture as fresh. The relay retains that stamp and
uses output frame ID `k1_head_stereo_optical`. Unequal or missing eyes do not
form a pair; there is no wrist-camera or single-eye fallback.

The relay subscribes to both sources only while output consumers exist and
retains at most one unmatched image per eye plus one latest complete pair.
Both source subscriptions use reliable, volatile delivery with a depth-one
queue; retransmitted fragments still have to meet the original freshness limit.
It uses OpenCV area downsampling without changing aspect ratio or upscaling,
and rejects malformed, oversized, duplicate, future, or expired frames before
publication. It never restamps old pixels. The default relay age limit is one
second. Unity subscribes only in Robot view, labels missing/stale data and
hides an expired image using its configured 1.5-second viewer timeout.

The ROS-enabled Isaac adapter enables this display source by default, with
`--no-head-stereo` as an opt-out. Its extra render products run only while an
eye's image or CameraInfo topic has a subscriber, at no more than 5 Hz.
Closing the last display consumer therefore releases both relay subscriptions
and render demand, unless another head-camera consumer remains. The virtual
eyes use a provisional 64 mm baseline around the vendor head optical mount;
this is simulation geometry, not a measured physical stereo calibration.

Unity places the toggle and monitor at a fixed world pose. The existing
controller ray hovers the button; Trigger (or Space over the button) toggles
it without selecting a target. V is a direct keyboard shortcut. Each XR eye
samples its corresponding half; desktop/mono rendering shows the left eye.
The default toggle center `(0, 1.25, 0.8)` meters is configured by
`cameraControlsWorldPosition`. Camera pixels are not target-selection surfaces;
return to User view to commit a target. Existing valid previews can still be
executed with A/Enter, and B/Escape cancels in either view.

For an already-running bridge, launch only this additional process in the same
sourced ROS environment/domain (do not start a second endpoint):

```bash
ROS_DOMAIN_ID=42 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  .venv-control/bin/python ros2/scripts/camera_view_relay.py
```

ROS parameters `left_source_topic`, `right_source_topic`, `output_topic`,
`max_width`, `max_height`, `display_hz`, and `max_age_s` configure this
display-only path. Width/height bounds are per eye; they and the rate may be
reduced from the 480×360/5 Hz caps. NumPy and system OpenCV are required.
The former single `source_topic` parameter has been replaced by the two eye
parameters. Update existing Unity scenes' `robotCameraTopic` and set
`robotCameraStereo=true` when using this relay.

## Contract

All distances are meters, joints radians, and message geometry uses ROS FLU.
`base_link` is an alias for the fixed K1 URDF `trunk`. The right-arm names are
read from the pinned vendor URDF; its first name really is
`aaright_shoulder_pitch_joint`. The tool is the terminal
`right_elbow_yaw_link` plus configurable local offset `[0,-0.10,0]` meters.

| Topic | ROS message | Meaning |
|---|---|---|
| `/k1/head_camera/left/image_raw` | `sensor_msgs/Image` | Left robot-head RGB8 display source; paired acquisition stamp |
| `/k1/head_camera/right/image_raw` | `sensor_msgs/Image` | Right robot-head RGB8 display source; paired acquisition stamp |
| `/deictic/camera_view/stereo/image_raw` | `sensor_msgs/Image` | Atomic left/right SBS display, up to 960×360 at 5 Hz; common acquisition stamp, frame `k1_head_stereo_optical` |
| `/deictic/headset_pose` | `geometry_msgs/PoseStamped` | Headset pose in `headset_world` |
| `/deictic/goal` | `geometry_msgs/PoseStamped` | Deliberate goal in `base_link`, fresh UTC stamp |
| `/deictic/preview` | `trajectory_msgs/JointTrajectory` | Validated preview; header stamp echoes its goal; empty clears it |
| `/deictic/execute_request` | `std_msgs/String` | Execute once only when the exact goal timestamp matches the current fresh preview |
| `/deictic/execute` | `std_msgs/Empty` | Legacy request, disabled unless `allow_legacy_execute=true` |
| `/deictic/cancel` | `std_msgs/Empty` | Discard preview and replace motion with a measured-position hold |
| `/deictic/status` | `std_msgs/String` | Atomic JSON transform/confidence/version and operation |
| `/deictic/base_from_headset_world` | `geometry_msgs/TransformStamped` | `T_base_link_headset_world`, retained/frozen on degraded tracking |
| `/deictic/registration` | `std_msgs/String` | Registration observation described below |
| `/deictic/registration_failure` | `std_msgs/String` | JSON reason; immediately invalidates confidence |
| `/deictic/time_sync/request` | `std_msgs/Float64` | Client UTC send time t0 |
| `/deictic/time_sync/reply` | `std_msgs/Float64MultiArray` | `[t0,server_receive,server_send]` for round-trip clock offset |
| `/joint_states` | `sensor_msgs/JointState` | Measured Isaac joints, one authoritative publisher |
| `/k1/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Immediate simulation command, zero header stamp |
| `/k1/end_effector_pose` | `geometry_msgs/PoseStamped` | Measured tool pose in `base_link` |

The status is authoritative and atomic: `stamp`, `alignment_version`,
`base_from_headset_world` (7-vector), `confidence`, `can_commit`, `reason`,
`registration_age`, `backend`, `mode`, `operation`, `preview_ready`, `executing`,
`execution_enabled`, and `robot_feedback`. Renderers may continue displaying
the frozen transform but must block commits when `can_commit=false`. Each
preview binds its original transform, base-frame goal and alignment epoch.
Healthy transform updates retain the exact preview trajectory only while the
goal mapped through `current_X @ inverse(original_X)` moves at most
`preview_target_tolerance` (default 0.005 m). This measures displacement at the
selected point, including rotation effects, against the original binding; many
small changes cannot accumulate unnoticed. `alignment_version` remains an
update counter. An epoch change, greater target displacement, failed
registration, stale feedback, cancellation, or expired execution request
invalidates the pending preview. Existing trajectories are monitored against
fresh feedback and a 0.025 rad final tolerance. Success is reported only after
the scheduled duration and measured convergence; deadline/stale-feedback
failures send a hold and report an abort.

Planning runs in one background worker so the ROS executor can continue
processing joints, registration, cancel and status messages. Each request
copies its model, goal, initial joints, alignment and collision settings;
the queue contains at most one running request and one latest pending request.
Cancellation or a newer request invalidates old work by generation and a
cooperative cancellation event. Workers do not publish or mutate controller
state. The executor accepts a completed plan only after rechecking fresh
registration/joints, the original alignment epoch and target-displacement
bound, the unchanged start pose (within 0.03 rad), and its planning deadline.
`plan_timeout` defaults to 10 seconds and must be finite and positive before
planning and execution. The preview still requires a separate correlated
execute request; planning does not command motion.

This corrects a recorded pre-motion failure where synchronous IK/path checking
starved joint callbacks and triggered `joint_feedback_stale` despite valid
visual registration. The resulting core/lifecycle/execution suite passed
**44/44 tests** in 12.124 seconds (`sim/artifacts/control_async_planner_tests.xml`).
The controller was restarted with a fresh graph; the simulator, image provider,
relay and endpoint were retained. The subsequent partial traversal completed
targets 1/2, with all 1507 observed gates open and 457/457 visual estimates
passing. Target 3 was rejected before motion for timestamp quality. A tighter
Unity clock-sync rule passed 32 EditMode tests, followed by a complete ordered
five-target traversal: grounding 0.17–0.78 mm and measured tool error
2.62–4.87 mm. That 39.176-second test began at the prior target-2 pose and
reviewed each preview for 4.5 seconds; it is not the paper's mobile-user or
neutral-return protocol. Earlier failed attempts are preserved in the
[validation record](../docs/validation.md).

Quest tracking lifecycle events (`source=quest_tracking`) start a new graph
epoch and stop the active trajectory. The prior display transform remains
frozen with commits blocked, but its constraints are discarded. Registrations
captured at or before the event are rejected. Loss/pause inhibits registration
until a restoration/resume/recenter/origin-change event; the next fresh strong
observation initializes the new origin without an old-frame jump comparison.
The `event_type` field distinguishes `lost`, `restored`, `paused`, `resumed`,
`recentered`, and `origin_changed`. Ordinary visual registration failures keep
the graph and freeze confidence.

Registration observations use this schema (all transforms `[x,y,z,qx,qy,qz,qw]`):

```json
{
  "schema_version": 1,
  "stamp": 1789570000.0,
  "headset_pose": [0, 0, 0, 0, 0, 0, 1],
  "camera_pose": [0, 0, 0, 0, 0, 0, 1],
  "camera_from_headset": [0, 0, 0, 0, 0, 0, 1],
  "inliers": 90,
  "matches": 100,
  "median_reprojection_error": 0.5,
  "source": "superpoint_pnp"
}
```

`headset_pose = T_headsetWorld_headsetOptical`,
`camera_pose = T_base_wristOptical`, and
`camera_from_headset = T_wristOptical_headsetOptical`. Poses must correspond to
the synchronized image captures. Headset optical-camera extrinsics must be
applied before sending the first pose. The JSON identity values above document
the shape only; they are not calibration defaults. Timestamps are synchronized
UTC/ROS wall time; `use_sim_time` is not supported by this bridge. Zero feedback
stamps use receipt time, but zero target/registration stamps are rejected. The
Unity client estimates server-minus-client clock offset from the time-sync
round trip, applies it to outgoing stamps, and checks received status age in
the server clock domain. Machine clocks are not modified.

The default logistic coefficients `(alpha,beta,gamma)=(12,1,6)`, confidence
threshold 0.7, minimum 12 inliers, registration age 1 s, jump bounds 0.15 m /
0.35 rad, and FK/VIO/registration noise are **engineering defaults**. The paper
does not supply these numbers. They require calibration before an experiment.

## Simulation geometry and validation limits

The synthetic Unity layout places the robot base at Unity `(0,0.9,1.2)` with
identity orientation, yielding `T_base_headsetWorld=[-1.2,0,-0.9,0,0,0,1]` in ROS
coordinates. Override the node's `synthetic_base_from_headset_world` parameter
when moving the workcell. The table guard uses base-frame z=-0.15 by default.

IK uses the URDF chain and bounds; quintic timing limits speed to 0.35 rad/s and
acceleration to 0.7 rad/s². Sampled link capsules are checked against the table,
a conservative trunk box, and configurable obstacle boxes. This is not a full
mesh collision checker, dynamic balance controller, certified physical safety
layer, or whole-body planner. The base and legs stay fixed in Isaac.

Five tested reaching sphere centers are `(x,-0.25,0.02)` with x =
0.08, 0.10, 0.12, 0.14, 0.16 m. Centers plus six ±12.5 mm axis offsets pass the
position solver and default clearance checks from rest `[0,1,0,0]`.

## Tests

Execution requests use JSON `{"schema_version":1,"goal_stamp_sec":123,"goal_stamp_nanosec":456}`,
copying the preview header's integer timestamp fields exactly. A delayed request
for an older goal cannot execute or erase a newer preview. Matching tokens still
require fresh registration/feedback, the same alignment epoch and original
target-displacement bound, an unexpired plan, unchanged starting joints, and a
command subscriber. This binding is checked again at execution; the exact
preview samples and token are retained during permitted small updates. The default disables
the uncorrelated legacy Empty request. Cancel remains an Empty message.

Core tests need NumPy, SciPy and pytest; graph tests additionally require a
compatible GTSAM environment. They cover noncommuting frame composition, drift
correction, malformed observations, confidence loss/freezing, freshness,
outliers, actual K1 geometry/limits, quintic bounds, and collision rejection.

```bash
PYTHONPATH=ros2/src/deictic_control .venv-control/bin/python -m pytest ros2/src/deictic_control/test -q
```

With the mock or Isaac bridge running, run an actual TCP protocol test in a
second sourced ROS terminal. `--execute` opts into one simulated reaching move
and a hold; omit it to check the preview only:

```bash
export PYTHONPATH="$PWD/ros2/src/deictic_control:${PYTHONPATH:-}"
python3 ros2/scripts/smoke_tcp.py --execute
```

This checks endpoint ROS2 handshake, all message registrations, stale-goal
rejection, preview, optional execution, measured FK convergence, and cancel.
Isolated ROS-domain tests also cover delayed/replayed execute tokens and each
freshness/epoch/target-displacement/start-state execution guard without publishing robot commands.
The supplied Reachy controller is not reused: it skipped the final joint,
ignored `time_from_start`, and allowed overlapping trajectory coroutines.

The embedded Connector also includes a documented ROS 2 `EmptyMsg` compatibility
patch: the original generated class omitted the IDL dummy byte and produced
header-only Execute/Cancel payloads rejected by Fast CDR. See the package's
`LOCAL_CHANGES.md` and Unity's wire-format regression test.

## Verified live result

On 2026-09-16, the Unity PlayMode test drove the full path through the SSH
tunnel, ROS-TCP endpoint, GTSAM backend, trajectory relay, and live Isaac K1:
fresh clock-corrected alignment → target commit → correlated preview → Execute
→ measured movement → feedback-confirmed completion → Cancel. The final Isaac
tool error was **2.42 mm** against the 3 cm test threshold. The test passed in
11.378 s; backend status then confirmed `cancel_hold_sent`, no active execution,
and no pending preview. This used real physics and GTSAM with explicitly
synthetic registration observations, not physical camera registration.

That Unity test back-solves its input through the estimated alignment, so it
checks transport/planning/control and does not independently measure grounding.
The separate opt-in `SelectFixedWorldTargetThroughMarkerlessRos` test uses the
fixed Unity sphere at `(0.25,0.92,1.32)` and scores measured tool feedback against
the independent base-frame target. Enable it only for an isolated Isaac fixture
with `DEICTIC_MARKERLESS_SIM_INTEGRATION=1`; it requires markerless GTSAM mode.
This independent test passed on 2026-09-16 at 16:49:06 UTC in 8.983 s with genuine
SuperPoint/LightGlue/PnP observations and GTSAM: **1.70 mm** target grounding
error and **2.27 mm** measured Isaac tool error. Clock synchronization RTT was
30.1 ms. Its world target was fixed independently of the estimated transform;
the test checked actual named-joint motion, fresh post-execution tool feedback,
completion, and teardown cancel. This is one reach in the calibrated textured
simulation fixture, not physical accuracy or general moving-view robustness.

The reached arm pose lost cross-camera scene overlap and subsequently produced
zero matches. The controller correctly disabled new commits and retained no
preview or active execution after cancel. Returning to a useful camera pose is
a separate simulator maintenance operation; confidence gates are not relaxed.
For the declared empty workcell, an explicit return to `[0,.5,0,0]` rad is
available in `ros2/scripts/return_sim_arm_to_rest.py`. It checks fresh simulator
feedback, exact joint limits, existing speed/acceleration limits and sampled
table/trunk clearance before publishing. Run without `--execute` to inspect
the plan; add `--execute` only for the intended maintenance return. This command
requires the Isaac node and relay and must run with the same sourced ROS and
Python paths. It is separate from normal learned target selection and should
not be used to substitute for a failed grounding estimate. The verified return
used 398 samples over 6.612 s and finished within 0.0096 rad of the rest joints.
Visual registration recovered naturally after that return. A separate idle
test paused only the registration provider for six seconds: the controller
closed its commit gate at observation age 4.544 s, preserved the last transform,
and had no preview or active execution. After resuming the provider, a fresh
observation reopened the gate about 2.70 s later (confidence 0.913, capture age
2.099 s). See `sim/artifacts/registration_liveness.json`. This verifies expiry
and fresh-input recovery at the rest viewpoint; it does not establish recovery
while the arm remains in a viewpoint without shared scene features.
The renderer now snapshots RGB, depth and camera parameters together and derives
capture poses from that same render reference. Accuracy throughout camera
motion still needs independent validation.

For a declared simulation camera-fixture experiment, `--joint-goal` accepts four
explicit angles in the published arm-joint order and uses the same checked
planner. Omit it for the rest pose. Stop the learned controller before such
maintenance moves, and clear/restart the trajectory relay before restarting
Isaac at a different initial pose; the relay otherwise retains its last hold.

The measured client/server clock offset was 2.1073 s with a 32.2 ms sync round
trip. The first live test exposed the Connector's missing Empty dummy byte;
the successful test includes the embedded wire-format fix described above.

After adding exact preview-token execution, the same known-alignment Unity test
passed again at 16:16:37 UTC in 10.325 s with **2.46 mm** measured tool error and
66.4 ms clock-exchange RTT. That backend revision passed 27 tests, including
delayed execute isolation and freshness/epoch/target-displacement/start-state guards. The live
test starts from the simulator's rest pose; rerunning it from the already reached
target needs a separate setup move because it deliberately requires measured
arm movement. The independent markerless result above used the separate fixed
world target test and explicit CPU timing profile.

After extracting the shared checked joint-space planner for the maintenance
return, all 20 core/tracking/execution tests were rerun successfully; the saved
report is `sim/artifacts/control_final_tests.xml`.

The first balanced-mount reaching attempt stopped before motion because a
healthy alignment update invalidated the preview based only on its version
counter. The mapped target had shifted just 0.048 mm. The preview now retains
its original geometry/token under the 5 mm original-target displacement bound
described above. The expanded suite passed 25 tests, including small accepted
updates, cumulative drift, rotation-induced displacement and epoch rejection;
see `sim/artifacts/control_preview_binding_tests.xml`.
A follow-up review added rejection of nonfinite tolerance values changed through
ROS parameters after startup; the complete 27-test run is saved in
`sim/artifacts/control_preview_binding_review_tests.xml`.

The updated balanced-mount test then passed in 15.448 s, including a deliberate
4.5 s preview-review pause spanning new registration updates: **0.91 mm**
independent grounding error and **2.78 mm** measured tool error. Sustained
registration at the reached pose did not pass: a subsequent 30 s record had
only 20 of 300 status samples permitting commits, corresponding to one newly
accepted observation. Other samples were blocked for poor registration quality
or age; the wrist camera image showed substantial forearm occlusion. The
successful single reach and this post-reach limitation are separate results.

A bounded GPU development check tested 512/1,024/2,048 keypoints with match
filters 0.8 and 0.9 on declared rest/reached development images. None passed the
unchanged geometry/confidence gates at both poses. GPU processing took 69–98 ms
per pose and the sampled process peak was 438 MiB, so the current limitation is
usable visual correspondence rather than inference capacity. All six outcomes
are retained in `sim/artifacts/balanced_gpu_development_selection.json`; no GPU
configuration was selected or deployed from that failed check.

A later fixed-mount trial extended the bracket from 6 cm to 10 cm, chosen by
mesh visibility analysis before viewing its registration results. The same
ordered GPU development comparison selected 1,024 keypoints/filter 0.9 at its
fifth candidate, with rest/end confidence 0.895/0.740. Six new captures after
freezing that configuration rejected it: all three rest captures passed, but
the three reached-pose confidences were 0.556, 0.270 and 0.391 (required 0.7).
The reached-pose transform errors were 39.8, 116.6 and 28.3 mm; the five-target
maximum grounding errors were 9.4, 24.2 and 8.2 mm. Warm GPU processing remained
76–85 ms, with a sampled development process peak of 438 MiB. No settings were
retuned on these validation images and this GPU candidate was not deployed.
The robot returned to rest using the checked maintenance planner. See
`sim/artifacts/bracket10_gpu_development_selection.json` and
`sim/artifacts/bracket10_gpu_frozen_validation.json`; earlier failures remain
separate artifacts.

The subsequent projection-based fixed mount used three declared development
captures at each pose before any configuration selection. None of the same six
ordered GPU candidates passed all six inputs: every rest input passed, but no
candidate passed all three reached-pose inputs. No held-out validation or live
profile followed. All 36 outcomes are retained in
`sim/artifacts/projection_gpu_development_selection.json`.

Read-only diagnostics of the preceding 10 cm mount labeled 64–65% of reached-view
wrist keypoints in floor-adjacent regions, versus 13–14% at rest. The reached
inliers covered only 3.3–5.4% of the wrist image, versus 21–23% at rest. These
approximate image-region measurements motivate future fixture investigations;
they do not isolate rotation, obliquity or occlusion as the sole cause, and no
image rotation augmentation or quality-gate adjustment was added.
These projected-table/nearby-blue-pixel labels are not semantic segmentation:
elevated object silhouettes can project outside the tabletop footprint and be
labeled floor-adjacent. They do not prove that a match uses a grid intersection.

A separate development diagnostic on the six projection-mount images held
CUDA 1,024/filter 0.8 fixed and tried wrist rotations of 0/90/180/270 degrees.
Matched pixels were inverse-mapped into the original camera calibration before
unchanged PnP. Only the three unrotated rest images passed; every quarter-turn
variant had at most three eligible matches and failed. All 24 outcomes and the
160 integer-pixel/16 subpixel coordinate checks are saved in
`sim/artifacts/projection_rotation_diagnostic.json`. This negative result is
separate from the earlier `rotation_cpu512_diagnostic.json`, which used a
different fixture. Neither diagnostic introduced runtime preprocessing.

A subsequent single-factor matte-floor ablation retained the projection mount,
table, objects and lighting. It again used three rest plus three reached-pose
development captures and the same six ordered GPU candidates. None passed all
six captures: all rest inputs passed, while every candidate failed at least one
reached-pose input. No held-out validation or live deployment followed. The 36
outcomes in `sim/artifacts/matte_gpu_development_selection.json` show that
removing the repeated grid alone is insufficient. The earlier region diagnostics
do not establish whether the remaining errors are image matches or source-depth
errors near elevated object silhouettes.

An independent-truth diagnostic on the matte development images found a
plausible depth-boundary explanation for 18/31, 22/32 and 15/33 fitted outliers:
holding both image pixels fixed, one depth in the source's 3×3 neighborhood
reduced reprojection error below 3 px, with a local depth spread above 2 cm.
Only 2, 2 and 4 outliers respectively violated the true epipolar line by more
than 3 px. This is diagnostic evidence, not proof of correspondence correctness;
no alternative depth was supplied to the estimator or used for selection.
See `sim/artifacts/matte_depth_boundary_diagnostic.json`.

The frontend now exposes `cuda_memory_fraction` (default 0.25, finite and within
`(0,1]`) and applies it before either model allocates on CUDA. CPU behavior is
unchanged. This bounds PyTorch's allocator, not driver/context memory. Eight
resource-policy tests pass, and the actual constructor and development inference
were checked against installed PyTorch 2.7.0+cu126. The cap resolves an explicit
CUDA device index because that version rejects an unindexed `torch.device` in
its allocator API. See `sim/artifacts/registration_resource_tests.xml`.
The configured CPU thread count also applies to preprocessing in CUDA mode,
matching the four-thread offline measurements.

The next controlled trial changed only native headset RGB/depth resolution to
1280×960 (effective focal length 640 px), retaining wrist 640×480/320 px,
the projection mount and matte scene. The first predeclared CUDA candidate,
512 keypoints/filter 0.8, passed all three rest and three reached-pose
development inputs. After freezing it, six new captures all passed unchanged
geometry/confidence gates and independent transform/target checks. Rest
confidence was 0.987/0.988/0.990, with five-target maximum grounding errors
2.75/1.24/3.22 mm; reached-pose confidence was 0.956/0.956/0.955, with errors
0.97/0.95/0.73 mm. These are repeated static captures at two poses in one
synthetic workcell, not physical accuracy or continuous-path validation.
Warm processing took 95–110 ms; the development process's sampled total GPU
peak was 1,196 MiB. The 0.25 limit applies to PyTorch allocation, not total
driver/context memory. See `head1280_gpu_development_selection.json` and
`head1280_gpu_frozen_validation.json` under `sim/artifacts/`.

A separate 30 s read-only native-resolution transport check received wrist
images at 5.93 Hz, headset RGB/depth at 5.63/5.80 Hz and joint feedback at
23.70 Hz. Maximum receipt ages were 6.3 ms for wrist images, 17.1/28.4 ms for
headset RGB/depth and 1.22 ms for joints. Total received image payload was about
54.4 MB/s. Some head frames were not received (largest RGB gap 0.679 s), so this
does not establish lossless transport or model delivery latency. The live
provider must independently satisfy the unchanged one-second age limit.
See `sim/artifacts/head1280_transport_idle.json`.

The same frozen CUDA profile then passed a 35 s live rest check with the real
GTSAM backend and the original one-second lifetime: all 350 controller status
samples allowed commits, and all 66 actual image registrations passed. No
registration failures occurred. Delivery age was 0.133–0.340 s; minimum
confidence was 0.9815, and the largest independently scored five-target error
was 5.94 mm. Controller observation age stayed below 0.885 s while graph
keyframes advanced from 45 to 111. No goal or motion was sent in this check.
See `head1280_live_rest_control.json` and `head1280_live_rest_registration.json`
under `sim/artifacts/`; motion validation is a separate stage.

The subsequent independent fixed-world center-target Unity test passed in
13.74 s (18:58:35–18:58:49 UTC), including the deliberate 4.5 s preview review
interval. Grounding error was 2.16 mm and measured Isaac tool error 4.51 mm;
the connection's measured round-trip time was 81.2 ms. This used actual
SuperPoint/LightGlue observations, GTSAM, token-bound execution and measured
robot feedback, with a fixed world target that was not back-solved from the
estimated alignment. Evidence: `playmode-markerless-head1280-results.xml`
and its matching Unity log under `.codex/unity-validation/`.

A dedicated 30 s reached-pose window then retained commit availability in all
300 status samples, with graph keyframes advancing from 360 to 416. All 57
actual registrations passed (minimum confidence 0.9161; delivery age
0.173–0.471 s; maximum five-target error 2.18 mm). No registration failure or
active execution was observed after the test's explicit cancel. Maximum
controller observation age was 0.972 s, still below the unchanged one-second
limit. This directly improves on the preserved CPU reached-pose failures; it
does not establish arbitrary viewpoint coverage. See
`head1280_center_postreach_control.json` and
`head1280_center_postreach_registration.json` under `sim/artifacts/`.

The first subsequent five-target traversal stopped before motion at target 1:
the original 0.5 s attempt period did not always leave enough freshness margin.
At 19:01:27.684 UTC, observation age reached 1.064 s and correctly cleared the
preview; it reached 1.164 s before a new observation restored tracking. The
alignment epoch was unchanged and there was no geometric-binding failure.
All 442 observations in the longer 240 s record passed individually, but 45
of 2,402 controller samples were stale. The preceding result had arrived with
0.616 s capture age and the next arrived 0.550 s later. Failed Unity evidence
and `head1280_five_target_{control,registration}.json` are retained.

The separate archived `isaac_head1280_cuda_05.yaml` preserves that cadence.
The primary profile changes only nominal attempts to 0.3 s; matcher, nearest
depth, quality gates, recovery period and one-second lifetime are unchanged.
At a nominal 3.33 accepted observations/s, 10,000 graph keyframes correspond
to about 50 minutes from an empty graph (rather than the 83-minute 2 Hz
synthetic example). Actual accepted rate and already-used capacity determine
the remaining session; the graph still stops at its cap rather than resetting.

The initial 35 s scheduling-only check did not establish sufficient margin:
346/351 status samples were open, with five stale samples and maximum age
1.262 s. All 63 image estimates individually passed, with receipt age no more
than 0.302 s. Camera production had fallen to about 1.86 Hz (median capture
interval 0.537 s), and joints to 7.49 Hz, under an additional shared GPU
workload. Result intervals reached 1.099 s when a synchronized pair was missed.
The provider used 1,196 MiB, with 10,035 MiB GPU memory free at the check's end;
free memory does not imply available compute throughput. No unrelated process
was changed and no five-target retry followed this failed check. Reports are
`head1280_cadence03_{control,registration,feedback_live}.json`. The strict
freshness gate behaved correctly; subsequent render-scheduling work is separate
from registration quality or threshold selection.

The minimal capture-cadence change preserved physics stepping and calibration,
but configured cameras to copy each actual rendered frame and publish each new
coherent pair with a 15 Hz wall-time ceiling. With WebRTC connected and the
other GPU workload still running, lightweight camera-pose timestamps became
steady: median interval 0.285 s and maximum 0.298 s. Nevertheless, the following
35 s estimator check still had six stale samples out of 351: all 101 estimates
passed quality, but processed capture/result gaps reached 1.285/1.300 s.
This separates the remaining large-image delivery issue from render scarcity.
Provider UDP socket diagnostics also recorded receive-buffer drops. These
observations motivate an explicit reliable camera subscription policy for this
large-image native profile; they do not justify relaxing the one-second gate.
The unchanged best-effort check is retained as
`head1280_rendercadence_{control,registration,feedback_live}.json`.

The camera QoS setting is validated before model startup; unknown values fail
closed. Seven policy tests and four existing image-message tests pass in the
actual ROS Jazzy environment (`registration_camera_qos_tests.xml`). The
archived 0.5 s profile retains the original default best-effort transport.

With that explicit reliable policy, the next 35 s live check passed all 350
controller samples and all 109 registrations with no failures. The existing
graph was retained and advanced from 3,904 to 4,012 keyframes. Maximum
controller age was 0.899 s; result receipt intervals stayed below 0.401 s
instead of the previous 1.300 s gap. Source camera-pose intervals stayed below
0.312 s. Result receipt age was 0.379 s median / 0.552 s p95 / 0.587 s maximum;
minimum confidence was 0.9781 and maximum independently scored target error
5.14 mm. Joint feedback was 6.92 Hz, with maximum capture gap 0.268 s and
receipt age 2.31 ms. WebRTC and the unrelated GPU workload remained active;
the provider used 1,196 MiB and GPU free memory was 3,970 MiB at the end.
These bounded results demonstrate fresh delivery under that observed load,
not guaranteed availability under arbitrary shared-resource contention.
See `head1280_reliable_{control,registration,feedback_live}.json`.
The UDP sampler's inner 34.15 s window recorded 23 dropped provider datagrams
and 45 host receive-buffer errors while application delivery stayed fresh;
Isaac discovery, userdata and WebRTC counters did not increase. Reliable
transport handled that observed loss without changing kernel settings. See
`head1280_reliable_udp_window.json` for timestamps and socket provenance.
