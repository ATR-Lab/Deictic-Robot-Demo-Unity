# Fixed-base Booster K1 simulation

Start with [complete setup](../docs/setup.md) or [the installed-stack runbook](../docs/start-demo.md). This page documents the simulator, camera profiles and their development history. Commands using lab account paths and addresses are examples of the tested installation; replace those values on another machine. Historical captures are generated artifacts, excluded from Git; their outcomes are retained in [the validation record](../docs/validation.md).

The current verified setup uses the `reach-projection-balanced` fixed camera
mount, matte floor, native 1280×960 synthetic headset RGBD and 640×480 wrist RGB.
Use the explicit continuous-run command below; the older CLI defaults and mount
profiles remain available for reproducing the archived experiments.

The supplied paper studies a fixed-base SO-101 manipulator. This adaptation fixes
K1's trunk and legs. Both four-joint arms are movable for
[bimanual controller teleoperation](../docs/arm-teleoperation.md); the original
deictic reaching path still controls the right arm and solves position only.
The two neck joints track headset yaw/pitch while Unity's Robot POV is selected.
Bimanual pose IK prioritizes position with a soft orientation objective: each
four-joint arm cannot reproduce arbitrary six-DoF tool poses. The supplied model
has no gripper or wrist camera. The right reaching tip is
`[0,-0.10,0]` in `right_elbow_yaw_link`; a virtual calibrated RGB camera is attached
at `[0,-0.10,0.08]`, looking along terminal +X with a 1.0 rad downward tilt.
Its fixed optical roll is -0.804 rad, leveling the image at the arm's rest pose;
the camera remains rigidly attached as the arm moves.
The actual terminal mesh has maximum local Z=0.03499958 m, giving the camera
45 mm of clearance above the arm instead of placing it on the mesh surface.
Neither script addresses a physical robot.

The optional `--camera-mount-profile reach-balanced` keeps a different, fully
fixed extrinsic: position `[0.06,-0.10,0.08]` and quaternion XYZW
`[0.867339508,-0.439688365,0.073361955,0.221391832]`. Its side bracket is 30 mm
outside the terminal mesh's maximum X, avoiding the hand when the camera points
downward. The default `rest` profile above remains available to reproduce the
earlier captures. Neither profile dynamically aims the camera or supplies scene
truth to registration. The balanced profile has passed static rendered checks;
its behavior during reaching must be assessed separately.

The experimental `--camera-mount-profile reach-balanced-10cm` extends the same
fixed bracket to X=0.10 m, preserving Y, Z, orientation, and intrinsics. An
offline raycast through all 239,521 pinned robot visual triangles and the scene
objects compared X=0.06/0.10/0.14 m over 107 poses. At a 120×120 tabletop grid,
the minimum jointly visible printed area was 11.69%/15.03%/14.44%, respectively.
The predetermined objective selected the 10 cm bracket by maximum worst-case
shared area, without learned features or estimated-transform truth scoring.
This is a virtual camera mount, not a mechanically certified physical bracket.
Its rendered registration validation is separate from that geometry analysis.
The analysis uses a separate environment because the controller's GTSAM runtime
requires a different NumPy version:

```bash
python3 -m venv .venv-geometry
.venv-geometry/bin/pip install numpy==2.5.3 trimesh==5.1.0 embreex==4.4.0
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv-geometry/bin/python \
  sim/analyze_camera_occlusion.py --grid 120 \
  --output sim/artifacts/mesh_mount_comparison_refined.json
```

The 10 cm trial passed development at GPU 1,024 features/filter 0.9, but all three
fresh END-pose images failed confidence (0.556/0.270/0.391); the three REST images
passed. The profile is retained as an unsuccessful experiment, not a deployment
configuration. Features outside the projected tabletop consumed detector
capacity while shared tabletop correspondences occupied a narrow image region.
These floor-adjacent image regions can also contain elevated object silhouettes;
their classification does not prove incorrect grid-identity matches.

The separate experimental `reach-projection-balanced` profile is fixed at
`[0.14,-0.10,0.12]`, quaternion XYZW
`[0.783193052,-0.454771444,-0.058639274,0.419944058]`. This places the camera 140 mm
to the terminal's side and 120 mm above its local origin (about 184 mm from the
reaching tip); physical bracket stiffness, clearance and calibration have not
been validated. It changes only the virtual fixed mount. A declared 36-candidate
geometry grid maximized minimum shared image support in the weakest projection
direction, using the smaller singular value of both planar projection Jacobians
and actual mesh occlusion. Its fixed aim was calculated at 75% of the nominal
middle-target path and never follows a target dynamically. The selected mount
raises the worst geometry score from 2,978 to 10,236 across the same 107 poses;
rendered learned-registration acceptance must be validated independently.
Reproduce this read-only design step with the geometry environment above and
`sim/analyze_camera_projection.py --grid 120`.
Before matcher selection, three REST and three END development pairs were
declared and captured. None of the six predeclared GPU configurations passed all
six images. Every REST result passed confidence, while END produced 21--44
inliers but only two isolated accepted results across 18 END evaluations. No
held-out evaluation or controller deployment was performed for this grid-floor,
640×480-headset trial. Its
better geometric visibility does not establish reliable visual registration.

The opt-in `--floor-profile matte` replaces the default floor's visual material
with uniform diffuse gray (roughness 1, metallic 0). Collision geometry, height,
friction, tabletop, lighting and robot remain identical. The default `grid`
profile preserves every earlier experiment. The matte-floor trial completed the
same six-configuration, six-development-image protocol. No configuration passed
all six; the best passed five of six (END confidence 0.701/0.593/0.799 at 1,024
features/filter 0.9). No held-out evaluation or live controller profile followed
for this 640×480-headset trial.

The optional `--headset-resolution-scale 2` requests a native 1280×960 headset
RGBD render product, with OpenCV K scaled to fx=fy=640, cx=640, cy=480 and matching
calibration imageSize. Field of view, pose and 15 Hz publication cap are unchanged; wrist
RGB remains 640×480 with fx=fy=320. RGB and optical-Z depth come directly from the
renderer at the requested size, with no image upsampling. The strict capture
checks accept different camera sizes but reject headset RGB/depth shape mismatch.
At 15 Hz, raw host-local ROS image payload rises from approximately 46 to
143 MB/s; the separate Unity head-stereo display stream is unaffected. Runtime GPU
usage and visual-registration quality must be checked separately.
The native-render check passed: PNG and depth dimensions, both calibration
image sizes, and K matched the requested resolutions. A known-floor
backprojection peak was -0.699 m versus the authored -0.700 m, using scene truth
only for calibration verification. The observed total GPU usage rose from
8,344 to 9,157 MiB, leaving 14,861 MiB free; this total includes other workloads.
The exact renderer reference and capture-pose provenance are retained in
`sim/artifacts/head1280_native_verification.json`.

With the projection-balanced mount, matte floor and native 1280×960 headset,
the first predeclared configuration (CUDA, 512 features, LightGlue filter 0.8)
passed all three REST and three END development captures. After freezing those
settings, six fresh captures also passed every unchanged geometric/confidence
gate: REST confidence 0.987/0.988/0.990 and END 0.956/0.956/0.955. The largest
grounding error over the five fixed targets was 3.22 mm. These are rendered
static-view checks; they do not establish physical-camera or continuous-motion
accuracy. Evidence and exact timestamps are in
`sim/artifacts/head1280_gpu_frozen_validation.json` and its referenced captures.
The live provider and fresh GTSAM controller then kept the gate open for all
350 samples in a 35-second REST check, with 66/66 accepted visual registrations
and observed registration age 0.133–0.340 s under a one-second TTL. In that initial
native-resolution run, image delivery measured about 5.6–5.9 Hz.

`models/K1/` includes the official pinned 22-DoF URDF and every referenced mesh.
The importer creates a temporary derivative with eight arm joints and two neck joints movable,
with the other twelve joints fixed. `base_link` equals vendor `trunk` and Isaac world; the
floor is at Z=-0.70 m, tabletop at Z=-0.15 m. The left arm starts at `[0,0,0,0]`
and the right at `[0,0.5,0,0]` radians; neck yaw/pitch start at zero. Actual measured state, not the last command, is
published. Position setpoints use PhysX drives, URDF limits, and a 0.5 s command
watchdog that holds measured position. Self-collision is disabled in this first
simulation scene; collision-free motion is not certified for hardware.

## Run on the GPU workstation

The verified inventory on 2026-09-16 is Ubuntu 24.04.5, ROS 2 Jazzy, Quadro RTX
6000 24 GiB, NVIDIA driver 570.211.01, Docker, and a locally available
`nvcr.io/nvidia/isaac-sim:5.0.0` image. The dedicated copy is
`~/Developer/deictic-k1-reproduction`. Existing Go2 containers are independent.

```bash
cd ~/Developer/deictic-k1-reproduction
# Bounded physics/render verification; require DEICTIC_K1_RESULT and no FAILED.
bash sim/run_isaac_container.sh --no-ros --synthetic-headset --smoke --steps 360

# Continuous simulation with ROS and WebRTC; no automatic arm trajectory.
ROS_DOMAIN_ID=42 bash sim/run_isaac_container.sh --synthetic-headset \
  --headset-resolution-scale 2 --textured-table --floor-profile matte \
  --camera-mount-profile reach-projection-balanced \
  --webrtc --public-ip 131.123.237.31 \
  --output /workspace/deictic/sim/artifacts/reach_head1280_matte_live
```

In a second terminal (host Python, outside Isaac's bundled Python):

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
/usr/bin/python3 sim/trajectory_relay.py
```

Start the repository's deictic ROS backend and TCP endpoint separately, using the
same domain and UDPv4 transport. Host networking plus Docker's private IPC makes
Fast DDS shared-memory discovery appear healthy while data are lost; explicit
UDPv4 was verified to deliver joint states between this container and native ROS.
Do not apply WSL's `LARGE_DATA` transport workaround to the native workstation.
Isaac's bundled Python 3.11 ROS runtime lacks `trajectory_msgs`, so
the host relay validates and samples `JointTrajectory` commands at 60 Hz and
sends an internal `JointState` setpoint topic. This avoids mixing ROS Jazzy's host
Python 3.12 binary extensions with Isaac's Python. Only immediate trajectories
(zero header timestamp) are supported. New trajectories replace older ones; an
empty trajectory is rejected. A cancel should send a one-point measured hold.

| Topic | ROS type | Meaning |
|---|---|---|
| `/k1/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | External right-arm trajectory input |
| `/k1/sim/joint_commands` | `sensor_msgs/JointState` | Internal relay setpoint |
| `/k1/teleop/command` | `std_msgs/String` | Leased eight-joint teleoperation setpoint from the controller |
| `/k1/teleop/relay_status` | `std_msgs/String` | Relay readiness, ownership and lease acknowledgement |
| `/k1/head/command` | `std_msgs/String` | Schema-1 headset orientation with tracking flags, timestamps and session/sequence |
| `/k1/head/status` | `std_msgs/String` | Accepted neck target, measured yaw/pitch, active state, limits and hold reason |
| `/joint_states` | `sensor_msgs/JointState` | Eight measured arm joints, two measured neck joints and twelve fixed-zero joints |
| `/k1/end_effector_pose` | `geometry_msgs/PoseStamped` | Tool tip in base_link |
| `/k1/wrist_camera/image_raw` | `sensor_msgs/Image` | Rendered rgb8, 640×480 |
| `/k1/wrist_camera/camera_info` | `sensor_msgs/CameraInfo` | Pinhole intrinsics |
| `/k1/wrist_camera/pose` | `geometry_msgs/PoseStamped` | Optical frame in base_link |

Optional `--synthetic-headset` publishes `/deictic/headset/image_raw`, `/depth`,
`/camera_info`, and `/camera_pose` under the same prefix. Depth is RGB-aligned
`32FC1` optical Z in meters. Optical axes are X right, Y down, Z forward. The
headset-world camera pose includes offset `[1.2,0,0.9]`; therefore the ground-truth
`T_base_headsetWorld` translation is `[-1.2,0,-0.9]`, rotation identity. This is
rendered simulation data, not Quest camera capture. The visual-registration node
must select the raw headset-image topic. Successful matching of the two views is
a separate test; publishing synthetic ground truth is not markerless registration.
`--textured-table` adds 60 deterministic, nonperiodic filled motifs approximately
30--80 mm across. The synthetic headset at `[0.05,-0.65,0.45]` looks toward
`[0.28,-0.30,-0.15]`, clearing the arm and exposing the printed surface. This
fixture does not alter robot dynamics or the reaching targets.

RGB, depth, and `CameraParams` are read together from each camera's cached frame.
The publisher requires equal renderer reference times across cameras, skips
duplicate frames, and derives optical poses from the captured view matrices.
Missing data are rejected; current-stage poses and latest-RGB fallbacks are not
used. ROS timestamps denote retrieval of a new coherent renderer frame. The
simulator's direct capture metadata also records its renderer reference and
simulation time; the separate ROS capture client records the exact common
seven-message timestamp because renderer references are not ROS topic fields.

Camera acquisition now retains every actual rendered frame (`frequency=-1` in
the installed Isaac 5.0 API). The existing loop still renders only every fourth
iteration. Coherent frames are inspected after each of those renders, with a
monotonic 15 Hz publication cap and duplicate-reference rejection. This avoids
discarding alternate usable renders when another workload slows the GPU. It
adds no render calls and changes neither physics stepping nor the wall-time
relay/watchdog. `World.step(render=True)` can advance multiple physics substeps,
so loop iterations must not be interpreted as a precise physics clock. Scene,
intrinsics, capture-time poses and all registration gates are unchanged.
Under concurrent GPU load, this change restored camera-pose intervals to a
0.285 s median and 0.298 s maximum. The first live check still recorded six stale
control samples: all 101 visual observations passed quality, but complete image
availability had gaps up to 1.285 s. Linux counters identified receive-buffer
drops on the registration process's UDP socket, so source cadence alone did not
resolve transport freshness. Before/after evidence is retained in
`sim/artifacts/capture_cadence_comparison.json` and the
`head1280_rendercadence_*` and `head1280_udp_diagnostic.json` reports.
The native-resolution registration profile then enabled reliable KEEP_LAST(2)
camera subscriptions without changing other profiles or kernel settings. With
WebRTC and the competing GPU workload active, the subsequent 35-second check
accepted 109/109 observations and kept all 350 control samples enabled; maximum
registration age was 0.8992 s under the unchanged one-second TTL. The provider
still recorded 23 UDP datagram drops during the sampled interior of that window,
so this demonstrates maintained freshness despite packet loss, not lossless
networking. The time-aligned evidence is in `head1280_reliable_*` reports.

## Robot head stereo display

Normal ROS launches now enable a separate robot-forward stereo display pair.
`--no-head-stereo` disables it; `--head-stereo` explicitly enables it, including
in an otherwise camera-only `--no-ros` smoke run. The two owned render products
are disabled without image/CameraInfo subscribers. With demand they remain
continuously enabled across render iterations, including between published pairs.
A two-second acquisition timeout reports a diagnostic without cycling the render
products off and on. The source publication cap defaults to **15 Hz**; set
`--head-stereo-rate HZ` within 1–30 Hz to change it. Actual delivered rate depends
on GPU and transport load; configured rates are not measured performance guarantees.
Bounded live checks are recorded in [the validation history](../docs/validation.md).
The wrist RGB and synthetic-headset RGBD registration topics and calibration are
retained.

Physics retains the authored 1/120-second step. `loop_timing.py` compares wall time
with Isaac's observed `world.current_time` to catch up after rendering, rather than
assuming a fixed number of physics steps per application update. Accumulated lag
is capped at 0.25 seconds. A due render receives a turn at the next scheduling
boundary after one render period (33 ms) of catch-up following the previous
render's completion, or after 32 catch-up iterations, whichever comes first.
This prevents persistent physics lag from starving the camera. Excess lag is
discarded instead of replayed indefinitely. Rendering
targets 30 Hz; this is a scheduling target, not a guaranteed render rate. Timing
diagnostics report the observed simulation/wall-time ratio and discarded lag, plus five-second aggregate costs for joint-state reads, control updates, rendering/physics, ROS callbacks and camera processing. Arm/head control shares one articulation snapshot before and one after each step.

The pair derives its orientation and parent transform from the vendor's
`head_booster_stereo_rgb_link` fixed joint under the movable `aahead_pitch_link`. At the
zero head pose, optical +Z points along robot/base/world +X, optical +X along
robot -Y, and optical +Y along robot -Z. It looks straight forward from the head;
headset yaw/pitch commands turn the neck and its attached camera pair rather than
aiming automatically at the wrist or reaching target. The nearby tabletop may
lie below the forward camera field of view until the head looks down.

The URDF yaw range is approximately ±58° and pitch is 18° up / 45.5° down;
roll and translation are not actuated. Head commands use `schema_version: 1`,
`frame_id: "base_link"` and xyzw quaternions. Unity's head publisher targets 50 Hz,
independently of the 20 Hz arm publisher. The independent command lease is
0.30 seconds, checked against both source timestamp and receipt time. Returning
to User view, losing tracking or expiry holds measured neck position. Invalid,
replayed and stale commands do not renew the lease. The status topic and
`/joint_states` expose actual neck feedback. This path does not require the
arm clutch or learned registration. See [head controls and protocol](../docs/arm-teleoperation.md#head-control-in-robot-pov).

The URDF supplies one stereo origin, not a calibrated left/right baseline or
intrinsics. This simulation places the eyes 32 mm either side of that origin in
optical X, with a 20 mm forward standoff in optical Z. The default baseline is
64 mm; `--head-stereo-baseline METRES` changes that simulation assumption. Both
origins are approximately 16.7 mm in front of the complete pinned head visual
mesh. This is an explicitly virtual mount, not a claim about Booster's physical
stereo calibration or lens positions.

Each eye defaults to native **320×240** RGB8 with simulated fx=fy=160, cx=160,
cy=120, zero distortion and matching OpenCV calibration imageSize. Use
`--head-stereo-width 640` for native 640×480 per-eye images with fx=fy=320,
cx=320 and cy=240. Matching `CameraInfo` includes the virtual right projection offset
`P[0,3]=-fx*baseline`. The shared capture is accepted only when both cached eyes
have the same renderer reference/time, newer than both the start of demand and
the last published pair.
A bounded reference history from the continuously rendered wrist camera records
the first observed UTC time for each renderer reference without copying pixels.
Both images and their CameraInfo use that same positive capture timestamp;
unknown references and captures at least one second old are rejected rather
than assigned a fresh timestamp. The optical frame IDs are distinct. Compact
`DEICTIC_HEAD_STEREO_DIAGNOSTIC` log entries every five seconds expose demand,
acquisition decisions and frame metadata without printing image data.

| Topic | ROS type | Meaning |
|---|---|---|
| `/k1/head_camera/left/image_raw` | `sensor_msgs/Image` | Left simulated head eye, RGB8 |
| `/k1/head_camera/right/image_raw` | `sensor_msgs/Image` | Right simulated head eye, RGB8 |
| `/k1/head_camera/left/camera_info` | `sensor_msgs/CameraInfo` | Left virtual calibration, `k1_head_left_camera_optical` |
| `/k1/head_camera/right/camera_info` | `sensor_msgs/CameraInfo` | Right virtual calibration, `k1_head_right_camera_optical` |
| `/deictic/camera_view/stereo/image_raw/compressed` | `sensor_msgs/CompressedImage` | Default Unity display: atomic JPEG-quality-80 left/right halves, normally 640×240 total |
| `/deictic/camera_view/stereo/image_raw` | `sensor_msgs/Image` | Legacy raw display on demand: atomic RGB8 halves, at most 960×360 total |

The display relay subscribes to the individual eyes only when a viewer requests
either output, and encodes/publishes only the requested outputs. It requires
equal-size, exact-stamp source pairs before joining the eyes, preserves their
timestamp, uses `k1_head_stereo_optical` for the composite, and never falls back to
wrist imagery. The default 320×240 eyes produce a 640×240 composite; the 640-pixel
source option is reduced to at most 480×360 per eye (960×360 total). JPEG quality
defaults to 80 and its payload size varies with image content. The relay checks
for the latest pair on a 5 ms timer with a separate 30 Hz maximum output rate;
the source's default 15 Hz cap remains the upstream limit. Unity decodes the
latest complete received image each update instead of applying a 5 Hz poll.
Returning to User view removes its subscription, and absent other consumers
both relay input subscriptions and the extra head rendering stop.

Deploy the same project revision to Unity and the workstation, including
`head_stereo.py`, `loop_timing.py`, the display relay and Unity's compressed-topic
settings. The launcher checks that required modules exist but transfers only
its supervisor; it does not synchronize project files or verify revision hashes.
The simulator saves `head_stereo_configuration.json` at startup, then the first
requested coherent pair as `head_left_camera.png`, `head_right_camera.png`, and
`head_stereo_capture.json` in its output directory.

After launching the simulator and display relay, this bounded read-only check
activates the **legacy raw display** demand for 15 seconds, records actual received pixels, and
sends no robot commands. Source ROS Jazzy and use the same DDS settings first:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 FASTDDS_BUILTIN_TRANSPORTS=UDPv4
python3 sim/validate_head_stereo.py --duration 15 \
  --output sim/artifacts/head_stereo_live_check
```

Use a new output directory. NumPy, OpenCV, Pillow and rclpy are required. The
command returns nonzero for missing pairs, stale or invalid messages, blank
images, or relay halves that differ from the exact-stamp source images resized
with the relay's area interpolation. `validation.json` records counts, capture
intervals, ages, per-eye statistics and differences; three PNGs retain the first
complete source/relay set. This validates live delivery/content, not physical
stereo calibration, moving-head accuracy, or registration freshness under load.

For a comparable head-response measurement, stop Unity Play first and ensure no
other head-command publisher remains. Keep the normal markerless registration
stack running, then run this opt-in simulated-motion check from the sourced
Ubuntu control environment:

```bash
.venv-control/bin/python sim/benchmark_head_latency.py --allow-sim-motion \
  --label updated --camera-topic /deictic/camera_view/stereo/image_raw/compressed \
  --output sim/artifacts/head_latency_updated.json
```

The helper checks graph identity, fresh feedback and output-file access before
motion, then requests six four-second small head poses at 50 Hz within an overall
45-second bound. It records first matching status, first measured movement,
90%-of-step response, convergence/tracking error, and received camera rate,
acquisition-header age and mean payload size. It finishes with an acknowledged
inactive hold. Status sampling (normally 10 Hz) limits response-time precision;
these metrics are not sensor-to-eye latency. JSON includes configuration and
source hashes for comparison. To measure an older raw-display deployment, use
`--label baseline --camera-topic /deictic/camera_view/stereo/image_raw` and a
different output path. A missing 90% crossing remains null. Final convergence
uses a separate 0.035-rad tolerance, so a small gravity offset can satisfy
convergence without entering the narrower 90% band. Compare the same moving
phases across runs; the initial neutral phase can already be near its goal.

## WebRTC and network

Connect the installed Isaac Sim WebRTC Streaming Client to `131.123.237.31`.
Isaac 5.0 uses `omni.services.livestream.nvcf`, TCP 49100 signaling and UDP 47998
video. `--webrtc` enables the official extension and visible UI in the headless
process. Docker uses host networking and NVIDIA GPU access.

As explicitly requested, the workstation firewalld public zone allows inbound
TCP/UDP 47995–48012, TCP/UDP 49000–49007, and TCP 49100 from the Windows host's
observed sources `131.123.224.11/32` and `131.123.224.5/32`; campus NAT changed
between these addresses during verification. Rules are both runtime and permanent.
Workstation OUTPUT policies already accept traffic. Windows must allow the same
ranges in both directions, scoped to the workstation address. The helper
`configure_remote_firewall.sh CLIENT_IPV4` recreates the Linux inbound rules.
No firewall is disabled. SSH forwarding of TCP 49100 alone cannot carry WebRTC's
UDP video. The ROS TCP endpoint can independently use an SSH TCP tunnel.

Only one client should connect at a time. Do not start a second streaming
container on the same ports. If the address changes, update both the public-IP
argument and client/firewall scope. If only the client's campus NAT source changes,
reconnect SSH, read `SSH_CONNECTION`, and add the newly observed client address
with the firewall helper; the simulator can stay running. See
[Isaac Sim 5.0 livestream instructions](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/manual_livestream_clients.html).

## Verification and scope

`python3 -m unittest discover -s sim` checks model completeness, joint limits,
locked-joint derivation, name-based mapping, trajectory timing, and strict camera
snapshot synchronization (the camera tests require NumPy). Bounded Isaac
smoke tests save `sim/artifacts/last_run.json` and real rendered camera images.
`DEICTIC_K1_READY` confirms the eight-joint arm articulation initialized;
`DEICTIC_K1_RESULT` reports measured tracking and camera-frame statistics.
Check these markers because Isaac's shutdown can hide Python exit codes.

On the remote Quadro RTX 6000, the 2026-09-16 bounded test completed 360 physics
loop steps, delivered 43 rendered wrist frames, and settled to a maximum joint
error of 0.007732 rad against the commanded `[-0.20,0.62,0.10,0.05]`. The measured
vector was `[-0.194776,0.627732,0.100108,0.048056]`. Six pure-Python model and
trajectory tests passed. This verifies model import, dynamic articulation control,
and rendering; it does not by itself verify ROS transport, WebRTC delivery, visual
registration quality, or physical robot behavior.

The independent read-only `sim/verify_feedback.py` also received 20 matched live
ROS samples and compared the reported tool pose with FK of the measured joint
state: maximum position disagreement was 1.85e-7 m. Run it with the control
virtual environment and UDPv4 settings above. Camera calibration is authored
after `Camera.initialize()` with explicit OpenCV pinhole intrinsics, following
the [Isaac 5.0 calibrated-camera example](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/sensors/isaacsim_sensors_camera.html).
The renderer schema's `opencvPinhole:imageSize` is explicitly set to each render
product's dimensions: `(1280,960)` for the current headset profile and `(640,480)`
for the wrist (or default-resolution headset).
Isaac 5.0's convenience setter leaves its `(2048,1024)` default untouched, which
would rescale the supplied pixel intrinsics. See the
[Omniverse camera schema documentation](https://docs.omniverse.nvidia.com/materials-and-rendering/latest/cameras.html).
`python3 sim/capture_cameras.py --output sim/artifacts/live_capture` independently
captures all seven camera messages with an identical ROS timestamp. Use a fresh
output directory because container-created artifacts belong to the container user.

The Unity PlayMode integration also passed against the live remote simulator on
2026-09-16: preview, explicit Execute, measured arm movement, and Cancel traversed
the ROS TCP endpoint and this relay. That execution test used the synthetic
registration provider; it is distinct from the rendered-image registration test.

The earlier fine-print RGBD camera pair independently reconstructed 2,817 sampled
tabletop points; after moving the camera above the arm, 2,399 projected into the
wrist image, with median RGB difference 1/255 (67.8% within 5/255).
The reconstructed floor was approximately -0.699 to -0.697 m against the
authored -0.70 m. The calibration came from the renderer configuration; scene
truth was used only to check it. Earlier captures with inconsistent intrinsics
failed registration and are retained under ignored `sim/artifacts/` directories.
That pair and its three held-out acquisitions remain in `fine_texture_pair/`
and `heldout_01/` through `heldout_03/`; they are distinct from the subsequent
coarse-print fixture and synchronized capture implementation.
On the earlier baseline, actual SuperPoint/LightGlue and PnP produced 41 inliers
from 87 depth-valid matches, 1.49 px median residual, and 5.98 mm transform
translation error against renderer truth used only for scoring. That is one
static simulated pair, not continuous or device accuracy; subsequent held-out
acquisitions did not provide reliable registration at the fixed confidence gates.
The ten model/trajectory/camera synchronization tests pass, including the mixed
camera-resolution guard. The first attempted
coarse-fixture launch was blocked by GPU memory allocation failure. After the
user explicitly authorized stopping the existing Go2 instance, only the verified
`go2-omniverse-local` container was gracefully stopped. Other users' applications
were untouched. The K1 simulator then reached READY at 16:26:34 UTC on
2026-09-16, published coherent camera frames, and listened on TCP 49100. Its
capture-time CameraParams recovered the authored headset eye to within 3e-8 m;
20 fresh measured-state/FK samples agreed within 1.56e-7 m. Four independent
static captures were saved under `coarse_fixture_development/` and
`coarse_fixture_validation_01/` through `coarse_fixture_validation_03/` with
distinct, exact seven-message ROS timestamps. Learned registration quality is
assessed separately against unchanged acceptance gates; capture synchronization
alone does not demonstrate reliable visual registration during arm motion.

The coarse fixture's first three held-out pairs did not clear the confidence
gate at LightGlue filter 0.5. A stricter filter 0.7 was selected using only the
development pair, then frozen with 512 features and four CPU threads before
three new `coarse_fixture_strict_validation_01/` through `03/` acquisitions.
Those new static pairs passed the unchanged gates: confidence was
0.870/0.785/0.783, with 85/110, 80/109, and 77/107 PnP inliers. Maximum grounding
error across the five fixed world targets was 3.84/3.46/1.04 mm; transform
translation error was 17.19/13.91/3.99 mm. These are different metrics, and neither
is a physical-device or continuous-motion result. Renderer truth was used only
for scoring, not to provide the estimated transform or choose the matcher setting.

The rest profile lost tabletop overlap after a successful markerless reach: the
position-only IK rotated the wrist optical axis nearly horizontal. The separate
balanced profile improves geometric shared-table coverage over 106 samples along
five nominal reach paths, from a 0.64% minimum to 33.9% (projection only, before
occlusion). Its first static development frame failed confidence at matcher filter
0.7. Filter 0.8 was then selected on that development frame alone and frozen
before three fresh `balanced_fixture_validation08_01/` through `03/` captures.
All three passed unchanged gates: confidence 0.824/0.912/0.911, maximum five-target
grounding error 7.96/1.365/1.889 mm. The rejected trial and unused initial held-out
captures remain archived. These static checks do not establish registration
continuity while the arm moves.

A subsequent markerless Unity reach using the balanced profile achieved 2.78 mm
measured tool error and 0.91 mm target-grounding error, including a 4.5 s preview
review delay. At the final arm pose, the frozen 512-feature/filter-0.8 frontend
lost acceptance: the forearm obscured much of the remaining shared print. A
fresh post-reach capture is retained in `balanced_fixture_postreach/`. The
proportion of geometrically reprojected tabletop pixels agreeing within 5/255
fell from 88.9% at rest to 46.2% after reaching; this photometric proxy includes
illumination differences and is not exact visibility segmentation. Registration
must remain gated when these observations fail, even though the reach completed.

The official [booster_train](https://github.com/BoosterRobotics/booster_train)
repository targets Isaac Lab 2.2 / Isaac Sim 5.0 and whole-body motion-policy
training. This paper's fixed-base reaching experiment does not require training a
walking policy, so this scene imports the same official K1 assets directly into
Isaac Sim 5.0. Locomotion, physical camera calibration, physical robot execution,
and the paper's human-subject metrics remain separate work.
