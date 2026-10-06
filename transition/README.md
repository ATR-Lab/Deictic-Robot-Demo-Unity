# Transition-aware shared autonomy

This package integrates the executable reference supplied in `Transition-deictic-paper`
with the Quest/Unity application. It adds a separate, world-fixed task panel, a
serialized operator API, immutable return snapshots and receive-only K1 telemetry.
Manual arm/head teleoperation remains available as a separate simulation mode.

## Start from Windows

**Hardware observation is held after a memory/reset incident.** Its launcher
refuses to reconnect, and the robot camera service is stopped and disabled.
Keep hardware launch/probe commands below for a future validated session;
see [the incident report](docs/K1_RESET_INCIDENT.md). Simulation remains available.

Run from the repository root. These launchers configure the next Unity Play session;
they do not start Unity or enter Play mode. Keep the terminal open.

```powershell
# Reference A -> B -> home task, native Isaac physics and simulated stereo eyes
powershell -ExecutionPolicy Bypass -File .\scripts\Start-Transition.ps1 -Mode Isaac

# Same task without opening the Windows viewer (WebRTC server still enabled)
powershell -ExecutionPolicy Bypass -File .\scripts\Start-Transition.ps1 -Mode Isaac -NoViewer

# Backend-only experiment without a WebRTC server
powershell -ExecutionPolicy Bypass -File .\scripts\Start-Transition.ps1 -Mode Isaac -NoWebRTC

# Fast task/scheduler development with the logical backend (no robot camera)
powershell -ExecutionPolicy Bypass -File .\scripts\Start-Transition.ps1 -Mode Logical

# Ordinary scheduler with the history display for a separate comparison run
powershell -ExecutionPolicy Bypass -File .\scripts\Start-Transition.ps1 -Mode Isaac -Policy ordinary -Display history

# Currently held: reference command for future K1 telemetry/camera diagnostics
powershell -ExecutionPolicy Bypass -File .\scripts\Start-Transition.ps1 -Mode HardwareObservation

# Existing manual IK/head tracking simulation
powershell -ExecutionPolicy Bypass -File .\scripts\Start-Demo.ps1
```

Use one launch at a time. Ports already occupied cause a preflight refusal; launchers
do not kill other workloads. An SSH password prompt uses your existing account;
credentials are not stored. Runtime connection settings are written to the ignored
`Deictic-Robot-Demo/Assets/StreamingAssets/transition-runtime.json` and loaded only
when Play starts. Stop Play before changing modes.

If startup reports a remote port conflict, inspect the listener on MLWorkstation:

```bash
ss -ltnp '( sport = :8766 or sport = :8767 or sport = :10000 or sport = :49100 )'
ss -aunp 'sport = :47998'
```

Port 8766 is the task console, 8767 the Isaac worker, and 10000 the ROS endpoint.
Close the terminal owning an earlier demo with Ctrl+C before launching another.
The check allows recently closed TCP connections to expire in the background;
it still refuses an active listener and never stops a process automatically.

The transition launcher forwards ROS TCP port **10000** and, for simulation,
operator HTTP port **8766** to workstation loopback. The isolated display ROS domain
is **174**. The vendor robot remains on domain **0**; the manual demo remains on **42**.
The transition endpoint refuses every Unity publisher and service registration.

Isaac mode also starts the WebRTC application stream and opens the installed
Isaac Sim Streaming Client. Wait for `Transition mode=isaac domain=174`, then connect the
client to **131.123.237.31**, signal port **49100**, stream port **47998**.
Use `-PublicIp` when the workstation address changes. WebRTC connects directly
to the workstation over TCP/UDP; the ROS/HTTP SSH tunnel does not carry its media.
The viewport starts on **TaskSpectatorCamera**, showing the full K1 and the green
A/orange B target markers. This is a monitoring view of the simulated scene;
Unity's robot POV still uses the separate head-camera stream.
If a connection attempt failed during startup, click Connect again after this message.

Startup waits for fresh, fault-free, settled Isaac observations before creating
the task runtime. Merely opening the worker HTTP port is not readiness: the
initial physical settling must finish first.

Simulation defaults to `-Policy consequence -Display summary`. Start a new run to
change policy or display; these choices never change during a session. The run
directory includes both choices. A/B/home checks integration under each policy;
its sequential dependencies do not by themselves demonstrate a scheduler benefit.

Once the incident hold is resolved, to verify the hardware wire path before entering Unity Play mode, keep the
hardware launcher open and run this from another Windows terminal with Python:

```powershell
python transition/scripts/probe_unity_endpoint.py --output output/hardware-wire-check.json
```

Use a new output filename each time. Run this while Unity is stopped: the pinned
ROS-TCP endpoint has one active consumer connection. The probe records received
counts and original image stamps; a successful joint check does not imply a
continuous camera feed or physical motion readiness.

## Workstation installation and commands

The deployed checkout is `/home/marnett5/Developer/deictic-k1-reproduction` on
MLWorkstation. Install or update the repository there before running its launcher.
Ubuntu ROS 2 Jazzy, system Python 3.12, the pinned ROS-TCP-Endpoint from
`ros2/scripts/setup.sh`, OpenCV, NumPy, Docker/NVIDIA and the existing
`k1-isaac-sim:5.0.0` image are used. Avoid Conda Python for ROS nodes.

```bash
cd ~/Developer/deictic-k1-reproduction
bash transition/scripts/run_stack.sh isaac
# Alternative: logical. hardware-observation is currently held.
# Explicit comparison condition: backend, scheduler, return display
bash transition/scripts/run_stack.sh isaac ordinary history
# Optional fourth argument: workstation IPv4 for WebRTC, or off
bash transition/scripts/run_stack.sh isaac consequence summary 131.123.237.31
bash transition/scripts/run_stack.sh isaac consequence summary off
```

For a fresh setup, fetch the hash-pinned official K1 assets:

```bash
/usr/bin/python3 transition/scripts/fetch_k1_assets.py
```

For local runtime development without ROS or Isaac:

```bash
cd transition
python3 -m venv .venv
.venv/bin/pip install -e '.[dev]'
.venv/bin/python -m pytest -q
.venv/bin/transition-autonomy demo --output runs/new-paired-demo
.venv/bin/transition-autonomy audit runs/new-paired-demo/ordinary.sqlite
```

Use a new run directory each time. Evidence is not overwritten or replayed.
The Linux-only physical ledger and ROS gateway tests use `fcntl`; run the complete
suite under Ubuntu/WSL rather than Windows Python.

## Task panel

The controller ray selects the panel in the world. Camera view selection remains
independent of task attention, authority and acknowledgement.
For the current Meta XR Simulator tests, use the browser console at
`http://127.0.0.1:8766` for task actions: simulator trigger input is still unreliable.

For the native K1 task:

1. Commit **Local ready**, grant task authority, acknowledge facts and select
   **Attend local**. Start sequencing.
2. After measured reach A, confirm local stability and cue a return. The cue starts
   its deadline immediately. The panel freezes and displays the lease's exact
   snapshot; display acknowledgement follows an actual eye-camera render.
3. Record **Execute / select A**, then separately commit **Local selected A**.
4. Repeat for B with **Execute / select B** and **Local selected B**.
5. After the observed return home, record **Defer / sequence complete**.

The logical launcher uses the same A/B/home task with explicitly synthetic effects;
it checks the operator workflow without physics. The separate paired scheduler demo
uses its own cue/reference task. Both have a web console at `http://127.0.0.1:8766`.
Response recording does not actuate the robot or commit local facts. An uncertain
response retry retains the same decision ID. Pause prevents new skills; revoke
requests task stop. Neither is an independent emergency stop.

The reference Isaac task fixes the trunk, head, left arm and legs and measures the
four right-arm joints. Its fixed head stereo view is synthetic. Use the existing
manual demo to test continuous bimanual IK and head tracking. These modes do not
concurrently own the same simulated arms.

## WebRTC verification

On 2026-09-24, the installed Windows streaming client (2.0.0.429) connected to
Isaac Sim 5.0 at 1920×1080. The client reported stream success and showed
`TaskSpectatorCamera` with visible arm pose changes. A disposable native Isaac
run completed A → B → home with three measured command completions, no runtime
faults, and authority revoked afterward. Streaming, port-preflight and startup
readiness tests passed (72 tests). This check covered the desktop monitor and
simulated motion; it was not a headset or human decision trial.

## Physical K1 observation

**Currently held after a memory/reset incident.** The robot camera service is
stopped and disabled, and the installed hardware launchers refuse to reconnect.
The commands below describe deployment; they are not instructions to resume it
now. See [the incident report](docs/K1_RESET_INCIDENT.md).

The K1 is reachable from MLWorkstation at `booster@192.168.10.102`. The observed
onboard system is Ubuntu 22.04/aarch64 with ROS Humble. The native joint topic is
`/joint_states`. Hardware mode shows the raw **left head camera in both eyes**;
it does not imply physical stereo. The raw left/right topics are
`/boostercamera/head/raw/rgb` and `/boostercamera/head/raw/right/rgb` (NV12,
544×448). Their publisher is intermittent. The alternative vendor compressed
topic `/booster_video_stream` repeats old images: one 20-second diagnostic saw
169 messages but only nine distinct source timestamps. It is therefore not the
default display source. Native joints are mapped by explicit names to
the supplied URDF; unexpected or duplicate names and nonfinite values are rejected.

The offline camera candidate compresses only the left image by default.
`--stereo` explicitly adds the second raw subscription and publishes a diagnostic
pair when both eyes advance within 40 ms. Both source stamps and optical frames
are retained in status. This does not certify stereo synchronization,
rectification or source clock offset. These changes have not been deployed to
the K1 or established as a fix for the memory incident.

The observer is installed on this K1 as the user service
`transition-k1-camera-observer.service`, currently disabled. It publishes derived
images only and does not change vendor services. Inspect its state with:

```bash
systemctl --user status transition-k1-camera-observer.service
journalctl --user -u transition-k1-camera-observer.service -n 30 --no-pager
```

For a future validated installation, these commands copy the observer without
starting it. Run on MLWorkstation from the repository root:

```bash
ssh booster@192.168.10.102 'mkdir -p ~/transition-observation/deployment ~/transition-observation/scripts ~/.config/systemd/user'
scp transition/deployment/{observation_codec.py,k1_camera_observer.py} booster@192.168.10.102:transition-observation/deployment/
scp transition/scripts/run_robot_camera.sh booster@192.168.10.102:transition-observation/scripts/
scp transition/deployment/transition-k1-camera-observer.service booster@192.168.10.102:.config/systemd/user/
ssh booster@192.168.10.102 'systemctl --user daemon-reload'
```

After the incident is resolved, the launcher generates a private UDP-only DDS profile for the observer's active
private interfaces. It does not overwrite the vendor profile. The user service
is disabled and has automatic restart off; this installation does not change lingering
or system boot policy. `--mono-source native` is available for future explicit diagnostics;
there is no automatic camera source switch.

Repeated/regressing stamps are rejected. Bridge samples expire after 0.5 seconds;
Unity hides the display after 1.5 seconds without a new image. This is receipt-based
diagnostic behavior, not proof of physical acquisition freshness. Intermittent
source cameras will therefore make the display blank between bursts.

The workstation bridge subscribes to vendor data and publishes only to domain174:

| Public topic | Type | Meaning |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Measured joints, explicit URDF name mapping; original source stamp |
| `/transition/hardware/head/image_raw/compressed` | `sensor_msgs/CompressedImage` | Compressed raw left head image, mono diagnostic view |
| `/deictic/camera_view/stereo/image_raw/compressed` | `sensor_msgs/CompressedImage` | Diagnostic head camera pair, only when the source observer explicitly enables stereo |
| `/k1/head_camera/{left,right}/camera_info` | `sensor_msgs/CameraInfo` | Unmodified vendor camera parameters |
| `/transition/hardware/status` | `std_msgs/String` | Counts, receipt ages, validation errors and release blockers |
| `/transition/hardware/camera_status` | `std_msgs/String` | Per-eye receipt ages, pair stamps, skew and limitations |
| `/transition/hardware/imu` | `sensor_msgs/Imu` | Vendor IMU, when supplied |

No physical control service/action is fabricated. Vendor RPC services remain on
their original domain, outside the Unity endpoint. The observation bridge creates
no command publisher or service client in that domain.

## Physical release and evidence

The installed hardware diagnostic entry points currently stop at the incident
hold before importing ROS/SDK dependencies or creating participants. Keep that
hold until the memory issue has been investigated offline and a bounded
supervised validation has been agreed.

For a future validated diagnostic session, the reference commands for vendor
motor/status telemetry and read-only hand transforms are below. The observer
command currently refuses to start because of the incident hold.

```bash
# One-time build of pinned official message/service definitions; no vendor demos.
bash transition/scripts/setup_booster_ros2.sh
# Observe continuously; Ctrl+C closes the observer and marks its snapshot stopped.
bash transition/scripts/run_physical_observer.sh --seconds 0
```

The setup script corrects invalid C++ comment/semicolon syntax in the pinned
upstream `Subtitle.msg`, preserving field types/order and recording hashes. The
observer uses system Python 3.12, clears Conda's Python overrides, and applies a
process-scoped DDS profile for the workstation's route to the robot. It subscribes
to `/low_state`, `/joint_states`, `/robot_states`, `/fall_down` and queries only
identity (2022), status (2018), and torso-to-hand transforms (2011). It never sends
movement, stop or mode-change requests. Evidence is saved under
`transition/.runtime/physical-observation/`.

In a second MLWorkstation terminal, expose the snapshot through the diagnostic
gateway on isolated domain 175:

```bash
cd ~/Developer/deictic-k1-reproduction
unset PYTHONHOME PYTHONPATH
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/transition/src:${PYTHONPATH:-}"
/usr/bin/python3 transition/deployment/physical_gateway.py \
  --mode observe --domain 175 \
  --observation-file "$PWD/transition/.runtime/physical-observation/latest.json" \
  --ledger "$PWD/transition/.runtime/physical-observation/observe-gateway.sqlite"
```

`/transition/physical/state` reports diagnostic state. Observe mode has no command
or stop subscription and constructs no SDK client. Shadow mode can record named
proposals without dispatch, effects or authority; use a separate ledger with
`--mode shadow`. Neither mode certifies acquisition freshness or commissions the
physical gateway. These diagnostics do not change Unity or Play mode.

Physical dispatch is disabled. The installed robot SDK lacks the specification's
`GetRobotInfo`/`GetStatus` API and package version metadata; it is not the pinned
1.6.3 artifact. A separate, hash-verified 1.6.3 client on MLWorkstation successfully
queried both identity and status. It reported Booster K1 firmware
`v1.6.1.1-release-01967-2026-04-27`. That verifies query connectivity only.
No onboard SDK, firmware or vendor controller was replaced.

The supplied [hardware specification](docs/HARDWARE_IMPLEMENTATION_SPEC.md) requires
approved identity/configuration, measured acquisition and stop bounds, independent
support/stop arrangements, exclusion of alternate command producers, and calibrated
A/B/home profiles before supervised commissioning. Fake-transport tests cannot
supply those records. Raw receipt times and ROS connectivity do not authorize motion.

See [integration status](docs/INTEGRATION_STATUS.md) for current evidence and remaining
stage gates, [operator protocol](docs/UNITY_OPERATOR_API.md) for Unity/API semantics,
and [architecture](docs/ARCHITECTURE.md) for the task model. Imported validation
documents describe the reference project's historical runs; they are not new hardware
or participant results. The sequential A/B/home task checks integration, not whether
one scheduling policy improves human performance.
