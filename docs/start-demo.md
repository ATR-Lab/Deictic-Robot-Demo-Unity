# Start the installed demo stack

This guide uses the already-installed **native Ubuntu workstation** profile. WSL is not needed. It starts Isaac, ROS, the Windows tunnel and the WebRTC viewer; it does **not** open Unity or change Play mode. For a new installation, follow the [complete setup guide](setup.md). The account, addresses and paths below are the tested lab installation; substitute your own on another machine.

Use one instance of each service. If the stack is already running, reuse it and go to the readiness checks instead of starting duplicates. Authenticate to SSH interactively; do not save passwords in scripts.

## One-command Windows launcher

From this repository's root in Windows PowerShell, run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\Start-Demo.ps1
```

Enter the workstation's SSH password when prompted (or use an already configured
SSH key). The launcher opens the installed WebRTC viewer, creates the ROS SSH
tunnel and starts the four workstation services below. It waits for Isaac's
startup marker before starting the ROS stack. Leave this terminal open and wait
for `DEICTIC_DEMO_RUNNING`; then enter Unity Play mode yourself. Registration
may still be initializing; use the Unity status and the readiness checks below
before requesting motion. In
the WebRTC client, connect to **131.123.237.31** once Isaac is ready. Unity, Play
mode and your selected Quest Link / Meta XR Simulator runtime are not changed.

Press **Ctrl+C** in the launcher terminal to stop the services from this launch
and close its tunnel. Close/disconnect the viewer separately. The launcher
refuses occupied ports or a detected existing demo; it does not replace running
services or stop other GPU jobs. Stop an earlier manual stack using its own
terminals before switching to this launcher.

Useful options:

```powershell
# Print the launch plan without starting or connecting anything.
.\scripts\Start-Demo.ps1 -DryRun
# Start the stack without opening the optional WebRTC application viewer.
.\scripts\Start-Demo.ps1 -NoViewer
# Override the installed workstation profile when needed.
.\scripts\Start-Demo.ps1 -Remote 'user@workstation' -RemoteRepo '~/path/to/repo' -PublicIp '192.0.2.10'
```

This starts an **already installed** stack. It does not install dependencies,
pull repository updates, download the Isaac Docker image, or change firewall
rules. It transfers the launcher itself over the single SSH connection, so the
installed workstation does not need a separate copy of this new launcher.
When upgrading, deploy the matching Unity and remote project revision first,
including `sim/head_control.py`, `sim/head_stereo.py`, `sim/loop_timing.py`,
`ros2/scripts/camera_view_relay.py` and `ros2/scripts/run_endpoint.py`.
The launcher checks that these installed files exist; it does not synchronize
or prove that their contents match the Windows checkout.
Per-service logs are retained on the workstation under
`<repository>/.codex/demo-sessions/<timestamp>-<session>/`.
The manual commands below remain available for debugging individual services.

## 1. Open four workstation sessions

Open four Windows PowerShell tabs, labeled **A–D**. Run this in each:

```powershell
ssh marnett5@mlworkstation.atr.cs.kent.edu
```

If a remote prompt shows a Conda environment such as `(base)`, run `conda deactivate`; repeat until Conda is inactive. Then paste this common environment into **every** remote tab, including later diagnostic sessions:

```bash
cd ~/Developer/deictic-k1-reproduction
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE
export PYTHONPATH="$PWD/ros2/src/deictic_control:$PWD/ros2/src/deictic_registration:$PWD/.codex/ros2/vendor/ROS-TCP-Endpoint:${PYTHONPATH:-}"
```

Use the explicit Python executables below. Jazzy requires the installed Python 3.12 environment; Conda Python 3.13 cannot load its `rclpy` extension. Do not substitute WSL's `LARGE_DATA` transport for `UDPv4`.

Before a fresh start, inspect existing services and GPU use:

```bash
docker ps --format '{{.Names}}\t{{.Image}}\t{{.Status}}'
ss -ltnup
nvidia-smi
```

An existing demo on TCP **10000**, TCP **49100**, or UDP **47998** should be reused or stopped through its own terminal. The launchers reject occupied ports and never kill another process. Leave unrelated containers and GPU jobs alone.

## 2. Terminal A — Isaac Sim

```bash
bash sim/run_isaac_container.sh --synthetic-headset \
  --headset-resolution-scale 2 --textured-table --floor-profile matte \
  --camera-mount-profile reach-projection-balanced \
  --webrtc --public-ip 131.123.237.31 \
  --output /workspace/deictic/sim/artifacts/reach_head1280_matte_live
```

Wait for `DEICTIC_K1_READY` and camera publication; startup can take several minutes. Keep the terminal open. Preserve these flags: they select the native 1280×960 headset RGB/depth, 640×480 wrist image, matte floor and fixed wrist-camera fixture used by the registration profile. The `/workspace/deictic` output path is inside the container and maps to this repository on the workstation.

Both arms and the neck yaw/pitch joints are available; the base and legs remain fixed. Unity targets 50 Hz neck commands while Robot POV is selected, independently of the 20 Hz arm clutch publisher. Starting this command alone requests no arm or neck motion.

The head-stereo source defaults to **320×240 per eye**, with a **15 Hz** publication cap. Add `--head-stereo-width 640` to the Isaac command for native 640×480 eyes; the display relay then reduces them to at most 480×360 per eye. `--head-stereo-rate 15` sets the source cap explicitly (supported range 1–30 Hz). These are configuration values, not measured delivery guarantees. The head render products stay enabled continuously while their image/CameraInfo topics have subscribers and stop when demand ends. They are not repeatedly disabled between pairs, and they do not render while idle without subscribers.

Physics keeps its authored 1/120-second step. Bounded catch-up follows Isaac's observed world time, with capped accumulated lag, so a render stall does not replay an unlimited backlog of old simulation steps. See [simulation timing](../sim/README.md#robot-head-stereo-display) for the limits and measurement tool.

## 3. Terminal B — simulation trajectory/teleoperation relay

```bash
/usr/bin/python3 sim/trajectory_relay.py
```

This process consumes measured joints and arbitrates deictic trajectories versus bimanual teleoperation before sending commands to Isaac. It runs on the host, outside the container.

## 4. Terminal C — ROS TCP endpoint, controller and stereo display relay

```bash
DEICTIC_PYTHON="$PWD/.venv-control/bin/python" \
DEICTIC_CONTROL_PARAMS="$PWD/ros2/src/deictic_registration/config/isaac_head1280_cuda.yaml" \
  bash ros2/scripts/run_bridge.sh markerless
```

Keep `markerless` explicit: omitting it starts mock mode. This command starts three services and binds the ROS TCP endpoint to workstation `127.0.0.1:10000`. It does not start the separate trajectory relay or registration provider.

The display relay publishes the paired head view as a quality-80 JPEG on `/deictic/camera_view/stereo/image_raw/compressed`, the current Unity default. Both eyes travel together with the source acquisition stamp. The raw `/deictic/camera_view/stereo/image_raw` output remains available on demand for legacy consumers. Deploy the matching Unity settings/scripts and relay together; the launcher does not update them. Unity decodes the latest received frame each update instead of waiting on a separate 5 Hz polling timer.

`run_bridge.sh` starts the endpoint through the tracked `ros2/scripts/run_endpoint.py` compatibility wrapper. The pinned ROS-TCP-Endpoint ROS2v0.7.0 lacks the Connector's subscriber-removal command; without the wrapper, returning to User view can disconnect the shared ROS connection. Deploy both files and restart this terminal when upgrading. The wrapper preserves the pinned dependency and adds that missing command; do not bypass it with the upstream endpoint module.

## 5. Terminal D — learned registration

```bash
.venv-registration/bin/python -m deictic_registration.node --ros-args \
  --params-file ros2/src/deictic_registration/config/isaac_head1280_cuda.yaml
```

Wait for initialization and any first-run weight download. This profile uses CUDA, 512 keypoints, LightGlue filter 0.8, reliable camera delivery, and the original one-second registration lifetime. Run only one provider.

## 6. Check readiness

Open another SSH session and apply the common environment from step 1, then run:

```bash
ros2 topic echo /deictic/status --once
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
```

Stop `topic hz` with Ctrl+C. For deictic reaching, look for `mode: markerless`, `backend: gtsam_isam2`, `can_commit: true`, and fresh registration age below one second. At idle, `executing` and `teleop_active` should be false. Independently confirm that `/joint_states` is producing fresh stamped feedback with all eight arm names; topic discovery alone is insufficient.

**`teleop_ready: false` before Unity sends a fresh tracked idle/release heartbeat is expected.** It does not mean the ROS connection or visual registration failed. Bimanual readiness additionally requires execution permission, fresh eight-joint feedback, a fresh `/k1/teleop/relay_status` acknowledgement, and fresh tracked controller inputs; it does not depend on learned camera alignment. These checks do not enter Unity Play mode or send motion requests.

The absolute pose implementation reports `teleop_protocol_version: 3`, `teleop_translation_scale` (default approximately 0.559), and the human/robot shoulder mapping. Update/restart Unity, the controller, trajectory relay and Isaac together; earlier arm protocols are rejected. The one-command launcher transfers only itself and reuses the installed remote project. The neck acknowledges commands on `/k1/head/status` and publishes measured yaw/pitch in `/joint_states`; its fresh clock/tracking requirements are separate from visual registration. Use `.venv-control/bin/python`, which includes NumPy 1.26.4 and SciPy 1.11.4; do not start ROS nodes with Conda's Python.

## 7. Windows — SSH tunnel for ROS

In a separate Windows PowerShell tab, run and leave open:

```powershell
ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 -o ServerAliveCountMax=3 -L 127.0.0.1:10000:127.0.0.1:10000 marnett5@mlworkstation.atr.cs.kent.edu
```

No output after authentication is normal. This forwards Windows `127.0.0.1:10000` to the remote ROS endpoint. If that local port is already occupied by the existing tunnel/helper, reuse it rather than starting another tunnel. A WSL mock endpoint on the same Windows port also conflicts.

## 8. Windows — WebRTC viewer

```powershell
Set-Location 'C:\Users\ATR Lab\Documents\GitHub\Deictic-Robot-Demo-Unity'
.\scripts\Start-IsaacStream.ps1 -Server '131.123.237.31'
```

The helper opens the installed client. In its connection screen, enter **131.123.237.31** and connect; signaling is **TCP 49100**, media is **UDP 47998**. Use one client per Isaac instance. After an Isaac restart, disconnect/reconnect the viewer, or use **Connection → Reload (F5)** if it retained the old session.

WebRTC connects directly to the workstation; the ROS SSH tunnel does not carry its UDP media. If the network/NAT address has changed, follow the [firewall instructions](setup.md#4-configure-networking) using the current SSH-observed Windows source address. The headset-filling robot POV uses a separate ROS head-stereo feed; the WebRTC application viewer shows Isaac's desktop and does not supply the headset image.

Unity remains as the user left it throughout this procedure.

## Stop or restart the foreground stack

With the simulator idle, Ctrl+C **D**, then **C**, then **B**, then **A**. The bridge script stops its own children, and Isaac's container uses `--rm`. Disconnect the WebRTC client and Ctrl+C the Windows tunnel when finished.

If a terminal was lost, identify the exact owned `deictic-k1-...` container with `docker ps` before using `docker stop CONTAINER_NAME`; do not stop unrelated instances. Restart the trajectory relay whenever Isaac restarts so retained targets cannot replay into a reset scene. For a clean session, restart A → B → C → D, reconnect WebRTC, and repeat readiness checks. Do not layer these commands on top of an already-running stack.
