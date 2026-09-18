# Start the installed demo stack manually

This guide uses the already-installed **native Ubuntu workstation** profile. WSL is not needed. It starts Isaac, ROS, the Windows tunnel and the WebRTC viewer; it does **not** open Unity or change Play mode. For a new installation, follow the [complete setup guide](setup.md). The account, addresses and paths below are the tested lab installation; substitute your own on another machine.

Use one instance of each service. If the stack is already running, reuse it and go to the readiness checks instead of starting duplicates. Authenticate to SSH interactively; do not save passwords in scripts.

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

Both arms are available, while the base, legs and neck remain fixed. The separate head-stereo display source is enabled on demand by default; no head-view subscribers means no extra head-camera rendering. This command does not request arm motion.

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

The corrected pose implementation reports `teleop_protocol_version: 2` and `teleop_translation_scale` (default approximately 0.559). Update/restart the controller and trajectory relay together, then reload the updated Unity scripts before starting Play. Old position-only Unity input is rejected explicitly. Use `.venv-control/bin/python`, which includes NumPy 1.26.4 and SciPy 1.11.4; do not start ROS nodes with Conda's Python.

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

WebRTC connects directly to the workstation; the ROS SSH tunnel does not carry its UDP media. If the network/NAT address has changed, follow the [firewall instructions](setup.md#4-configure-networking) using the current SSH-observed Windows source address. The ROS head-stereo monitor and the WebRTC application viewer are separate feeds.

Unity remains as the user left it throughout this procedure.

## Stop or restart the foreground stack

With the simulator idle, Ctrl+C **D**, then **C**, then **B**, then **A**. The bridge script stops its own children, and Isaac's container uses `--rm`. Disconnect the WebRTC client and Ctrl+C the Windows tunnel when finished.

If a terminal was lost, identify the exact owned `deictic-k1-...` container with `docker ps` before using `docker stop CONTAINER_NAME`; do not stop unrelated instances. Restart the trajectory relay whenever Isaac restarts so retained targets cannot replay into a reset scene. For a clean session, restart A → B → C → D, reconnect WebRTC, and repeat readiness checks. Do not layer these commands on top of an already-running stack.
