# Complete setup: Quest 3S + Booster K1

For an overview, see the [project README](../README.md). If dependencies are already installed, use [the shorter startup runbook](start-demo.md).

This project adapts the supplied paper's mixed-reality reaching method to a Meta Quest 3S and a fixed-base Booster K1 in Isaac Sim. Unity handles pointing, target previews, execution requests, controller-driven bimanual IK, and the user/robot camera toggle. ROS 2 estimates camera alignment and plans arm motion. Isaac provides physics, measured joints, and simulated cameras.

**Start with simulation.** The setup below uses actual rendered images with SuperPoint/LightGlue, depth/PnP, and GTSAM iSAM2. A five-target simulation traversal has passed. Physical camera calibration, physical K1 control, and the paper's user study remain unfinished. Quest head direction and a controller replace eye gaze and the stylus. Deictic reaching uses four right-arm joints and controls position only; bimanual teleoperation uses both arms with position-prioritized pose IK. Speech recognition is not configured.

## Contents

- [1. Machines and prerequisites](#1-machines-and-prerequisites)
- [2. Get the project onto both machines](#2-get-the-project-onto-both-machines)
- [3. Install workstation dependencies](#3-install-workstation-dependencies)
- [4. Configure networking](#4-configure-networking)
- [5. Start the simulation stack](#5-start-the-simulation-stack)
- [6. Open Unity and use the demo](#6-open-unity-and-use-the-demo)
- [7. Stop and restart](#7-stop-and-restart)
- [8. Build and connect a physical Quest](#8-build-and-connect-a-physical-quest)
- [9. Optional WSL mock demo](#9-optional-wsl-mock-demo)
- [10. Verification and troubleshooting](#10-verification-and-troubleshooting)
- [Further documentation](#further-documentation)

## 1. Machines and prerequisites

The main setup runs ROS and Isaac on **native Ubuntu**, with Unity and the WebRTC viewer on **Windows**. WSL is optional for the mock demo; it is not needed between Windows and the remote workstation.

| Machine | Runs here | Tested configuration |
| --- | --- | --- |
| Windows host | Unity, Meta XR Simulator, SSH tunnel, Isaac WebRTC client | Unity **6000.6.0f1**, Meta XR SDK/Simulator **205** |
| Ubuntu workstation | Isaac container, trajectory relay, ROS endpoint/controller/display relay, learned registration | Ubuntu **24.04**, ROS **Jazzy**, host Python **3.12**, Isaac Sim **5.0.0** |
| Optional Quest 3S | Quest Link editor session or Android application | Link camera UI confirmed; ARM64/IL2CPP APK built, standalone-device validation pending |

The supplied workstation is `marnett5@mlworkstation.atr.cs.kent.edu`, with documented streaming address `131.123.237.31`. Substitute your own account/address on another installation. Authenticate interactively or use a configured SSH key. Do not put passwords in project files.

**Windows prerequisites:**

- Unity Hub and Unity **6000.6.0f1**. For Quest builds, install its **Android Build Support**, **Android SDK & NDK Tools**, and **OpenJDK** modules.
- Meta XR Simulator v205, already installed on the supplied host. Unity resolves Meta XR SDK **205.0.0**, OpenXR **1.18.0**, and Input System **1.20.0** from the project manifest.
- Isaac Sim WebRTC Streaming Client, already installed on the supplied host. Use a client compatible with Isaac Sim 5.0; see [NVIDIA's client instructions](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/manual_livestream_clients.html).
- Git, PowerShell, and the Windows OpenSSH client (`ssh`).

**Workstation prerequisites:**

- An NVIDIA RTX GPU, compatible driver, and available GPU memory for Isaac rendering plus learned registration. The recorded machine uses a 24 GiB Quadro RTX 6000 with driver 570.211.01; this is the tested configuration, not a minimum-memory guarantee.
- Docker Engine and NVIDIA Container Toolkit configured for GPU containers. Follow [NVIDIA's Isaac Sim 5.0 container installation instructions](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_container.html).
- Native ROS Jazzy and the Python environments in section 3; internet access for packages, the Isaac image, and model weights.

The initial image pull and shader compilation can take several minutes. Check shared GPU use before launching another simulator; leave other users' workloads untouched.

## 2. Get the project onto both machines

Use the **same complete source revision** on Windows and Ubuntu. Clone the repository on each machine, then check out the same commit or branch:

```bash
git clone https://github.com/ATR-Lab/Deictic-Robot-Demo-Unity.git
```

If transferring a source archive instead, preserve `docs`, `models`, `ros2`, `scripts`, `sim`, and the Unity project's `Assets`, `Packages`, and `ProjectSettings`, including Unity `.meta` files. Omit Unity `Library`, `Temp`, `Logs`, and `obj`; `.venv-*`; `.codex`; `output`; and generated `sim/artifacts`. Use a separate destination rather than overwriting a modified checkout.

The examples use the tested lab repository roots below; adjust every `cd`/`Set-Location` if you use another path. The clone command above normally creates `Deictic-Robot-Demo-Unity`; pass your desired destination as its final argument if you want a different directory name, such as the workstation's `deictic-k1-reproduction`:

| Machine | Repository root |
| --- | --- |
| Windows | `C:\Users\ATR Lab\Documents\GitHub\Deictic-Robot-Demo-Unity` |
| Workstation | `~/Developer/deictic-k1-reproduction` |

The **Unity project is the nested `Deictic-Robot-Demo` folder**, not the repository root. Keep `models/K1` alongside it; asset generation reads that sibling directory.

On Ubuntu, confirm that the implementation is present:

```bash
cd ~/Developer/deictic-k1-reproduction
test -f sim/run_isaac_container.sh && \
test -f ros2/scripts/run_bridge.sh && \
test -f ros2/src/deictic_registration/config/isaac_head1280_cuda.yaml && \
test -f Deictic-Robot-Demo/Assets/Scenes/DeicticDemo.unity && \
echo "Demo source files found"
```

The pinned K1 URDF and 24 meshes are included in `models/K1`. If missing, restore them with:

```bash
python3 sim/fetch_k1_assets.py
```

This restores upstream revision `3c2dfa99e09beddf092e0d6521dbbcec7e7903ed`; see [asset provenance and license](../models/K1/PROVENANCE.md). The supplied ROS-TCP-Connector and URDF-Importer Unity packages are embedded in `Packages`; importing the Reachy packages again is unnecessary. This fixed-arm demo does not require training a locomotion policy or installing the full [Booster training stack](https://github.com/BoosterRobotics/booster_train).

## 3. Install workstation dependencies

Run this section **once on native Ubuntu 24.04**, not inside Isaac's container or the supplied WSL installation. Existing installations can reuse their environments after the import checks below.

If the terminal prompt shows a Conda environment such as `(base)`, run `conda deactivate` before sourcing ROS. Repeat for nested environments until Conda is inactive. Jazzy's native modules require system Python 3.12; sourcing ROS does not replace Conda's Python. The commands below explicitly use `/usr/bin/python3` when creating environments or running host scripts.

First configure Ubuntu's ROS package repository using the [official ROS Jazzy installation instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). Then:

```bash
sudo apt update
sudo apt install git python3-venv python3-pip python3-numpy python3-scipy python3-opencv \
  python3-pil python3-pytest ros-jazzy-ros-base ros-jazzy-message-filters \
  ros-jazzy-trajectory-msgs ros-jazzy-rmw-fastrtps-cpp

cd ~/Developer/deictic-k1-reproduction
source /opt/ros/jazzy/setup.bash

/usr/bin/python3 -m venv --system-site-packages .venv-control
.venv-control/bin/python -m pip install numpy==1.26.4 scipy==1.11.4 gtsam==4.2.2

/usr/bin/python3 -m venv --system-site-packages .venv-registration
.venv-registration/bin/python -m pip install numpy==1.26.4
.venv-registration/bin/python -m pip install \
  torch==2.7.0 torchvision==0.22.0 \
  --index-url https://download.pytorch.org/whl/cu126
.venv-registration/bin/python -m pip install kornia==0.8.3
.venv-registration/bin/python -m pip install --no-deps \
  git+https://github.com/cvg/LightGlue.git@eb42fee2d71449efb0aa5c10549752b5d75384d8

bash ros2/scripts/setup.sh
```

The CUDA wheel pair follows [PyTorch's versioned installation instructions](https://pytorch.org/get-started/previous-versions/#v270). Both environments inherit native ROS and OpenCV through `--system-site-packages`. **Keep NumPy 1.26.4 in the controller environment:** GTSAM 4.2.2 is incompatible with NumPy 2. Do not install a different `rclpy` from pip or substitute Isaac's bundled Python for the host interpreter.

`setup.sh` only fetches and verifies Unity's ROS-TCP-Endpoint at `ROS2v0.7.0`, commit `54c1a64b6d5ef6ffa0a0431570bb74329b79b15b`, into `.codex/ros2/vendor`. It does not install ROS, Docker, Isaac, or Python dependencies. These launchers run source modules directly; this path does not require `colcon build`.

Check imports and GPU availability:

```bash
.venv-control/bin/python -c "import rclpy, cv2, numpy, scipy, gtsam; print('Control imports OK; NumPy', numpy.__version__, 'SciPy', scipy.__version__)"
.venv-registration/bin/python -c "import rclpy, cv2, message_filters, torch, torchvision, kornia, lightglue; print('Torch', torch.__version__, 'CUDA available:', torch.cuda.is_available())"
nvidia-smi
docker run --rm --gpus all ubuntu nvidia-smi
docker pull nvcr.io/nvidia/isaac-sim:5.0.0
```

The registration check must report `CUDA available: True`. If Docker requires elevated access, configure the intended account's Docker access using the installation guide before using the launcher.

SuperPoint and LightGlue download pretrained weights on first initialization; allow that download before expecting tracking readiness. Weights are cached in the account's Torch cache and are not included in the repository. [Upstream LightGlue documentation](https://github.com/cvg/LightGlue) covers downloads and model licenses.

The simulator launcher sets `ACCEPT_EULA=Y` for the NVIDIA container; review the linked terms before using it. Its ROS bridge uses bundled Python 3.11 libraries. The external ROS processes below use native Jazzy/Python 3.12.

## 4. Configure networking

There are two separate connections:

- **Unity ROS traffic:** Windows `127.0.0.1:10000` → SSH → workstation `127.0.0.1:10000`.
- **Isaac WebRTC:** Windows streaming client → workstation directly. Signaling uses **TCP 49100** and media **UDP 47998**. The SSH tunnel does not carry WebRTC media.

Allow the requested compatibility ranges **inbound and outbound on both machines**:

| Protocol | Ports |
| --- | --- |
| TCP and UDP | `47995-48012` |
| TCP and UDP | `49000-49007` |
| TCP | `49100` |

SSH also needs TCP 22 on its route. The ROS endpoint remains bound to loopback; this setup does not require exposing port 10000 publicly.

### Windows firewall

From the repository root in PowerShell, inspect the proposed peer-scoped rules:

```powershell
Set-Location 'C:\Users\ATR Lab\Documents\GitHub\Deictic-Robot-Demo-Unity'
.\scripts\Enable-WebRTC-Firewall.ps1 -RemoteAddress '131.123.237.31' -WhatIf
```

Apply them from **PowerShell running as Administrator**:

```powershell
Set-Location 'C:\Users\ATR Lab\Documents\GitHub\Deictic-Robot-Demo-Unity'
.\scripts\Enable-WebRTC-Firewall.ps1 -RemoteAddress '131.123.237.31'
```

The script creates or updates four named rules for both protocols/directions, scoped to the workstation. Organization-managed blocking rules may require a network administrator to adjust them.

### Workstation firewall

The supplied helper is for **firewalld's `public` zone**. On another Ubuntu machine, inspect its active firewall first; do not enable firewalld on top of an existing UFW configuration just to run this script.

In a direct SSH session **from Windows to the workstation**, inspect the observed client source address and active zone:

```bash
cd ~/Developer/deictic-k1-reproduction
printf '%s\n' "$SSH_CONNECTION"
sudo firewall-cmd --get-active-zones
```

If the network interface uses the `public` zone, use the first field of `SSH_CONNECTION`:

```bash
read -r windows_source_ip _ <<< "$SSH_CONNECTION"
bash sim/configure_remote_firewall.sh "$windows_source_ip"
```

The helper accepts IPv4 and adds persistent/runtime **inbound** rules scoped to that source. It inspects the output policy but does not change it: outbound traffic must also allow the table's ranges through the active firewall. The supplied workstation previously allowed outbound traffic. If another firewall or a restrictive outbound policy is in use, configure equivalent peer-scoped rules there.

Campus NAT has changed the Windows source address during testing. Use the current observed address, not a Windows/WSL private address or old NAT address. If it changes, update the source rules and reconnect. Intervening campus/network firewalls must also permit the WebRTC route.

## 5. Start the simulation stack

The full stack uses **four workstation terminals**, plus a Windows SSH tunnel and the two Windows applications. Run one instance of each service. Inspect `docker ps` and existing demo terminals first; a second bridge can duplicate ROS publishers or collide on port 10000.

The launchers now check their ports before starting: Isaac with `--webrtc` checks TCP 49100/UDP 47998, and the bridge checks its configured TCP endpoint. If a check fails, inspect and reuse the existing demo or stop its identified processes before relaunching. These checks never terminate another process automatically.

### Common workstation environment

Open four SSH sessions to the workstation. If a shell has Conda active (`(base)`), run `conda deactivate` there before continuing. Run this block in **each** terminal, and in any later diagnostic terminal:

```bash
cd ~/Developer/deictic-k1-reproduction
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE
export PYTHONPATH="$PWD/ros2/src/deictic_control:$PWD/ros2/src/deictic_registration:$PWD/.codex/ros2/vendor/ROS-TCP-Endpoint:${PYTHONPATH:-}"
```

Native `UDPv4` is intentional: the container's host networking and separate IPC namespace caused sample-delivery failures with other settings. Do not carry WSL's `LARGE_DATA` setting into the native stack.

### Terminal A: Isaac Sim

```bash
bash sim/run_isaac_container.sh --synthetic-headset \
  --headset-resolution-scale 2 --textured-table --floor-profile matte \
  --camera-mount-profile reach-projection-balanced \
  --webrtc --public-ip 131.123.237.31 \
  --output /workspace/deictic/sim/artifacts/reach_head1280_matte_live
```

Wait for `DEICTIC_K1_READY` and camera publication. Keep this terminal open. The container mounts the repository at `/workspace/deictic`, so `--output` uses a **container path**, with results under the local `sim/artifacts` directory.

Keep every scene flag: the verified profile uses native **1280×960 headset RGB/depth**, **640×480 wrist RGB**, a textured table, matte floor, and fixed projection-balanced wrist-camera mount. Older CLI defaults select a different fixture. `--synthetic-headset` means Isaac renders the headset input; registration still estimates alignment from images rather than receiving a known transform as its answer.

The ROS-enabled simulator also supplies a separate robot-head stereo display pair by default. Its two 640×480 RGB eyes use the same renderer acquisition and publish at most 5 Hz, only while their image or CameraInfo topics have subscribers. The provisional **64 mm simulation baseline** is an assumed virtual geometry, not physical K1 camera calibration. `--no-head-stereo` disables this optional display source; it does not disable the wrist/headset registration cameras.

The trunk, legs and head are fixed. Both four-joint arms are movable for bimanual teleoperation; the original deictic reaching path uses the right arm. This command does not automatically execute an arm trajectory.

### Terminal B: trajectory relay

```bash
/usr/bin/python3 sim/trajectory_relay.py
```

This host process arbitrates `/k1/arm_controller/joint_trajectory` and `/k1/teleop/command`, then sends `/k1/sim/joint_commands` using measured `/joint_states`. It publishes `/k1/teleop/relay_status` acknowledgements for the teleoperation lease. The explicit system interpreter avoids selecting Conda's Python. Keep it outside Isaac's container, whose bundled ROS Python lacks the required trajectory message package.

### Terminal C: ROS endpoint, controller, and display relay

```bash
DEICTIC_PYTHON="$PWD/.venv-control/bin/python" \
DEICTIC_CONTROL_PARAMS="$PWD/ros2/src/deictic_registration/config/isaac_head1280_cuda.yaml" \
  bash ros2/scripts/run_bridge.sh markerless
```

**Specify `markerless` explicitly.** Omitting the argument starts mock mode. This script starts the TCP endpoint, GTSAM/controller, and head-stereo display relay; it does not start Isaac, the trajectory relay, or learned registration.

### Terminal D: learned registration provider

```bash
.venv-registration/bin/python -m deictic_registration.node --ros-args \
  --params-file ros2/src/deictic_registration/config/isaac_head1280_cuda.yaml
```

Allow first-run model downloads and initialization to finish. Run one provider. This YAML pairs with Terminal A's fixture: CUDA, 512 keypoints, LightGlue filter 0.8, reliable camera delivery, and unchanged one-second registration age limits. Historical CPU profiles in the detailed runbooks use different fixtures/timing and are not substitutes for this command.

### Check workstation readiness

In another terminal with the common environment:

```bash
ros2 topic list
ros2 topic echo /deictic/status --once
ros2 topic hz /joint_states
```

Stop `topic hz` with Ctrl+C. Status should eventually show `mode` = `markerless`, `backend` = `gtsam_isam2`, and `can_commit` = `true`. At idle, `executing` should be false. Discovery alone does not prove messages arrive: check measured joints and fresh registration before using the demo.

### Windows: ROS SSH tunnel

Run in PowerShell and leave it open:

```powershell
ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 -o ServerAliveCountMax=3 -L 127.0.0.1:10000:127.0.0.1:10000 marnett5@mlworkstation.atr.cs.kent.edu
```

No output after authentication is normal. Unity connects to `127.0.0.1:10000`. Use one tunnel on this port; stop any WSL mock endpoint using the same local port. An optional reconnecting helper is in the [ROS runbook](../ros2/README.md).

### Windows: WebRTC viewer

```powershell
Set-Location 'C:\Users\ATR Lab\Documents\GitHub\Deictic-Robot-Demo-Unity'
.\scripts\Start-IsaacStream.ps1 -Server '131.123.237.31'
```

The script opens the installed client and prints the address; **enter the address and connect manually in the client**. Use TCP 49100 if a signaling-port field is shown. If the installation path differs, open the client directly. Use one streaming client per Isaac instance.

WebRTC displays the remote Isaac application. Unity's **Robot camera** view is a separate ROS head-stereo feed from Terminal C's display relay. Put the WebRTC viewer on the second monitor if useful.

## 6. Open Unity and use the demo

1. In Unity Hub, open the repository's **`Deictic-Robot-Demo`** folder with **6000.6.0f1**. Let package import and compilation finish.
2. Open `Assets/Scenes/DeicticDemo.unity`. If generated assets are missing, use **Deictic → Create Demo Scene**. Avoid regenerating an already customized scene. The original `SampleScene` is preserved.
3. Inspect `Assets/Resources/DeicticSettings.asset`: `rosHost = 127.0.0.1`, `rosPort = 10000`, `connectOnStart = true`, `syntheticScene = true`, and `publishHeadsetCamera = false` for simulation. Use `robotCameraTopic = /deictic/camera_view/stereo/image_raw` and `robotCameraStereo = true` for the paired head display. Isaac publishes the camera inputs. Unity's `syntheticScene` setting is independent of ROS registration mode.
4. Activate **Meta → Meta XR Simulator → Activate**. In the simulator, select **Settings → Simulated Headset → Headset Type → Meta Quest 3S**, then restart Play mode if changing device type.
5. The demo uses a stationary `OVRCameraRig`, FloorLevel tracking, and disabled artificial locomotion. **Deictic → Configure Stationary Demo Rig** reapplies these overrides if needed. Do not add an eye-height offset or a second Unity physics controller; Isaac owns robot physics.
6. Enter Play mode and click inside **Game** view so keyboard input reaches Unity. Wait for a connected bridge, clock synchronization, measured state, and accepted alignment.

| Action | Quest/controller input | Desktop input |
| --- | --- | --- |
| Point at a target | Right-controller ray; head direction supplies gaze substitute | Mouse in the synthetic scene when the right controller is untracked |
| Request a plan | Press and release the right index trigger without the left trigger | Space |
| Execute the preview | A | Enter |
| Cancel/hold and clear preview | B | Escape |
| Switch user/robot view | Aim the controller ray at the world-fixed UI button and press/release the right trigger | V; mouse click when using the mouse fallback; Space while pointing at the button |
| Teleoperate both arms | Hold both index triggers; move each controller to move its corresponding arm | Use the two controllers in Meta XR Simulator |
| End teleoperation | Release either trigger, then release both before clutching again | Same in Meta XR Simulator |

With a tracked Meta XR Simulator right controller, aim its controller ray; mouse position does not override that ray. Space still requests a plan at the ray's target, and V switches views independently. Mouse pointing/clicking is the fallback when the right controller is untracked.

Bimanual teleoperation captures controller and measured robot poses at each clutch, then follows scaled relative translation and rotation through bounded pose IK. Controller poses use the current head-facing frame; the four-joint arms prioritize position with a soft orientation objective. It clears pending reaches and holds both arms on release or input loss, while keeping the base, legs, and neck fixed. See [arm teleoperation](arm-teleoperation.md) for the Unitree reference adaptation, protocol-v2 update, limits, and simulator verification workflow.

In **User view**, point at one of five target spheres, request a plan, inspect the committed-target marker and ghost trajectory, then execute. Each execution needs a valid preview; tracking/alignment failures block a new commit. The status panel explains rejected requests.

The **User view / Robot camera** button and monitor stay fixed in the Unity world rather than following the head. The default toggle center is `(0, 1.25, 0.8)` meters, configurable through `cameraControlsWorldPosition`; the monitor sits above it. Hover with the controller ray and press Trigger to switch. That click takes precedence over target selection and never requests motion. In the editor, User view is the simulated workcell. On Quest, User view enables passthrough with virtual overlays over the wearer's surroundings. Robot view shows the robot's forward-facing head cameras: each XR eye sees its corresponding camera image, while a desktop/mono view shows the left eye. Select targets in User view; displayed camera pixels do not select targets. A/Enter can execute an existing valid preview, and B/Escape still cancels.

The panel subscribes to `/deictic/camera_view/stereo/image_raw` only while visible. This is one atomic RGB8 image with the left eye in the left half and right eye in the right half, up to **960×360 total / 480×360 per eye at 5 Hz**. The relay requires equal-size images with exactly matching acquisition stamps; it never substitutes an unmatched eye or wrist frame. Both source subscriptions stop when the viewer closes, so the simulator also stops these extra renders unless another head-camera consumer remains. Maximum display pixel payload is 5.184 MB/s (about 41.5 Mbit/s), before TCP/SSH overhead. A missing/stale frame is labeled and hidden after the configured 1.5-second viewer timeout; the return button remains available. The full-resolution wrist camera remains dedicated to learned registration with the headset RGBD input.

## 7. Stop and restart

1. Cancel/hold in Unity with B/Escape, and confirm execution has stopped before replacing control processes.
2. Exit Unity Play mode and disconnect the WebRTC client.
3. Ctrl+C the registration provider, then bridge, then trajectory relay, then Isaac. The bridge cleans up its own three child processes.
4. Ctrl+C the Windows SSH tunnel when finished.

The container uses `--rm` and a name beginning `deictic-k1-`. If its terminal was lost, identify **your exact demo container** with `docker ps` and stop only it with `docker stop CONTAINER_NAME`. Do not stop unrelated containers or other users' GPU jobs.

**Restart the trajectory relay whenever Isaac restarts:** it retains its last trajectory target, which must not be replayed into a reset scene. For a clean session, restart all four processes in section 5's order.

The GTSAM graph stops accepting updates at 10,000 accepted keyframes, roughly 50 minutes at nominal 3.33 Hz. It fails closed rather than silently discarding state; cancel and restart the session at capacity.

## 8. Build and connect a physical Quest

This produces the simulation application's Quest APK. It does not add a physical K1 driver or calibrate physical cameras.

Install Unity's Android modules from section 1. **Close the editor using this project** before a batch build; stopping Play mode alone is insufficient. Alternatively, pass a separate complete project copy with `-ProjectPath`.

From Windows PowerShell:

```powershell
Set-Location 'C:\Users\ATR Lab\Documents\GitHub\Deictic-Robot-Demo-Unity'
.\scripts\Build-Quest.ps1
```

The script builds ARM64/IL2CPP with Android OpenXR, Quest 3S support, and ROS2 serialization. Output is `output/DeicticK1.apk`; the log is `output/quest-build.log`. It applies the Java temporary-directory workaround needed on this host. Build output is ignored by Git, so a fresh copy may not include an APK.

For the simplest headset route, keep the Windows SSH tunnel running and connect Quest by USB. Enable developer mode and accept its USB debugging prompt, then:

```powershell
$adb = 'C:\Program Files\Unity\Hub\Editor\6000.6.0f1\Editor\Data\PlaybackEngines\AndroidPlayer\SDK\platform-tools\adb.exe'
& $adb devices
& $adb install -r '.\output\DeicticK1.apk'
& $adb reverse tcp:10000 tcp:10000
```

Open the installed application on Quest. With USB reverse, the APK's `127.0.0.1:10000` reaches Windows and then SSH. Reapply `adb reverse` after reconnecting if needed. For wireless use, configure a reachable endpoint and routing before rebuilding; headset localhost does not refer to the workstation. See the [Quest/endpoint runbook](unity-bridge.md).

For **Quest Link**, run the Unity editor on this PC instead: connect/enter Link,
keep the Windows SSH tunnel running, leave **Meta > Meta XR Simulator** inactive,
and Play `DeicticDemo` with Meta's OpenXR runtime selected. The editor connects to
`127.0.0.1:10000` directly through the tunnel. This route does **not** use an APK,
ADB installation, or `adb reverse`. The headset mirror's **Left Eye** setting
only chooses the desktop preview; both eyes should still render in the headset.

Passthrough presentation does not enable physical RGB/depth capture. Keep `publishHeadsetCamera=false` while Isaac supplies inputs. The physical phase still needs a calibrated wrist camera, headset image/depth/pose acquisition, hand-eye calibration, and a separately validated hardware-control path.

## 9. Optional WSL mock demo

This checks Unity interaction and ROS TCP transport **without Isaac or a GPU**. It supplies known reference alignment and mock joint feedback; it does not run learned registration or physics.

Stop the remote SSH tunnel and any conflicting endpoint first. In the **supplied WSL Ubuntu installation**:

```bash
cd '/mnt/c/Users/ATR Lab/Documents/GitHub/Deictic-Robot-Demo-Unity'
source /opt/ros/lyrical/setup.bash
bash ros2/scripts/setup.sh
bash ros2/scripts/run_bridge.sh mock
```

This WSL installation uses ROS Lyrical/Python 3.14; these commands are specific to that machine. Fresh WSL installations need their own supported ROS distribution, NumPy, and OpenCV. Do not install native Python 3.12/GTSAM pins into WSL's Python 3.14 environment or downgrade system NumPy.

Use Unity's localhost endpoint and the same scene. Status must identify mock/reference mode. Without a camera publisher, Robot view shows a missing feed. Do not run mock feedback alongside Isaac in the same domain. Stop with Ctrl+C before returning to native simulation.

For comparison, `run_bridge.sh synthetic` uses real GTSAM with known simulated camera observations and external joint feedback. Only **`markerless` plus the separate learned provider** follows the image-estimation path above.

## 10. Verification and troubleshooting

The recorded full-loop check traversed targets 1→5 with a 4.5-second preview review per execution. It measured 0.17–0.78 mm grounding error and 2.62–4.87 mm tool error in a static simulated workcell, starting from a previous target-2 pose rather than standardized neutral. These are engineering checks, not physical results or a reproduction of the paper's mobile-user task. See the [validation record](validation.md) for conditions, failed experiments, and artifacts.

### Repeatable source checks

On Ubuntu, after section 5's common environment block:

```bash
/usr/bin/python3 -m unittest discover -s sim
.venv-control/bin/python -m pytest \
  ros2/src/deictic_control/test/test_core.py \
  ros2/src/deictic_control/test/test_node_tracking.py \
  ros2/src/deictic_control/test/test_node_execution.py -q
.venv-registration/bin/python -m pytest \
  ros2/src/deictic_registration/test/test_registration.py -q
```

In Unity, use the Test Runner's **EditMode** tests for `Deictic.Tests`. The latest teleoperation correction passed 53 Unity EditMode cases and a separate scripted Unity-to-Isaac PlayMode check; earlier reaching and camera suites are recorded separately in [the validation history](validation.md). The commands above cover the core reaching/registration selection. Follow [the teleoperation guide](arm-teleoperation.md) for its additional source and live-motion checks. Live tests have explicit setup and opt-in requirements.

An optional standalone simulator smoke check, with the continuous stack stopped:

```bash
bash sim/run_isaac_container.sh --no-ros --synthetic-headset --smoke --steps 360
```

**This intentionally moves the simulated arm.** Look for successful `DEICTIC_K1_RESULT` output and no `DEICTIC_K1_FAILED`. This checks import/render/physics rather than the full markerless loop.

### Common problems

| Symptom | Check or recovery |
| --- | --- |
| Port 10000 is already in use | Keep one endpoint and one Windows tunnel. Check for WSL mock or the reconnecting helper before starting another. |
| Unity disconnected | Keep SSH alive, verify Terminal C, and check `127.0.0.1:10000` in settings. Quest also needs USB reverse or an explicitly reachable route. |
| Topics exist but messages do not arrive | Source Jazzy in every native shell; match domain 42 and `rmw_fastrtps_cpp`; use `UDPv4`, not WSL's `LARGE_DATA`. |
| `No module named rclpy._rclpy_pybind11`, with Python 3.13/Conda in the traceback | Deactivate Conda, source Jazzy again, and run `/usr/bin/python3 sim/trajectory_relay.py`. Jazzy uses Python 3.12; reinstalling `rclpy` into Conda is not the fix. |
| Alignment unavailable/stale | Check Terminal D, weight downloads, CUDA, and all four services. Use the matching YAML and every scene flag. Images, depth, calibration, and poses need coherent capture stamps. |
| `joint_feedback_stale` | Check `/joint_states` flow, Isaac health, GPU load, and transport. Do not mask this with mock feedback. |
| Preview executes but Isaac does not move | Check Terminal B's separate trajectory relay and ROS environment. The bridge does not launch it. |
| `goal_timestamp_out_of_range` or clock sync unavailable | Use current Unity source/APK and wait for fresh clock sync after reconnecting. Accepted RTT must be at most 80 ms; check latency rather than loosening timestamp gates. |
| `robot_busy_cancel_before_replanning` | Cancel/hold before requesting a replacement plan. |
| Robot camera blank | Switch to Robot view to request the head cameras. Check both `/k1/head_camera/left/image_raw` and `/k1/head_camera/right/image_raw`, exact paired stamps, Terminal C's stereo relay, and `robotCameraStereo=true`. The simulator must not use `--no-head-stereo`. Missing/stale pairs are hidden; there is no wrist fallback. This feed is separate from WebRTC. |
| V/Space/Enter do nothing | Click inside Unity's Game view. Automated desktop keystrokes may not reach Unity's input system. |
| WebRTC fails or stays blank | Wait for Isaac, match `--public-ip` to the reachable address, and check TCP 49100, UDP 47998, all requested firewall ranges, current NAT source, GPU encoder support, and other connected clients. Reconnect after restarting Isaac. |
| WebRTC connects but logs `No offer is received from remote` | Check `docker ps` and `sudo ss -lntup` for a stale/duplicate Isaac server. Two instances can bind the streaming port yet fail to create a video session. Remove the identified duplicate, restart the affected demo cleanly if its stream remains stuck, then use **Connection → Reload (F5)** and reconnect in the installed client. Restart the trajectory relay with Isaac. |
| GTSAM import fails or Python crashes | Use native Python 3.12 and `.venv-control` with NumPy 1.26.4/GTSAM 4.2.2. Do not mix NumPy 2 or container ROS libraries into it. |
| CUDA out of memory | Inspect `nvidia-smi` and duplicate demo instances. Free only your own unwanted processes or coordinate GPU availability; leave other users' jobs untouched. |
| Quest build fails on project lock | Close its editor or build from a separate complete copy. |
| Android Java temporary-socket error | Use `scripts/Build-Quest.ps1`, which scopes the temporary-directory workaround to the build. |
| Graph capacity reached | Cancel and restart; the controller intentionally blocks after 10,000 accepted keyframes. |

## Further documentation

- [Start the installed demo](start-demo.md): copy-paste workstation, Windows tunnel, viewer, readiness and shutdown commands.
- [Paper review](paper-review.md): method, adaptations, missing parameters, and numerical caveats.
- [Unity and Quest runbook](unity-bridge.md): frames, camera toggle, builds, and live integration tests.
- [Arm teleoperation](arm-teleoperation.md): two-trigger clutch, relative bimanual IK, and simulator controls.
- [ROS control runbook](../ros2/README.md): modes, pose graph, planning contract, and display relay.
- [Registration provider](../ros2/src/deictic_registration/README.md): sensor contract and historical profiles. Its initial WSL/CPU example is separate from this README's native CUDA setup.
- [Isaac simulation runbook](../sim/README.md): fixed-arm model, cameras, fixture profiles, and capture tools.
- [Experiment plan](experiment-plan.md): remaining baselines and study work.
- [Validation record](validation.md): measured results and limitations.
