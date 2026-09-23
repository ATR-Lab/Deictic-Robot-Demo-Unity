# Deictic Robot Demo

Mixed-reality reaching and bimanual teleoperation for **Meta Quest 3S** and the **Booster K1**, built with Unity, ROS 2 and Isaac Sim.

The project adapts *Deictic Shared-Autonomy Manipulation via Continuous Markerless Egocentric MR-to-Robot Frame Fusion* to a fixed-base K1. Point at a tabletop target, inspect the planned motion, then execute it. Alternatively, hold both controller triggers to move the robot's arms through pose IK. A world-fixed button switches between the user's view and an immersive, headset-filling stereo view from the robot's head cameras.

**Simulation first:** the rendered-image registration, reaching and scripted teleoperation paths have been exercised against Isaac physics. This repository does not include a physical K1 motor driver or reproduce the paper's participant study.

[Set up from scratch](docs/setup.md) · [Start the installed demo](docs/start-demo.md) · [Teleoperation controls](docs/arm-teleoperation.md) · [Validation record](docs/validation.md)

## What is included

| Capability | Implementation |
| --- | --- |
| Deictic reaching | Controller pointing, head-direction input, target/trajectory preview, explicit Execute and Cancel |
| Markerless frame alignment | SuperPoint/LightGlue matches, RGB-aligned depth, PnP and GTSAM iSAM2 fusion |
| Bimanual teleoperation | Both-trigger absolute pose acquisition in a stable body frame, measured-state bounded IK and soft orientation tracking |
| Robot camera view | Headset yaw/pitch steer the simulated neck in Robot POV; paired head-camera video fills both eyes; a world-fixed button returns to User view |
| K1 simulation | Official pinned URDF/meshes, eight movable arm joints plus two neck joints, fixed trunk/legs, measured joint feedback and simulated cameras |
| Remote operation | Unity ROS TCP traffic over SSH; a separate Isaac WebRTC application stream |

The controller enforces joint/motion limits, conservative geometry checks, input and feedback freshness, and relay ownership acknowledgements. These are simulation controls, not a physical robot safety certification.

## Quick start

Use the same repository revision on Windows and the Ubuntu GPU workstation:

```bash
git clone https://github.com/ATR-Lab/Deictic-Robot-Demo-Unity.git
```

1. Follow the [complete setup guide](docs/setup.md) to install dependencies and configure networking. The tested stack is Unity **6000.6.0f1**, Meta XR **205**, Ubuntu **24.04**, ROS **Jazzy** and Isaac Sim **5.0.0**.
2. Start the four workstation processes using [the run commands](docs/start-demo.md): Isaac, the trajectory/teleoperation relay, the ROS endpoint/controller/display relay, and learned registration.
3. Start the Windows SSH tunnel. Connect the Isaac WebRTC client directly to the workstation if you want the simulator application view.
4. Open the nested **`Deictic-Robot-Demo`** Unity project, then `Assets/Scenes/DeicticDemo.unity`. Activate Meta XR Simulator with the Quest 3S profile and enter Play mode. The default Unity endpoint is `127.0.0.1:10000` through the tunnel.

ROS and Isaac run on native Ubuntu; WSL is optional for the [lightweight mock workflow](docs/setup.md#9-optional-wsl-mock-demo). WebRTC media does not pass through the ROS SSH tunnel. The runbook explains ports, readiness, shutdown and recovery.

For the already installed lab stack, run `powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\Start-Demo.ps1` from the repository root on Windows. It starts the remote services, ROS tunnel and WebRTC viewer while leaving Unity Play mode to you. See [one-command startup and shutdown](docs/start-demo.md#one-command-windows-launcher).

## Controls

| Action | Controller | Desktop |
| --- | --- | --- |
| Point | Right-controller ray | Mouse when the right controller is untracked |
| Preview a reach | Press/release right index trigger alone | Space |
| Execute preview | A | Enter |
| Cancel / hold | B | Escape |
| Switch user / robot view | Aim at the world-fixed button and press/release trigger | V |
| Look around as the robot | Turn/tilt the headset while Robot POV is selected | Move the simulated headset |
| Teleoperate arms | Hold both index triggers; move the corresponding controllers | Two simulated controllers |
| End teleoperation | Release either trigger; release both before restarting | Same |

Pressing both triggers acquires the controllers' current absolute poses through the human-to-K1 shoulder mapping. Commands start from measured joints and approach those goals under motion limits; reclutching acquires the new poses. Raw trigger values use a 0.5 press threshold and 0.1 full-release threshold; one sample with both fully released rearms the clutch. The complete two-trigger gesture is consumed so it cannot also click the view button. Looking around does not rotate the arm-control frame. K1 has **four joints per arm**, so position takes priority and orientation is a soft objective. The [teleoperation guide](docs/arm-teleoperation.md) explains proportions, protocol v3, limits and the adaptation from [Unitree xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate).

Robot view takes over the headset image. Turning the headset drives the simulated neck within K1's yaw/pitch limits, independently of the arm clutch, and its attached stereo cameras turn with it. Head commands target **50 Hz**, independently of the arms' **20 Hz**. The return button stays in the world and remains available if images are missing or stale; V also switches views on desktop. The stereo source defaults to **320×240 pixels per eye**, capped at **15 Hz**, and the display sends an atomic side-by-side JPEG at quality 80. Unity decodes the latest received frame without a separate 5 Hz polling limit. These are configured rates, not guarantees of delivery rate or motion-to-display latency. The stereo baseline is provisional and the cameras retain their ordinary field of view. See [Unity and Quest interaction](docs/unity-bridge.md#camera-view-toggle).

## Validation and limits

The [dated validation record](docs/validation.md) separates successful checks, failed experiments and remaining gaps:

- The September 23 latency/reclutch update passed **96 project EditMode checks** and controller/simulator regressions. Measured stereo delivery increased from **1.48 to 9.33 Hz** on the workstation; source-stamp-to-ROS-receipt median fell from **299 to 41 ms**. The default now trades display resolution for responsiveness. Full neck settling remains around half a second under rendering load; see [measurement scope and limitations](docs/validation.md#head-latency-and-repeated-trigger-acquisition--2026-09-23).
- A five-target markerless simulation traversal completed with **2.62–4.87 mm measured tool error** in the declared static workcell.
- Protocol-v3 absolute arm acquisition and moving-head control passed **72 unique Unity EditMode checks**, a **227-check native controller/simulator selection**, focused regressions, and live Unity–ROS–Isaac integration with the native Meta XR Simulator display. Scripted head/arm motion and native OVR head-pose delivery are verified separately; see [the dated results](docs/validation.md#head-tracking-and-absolute-arm-acquisition-protocol-v3--2026-09-22).
- The immersive stereo POV passed GPU, offline rendering and native simulator display checks; live Isaac head-camera movement and return-to-User-view now pass too. The earlier Android APK predates head/arm v3 and must be rebuilt. Physical Quest display testing remains pending for this presentation.

These results do not establish physical K1 performance. Native simultaneous-trigger delivery in Meta XR Simulator still needs a manual check; scripted tests exercise the production clutch/bridge but do not replace that input test. Standalone Quest camera capture, physical calibration, arbitrary-scene alignment and the paper's user study remain unvalidated. The robot-head baseline is a virtual simulation assumption. Head direction substitutes for eye gaze, the controller substitutes for the stylus, and speech recognition is not configured.

The original right-arm deictic planner controls position only. Bimanual pose IK is a separate mode. Neither mode supplies walking, balance, grasping or force control.

## Repository guide

| Path | Purpose |
| --- | --- |
| [`Deictic-Robot-Demo/`](Deictic-Robot-Demo) | Unity project, Meta XR interaction, ROS bridge, shaders and tests |
| [`ros2/src/deictic_control/`](ros2/src/deictic_control) | Frame fusion, reaching planner and bimanual IK/controller |
| [`ros2/src/deictic_registration/`](ros2/src/deictic_registration) | Learned image registration and sensor profiles |
| [`ros2/scripts/`](ros2/scripts) | ROS launchers, camera display relay and endpoint setup |
| [`sim/`](sim) | Isaac scene, command relay, camera tools and simulation checks |
| [`models/K1/`](models/K1) | Pinned Booster URDF/meshes and asset provenance |
| [`scripts/`](scripts) | Windows build, WebRTC and firewall helpers |
| [`docs/`](docs) | Setup, interfaces, paper review, experiments and validation history |

Unity caches, virtual environments, build output and generated experiment captures are excluded from Git. Model/package provenance is retained with the vendored assets; pretrained registration weights download separately.

## Documentation

- [Complete setup](docs/setup.md) and [daily startup/shutdown](docs/start-demo.md)
- [Unity/Quest interaction and frames](docs/unity-bridge.md)
- [Bimanual pose IK and protocol](docs/arm-teleoperation.md)
- [ROS control runbook](ros2/README.md), [registration provider](ros2/src/deictic_registration/README.md) and [Isaac simulation](sim/README.md)
- [Paper review](docs/paper-review.md), [experiment plan](docs/experiment-plan.md) and [validation history](docs/validation.md)
- [Booster asset provenance](models/K1/PROVENANCE.md)
- [Third-party components and notices](THIRD_PARTY_NOTICES.md)
