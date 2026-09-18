# Deictic Robot Demo

Mixed-reality reaching and bimanual teleoperation for **Meta Quest 3S** and the **Booster K1**, built with Unity, ROS 2 and Isaac Sim.

The project adapts *Deictic Shared-Autonomy Manipulation via Continuous Markerless Egocentric MR-to-Robot Frame Fusion* to a fixed-base K1. Point at a tabletop target, inspect the planned motion, then execute it. Alternatively, hold both controller triggers to move the robot's arms through pose IK. A world-fixed UI switches between the user's view and the robot's stereo head cameras.

**Simulation first:** the rendered-image registration, reaching and scripted teleoperation paths have been exercised against Isaac physics. This repository does not include a physical K1 motor driver or reproduce the paper's participant study.

[Set up from scratch](docs/setup.md) · [Start the installed demo](docs/start-demo.md) · [Teleoperation controls](docs/arm-teleoperation.md) · [Validation record](docs/validation.md)

## What is included

| Capability | Implementation |
| --- | --- |
| Deictic reaching | Controller pointing, head-direction input, target/trajectory preview, explicit Execute and Cancel |
| Markerless frame alignment | SuperPoint/LightGlue matches, RGB-aligned depth, PnP and GTSAM iSAM2 fusion |
| Bimanual teleoperation | Both-trigger clutch, current-head-relative controller poses, measured-state bounded IK, soft orientation tracking and no-jump rearming |
| Robot camera view | On-demand, paired head-camera images; one view per XR eye; world-fixed ray-selectable toggle |
| K1 simulation | Official pinned URDF/meshes, eight movable arm joints, fixed trunk/legs/head, measured joint feedback and simulated cameras |
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

## Controls

| Action | Controller | Desktop |
| --- | --- | --- |
| Point | Right-controller ray | Mouse when the right controller is untracked |
| Preview a reach | Press/release right index trigger alone | Space |
| Execute preview | A | Enter |
| Cancel / hold | B | Escape |
| Switch user / robot view | Aim at the world-fixed button and press/release trigger | V |
| Teleoperate arms | Hold both index triggers; move the corresponding controllers | Two simulated controllers |
| End teleoperation | Release either trigger; release both before restarting | Same |

Teleoperation captures the current controller and robot poses when the clutch begins. Subsequent controller translation and rotation modify those captured targets. K1 has **four joints per arm**, so position takes priority and orientation is a soft objective. The [teleoperation guide](docs/arm-teleoperation.md) explains scaling, frame conversion, status, limits and the adaptation from [Unitree xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate).

## Validation and limits

The [dated validation record](docs/validation.md) separates successful checks, failed experiments and remaining gaps:

- A five-target markerless simulation traversal completed with **2.62–4.87 mm measured tool error** in the declared static workcell.
- Protocol-v2 teleoperation passed **53 Unity EditMode checks**, a **176-check native controller/relay/verifier selection**, focused follow-up regressions, and live inward-motion, small-rotation, timeout and rearming checks. A separate scripted Unity → ROS → Isaac test also passed.
- A Quest Link check confirmed the world-fixed camera toggle, controller-ray interaction and image visibility through both eyes.

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
