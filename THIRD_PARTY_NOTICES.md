# Third-party components and research references

Upstream components retain their own licenses. This document records provenance;
it does not replace those licenses or establish a repository-wide license grant.

The two ROS packages already declare Apache 2.0 in their package metadata;
their license texts are included with [control](ros2/src/deictic_control/LICENSE)
and [registration](ros2/src/deictic_registration/LICENSE).

## Included components

| Component | Location | License and local changes |
| --- | --- | --- |
| Booster K1 URDF and meshes | `models/K1/` | [BSD 3-Clause](models/K1/LICENSE); pinned revision and adaptations in [PROVENANCE.md](models/K1/PROVENANCE.md). The Unity mesh assets and robot prefab are generated from these meshes. |
| Unity ROS-TCP-Connector 0.7.0-preview | `Deictic-Robot-Demo/Packages/com.unity.robotics.ros-tcp-connector/` | [Apache 2.0](Deictic-Robot-Demo/Packages/com.unity.robotics.ros-tcp-connector/LICENSE); [local ROS 2 serialization fix](Deictic-Robot-Demo/Packages/com.unity.robotics.ros-tcp-connector/LOCAL_CHANGES.md). |
| Unity URDF Importer 0.5.2-preview | `Deictic-Robot-Demo/Packages/com.unity.robotics.urdf-importer/` | [Apache 2.0](Deictic-Robot-Demo/Packages/com.unity.robotics.urdf-importer/LICENSE); [local platform metadata changes](Deictic-Robot-Demo/Packages/com.unity.robotics.urdf-importer/LOCAL_CHANGES.md). |
| UnityMeshImporter | Embedded in the URDF Importer | [MIT](Deictic-Robot-Demo/Packages/com.unity.robotics.urdf-importer/Runtime/UnityMeshImporter/LICENSE.md). |
| AssimpNet and native Assimp libraries | Embedded in the URDF Importer | [Retained MIT/BSD notices](Deictic-Robot-Demo/Packages/com.unity.robotics.urdf-importer/Runtime/UnityMeshImporter/Plugins/AssimpNet/License_AssimpNet.txt). |
| V-HACD | Embedded in the URDF Importer | [BSD 3-Clause](Deictic-Robot-Demo/Packages/com.unity.robotics.urdf-importer/Runtime/VHACD/LICENSE.MD). |

The existing Unity project also contains Meta XR/OpenXR settings and a Meta XR
operator API-layer binary. Meta and Unity packages referenced in `manifest.json`
remain subject to their respective upstream terms; the project does not
relicense those SDKs. Unity package caches, the installed Meta XR Simulator,
Quest software and Isaac Sim installations are not included in this commit.

## Downloaded dependencies

The ROS-TCP-Endpoint is downloaded into ignored local state at a pinned revision
by `ros2/scripts/setup.sh`; see its upstream Apache 2.0 license. GTSAM, SciPy,
NumPy, OpenCV, PyTorch and other Python dependencies are installed separately.

The registration frontend installs [LightGlue](https://github.com/cvg/LightGlue)
at the revision documented in the [registration README](ros2/src/deictic_registration/README.md).
LightGlue and the SuperPoint extractor/weights have separate upstream licensing
terms. Pretrained weights and Python environments are not distributed here.

## Research and implementation references

- The user-supplied `Transition-deictic-paper` implementation, specifications,
  tests and configurations are incorporated under `transition/` for this integration.
  [REFERENCE_SOURCE.json](transition/REFERENCE_SOURCE.json) records source hashes.
  Its research PDF archive, generated run evidence and compiled artifacts are not
  included. No new license grant is inferred for supplied material. New integration
  and physical limitations are described in [transition/README.md](transition/README.md).

- The user-supplied paper is described in [the paper review](docs/paper-review.md).
  The supplied manuscript and Reachy demo archive are not redistributed.
- [Unitree xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate/tree/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b)
  informed the pose-IK correction. The K1 implementation and differences are
  described in [arm teleoperation](docs/arm-teleoperation.md); Unitree hardware
  drivers, robot assets and cloned source trees are not included.
- [Isaac Lab teleoperation examples](https://isaac-sim.github.io/IsaacLab/main/source/overview/imitation-learning/teleop_imitation.html)
  provide additional task-space teleoperation context. This project does not
  redistribute Isaac Sim or implement the Mimic data-generation workflow.
