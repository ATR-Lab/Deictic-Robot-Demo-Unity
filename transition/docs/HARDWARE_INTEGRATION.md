# Booster K1 integration: reaching and pointing

See [Physical release status](PHYSICAL_RELEASE_STATUS.md) for the current gate
and API behavior. The prototype description below is retained as provenance;
`BoosterSDKTransport.move` now refuses uncommissioned motion, and receipt-string
equality alone cannot arm the backend.

For the complete transfer procedure, see the [hardware implementation and test specification](HARDWARE_IMPLEMENTATION_SPEC.md). It distinguishes the existing adapter documented here from required production changes, including stop handling, final admission checks, observation timing, device ownership and commissioning. It also provides the staged hardware test matrix and operator runbook.

**Checked 2026-09-23.** The selected robot is the standard **Booster K1**, and the simulator is **Isaac Sim**. The first task uses named, bounded reaching/pointing poses. It requires no gripper, walking, learned balance, or replacement whole-body controller. K1 has 22 joints, including four per arm; requesting a Cartesian pose does not establish independent control of all six pose components. [Official K1 model description](https://github.com/BoosterRobotics/booster_assets).

The adapter is `src/transition_autonomy/backends/booster_backend.py`, **disabled for physical dispatch by default**. Its tests use a fake transport. The actual SDK was imported on the simulation host only for API inspection: no robot client, DDS factory, subscription, mode switch or physical motion was created. Hardware commissioning remains outstanding. ROS 2 Jazzy on the Ubuntu 24.04 simulation host does not establish physical-robot compatibility.

## Verified SDK and version pin

The official repository is `BoosterRobotics/booster_robotics_sdk`, checked at commit `87a9a26d06a94ddd50968e7576f1170f6fa5a80f`. The Python distribution is **`booster_robotics_sdk_python==1.6.3`**, not an assumed SDK2 package. The separately documented `boosteros` client is another interface, unused here. The repository advertises C++ binaries built on Ubuntu 22.04; each physical host needs its own compatibility check. [Pinned official SDK](https://github.com/BoosterRobotics/booster_robotics_sdk/tree/87a9a26d06a94ddd50968e7576f1170f6fa5a80f), [BoosterOS scope](https://docs.booster.tech/docs/developer-guide/booster-os-python-sdk/sdk-overview/).

The inspected CPython 3.12/Linux x86-64 wheel has SHA-256 `144d50f13ddbe31de56f508df615d070352eebb2ca439be8f6f3723d98067ebe`. Other architectures/Python versions require their own matching wheel/hash. `docs/booster_sdk_evidence.json` records the wheel URL/hash, source revision/file hashes and inspection boundary. The adapter checks installed package version before explicit connection; deployment must also use a hash-locked artifact.

| Verified Python interface | Project use |
|---|---|
| `ChannelFactory.Instance().Init(domain_id, network_interface)` | Explicit configured connection, never at import |
| `B1LocoClient.Init()` / `InitWithName(robot_name)` | One client for the selected robot |
| `MoveHandEndEffectorV2(Posture, duration_ms, HandIndex)` | Approved torso-frame endpoint request; returns `None` |
| `StopHandEndEffector()` | Endpoint-planner stop request; returns `None` |
| `GetRobotInfo()` | Exact serial, firmware and model check |
| `GetStatus()` | Commissioned mode/body-control check |
| `GetFrameTransform(Frame.kBody, Frame.kLeftHand/kRightHand)` | Computed torso-to-hand transform |
| `B1LowStateSubscriber(callback)` + `InitChannel[WithName]()` | `motor_state_serial` positions, velocities and lost flags |

These signatures were checked against the downloaded wheel examples and imported binding docstrings. The C++ header specifies K1 support for V2 and a torso reference frame. It deprecates the older `MoveHandEndEffector` because of different orientation handling; this adapter never falls back to that legacy method. [Pinned motion header](https://github.com/BoosterRobotics/booster_robotics_sdk/blob/87a9a26d06a94ddd50968e7576f1170f6fa5a80f/include/booster/robot/b1/b1_loco_client.hpp).

The SDK `ArmController` wrapper is absent: its constructor enables upper-body custom control and creates a low-level publisher. Publishing completion is not measured completion. Our scheduler leaves servo control and standing/balance with the commissioned vendor controller. It never calls `ChangeMode`, `GetUp`, `Move`, `UpperBodyCustomControl`, gait interfaces or `rt/joint_ctrl`. [Controller-wrapper behavior](https://docs.booster.tech/docs/developer-guide/cpp/controllers/), [SDK communication model](https://docs.booster.tech/docs/developer-guide/cpp/architecture/).

## Shared simulation/real commands

Both backends accept `SkillCommand(kind="point", parameters={"profile": "point_a"})`. Allowed profile identifiers are `point_a`, `point_b` and `home`. Profiles are approved local configurations. Task/network commands cannot supply raw joint arrays, arbitrary poses, gains, durations or mode changes.

An Isaac profile contains validated arm joint targets and its measured hand-reference target. A hardware `PointProfile` contains hand, torso-frame position, V2 orientation parameters, duration, endpoint tolerance, settling interval and semantic label. `point_a`/`point_b` yield `remote.pointed_target="A"`/`"B"`; `home` yields `None`. Each backend's geometry needs separate commissioning. Equal labels establish task semantics, not identical trajectories or dynamics.

The pointing fact becomes unknown when motion is accepted. Hardware success requires fresh state, both arms below the speed threshold and the requested hand's computed/measured torso-frame endpoint within tolerance for the dwell. Completion facts have a bounded validity window; subsequent observations refresh them only while the arm remains within tolerance and stationary, otherwise making them unknown. A gap exceeding the allowed observation age resets settling; timestamp regression yields an unknown outcome. Receipts explicitly record `orientation_verified: false`. Here, “point” means reaching a commissioned endpoint associated with a target. It does **not** prove a finger ray intersects an object or verify orientation. A directional-pointing study must add that postcondition.

A normal RPC return is an acknowledgement. Elapsed requested duration is not completion. Timeouts, stale state, identity/mode changes and invalid measurements produce `unknown`, disarm dispatch and require reconciliation. After a stop request, measured arm stillness over the dwell is required for `canceled`; that status does not certify a whole-body emergency stop. Partial task effects remain unresolved.

The runtime owns task dependency/authority validation, A0/A1 sequencing and common return holds. The backend supplies observations, never policy decisions or predicted outcomes. Return deadlines include waiting for an observed atomic boundary. A fixed-root/support-qualified Isaac scene validates the arm task layer, not unsupported real standing stability.

## Joint mapping

The pinned SDK uses `JointIndexK1`: head indices 0–1, arms 2–9 and legs 10–21; no waist. This adapter only **reads** indexed joints. [Pinned K1 enum](https://github.com/BoosterRobotics/booster_robotics_sdk/blob/87a9a26d06a94ddd50968e7576f1170f6fa5a80f/include/booster/robot/b1/b1_api_const.hpp).

| SDK index | SDK enum |
|---|---|
| 2 / 6 | `kLeftShoulderPitch` / `kRightShoulderPitch` |
| 3 / 7 | `kLeftShoulderRoll` / `kRightShoulderRoll` |
| 4 / 8 | `kLeftElbowPitch` / `kRightElbowPitch` |
| 5 / 9 | `kLeftElbowYaw` / `kRightElbowYaw` |

The official URDF uses `aaleft_shoulder_pitch_joint` and `aaright_shoulder_pitch_joint`, differing from its companion motion-name list. Isaac articulation order is not assumed to match SDK order. Maintain explicit names and verify the imported model, never zip arrays merely because both contain 22 entries. The simulator integration owns its full model mapping and asset hash.

## Ownership, durable receipts and reconnection

`BoosterK1Backend` commits full command/configuration and dispatch intent to SQLite before RPC. Its append-only receipt table retains terminal evidence. Identical command-ID retries return the existing receipt; changed payload/configuration with the same ID is rejected. A local file lock excludes a second adapter using the same ledger.

A nonterminal command at restart blocks arming. The verified endpoint API exposes no project command ID or queryable per-request history, so the adapter cannot prove whether a lost RPC ran. It **never automatically replays ambiguous motion**. A robot-local operator/supervisor must reconcile state and effects with the retained journal under a recorded recovery procedure. There is no “clear all” or generic hardware-reset method. Deleting the ledger is not a readiness procedure.

The journal/file lock is not a DDS-wide exclusive-control lease. Required `site_ready` evidence must come from a robot-local workcell/ownership/stop/support monitor; it must exclude competing apps, gestures, teleoperation and raw publishers. An untrusted UI Boolean is insufficient. Owner loss and robot-local stop/watchdog behavior must be commissioned independently of the experiment UI/network.

Synchronous SDK query blocking/exception behavior is another deployment requirement: run and bound it in a robot-local supervised process before use. Client timeout/process termination cannot retract a delivered RPC. The implementation does not assume K1 has a ROS `FollowJointTrajectory` server, ros2_control hardware interface or MoveIt driver.

## Exact readiness prerequisites

No physical pose defaults are supplied. `K1Config` requires serial/firmware, commissioned allowed mode/body-control values, profiles and a commissioning receipt. `physical_actuation_enabled` defaults to false. `arm()` additionally checks that receipt, unresolved ledger entries, identity/mode, fresh finite state, observed arm stillness and positive `site_ready` evidence. Construction/import do not arm it.

Before physical execution, supply and validate:

1. **Identity/runtime:** K1 serial/edition, firmware, SDK artifact/hash, host OS/Python/architecture, explicit DDS domain/interface/robot name and network isolation. The sim host's Jazzy does not justify replacing the vendor-supported stack.
2. **Model/profiles:** actual model/calibration, torso and hand-reference definitions, verified joints, SI units, reachable profiles, collision/body clearance, registered targets, approved conservative durations and bounds.
3. **Support/ownership:** vendor-supported mode for endpoint control in the actual stance/fixture; local monitor of workcell, support, fault/fall/stop state, exclusive arm control, owner loss and watchdog. No balance controller is implemented here.
4. **Observation timing:** commissioned DDS latency bound, freshness threshold, arm-speed threshold, endpoint repeatability, dwell and stop deadline. Inspected `LowState` exposes no source timestamp: callback receipt time alone cannot exclude delayed old packets. RPC transforms and streaming joints are not an atomic sample.
5. **Recovery:** physical stop access, verified `StopHandEndEffector` behavior, intervention responsibility and lost-reply/restart procedure. The vendor documents that stopping an inactive planner can fail; such failure is not proof of stillness. [Motion interface](https://docs.booster.tech/docs/developer-guide/cpp/rpc/motion/).
6. **Commissioning receipts:** reduced-risk supported reach trials for every profile, endpoint accuracy, mid-motion cancellation, stale state, disconnect and duplicate/restart behavior. These are future physical tests, not completed tests. Receipts must identify model/profile/software hashes.

Only an authorized robot-local session should construct `BoosterSDKTransport(permit_connection=True, ...)` with the local monitor, inspect readiness and call `arm()`. The current work has made no robot connection. Thirteen fake-transport tests validate protocol behavior, including durable completion evidence; they do not qualify hardware, site monitoring or the pose library.

## ROS 2 and experiment evidence

The experiment protocol is Python and transport-independent. The implemented isolated ROS 2 gateway wraps the logical or Isaac backend and preserves command IDs, measured results and durable journal semantics. Actual Jazzy/DDS logical and native fixed-base Isaac roundtrips have passed; see [ROS 2 gateway contract and evidence](ROS2_GATEWAY.md). Its executable cannot select physical hardware, and no ROS extension is required inside Isaac. ROS action acceptance/cancel acknowledgement must not be interpreted as observed completion. Lifecycle activation alone is not K1 readiness. [ROS action state model](https://design.ros2.org/articles/actions.html).

Jazzy exists on the simulator host. Newer Lyrical is also LTS, but driver/host compatibility determines deployment; freeze supported versions rather than Rolling. [Official release table](https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases.rst).

Retain model/asset hashes, profiles, runtime revision, SDK evidence, readiness receipt, ledger, observations and recovery results. Physical hardware performance, simulator validity and human return-to-task effectiveness remain separate claims.

## Python assembly and clock ownership

The SDK transport stamps incoming samples with the robot host's `time.monotonic()`. Keep the runtime in that same absolute monotonic domain. Do not use the logical fixture's zero-based clock for a live SDK transport. This factory does not create a network client or arm the backend; the supplied `transport` can be a fake implementing `K1Transport`, or a separately authorized and configured `BoosterSDKTransport`.

```python
from pathlib import Path
import time

from transition_autonomy.backends.booster_backend import BoosterK1Backend, K1Config
from transition_autonomy.journal import Journal
from transition_autonomy.model import load_task
from transition_autonomy.runtime import Runtime


def assemble_k1(config: K1Config, transport, task_path: Path, run_dir: Path):
    # config.physical_actuation_enabled remains False unless explicitly commissioned.
    run_dir.mkdir(parents=True, exist_ok=False)
    backend = BoosterK1Backend(config, transport, run_dir / "adapter.sqlite")
    journal = Journal(run_dir / "runtime.sqlite")
    runtime = Runtime(
        load_task(task_path), backend, journal,
        now=time.monotonic(), clock=time.monotonic,
        policy="consequence",
    )
    return runtime, backend, journal
```

Call `runtime.tick(time.monotonic(), auto_dispatch=False)` from its single owner to inspect the integration before any authority/arming decision. The SDK readiness/sample stream must use that same host and clock. A fake transport must deliberately provide timestamps in the test's chosen clock; the Isaac HTTP adapter performs its documented remote-clock conversion instead.

Construction may occur on the main thread and then transfer ownership to one serialized runtime worker. The adapter SQLite connection permits that handoff; it does not make concurrent adapter access safe. Do not call `backend.arm/start/poll/observe/close` independently while another thread owns it. Shut down/join the runtime owner before closing either journal. The supplied CLI exposes only logical and Isaac backends; it has no physical-hardware selection. Use the separately reviewed deployment configuration and commissioning gate before introducing any real command path.
