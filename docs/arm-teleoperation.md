# Bimanual arm teleoperation

The arm mode adds controller-driven pose IK to the existing deictic reaching demo. Test it with **Meta XR Simulator on Windows and the fixed-base K1 in Isaac Sim**. Physical Quest and physical robot tests are a later phase. This code does not provide a physical Booster motor driver.

## Controls and mapping

Release both index triggers before starting. Hold the left and right index triggers together to clutch both arms. Move the left controller to move the left tool position and the right controller to move the right tool position. Release either trigger to hold both arms. Release both fully before clutching again.

Each frame expresses both controller poses relative to the **current** head position and horizontal head direction. Translating or yawing the whole body together with both controllers therefore leaves the robot targets unchanged. Head pitch and roll do not tilt the control frame. Looking almost vertically makes heading ambiguous and requires release before restarting.

Each clutch records these controller poses and the measured robot tool poses. Translation and rotation changes then modify the captured tool targets, avoiding a jump to absolute human hand coordinates. In ROS forward/left/up axes the mapping is:

```text
p_hand = inverse(R_head_yaw) * (p_controller - p_head)
R_hand = inverse(R_head_yaw) * R_controller
p_goal = p_tool_at_clutch + scale * (p_hand - p_hand_at_clutch)
R_goal = R_hand * inverse(R_hand_at_clutch) * R_tool_at_clutch
```

The default scale is derived from the K1 URDF chain reach divided by a nominal 0.60 m human arm reach (approximately 0.56). The controller's `teleop_translation_scale` parameter can override it. Status reports the effective value. Releasing, repositioning the controllers, and clutching again resets the references without resetting the robot pose.

Each K1 arm has four joints, so it cannot match arbitrary six-dimensional poses. Bounded nonlinear IK prioritizes position and uses orientation as a soft objective, with posture and solution-continuity terms. Measured joints seed every solve; additional bent-arm seeds escape the straight arm's inward-motion singularity. Zero displacement and rotation preserve the captured joint pose. The base, legs, and neck remain fixed. Joint limits and conservative table/trunk/arm separation checks restrict motion; unreachable or obstructed targets need not match the controller pose. Status exposes measured position/orientation errors and whether the target is limited.

The current table-clearance guard uses an infinite horizontal plane at the configured table height. Downward movement may therefore be limited even outside the visible table footprint. This conservative workspace restriction remains in the simulation; the solver does not bypass it.

The default servo limits are 0.35 rad/s per joint, nominal 0.7 rad/s² joint acceleration,
and 0.10 m/s commanded tool velocity. Robot target displacement beyond 0.30 m
from the clutch anchor is projected into that workspace; bringing the target
back allows continued control. Input older than
0.25 seconds stops the servo. The relay has its own 0.30-second command lease,
and the simulator independently holds after 0.50 seconds without a valid
setpoint. The timing limits are watchdog thresholds, not measured network or
motion latency guarantees. Hard geometry holds, abrupt measured-feedback
reversals and final tool-rate protection can stop faster than the nominal
acceleration bound; collision/feedback containment takes priority.

The right trigger's ordinary target selection or camera-button click occurs on release. If the left trigger joins that press, it becomes a teleoperation chord and does not also select a target. Target execution is suppressed during teleoperation. The user/robot view toggle remains available outside a clutch.

Teleoperation is relative control and does not depend on the wrist camera's learned registration. The original deictic pointing workflow still requires camera alignment. Losing visual features while moving an arm does not itself redefine the teleoperation coordinate frame.

## Runtime path

Use the normal four-process Ubuntu stack and Windows ROS SSH tunnel from the [setup guide](setup.md). The bridge script explicitly enables teleoperation in `markerless` and `synthetic` modes. When starting the controller directly, both `allow_execution` and `allow_teleoperation` must be enabled; both default to false in the controller. The existing single-arm `mock` mode preserves its original reaching behavior and does not supply the eight measured joints required for bimanual control.

In Unity, activate **Meta > Meta XR Simulator**, then Play `DeicticDemo`. Keep the simulator active for this implementation's verification. Do not switch back to Quest Link for a hardware test until the simulator work is complete.

| Stage | Interface | Responsibility |
| --- | --- | --- |
| Unity to controller | `/deictic/teleop/input`, `std_msgs/String` | One timestamped pair of controller positions and rotations, tracking flags, clutch, client session and sequence |
| Controller | URDF-derived bimanual pose IK | Capture clutch references, apply bounded IK, invalidate reach previews, and report teleoperation status |
| Controller to relay | `/k1/teleop/command`, `std_msgs/String` | Atomic pair of arm joint targets or a hold/release command, with backend session and sequence |
| Relay to controller | `/k1/teleop/relay_status`, `std_msgs/String` | Fresh feedback/actuator readiness, active lease, owner session, accepted sequence and lease/hold reason |
| Relay to Isaac | `/k1/sim/joint_commands`, `sensor_msgs/JointState` | Arbitrate teleoperation versus reaching and stream validated setpoints |
| Isaac to controller/Unity | `/joint_states`, `sensor_msgs/JointState` | Actual simulated positions for both four-joint arms |

Input uses `schema_version: 2`, `frame_id: "teleop_head"`, metres, ROS forward/left/up axes and unit quaternions in xyzw order. `left_position`, `right_position`, `left_rotation`, and `right_rotation` always travel in one message. The publisher targets 20 Hz. Sessions, increasing sequence numbers, source timestamps, feedback freshness, and short command leases prevent an old queued packet from restarting movement. An existing ROS subscriber alone is insufficient: the controller requires fresh relay acknowledgement and checks that the relay retains its active lease. Fault reasons remain visible after the triggering packet.

**Update Unity, the controller and the trajectory relay together.** Version 1 position-only input is rejected with `protocol_version_mismatch`; an older running Unity session cannot silently use the new mapping. Stop/restart the backend and relay using [the run commands](start-demo.md), then reload the updated Unity scripts and start Play yourself. The existing SSH endpoint and camera topics are unchanged. SciPy >=1.11,<2 is required by the controller; the supplied `.venv-control` already has 1.11.4.

## Reference implementation

The correction follows [Unitree xr_teleoperate at commit 817fb00](https://github.com/unitreerobotics/xr_teleoperate/tree/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b), especially the four-joint-per-arm H1 solver in [`robot_arm_ik.py`](https://github.com/unitreerobotics/xr_teleoperate/blob/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b/teleop/robot_control/robot_arm_ik.py). It uses full wrist poses, head-yaw-relative input, joint bounds, measured-state initialization, and posture/continuity objectives. The K1 implementation uses its own URDF, SciPy solver, motion/geometry guards, ROS transport and the requested two-trigger clutch. Unitree's fixed waist offsets, H1 arm-length ratio, joint layout, motor SDK and gravity feedforward are not K1 calibrations and are not reused. This is an adaptation of the control approach, not integration of Unitree hardware drivers.

The previous local differential position solver could return no movement for a reachable inward target from the straight left arm, and could reach a right-arm joint limit on an alternative solvable branch. The earlier small forward-only checks missed this. The new regression coverage includes inward motion, held-out FK-generated poses, common body motion, rotations, limit recovery and relay lease loss.

Clutch entry cancels an existing reach and clears its preview. Release, tracking loss, focus/pause changes, recentering, or a communication timeout ends active control. A lost stream cannot resume merely because old held-trigger packets arrive later. The relay clears its prior trajectory when teleoperation acquires or releases control and holds measured joints on timeout. These are simulation guards, not a physical safety certification or full mesh collision checking.

## Validation

The implementation is checked in layers: pure clutch/frame and IK tests, ROS arbitration/freshness tests, then scripted Unity–ROS–Isaac integration. Native Meta XR Simulator input is checked separately. Recorded results and remaining limitations are in [the validation record](validation.md). Do not interpret a scripted integration test as evidence of native controller input or real robot operation.

For source checks, use Unity Test Runner's EditMode tests and the native Ubuntu control environment. From the repository root, after sourcing ROS Jazzy:

```bash
ROS_DOMAIN_ID=223 PYTHONPATH="$PWD/ros2/src/deictic_control:${PYTHONPATH:-}" \
  .venv-control/bin/python -m pytest ros2/src/deictic_control/test \
  sim/test_command_control.py sim/test_relay_status.py \
  sim/test_teleop_contract.py sim/test_verify_teleop_live.py -q
```

These source tests use isolated ROS domains and do not command the live simulator. The opt-in live checks below intentionally move its arms.

Meta XR Simulator v205 provides one shared index-trigger keyboard binding
(`T`) and a left/right action-input selector. Its “Use same input for both sides”
setting copies the input type, not button presses. Both simulated controllers
and the idle ROS handshake have been verified; sustained native two-trigger
activation has not been verified with the available keyboard automation.
See Meta's [input controls](https://developers.meta.com/horizon/documentation/native/xrsim-input-keyboard/)
and [Point & Click controls](https://developers.meta.com/horizon/documentation/unity/xrsim-point-and-click/).

The repeatable motion check below uses **synthetic ROS controller input** and
the real Isaac physics instance. Stop Unity Play first, and restart the bridge
terminal if its endpoint retains the previous Unity publisher. In the sourced
Ubuntu project terminal, run:

```bash
.venv-control/bin/python sim/verify_teleop_live.py \
  --allow-sim-motion \
  --output sim/artifacts/bimanual_synthetic_ros_validation.json
```

The default profile makes independent 5 cm inward robot-tool requests, checks measured FK progress, then
checks release, timeout, held-input rejection and rearming. It requires the
identified Isaac nodes, fresh feedback, no competing input publisher and a
checked collision-free path. The default requires the declared rest pose;
`--from-current` explicitly permits a repeat from the current held pose with
the same geometry checks. It leaves the arms held at their final positions.
Restart Unity Play afterward to create a fresh controller session. `--motion-profile forward` retains the earlier 2 cm forward fixture for comparison. Add `--orientation-check` for two modest orientation-only phases chosen from each measured arm's local positional null direction, with independent measured FK angular-error and position-drift checks. The test refuses an unsuitable starting pose instead of weakening the geometry gate.

`BimanualRosLoopTests` provides a separate Unity PlayMode integration test,
enabled only by `DEICTIC_BIMANUAL_SIM_INTEGRATION=1` with the isolated Isaac
endpoint at `127.0.0.1:10000`. It passes scripted samples through the production
clutch and Unity TCP bridge, checks unchanged commands under common body translation/yaw, independent inward arm motion and
release/reclutch behavior, and observes actual backend joint commands. This
also does not substitute for native Meta trigger delivery.

The related [Isaac Lab teleoperation example](https://isaac-sim.github.io/IsaacLab/main/source/overview/imitation-learning/teleop_imitation.html) uses task-space input and IK, including wrist targets for humanoids. This project applies that control pattern to the existing Unity/ROS bridge; it does not add Isaac Lab Mimic dataset generation or policy training.
