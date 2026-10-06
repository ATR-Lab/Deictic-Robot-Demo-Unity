# Booster K1 / Isaac Sim implementation

The reference backend runs native **Isaac Sim 5.0.0** on MLWorkstation. The experiment process communicates with a loopback HTTP worker using the common `Backend` protocol. Unity and the Meta XR Simulator run on the Windows host, with loopback ports forwarded over SSH. The first task is a bounded right-arm reference-point reach to A or B, followed by a return-home profile. These are simulator profile names, not a claim of physical fingertip calibration or general six-dimensional pose control.

## Model and physical coverage

The official [Booster assets](https://github.com/BoosterRobotics/booster_assets/tree/3c2dfa99e09beddf092e0d6521dbbcec7e7903ed/robots/K1) are pinned to revision `3c2dfa99e09beddf092e0d6521dbbcec7e7903ed`. The original URDF SHA-256 is `834e17faf4681e5fcae116130a9da6d81b696c7f8278b95f281cec0551ca6124`. `scripts/fetch_k1_assets.py` retrieves that URDF, its 24 referenced meshes, and the upstream BSD 3-Clause license. `assets/booster_k1/manifest.json` records source URLs and hashes. Vendor files are unmodified.

The original model has 22 revolute joints, four per arm, and no gripper. A separate generated URDF fixes every joint except the four right-arm joints and fixes the trunk to the world. The exact joint names, ordering, limits and target profiles are checked against the pinned model. In particular, the official URDF prefixes the shoulder pitch joint with `aa`; joint names must not be inferred from a motor-array position or a different SDK naming convention.

The worker uses position-drive targets, the URDF inertia data, gravity and PhysX integration. Joint positions are set directly **only during the initial reset**; motions use the articulation controller. Self-collision is disabled in this bounded reference scene. The torso constraint supplies external support. Thus this tests arm actuation, observations, transport and completion handling; it does not test balance, walking, self-collision avoidance, human safety, grasping, camera-based target recognition, or physical K1 servo equivalence. The virtual tool point is 0.10 m along the terminal arm link's negative Y axis. That point is defined for reproducibility and is not a measured physical fingertip location.

The first native run exposed an importer detail: the resulting USD had acceleration-drive stiffness 625 and damping zero despite the requested import defaults. The worker therefore sets native controller gains explicitly (`kp=4000`, `kd=126.5`, acceleration drive), solver iteration counts (32 position, 8 velocity), and disables articulation sleeping. These are simulator tuning parameters, not identified physical K1 gains.

A second diagnostic found inconsistent native velocity telemetry: joint positions and the USD reference point remained stationary across fresh physics samples while native joint velocity stayed nonzero. Disabling articulation sleeping did not eliminate this discrepancy. The completion gate therefore uses **an explicitly labeled velocity estimate from consecutive measured joint positions divided by the positive simulation-time increment**. Both samples come from the same articulation after physics updates. Native velocity is retained separately in observations and terminal receipts; it is not silently relabeled as the estimate. The 0.04 rad/s threshold and 0.3 s dwell were not relaxed. This observer supports measured pose stability; the native velocity discrepancy remains a simulator limitation to investigate before claims requiring accurate generalized-velocity telemetry.

## Command and observation contract

`SkillCommand(kind="point", parameters={"profile": "point_a"})` selects an approved profile. The only supported profiles are:

| Profile | Verified terminal fact | Joint target in config order, radians |
|---|---|---|
| `point_a` | `remote.pointed_target = "A"` | `[-0.4, 0.8, -0.2, 1.2]` |
| `point_b` | `remote.pointed_target = "B"` | `[-0.8, 0.65, 0.2, 0.9]` |
| `home` | `remote.pointed_target = None` with known status | `[0.0, 0.8, 0.0, 0.0]` |

Startup reports unknown point identity. During motion it remains unknown. A target becomes known only when measured joint error is at most 0.035 rad, measured reference-point distance is at most 0.012 m, and measured joint speeds stay at or below 0.04 rad/s for 0.3 s. The reference target comes from forward kinematics of the approved URDF and configuration. The observed reference point comes from the simulated terminal-link transform after a physics step. Elapsed motion duration alone cannot cause success.

Settling dwell accumulates **advancing physics simulation time**, independently of HTTP reads. A nonadvancing/reset clock, a source gap over 0.1 s, a wall-clock sampling gap over 0.5 s, or contradictory measurements resets the dwell. Paused physics cannot renew measurement freshness or finish a command. Human-facing response and completion durations remain wall-clock measures; terminal receipts label the physics dwell clock separately.

Current joint/reference measurements appear in `Observation.facts`; terminal events contain only the declared point-state fact, with numerical evidence in `BackendEvent.evidence`. Each fact has a 0.5 s lifetime. A fresh heartbeat renews a known point only while measurements still match. A drift or ongoing motion makes its identity unknown. The adapter converts remote timestamps by subtracting remote age and the full measured round-trip time from the caller's monotonic clock. Stale observations, future timestamps or excessive round-trip times fail closed as disconnected and non-quiescent.

Stop requests hold the measured arm position. Cancellation is acknowledged only after observed velocity and position settle. A deadline initiates a measured hold and produces failure only after settling. The append-only, fsynced worker ledger deduplicates unchanged command IDs and rejects changed payloads. On restart, previous commands become unknown and cannot be automatically re-executed; their previous evidence is retained for reconciliation. This is a simulator command ledger, not a physical safety controller.

## Deployment and reference test

From an isolated copy of this project on the Linux GPU host:

```sh
python3 scripts/fetch_k1_assets.py
bash scripts/run_isaac_worker.sh
```

The launcher uses the host's existing `k1-isaac-sim:5.0.0` image, native `/isaac-sim/python.sh`, the GPU, and a cache/output directory inside this project. It creates the named `transition-k1-isaac` container. It never starts or changes an existing K1 project. For a clean host, the standard NVIDIA 5.0.0 image can be supplied after verifying that its bundled dependencies match; that alternative is not claimed tested here.

The service binds `127.0.0.1:8767`. Access it from the same host or through an SSH local port forward. Do not expose this unauthenticated worker publicly. `IsaacBackend(endpoint="http://127.0.0.1:8767")` supplies the shared Python protocol. The worker exposes `/observation`, `/events`, `/commands/status?id=...`, `/commands/start`, `/stop`, and `/provenance`. HTTP threads only operate on the locked command ledger; the main thread owns the simulation and all articulation changes.

After native startup, run `python3 scripts/validate_isaac.py` from another terminal on that host. It exercises A → B → home, repeated command IDs, a stop during motion, and an actual loopback TCP response-loss injection. The proxy discards an accepted command's response and subsequent reads; the adapter reports unknown/disconnected/non-quiescent, then reconciles the same command after reconnection without redispatch. This is a controlled transport fault test, not a physical network or hardware safety certification. It writes a JSON receipt with measured terminal evidence. The worker must have one command owner during this check. The adapter's event polling follows IDs it has started or explicitly status-queried, while observation still reports any other active motion owner. Old validation events therefore cannot masquerade as a new experiment's command events.

Outputs are `artifacts/isaac/scene_provenance.json`, the generated fixed-support URDF, a scene export, `command_ledger.jsonl`, `latest_observation.json`, and the actual rendered `latest_frame.png`. Receipt timestamps distinguish wall-clock command duration from accumulated simulation time. Local unit tests use synthetic samples and do not count as physics evidence. The remote run report records separately whether native import, rendering, measured completion, stop, and replay checks actually passed.

## Primary API references

- [NVIDIA 5.0 Python installation and `SimulationApp` ordering](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_python.html).
- [NVIDIA URDF importer](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/py/source/extensions/isaacsim.asset.importer.urdf/docs/index.html).
- [NVIDIA articulation controller](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_simulation/articulation_controller.html).
- [NVIDIA replication and image acquisition](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/replicator_tutorials/tutorial_replicator_isaac_snippets.html).

The available Quadro RTX 6000 and driver are recorded in the run receipt. A successful bounded run on that machine is the compatibility evidence; it is not a claim that the card satisfies every current recommended Isaac hardware configuration.
