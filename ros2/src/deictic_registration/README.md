# Calibrated image registration

This package implements the paper's visual frontend: pretrained SuperPoint features, LightGlue descriptor matching, aligned headset depth backprojection, OpenCV RANSAC PnP, and robust Huber reprojection refinement. It publishes measured camera constraints to `deictic_control`'s frame-fusion graph. LightGlue is a chosen matcher because the paper does not specify one. All thresholds here are implementation defaults; they were not provided by the paper.

The live node never invents an identity transform, camera calibration, image, or depth plane. Missing dependencies, invalid calibration, insufficient inliers, image failures, unsynchronized clocks, and stale input/result messages produce `/deictic/registration_failure`. Synthetic test data is confined to explicitly named tests/simulator providers.

## Inputs

| Default topic | Type | Contract |
|---|---|---|
| `/deictic/headset/image/compressed` | `sensor_msgs/CompressedImage` | Headset RGB sensor image, JPEG/PNG |
| `/deictic/headset/camera_info` | `sensor_msgs/CameraInfo` | Effective intrinsics at the published RGB resolution; calibrated pinhole/plumb-bob/rational model |
| `/deictic/headset/depth` | `sensor_msgs/Image` | `32FC1`, RGB-aligned optical z in meters; same dimensions as headset RGB; invalid pixels are NaN or zero |
| `/deictic/headset/camera_pose` | `geometry_msgs/PoseStamped` | `T_headsetWorld_headsetOptical` at capture, parent `headset_world` |
| `/k1/wrist_camera/image_raw` | `sensor_msgs/Image` | `rgb8`, `bgr8`, or `mono8` wrist sensor image |
| `/k1/wrist_camera/camera_info` | `sensor_msgs/CameraInfo` | Calibrated wrist camera intrinsics at published resolution |
| `/k1/wrist_camera/pose` | `geometry_msgs/PoseStamped` | `T_base_wristOptical` from FK and calibrated camera mounting, parent `base_link` |

All seven messages need acquisition timestamps on the same ROS clock; the default maximum skew is 20 ms. Publish CameraInfo for each capture, not just once at startup. RGB, depth, and CameraInfo frame IDs must match the corresponding optical frame. Both optical frames use x right, y down, z forward. A raycast distance is not optical-z depth: multiply ray range by the camera-space unit ray's z component. Camera pose means the sensor pose, not the latest headset center-eye pose. Publish rectified images with rectified effective K and zero D, or consistent unrectified images/K/D. Cropped/binned CameraInfo requires effective intrinsics with ROI/binning fields cleared.

Topic names, compression flags, frame names, device, CPU thread count, quality thresholds, input timeout, synchronization tolerance, and registration period are ROS parameters. `match_filter_threshold` controls LightGlue's minimum matching score (upstream default 0.1); increasing it rejects more uncertain feature pairs before PnP, without changing the geometric or controller gates. Fresh `/deictic/status` feedback sets the nominal period when confidence permits commits; low/missing confidence uses the faster recovery period (default 0.25 seconds), subject to available captures and inference throughput. For raw simulated headset images use `headset_image_topic:=/deictic/headset/image_raw` and `headset_compressed:=false`. Do not run two camera providers on the same topics simultaneously.

## Output

`/deictic/registration` is `std_msgs/String` containing JSON:

```json
{
  "schema_version": 1,
  "stamp": 1234.5,
  "headset_pose": [0, 0, 0, 0, 0, 0, 1],
  "camera_pose": [0, 0, 0, 0, 0, 0, 1],
  "camera_from_headset": [0, 0, 0, 0, 0, 0, 1],
  "inliers": 50,
  "matches": 60,
  "median_reprojection_error": 0.6,
  "source": "superpoint_pnp"
}
```

These are schema examples, not calibration defaults. Pose arrays are `[tx,ty,tz,qx,qy,qz,qw]` in meters. `camera_from_headset = T_wristOptical_headsetOptical`; the graph derives `X = camera_pose * camera_from_headset * inverse(headset_pose)`. `matches` counts pairs remaining after valid metric-depth and image-bound checks. Confidence calculation belongs to the graph; this frontend also rejects poor geometry before publishing. Failure JSON contains `schema_version`, `stamp`, `reason`, and `source`.

## Install and run

System ROS dependencies are listed in `package.xml`. For the learned frontend use an isolated Python environment that can import the installed ROS distribution and system OpenCV/NumPy. On this host ROS Lyrical uses Python 3.14. The tested CPU dependencies are PyTorch 2.14.0+cpu and torchvision 0.29.0+cpu. This is roughly a 200 MB wheel download, not a CUDA installation.

```bash
# Run in the repository root inside WSL Ubuntu.
python3 -m venv --system-site-packages .venv-registration
.venv-registration/bin/python -m pip install \
  torch==2.14.0+cpu torchvision==0.29.0+cpu \
  --index-url https://download.pytorch.org/whl/cpu
.venv-registration/bin/python -m pip install kornia==0.8.3
.venv-registration/bin/python -m pip install --no-deps \
  git+https://github.com/cvg/LightGlue.git@eb42fee2d71449efb0aa5c10549752b5d75384d8
source /opt/ros/lyrical/setup.bash
export PYTHONPATH="$PWD/ros2/src/deictic_registration:$PYTHONPATH"
.venv-registration/bin/python -m deictic_registration.node
```

For another ROS/Python platform select supported PyTorch wheels for that interpreter instead of copying this host-specific pin. Upstream downloads published SuperPoint/LightGlue pretrained weights to the user's torch cache during initialization. We do not vendor model weights. [Upstream installation and model documentation](https://github.com/cvg/LightGlue) describes the separate SuperPoint and LightGlue licenses.

With the frontend installed in the same Python environment used by `colcon`, the package can instead be built and run as `ros2 run deictic_registration registration_node`. The direct venv command above avoids accidentally launching a system-Python entry point without PyTorch.

The workstation also has an isolated `.venv-registration` in `~/Developer/deictic-k1-reproduction`, using native Python 3.12, existing PyTorch 2.7.0+cu126, torchvision 0.22.0+cu126, Kornia 0.8.3, and the same LightGlue commit. Its exported-camera check is:

```bash
cd ~/Developer/deictic-k1-reproduction
PYTHONPATH=ros2/src/deictic_registration .venv-registration/bin/python \
  -m deictic_registration.offline sim/artifacts --device cpu --max-keypoints 512
```

Select `--device cuda` when GPU memory is available. Offline evaluation publishes no ROS messages and does not change the running controller.

## Verification and remaining limits

```bash
PYTHONPATH=ros2/src/deictic_registration python3 -m pytest \
  ros2/src/deictic_registration/test/test_registration.py -q
PYTHONPATH=ros2/src/deictic_registration .venv-registration/bin/python \
  ros2/src/deictic_registration/test/smoke_features.py
```

The first test suite checks transform recovery with planar/nonplanar scenes, noise and outliers, and rejection of missing/mismatched depth, invalid calibration, and random matches. The optional second test runs the actual pretrained networks on a generated image pair, estimates its pose, and compares against synthetic truth used only for scoring.

On this host on 2026-09-16, all 11 geometry/ROS-image tests passed. The optional 640x480 generated-pair test using 512 features and four CPU threads produced 459 eligible matches, 434 inliers, 0.834-pixel median residual, and 0.00105 m / 0.000787 rad error against synthetic truth. Feature extraction through pose estimation took 1.17 seconds, excluding model loading. This is one synthetic test, not a device-accuracy or latency guarantee. A live 640x480 CPU run therefore needs an explicitly configured age budget above this observed processing time; do not silently relax the controller's default freshness gate.

For exported camera captures, `python -m deictic_registration.offline CAPTURE_DIRECTORY --output REPORT.json` runs the same frontend. Its module docstring specifies `manifest.json`, image, calibration, pose, and aligned-depth files. Optional simulator truth is accessed only after pose estimation and appears under `scoring_only`; it never supplies correspondences or the estimated transform.

A separate Isaac Sim camera pair was validated after correcting the renderer's calibration image size and moving the wrist camera clear of its link mesh. Actual 512-feature GPU inference yielded 89 image matches, 87 depth-valid matches, 41 PnP inliers and 1.490 px median residual. Independent simulator truth scored the estimated alignment at 5.98 mm translation and 0.006375 rad rotation error. Cold processing took 1.07 s, including first CUDA use. This is one rendered pair, not continuous registration or physical accuracy. Its inlier ratio produces only about 0.138 confidence under the controller's documented default formula, so it cannot authorize commits. Increasing the feature budget to 1,024 or 2,048 did not improve this pair's pose score or confidence. See the repository validation record for subsequent acceptance testing.

On that same capture, a stricter `--match-filter-threshold 0.5` retained 22 depth-valid matches with 17 inliers, 1.749 px residual and 13.92 mm truth-scored translation error. Its default-formula confidence was 0.821; approximately 1.97 s remote CPU processing exceeds the controller's default one-second registration lifetime. However, three subsequent fresh acquisitions at this fixed setting produced only 3, 3 and 9 matches and were all rejected below the 12-inlier minimum. No accepted transforms existed for target-grounding scoring on those captures. This setting therefore failed the repeatability check and was not enabled for live control. Runtime defaults remain unchanged. Neither tuning on one capture nor a high inlier ratio establishes general accuracy.

The five independently defined scene targets gave mean/max grounding error 2.400/2.506 mm for the original 512-feature, default-filter estimate, and 6.920/6.951 mm for the single stricter-filter estimate. These are offline coordinate-mapping scores, separate from measured robot execution. Disabling LightGlue pruning and early stopping did not rescue the three later captures' confidence: the full-depth matcher at filter 0.1 produced confidence 0.17-0.22; at 0.5 only 4, 3 and 11 matches remained. Production inference still uses the upstream adaptive defaults.

Those measurements used the earlier fine-pattern fixture and static poses. The revised simulator synchronizes cached RGB/depth/camera parameters to one renderer reference and uses larger distinctive printed features, clearer headset framing, and a leveled rigid wrist camera. Nine simulator source tests passed, and four new static ROS captures confirmed the same-frame camera pipeline runs. With filter 0.5, the three validation captures had accurate target mapping but confidence 0.640–0.679, below the unchanged 0.7 gate; that validation failed.

Filter 0.7 was then selected on the development capture alone and frozen before three **new** acquisitions. Those captures passed unchanged geometric/confidence gates: 85/110, 80/109 and 77/107 inliers; 1.370, 1.510 and 1.355 px median residual; confidence 0.870, 0.785 and 0.783. Independent simulator truth scored maximum error across the five fixed scene targets at 3.84, 3.46 and 1.04 mm, respectively. Alignment-transform translation errors were 17.19, 13.91 and 3.99 mm. Different transform and target errors are expected because rotation and translation interact at the queried target locations. This is a repeatability check in one static, deliberately textured simulation fixture, not evidence for arbitrary rooms or moving cameras.

The tracked `scripts/validate_registration_fixture.py` reproduces this development/frozen-validation protocol; `--thresholds 0.7` holds the accepted simulation setting fixed. The saved reports include image hashes and capture provenance. The three ROS datasets had distinct equal-across-seven-message capture timestamps; renderer references are enforced internally by the publisher but are not carried in those ROS messages. Simulator truth enters only after estimation for scoring.

Warm full-resolution CPU inference took 1.96–1.99 s on the three accepted captures. A separate same-image timing check gave median 4.595 s with one CPU thread, 2.850 s with two, and 1.973 s with four, with identical geometric results. The explicit opt-in [`config/isaac_cpu.yaml`](config/isaac_cpu.yaml) therefore sets 512 features, filter 0.7, four CPU threads, frontend maximum age 3 s, and controller registration timeout 4.5 s (processing plus an update interval). It selects the genuine GTSAM/visual path and does not enable trajectory execution. Core default confidence, geometric thresholds and one-second controller timeout remain unchanged. This slow static simulation profile does not reproduce the paper's latency; physical or moving-scene use needs separate latency validation.

For a read-only live check, launch the frontend with this profile and remap both outputs away from the controller:

```bash
.venv-registration/bin/python -m deictic_registration.node --ros-args \
  --params-file ros2/src/deictic_registration/config/isaac_cpu.yaml \
  -r /deictic/registration:=/deictic/registration_candidate \
  -r /deictic/registration_failure:=/deictic/registration_candidate_failure
```

While that node runs, `python scripts/observe_registration_candidates.py --synthetic-scene coarse_fixture` records a 60-second read-only stream check using the declared fixture's independent truth. Run it with the ROS/NumPy environment sourced. The explicit scene flag prevents presenting those fixed simulator coordinates as physical calibration.

A subsequent 60-second candidate-only live ROS check produced 30 observations, all passing the unchanged confidence threshold and independent five-target error check, with zero frontend failures. Minimum confidence was 0.745568 and maximum target-mapping error was 4.102 mm. Source-header-to-arrival age was 2.012–2.264 s; the largest interval between delivered observations was 2.100 s. The Isaac publisher assigns that header when retrieving the synchronized cached pair, so rendering-to-publication delay is not independently included. Those measurements explain the explicit 4.5-second static simulation lifetime. Outputs were remapped away from the controller during this check, separately validating live camera-to-registration transport and estimation.

After the coordinated switch to genuine visual observations and GTSAM, the fixed-world Unity-to-Isaac test passed once at 16:48:57–16:49:06 UTC. It selected the independently fixed Unity target `(0.25,0.92,1.32)`, required `markerless` mode and `gtsam_isam2`, measured 1.70 mm target-grounding error and 2.27 mm actual tool error, and verified joint movement plus fresh post-execute tool feedback. The test took 8.983 s, including its connection/planning/checking steps; that is not a paper task-time measurement. This establishes one complete learned-registration/fusion/reaching loop in the declared simulation fixture.

Static registration, live candidates and the single executed reach are distinct validation stages. They do not establish robustness to changing headset views, dynamic objects, arbitrary rooms or physical camera motion. Physical motion-time image/pose synchronization remains a separate validation requirement.

The wrist mount used by that first reaching test lost usable shared scene content after arrival, causing visual-registration failure and closed commit gates. An explicit simulator maintenance return to rest restored registration; it is separate from the learned target controller and is not automatic viewpoint recovery. In a separate idle test, pausing only the provider for six seconds closed commits at observation age 4.544 s, froze the transform, and left no preview/execution active. Resuming it recovered fresh accepted registration after approximately 2.70 s (confidence 0.91268, age 2.099 s). Workspace-wide view coverage remains under evaluation; no continuous multi-reach success is claimed from the single passed test.

A distinct optional simulator mount, `--camera-mount-profile reach-balanced`, is being evaluated to preserve tabletop visibility over the arm path. On its development frame, filter 0.7 failed confidence at 0.5605; development-only selection then accepted 0.8, so 0.9 was not evaluated. The fixed 0.8 setting passed three **new** rest captures: 44/59, 48/60 and 49/61 inliers; confidence 0.8241, 0.9119 and 0.9108; maximum five-target errors 7.96, 1.36 and 1.89 mm. CPU processing took 1.88–1.91 s, with truth only used for scoring.

Its first live reach attempt was rejected before motion because a healthy optimizer version update discarded the preview, despite only approximately 0.0481 mm target remapping. After replacing version equality with same-epoch/original-preview target displacement bounded to 5 mm, a new full test passed at 17:20:11–17:20:27 UTC: 0.91 mm fixed-world grounding error, 2.78 mm measured tool error, and 15.448 s total test duration including a deliberate 4.5-second review interval. The exact approved trajectory, execute token and other freshness/quality/joint-state gates remain unchanged. A subsequent 30-second postreach monitor had commits open in only 20/300 status samples and one new accepted graph key. Forearm occlusion reduced shared texture; a captured postreach pair had only 15/35 inliers and confidence 0.1122. This is another single-reach result rather than demonstrated continuous operation.

Use the separate [`config/isaac_balanced_cpu.yaml`](config/isaac_balanced_cpu.yaml) for that explicitly selected simulator mount. It uses filter 0.8 and keeps the same feature budget, CPU threads, ages and geometric/confidence gates. The original `isaac_cpu.yaml` remains the preserved filter-0.7 profile used by the earlier verified reach and recovery checks.

The bounded two-pose development script `scripts/select_simulation_registration_profile.py` tests CUDA feature budgets 512, 1,024 and 2,048 at filter 0.8, then 0.9 only if necessary. It chooses the first setting that passes the unchanged geometry/confidence gates on every declared development capture. The two positional arguments retain the earlier one-capture-per-pose protocol; `--rest-development R1 R2 R3 --end-development E1 E2 E3` requires all six captures to pass. Simulator truth only scores the resulting transforms and never selects a configuration. Its 25% limit applies to this process's PyTorch allocator, not all GPU process memory; synchronized warm timings and allocator peaks are recorded. A selected profile still requires `scripts/validate_frozen_simulation_registration.py` to pass three new captures at each pose, acquired after the recorded configuration-freeze time. These offline scripts do not alter the running provider or controller.

This comparison completed on the balanced mount without selecting a profile (`sim/artifacts/balanced_gpu_development_selection.json`). All six settings failed at postreach: 512/0.8 had 11/33 refined inliers; 1,024/0.8 and 2,048/0.8 had confidence 0.1306/0.1726; the three 0.9 settings had 11/26, 10/21 and 11/22 RANSAC inliers, below the unchanged minimum of 12. Five of six rest results passed. Warm feature extraction through PnP took 69–98 ms; maximum PyTorch allocated/reserved memory was 226,789,888/249,561,088 bytes and sampled whole-process usage was 438 MiB. CPU/CUDA may produce different keypoint pruning/matching results, so the device itself is part of the frozen profile. No heldout or live-GPU stage followed: fast computation alone did not resolve the forearm-occluded scene's unreliable matching.

A separate `reach-balanced-10cm` bracket trial used a local X offset of 0.10 m selected by mesh visibility, with orientation, Y/Z offsets, intrinsics and texture unchanged. Development selection chose CUDA/1,024 features/filter 0.9 as the first setting passing at both rest and end (confidence 0.895/0.740). The frozen setting then **failed fresh validation**: rest 3/3 passed, but end 0/3 passed, with confidence 0.556/0.270/0.391 and 12/20, 15/28, 14/25 inliers. The independently scored end target errors were 9.42/24.19/8.21 mm; low target error alone does not override the confidence gate. Warm processing was 76–85 ms. Full records are `bracket10_gpu_development_selection.json` and `bracket10_gpu_frozen_validation.json`; all six validation stamps follow the recorded freeze. No live GPU profile was created or provider enabled from this result.

A further projection-oriented virtual mount was tested using three rest plus three end development frames for each predefined setting (`projection_gpu_development_selection.json`). None of the six configurations passed all six frames: every rest frame passed, but at most one end frame passed per configuration. The 36 outcomes are retained; warm processing took 62–84 ms. No profile freeze or fresh-heldout stage followed. This larger development sample exposes the instability that a single successful frame could hide.

Two follow-up diagnostics did not qualify a profile. `scripts/diagnose_registration_rotation.py` tested fixed CUDA/1,024/filter 0.8 over the six declared projection frames and four wrist quarter-turns, inverse-mapping features into original pixels before unchanged PnP. All rotated cases failed; only the three unrotated rest frames passed. Exact rectangular-pixel and fractional-coordinate inverse tests run before that diagnostic. Separately, a matte-floor-only fixture ablation repeated the predefined six configurations on three rest plus three end development frames. None passed all six (`matte_gpu_development_selection.json`); the best two settings passed 5/6. Warm processing was 70–89 ms. No heldout stage or live provider followed either failed comparison.

The projected-plane/blue-neighborhood region labels in earlier plots describe **floor-adjacent regions**, not established floor-grid correspondence errors. Raised-object silhouettes can project outside the table-plane polygon and be mislabeled. A later read-only check (`scripts/diagnose_depth_boundaries.py`, `matte_depth_boundary_diagnostic.json`) ran the same CUDA/1,024/filter 0.8 estimator first, then used independent simulator truth to inspect its 31/32/33 fitted outliers. Only 2/2/4 were more than 3 px from the true wrist epipolar line. Keeping each continuous headset ray fixed, a 3×3 depth neighborhood with span above 2 cm reduced truth reprojection from above 3 px to at most 3 px for 18/22/15 outliers; the corresponding 5×5 counts were 19/22/15. For the narrower both-table-footprint subset, 3×3 counts were 5/7/3 out of 15/14/16. These are plausible depth-boundary explanations, not proof of correct matching, corrected registration or a deployable algorithm. Oracle depth choices never entered PnP or control.

The runtime `cuda_memory_fraction` parameter defaults to 0.25, accepts only finite values in `(0,1]`, and applies the cap before either learned model allocates on CUDA. CPU mode does not initialize that allocator. Eight resource-contract tests pass (`registration_resource_tests.xml`), and the actual CUDA constructors/inference were exercised by the development comparison. PyTorch 2.7 requires resolving an unindexed `cuda` device to the active integer index; an initial configuration failure was preserved separately and is not counted as a registration trial.

## Native-resolution CUDA simulation profile

The declared `reach-projection-balanced`/matte fixture now has a separately tested **native 1280×960 headset RGBD** source, with fx=fy=640 and aligned depth at that resolution. Wrist RGB remains 640×480/fx=fy=320. It changes renderer sampling, not post-render upsampling; nearest-depth lookup, matching geometry and confidence gates are unchanged.

The first ordered CUDA candidate, **512 features/filter 0.8**, passed all three rest and three end development captures. After freezing, **six fresh captures also passed**: rest confidence 0.987/0.988/0.990 and worst five-target errors 2.75/1.24/3.22 mm; end confidence 0.956/0.956/0.955 and errors 0.97/0.95/0.73 mm. Transform translation errors were respectively 19.34/8.84/19.02 mm and 5.24/2.22/4.70 mm, distinct from target-mapping error. Synchronized warm processing was 95–110 ms; sampled development process GPU peak was 1,196 MiB. See `head1280_gpu_development_selection.json` and `head1280_gpu_frozen_validation.json`.

Use the separate [`config/isaac_head1280_cuda.yaml`](config/isaac_head1280_cuda.yaml) only with that declared simulator setup. It specifies CUDA/512/filter 0.8, four CPU threads, 25% PyTorch allocator cap, one-second frontend/controller lifetimes, and 0.30/0.25-second registration/recovery periods. The initial 0.50/0.25-second schedule with best-effort subscriptions is preserved in [`config/isaac_head1280_cuda_05.yaml`](config/isaac_head1280_cuda_05.yaml). The primary profile explicitly sets `camera_qos: reliable_latest` for the seven synchronized subscriptions: RELIABLE, KEEP_LAST depth 2, VOLATILE. This requires compatible reliable source publishers, as provided by this Isaac adapter. Other configurations retain the default `sensor_data` best-effort policy. Model, depth and quality gates are identical. Neither file enables execution; supply URDF/controller settings separately, as in the [ROS runbook](../../README.md). The earlier CPU profiles remain unchanged.

```bash
# In the configured ROS/Jazzy environment, with this simulator already running:
.venv-registration/bin/python -m deictic_registration.node --ros-args \
  --params-file ros2/src/deictic_registration/config/isaac_head1280_cuda.yaml
```

Run only one observation provider on the controller's topic. A fresh-stack controller launch can select the same file with `DEICTIC_CONTROL_PARAMS`; do not launch a duplicate TCP endpoint alongside an existing stack. For isolated candidate scoring, remap both registration and failure outputs as above and set the observer's `--max-age-s 1` and a distinct `--output` path.

At the original 0.50-second nominal period, the profile passed a 35-second live rest check: all 350 controller statuses allowed commits, all 66 observations passed, no failures, minimum confidence 0.9815, delivery age 0.133–0.340 s, and maximum independently scored five-target error 5.944 mm. A fixed-world center reach then passed at 18:58:35–18:58:49 UTC, with 2.16 mm grounding error and 4.51 mm measured tool error after 4.5 seconds of preview review. A separate 30-second reached-pose recording passed all 300 status and 57 observation samples: confidence at least 0.9161, delivery age 0.173–0.471 s, maximum target-mapping error 2.179 mm, and maximum controller observation age 0.972 s. Those bounded windows passed, but a longer 240-second recording had 45/2,402 status samples closed by expiration even though all 442 delivered observations passed individually. The first five-target attempt therefore failed before target 1 moved: ages 1.063835 and 1.164125 s cleared the preview, followed by valid recovery without reviving that preview. This was not an epoch or geometric-binding failure.

The provider-only 0.30-second trial still had 5/351 stale status samples under concurrent GPU load, despite 63/63 delivered observations passing. The source was only about 1.86 Hz. The Isaac camera adapter then began reading each actual render (`frequency=-1`, existing render schedule and 15 Hz wall cap), keeping the scene, physics and strict same-frame pairing unchanged. Source cadence improved to roughly 3.2–3.8 Hz, but best-effort transport still had 6 stale status samples and a 1.285-second result gap despite 101/101 good estimates. Those unsuccessful stages are preserved in `head1280_cadence03_*.json` and `head1280_rendercadence_*.json`.

The subsequent reliable-subscription check **passed its 35-second idle window**: 109/109 visual estimates and 350/350 gate-open statuses, maximum controller age 0.8992 s, and graph keys 3904→4012. Delivery age was median 0.379 s, p95 0.552 s and maximum 0.587 s; maximum result interval was 0.400 s. WebRTC remained on, the unrelated user's GPU workload was untouched, and no system socket-buffer setting changed. UDP drop counters still increased, so the result establishes maintained freshness in this window rather than lossless transport. Seven QoS policy cases plus four existing image-message cases passed in ROS Jazzy. Evidence is `head1280_reliable_{control,registration,feedback_live,udp_window}.json`.

The fresh five-target retry failed before motion/preview acceptance on target 1 (`playmode-markerless-five-targets-reliable-results.xml`, 19:37:38–19:37:51 UTC, 13.3095 s, grounding 2.35 mm). Synchronous planning blocked joint-feedback processing, triggering `joint_feedback_stale` while registration remained valid (age 0.879/0.880 s, confidence 0.989). Tracking recovered but the preview stayed cleared. This differs from the earlier registration timeout; the nonblocking planner correction passed 44 controller tests without relaxing timeouts. After restarting the controller with a fresh graph, a partial run completed targets 1/2 (grounding 1.11/0.15 mm; tool 2.16/4.45 mm), then rejected target 3 before motion for timestamp quality. Its 150-second recording had 1507/1507 open gates and 457/457 passing estimates; a separate target-2 postcheck passed 301/301 statuses and 92/92 estimates. The tighter Unity clock-sync rule passed 32 tests and then the complete ordered five-target traversal at 20:03:28–20:04:07 UTC. Independent grounding was 0.17/0.78/0.21/0.37/0.66 mm and tool error 2.62/2.83/3.93/3.56/4.87 mm. Every preview was reviewed for 4.5 seconds and completion required movement plus fresh feedback. The 39.176-second run started at the prior target-2 pose, then reached 1→5; it is not a neutral-start or paper mobile-user trial. Separately, both short camera UI tests passed with the full live stack and a 480×360 wrist frame aged 0.276 s; that does not establish bandwidth endurance.

The successful traversal's associated 150-second observer passed **1516/1516 gate-open statuses and 453/453 estimates**, with no failures and maximum controller registration age **0.928550 s**. A dedicated 30-second target-5 observer passed **299/299 statuses and 92/92 estimates**, with no failures, maximum age **0.907099 s**, minimum confidence **0.929153** and maximum independent diagnostic five-target error **3.947 mm**. Its graph advanced from 2026 to 2116 accepted keyframes. These are bounded observed windows in the declared static-headset fixture, not a general reliability guarantee; the windows and the separately retained `segment2` continuation overlap and must not be summed. Reports are `head1280_five_target_clock_{control,registration}.json` and `head1280_five_target_clock_post_{control,registration}.json`.

With 10,000 accepted keyframes, nominal 3.33 Hz would reach the graph's explicit session limit in about 50 minutes, compared with about 83 minutes at 2 Hz. Actual accepted rate determines capacity use; the graph does not automatically reset or discard uncertainty.

All reported measurements are simulation/software checks, not physical Quest/robot accuracy. Physical camera extrinsics, sensor synchronization, real depth quality, motion distortion, camera visibility, texture coverage, and confidence calibration still require captured data. Planar scenes can be geometrically ambiguous even with small reprojection residuals; the temporal graph and independent ground-truth evaluation remain necessary. Registration attempts are periodic with confidence-driven recovery; a predicted-residual trigger is not implemented.
