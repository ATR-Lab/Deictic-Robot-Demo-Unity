# Paper review and reproduction contract

Reviewed 2026-09-16 against the user-supplied manuscript *Deictic Shared-Autonomy Manipulation via Continuous Markerless Egocentric MR-to-Robot Frame Fusion* (Arnett et al.). The supplied PDF is not redistributed in this repository. Pages 3–6 were rendered and visually checked, including the architecture, algorithm, experimental design, and results tables. This is a review of reported methods and numbers, not a replication of measurements.

## What the paper actually implements

The contribution is continuous cross-device registration, not just a ROS bridge or an MR robot visualization. Sections III-B to III-E and Figures 2-3 require:

1. Timestamped headset VIO, scene RGB, metric spatial geometry, gaze and tracked-stylus rays; timestamped robot joint states and a calibrated wrist RGB camera.
2. A maintained transform `X = T_base_headsetWorld`, with `p_base = X * H(t) * p_headset`. Robot camera pose comes from forward kinematics and a measured wrist-to-camera extrinsic.
3. SuperPoint image features matched across headset and wrist views. Headset mesh depth gives 3D correspondences; RANSAC PnP followed by robust reprojection refinement estimates relative camera pose.
4. A pose graph containing headset poses, robot camera poses, and `X`, with VIO, kinematic, and cross-device factors; incremental iSAM2 optimization through GTSAM.
5. Confidence `sigmoid(alpha * inlier_ratio - beta * median_reprojection_error - gamma)`. Low confidence freezes the last accepted transform, prevents new commits, increases registration attempts, and warns the operator.
6. Gaze and stylus ray intersections with scene geometry. Average agreeing candidates within `delta`; otherwise prefer the stylus. Voice deliberately commits the target. Surface normals set approach direction, with a downward fallback.
7. Workspace/obstacle checks, damped IK, a timed joint trajectory, cancellation/refinement, and inverse-transform MR feedback for current robot pose, target, and plan.

For calibrated optical camera poses `A = T_headsetWorld_headsetRGB` and `C = T_base_wristRGB`, a PnP result `Z = T_wristRGB_headsetRGB` implies `X = C * Z * inverse(A)`. Camera sensor poses must be evaluated at acquisition time; the center-eye transform at processing time is not interchangeable. A Unity point already in headset-world coordinates needs `X` only, without a second multiplication by `H`.

## Quest 3S and Booster K1 adaptations

| Paper setup | Required adaptation and claim boundary |
|---|---|
| Apple Vision Pro eye gaze | Quest 3S has no eye-tracking hardware. A center-eye forward ray is **head direction**, not measured eye gaze. Label and log this adaptation; retain an input-provider boundary for a real gaze sensor. [Meta input documentation](https://developers.meta.com/horizon/essentials/horizon-os-input/) |
| Tracked rigid stylus | A tracked Touch controller with a calibrated pointer origin/direction is an explicit pointing substitute. A hand ray also changes the modality. Neither establishes tracking of an arbitrary physical stylus. |
| Headset scene RGB | Quest 3S supports the Passthrough Camera API with passthrough enabled and headset-camera permission. MRUK exposes images, intrinsics, extrinsics, and timestamps. Camera pose, optical axes, resolution, and image orientation must all agree. [Meta Unity camera guide](https://developers.meta.com/horizon/documentation/unity/unity-pca-documentation/) |
| Spatial mesh/depth | Quest 3S lacks Quest 3's dedicated depth sensor, but supports software-derived environment depth. Scene geometry and live depth are different sources: scanned static geometry cannot be assumed current after objects move. Use capability checks and record the actual provider. [Device comparison](https://developers.meta.com/horizon/essentials/compare-devices/), [Depth API overview](https://developers.meta.com/horizon/documentation/unreal/unreal-depthapi-overview/) |
| Physical headset | Meta XR Simulator provides an OpenXR API/device profile and synthetic scene/depth data on Windows; it does not emulate Android or device hardware. Passthrough Camera API is explicitly unsupported in the simulator. Synthetic camera frames must come from a separate declared provider. Simulator success does not validate device camera permissions, real VIO drift, or sensor latency. [Simulator overview](https://developers.meta.com/horizon/documentation/unity/xrsim-intro/), [camera API limitations](https://developers.meta.com/horizon/documentation/spatial-sdk/spatial-sdk-pca-overview/) |
| Fixed SO-101 tabletop arm | K1 requires its own URDF joint limits, end-effector frame, FK, IK, collision geometry, trajectory controller, and wrist-camera mounting/extrinsic. A fixed-base arm-reaching demonstration is the closest initial match. Walking adds base motion and balance control outside the paper's scope; a static `T_base_headsetWorld` cannot silently absorb an independently moving base. |
| Wrist-mounted RGB camera | An Isaac camera attached to the chosen hand frame can test the algorithm. A physical camera and its calibration must be supplied for robot deployment; the URDF alone is not evidence that this sensor exists. |
| Voice commit | A controller/keyboard commit can test the state machine, but must be logged as a substitute until actual speech recognition is enabled and tested. |

The requested hardware changes make this an architectural reproduction with disclosed modality/embodiment changes. Matching the paper's user-study scores is a separate new experiment.

## Missing information that prevents exact replication

- The paper gives no executable source, recorded trials, calibration files, or model weights. It omits camera models/resolutions/distortion, image-to-depth alignment, camera latency and timestamp conversion, joint zero offsets, robot link calibration, and tracked-stylus calibration.
- SuperPoint model/version, descriptor matcher, matching thresholds, keypoint count, minimum inliers, RANSAC settings, reprojection loss, registration interval and residual trigger are unspecified. The confidence coefficients, acceptance threshold, candidate agreement distance, and confidence expiration are unspecified. Implementation values must be documented as chosen defaults, not recovered paper parameters.
- Equation (4) does not fully specify the registration residual, covariance matrices, priors/gauge anchoring, graph initialization, marginalization/windowing, or robust factor losses. Relative VIO/kinematic chains alone do not anchor absolute frames. It is also unclear whether hand-eye/kinematic errors are explicit calibration variables or merely absorbed into pose corrections.
- Algorithm 1 has no explicit failure branch for insufficient inliers. A practical implementation must invalidate confidence for failed or stale observations, not keep permitting commits indefinitely. Recovery hysteresis, tracking recenter/map reset behavior, and the policy for an already executing trajectory when confidence falls are absent.
- Workspace bounds, obstacle geometry/margins, IK damping/tolerances, orientation priorities, controller limits, normal reliability criteria, and cancellation behavior are not supplied. A surface normal alone leaves tool roll unspecified. The paper's five-joint arm cannot generally achieve arbitrary six-dimensional end-effector poses.
- Reproduction needs exact task target coordinates, Task 3 reach count/movement schedule, visibility protocol for the fiducial condition, questionnaire anchors, timeout treatment, trial logs, and the R analysis scripts. Independent ground truth must never feed the estimator during scored trials.

## Reported findings and numerical audit

The paper reports 20 participants, three tasks and four conditions, one trial per cell (240 trials), a 120-second timeout, 3 cm reach tolerance, and offline 120 Hz OptiTrack scoring (Sections IV-C to IV-F). No such study has been performed here.

| Reported result | Audit |
|---|---|
| Task 2 alignment: full 0.9 cm mean/2.4 cm max/3.8 degrees; one-shot 3.4 cm/8.2 cm/12.7 degrees; fiducial 0.7 cm/2.1 cm/2.9 degrees | Table III and Figure 5 agree. No raw trajectories or uncertainty are provided for independent verification. |
| Tasks 2-3 grounding: full 2.4 and 3.1 cm; one-shot 4.7 and 6.1 cm | Means of 2.75 and 5.4 cm imply a 49.07% reduction, consistent with the headline 49%. |
| Interventions: 1.9 to 0.9 per task | Displayed values imply 52.63%; the stated 52% may use unrounded data or truncation. Raw values are needed. |
| Tables IV-V aggregate means | Displayed condition means equal the equally weighted means of the three task means. Reported partial eta-squared values are consistent with their displayed F statistics and degrees of freedom after rounding. This does not verify the tests from data. |
| Success percentages and standard deviations | The declared one binary task outcome per participant/condition/task implies task-level percentages in increments of 5%. Table V includes 98%, 97%, 89.6%, 61.9%, etc. A 95% binary success rate across 20 people has sample SD about 22.36 percentage points, not the listed 7.0. Table IV also includes rates incompatible with 60 binary trials per condition. Clarify whether these are per-reach scores, modeled estimates, additional repetitions, or another denominator; those are not the stated metric. |
| Alignment full versus fiducial: `p=.041`, paired `d_z=.46` | If these describe the same ordinary two-sided paired comparison with N=20, `t = d_z * sqrt(20) = 2.057`, df=19 gives unadjusted `p=.05366`, before any Bonferroni correction. The comparison method/effect-size definition needs clarification. A model contrast using another error term might differ, but that is not documented. |

Non-significant full-versus-fiducial tests are not equivalence tests. Lower grounding error does not establish end-effector tracking accuracy or obstacle avoidance by itself. Reported failures include rapid head motion, low texture, cross-camera occlusion, and arm self-occlusion. The study concerns supervised reaching, not grasp success, physical contact, bimanual manipulation, or humanoid locomotion.

## Verification sequence

1. Verify frame directions, handedness, optical conventions, timestamps, confidence expiration, cancellation, and joint limits with deterministic unit/integration tests.
2. Exercise genuine feature matching and PnP on paired synthetic RGB/depth observations with known transforms; vary viewpoint, texture, depth noise, motion, and occlusion. Keep synthetic ground truth isolated for scoring.
3. Run ROS/Unity/Isaac round trips and measured trajectory execution with K1 fixed at the base. Record observed latency and residuals instead of inheriting the paper's latency claims.
4. On a physical Quest 3S, validate permission handling, real camera/depth alignment, timestamped poses, head/controller inputs, and tracking reset behavior. Calibrate the real robot camera before physical robot execution.
5. Compare continuous registration, one-shot, fiducial, and tablet conditions using preregistered outcome definitions and raw trial logs. Reconcile the paper's success denominators and statistical discrepancy before treating its tables as numerical replication targets.
