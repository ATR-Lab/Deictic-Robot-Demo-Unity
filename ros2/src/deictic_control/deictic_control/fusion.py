"""Paper pose graph and conservative confidence policy.

GTSAM is an explicit optional dependency. Absence never falls back to identity,
ground truth, an average, or a one-shot transform under the fusion label.
"""
from dataclasses import dataclass
import json
import math
import numpy as np
from .se3 import from_pose, inverse, to_pose


@dataclass
class FusionConfig:
    alpha: float = 12.0
    beta: float = 1.0
    gamma: float = 6.0
    threshold: float = 0.7
    min_inliers: int = 12
    max_registration_age: float = 1.0
    max_future_skew: float = 0.05
    max_translation_jump: float = 0.15
    max_rotation_jump: float = 0.35
    allow_synthetic: bool = False


@dataclass
class Observation:
    stamp: float
    headset: np.ndarray
    camera: np.ndarray
    relative: np.ndarray
    inliers: int
    matches: int
    residual: float
    source: str

    @classmethod
    def parse(cls, payload):
        data = json.loads(payload) if isinstance(payload, str) else payload
        if not isinstance(data, dict):
            raise ValueError('Registration must be a JSON object')
        if data.get('schema_version') != 1:
            raise ValueError('Unsupported registration schema')
        inliers, matches = data['inliers'], data['matches']
        if (isinstance(inliers, bool) or isinstance(matches, bool)
                or not isinstance(inliers, int) or not isinstance(matches, int)
                or not 0 <= inliers <= matches or matches < 1):
            raise ValueError('Invalid inlier/match counts')
        stamp, residual = float(data['stamp']), float(data['median_reprojection_error'])
        if not math.isfinite(stamp + residual) or stamp <= 0 or residual < 0:
            raise ValueError('Invalid registration timestamp/residual')
        return cls(stamp, from_pose(data['headset_pose']), from_pose(data['camera_pose']),
                   from_pose(data['camera_from_headset']), inliers, matches, residual,
                   str(data['source']))


class GtsamPoseGraph:
    """iSAM2 H/C chains plus robust ternary log(Z^-1 C^-1 X H) factors.

    H0 anchors headset coordinates; calibrated FK anchors C. Output includes the
    optimized-current-headset correction so drift corrected inside H is applied
    to targets expressed in the still-drifting headset VIO world.
    """
    def __init__(self, max_keyframes=2500):
        if int(np.__version__.split('.')[0]) >= 2:
            raise ImportError('GTSAM 4.2 requires NumPy <2; use Python 3.12 / NumPy 1.26 environment')
        import gtsam
        self.g = gtsam
        self.isam = gtsam.ISAM2()
        self.index = 0
        self.previous = None
        self.max_keyframes = max_keyframes
        self.callbacks = []  # Retain Python callbacks for the lifetime of factors.
        self.name = 'gtsam_isam2'

    def reset(self):
        self.isam = self.g.ISAM2()
        self.index = 0
        self.previous = None
        self.callbacks.clear()

    def update(self, obs):
        g = self.g
        if self.index >= self.max_keyframes:
            raise RuntimeError('Pose graph capacity reached; restart the session')
        i = self.index
        x, h, c = g.symbol('x', 0), g.symbol('h', i), g.symbol('c', i)
        graph, values = g.NonlinearFactorGraph(), g.Values()
        H, C, Z = g.Pose3(obs.headset), g.Pose3(obs.camera), g.Pose3(obs.relative)
        noise = lambda r, t: g.noiseModel.Diagonal.Sigmas(np.array([r]*3+[t]*3))
        values.insert(h, H)
        values.insert(c, C)
        if i == 0:
            values.insert(x, C.compose(Z).compose(H.inverse()))
            graph.add(g.PriorFactorPose3(h, H, noise(1e-5, 1e-5)))
        else:
            old_h, old_c = self.previous
            graph.add(g.BetweenFactorPose3(g.symbol('h', i-1), h,
                      g.Pose3(old_h).between(H), noise(0.015, 0.01)))
            graph.add(g.BetweenFactorPose3(g.symbol('c', i-1), c,
                      g.Pose3(old_c).between(C), noise(0.01, 0.005)))
        graph.add(g.PriorFactorPose3(c, C, noise(0.01, 0.005)))

        def residual(poses):
            Xp, Hp, Cp = poses
            return g.Pose3.Logmap(Z.inverse().compose(Cp.inverse().compose(Xp).compose(Hp)))

        def error(factor, estimates, jacobians):
            poses = [estimates.atPose3(key) for key in factor.keys()]
            result = residual(poses)
            if jacobians is not None:
                eps = 1e-6
                for j in range(3):
                    derivative = np.empty((6, 6), order='F')
                    for k in range(6):
                        delta = np.zeros(6)
                        delta[k] = eps
                        plus, minus = list(poses), list(poses)
                        plus[j], minus[j] = poses[j].retract(delta), poses[j].retract(-delta)
                        derivative[:, k] = (residual(plus)-residual(minus))/(2*eps)
                    jacobians[j] = derivative
            return result

        self.callbacks.append(error)
        robust = g.noiseModel.Robust.Create(g.noiseModel.mEstimator.Huber.Create(1.345), noise(0.025, 0.015))
        graph.add(g.CustomFactor(robust, [x, h, c], error))
        self.isam.update(graph, values)
        self.isam.update()
        result = self.isam.calculateEstimate()
        correction = result.atPose3(x).matrix() @ result.atPose3(h).matrix() @ inverse(obs.headset)
        self.previous = (obs.headset, obs.camera)
        self.index += 1
        return correction


class SimulationReference:
    """Known simulated frame transform; NOT a markerless pose estimator."""
    name = 'simulation_reference'

    def reset(self):
        pass

    def update(self, obs):
        if obs.source != 'simulation_ground_truth':
            raise ValueError('Reference mode accepts synthetic ground truth only')
        return obs.camera @ obs.relative @ inverse(obs.headset)


class FusionState:
    def __init__(self, backend=None, config=None):
        self.config = config or FusionConfig()
        self.backend = backend
        self.transform = None
        self.confidence = 0.0
        self.last_accepted = None
        self.last_seen = None
        self.version = 0
        self.epoch = 0
        self.initialized = False
        self.minimum_observation_stamp = float('-inf')
        self.reason = 'awaiting_registration' if backend else 'gtsam_unavailable'
        self.healthy = False

    def invalidate(self, reason):
        self.healthy, self.confidence, self.reason = False, 0.0, str(reason)

    def reset(self, reason, minimum_stamp):
        """Start a new headset-origin epoch; keep only the frozen display pose."""
        if not math.isfinite(minimum_stamp):
            raise ValueError('Reset timestamp must be finite')
        self.invalidate(reason)
        self.last_seen = self.last_accepted = None
        self.minimum_observation_stamp = minimum_stamp
        self.initialized = False
        self.version += 1
        self.epoch += 1
        if self.backend is not None:
            self.backend.reset()

    def ingest(self, obs, now):
        cfg = self.config
        if self.backend is None:
            self.invalidate('gtsam_unavailable')
            return False
        if obs.source != 'superpoint_pnp' and not (cfg.allow_synthetic and obs.source == 'simulation_ground_truth'):
            self.invalidate('unapproved_registration_source')
            return False
        if obs.stamp <= self.minimum_observation_stamp:
            self.invalidate('pre_reset_registration')
            return False
        if now-obs.stamp > cfg.max_registration_age or obs.stamp-now > cfg.max_future_skew:
            self.invalidate('registration_timestamp_out_of_range')
            return False
        if self.last_seen is not None and obs.stamp <= self.last_seen:
            self.invalidate('out_of_order_registration')
            return False
        self.last_seen = obs.stamp
        value = cfg.alpha*obs.inliers/obs.matches - cfg.beta*obs.residual - cfg.gamma
        confidence = 1/(1+math.exp(-max(-60, min(60, value))))
        if obs.inliers < cfg.min_inliers or confidence < cfg.threshold:
            self.invalidate('registration_quality_low')
            return False
        candidate = obs.camera @ obs.relative @ inverse(obs.headset)
        if self.transform is not None and self.initialized:
            delta = inverse(self.transform) @ candidate
            angle = math.acos(float(np.clip((np.trace(delta[:3,:3])-1)/2, -1, 1)))
            if np.linalg.norm(delta[:3,3]) > cfg.max_translation_jump or angle > cfg.max_rotation_jump:
                self.invalidate('registration_jump_rejected')
                return False
        try:
            result = self.backend.update(obs)
        except Exception as error:
            self.invalidate('fusion_error: '+str(error))
            return False
        if result.shape != (4, 4) or not np.all(np.isfinite(result)):
            self.invalidate('invalid_optimizer_result')
            return False
        changed = self.transform is None or np.linalg.norm(self.transform-result) > .0005
        self.transform, self.confidence = result, confidence
        self.last_accepted, self.healthy = obs.stamp, True
        self.initialized = True
        if changed:
            self.version += 1
        self.reason = 'tracking'
        return True

    def snapshot(self, now):
        age = None if self.last_accepted is None else max(0, now-self.last_accepted)
        if age is not None and age > self.config.max_registration_age:
            self.invalidate('registration_stale')
        return dict(schema_version=1, alignment_version=self.version, alignment_epoch=self.epoch, confidence=self.confidence,
                    can_commit=bool(self.healthy and self.transform is not None), reason=self.reason,
                    stamp=now, registration_age=age, backend=self.backend.name if self.backend else 'unavailable',
                    mode='synthetic_test' if self.config.allow_synthetic else 'markerless',
                    base_from_headset_world=None if self.transform is None else to_pose(self.transform))
