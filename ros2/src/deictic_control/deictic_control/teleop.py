"""Simulation-only absolute bimanual clutch and bounded pose servo.

No visual-registration transform enters this path. Inputs use a stable body yaw.
The pose/posture/continuity objective follows the approach of Unitree's
xr_teleoperate H1_ArmIK (817fb00), adapted to K1 geometry with SciPy; no Unitree
driver, solver implementation, or unconstrained actuator output is copied.
"""
from dataclasses import dataclass
import json
import math
import re
import time
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .kinematics import segment_intersects_box
from .se3 import rotation, skew


@dataclass(frozen=True)
class ClutchInput:
    session_id: str
    sequence: int
    stamp: float
    clutch: bool
    left_tracked: bool
    right_tracked: bool
    positions: np.ndarray
    rotations: np.ndarray

    @classmethod
    def parse(cls, payload):
        if not isinstance(payload, str) or len(payload) > 4096:
            raise ValueError('invalid_teleop_payload')
        data = json.loads(payload)
        if not isinstance(data, dict) or type(data.get('schema_version')) is not int or data['schema_version'] != 3:
            raise ValueError('protocol_version_mismatch')
        if data.get('frame_id') != 'teleop_body':
            raise ValueError('invalid_teleop_frame')
        session, sequence = data.get('session_id'), data.get('sequence')
        if (not isinstance(session, str) or re.fullmatch(r'[A-Za-z0-9_-]{1,128}', session) is None
                or type(sequence) is not int or not 0 <= sequence <= 9_007_199_254_740_991):
            raise ValueError('invalid_teleop_session_or_sequence')
        for key in ('clutch', 'left_tracked', 'right_tracked'):
            if type(data.get(key)) is not bool:
                raise ValueError('invalid_teleop_tracking_or_clutch')
        stamp = data.get('stamp')
        if type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp <= 0:
            raise ValueError('invalid_teleop_stamp')
        positions = np.asarray([data.get('left_position'), data.get('right_position')], dtype=float)
        if positions.shape != (2, 3) or not np.all(np.isfinite(positions)) or np.max(np.abs(positions)) > 100:
            raise ValueError('invalid_teleop_positions')
        quaternions = np.asarray([data.get('left_rotation'), data.get('right_rotation')], dtype=float)
        if (quaternions.shape != (2, 4) or not np.all(np.isfinite(quaternions))
                or np.max(np.abs(np.linalg.norm(quaternions, axis=1)-1.)) > .001):
            raise ValueError('invalid_teleop_rotations')
        return cls(session, sequence, float(stamp), data['clutch'], data['left_tracked'],
                   data['right_tracked'], positions.copy(), Rotation.from_quat(quaternions).as_matrix())


def position_jacobian(model, q):
    pose, jacobian = pose_jacobian(model, q)
    return pose[:3, 3], jacobian[:3]


def pose_jacobian(model, q):
    transform, cursor, axes, origins = np.eye(4), 0, [], []
    for joint in model.joints:
        transform = transform @ joint.origin
        if joint.movable:
            axes.append(transform[:3, :3] @ joint.axis)
            origins.append(transform[:3, 3].copy())
            motion = np.eye(4)
            motion[:3, :3] = rotation(joint.axis, q[cursor])
            transform = transform @ motion
            cursor += 1
    tip = transform[:3, 3] + transform[:3, :3] @ model.tool_offset
    jacobian = np.array([np.cross(axis, tip-origin) for axis, origin in zip(axes, origins)]).T
    transform[:3, 3] = tip
    return transform, np.vstack((jacobian, np.asarray(axes).T))


def chain_reach(model):
    """Upper bound from shoulder pivot to tool, excluding the trunk mount."""
    after_shoulder, length = False, float(np.linalg.norm(model.tool_offset))
    for joint in model.joints:
        if after_shoulder:
            length += float(np.linalg.norm(joint.origin[:3, 3]))
        if joint.movable:
            after_shoulder = True
    return length


class AnatomicalArmMapping:
    """Match shoulder-to-controller vectors to K1 shoulder-to-tool vectors.

    Positions are relative to the headset in a fixed horizontal body frame.
    Human shoulders are estimates, not inferred from gaze or clutch posture.
    The URDF supplies robot shoulder pivots and maximum arm reach. Fixed local
    yaw offsets align the controller's forward X with each tool's +/-Y axis.
    """
    def __init__(self, models, scale=1., shoulder_forward=-.05,
                 shoulder_half_width=.18, shoulder_down=.20,
                 tool_yaw_degrees=(-90., 90.)):
        dimensions = (scale, shoulder_forward, shoulder_half_width, shoulder_down)
        yaw = np.asarray(tool_yaw_degrees, dtype=float)
        if (not all(math.isfinite(v) for v in dimensions) or not 0 < scale <= 2
                or not -.3 <= shoulder_forward <= .3 or not .05 <= shoulder_half_width <= .4
                or not .05 <= shoulder_down <= .5 or yaw.shape != (2,)
                or not np.all(np.isfinite(yaw))):
            raise ValueError('invalid_teleop_anatomical_mapping')
        if len(models) != 2:
            raise ValueError('teleop_requires_two_arm_models')
        self.scale = scale
        self.human_shoulders = np.array([[shoulder_forward, shoulder_half_width, -shoulder_down],
                                         [shoulder_forward, -shoulder_half_width, -shoulder_down]])
        shoulders = []
        for model in models:
            transform = np.eye(4)
            for joint in model.joints:
                transform = transform @ joint.origin
                if joint.movable:
                    shoulders.append(transform[:3, 3].copy())
                    break
            else:
                raise ValueError('teleop_arm_missing_shoulder')
        self.robot_shoulders = np.array(shoulders)
        self.reach = np.array([chain_reach(model) for model in models])
        self.tool_yaw_degrees = yaw.copy()
        self.tool_rotations = Rotation.from_euler('z', yaw, degrees=True).as_matrix()

    def targets(self, positions, rotations):
        target = np.tile(np.eye(4), (2, 1, 1))
        target[:, :3, 3] = self.robot_shoulders+self.scale*(positions-self.human_shoulders)
        target[:, :3, :3] = rotations@self.tool_rotations
        return target

    def project(self, targets):
        projected = targets.copy()
        offset = targets[:, :3, 3]-self.robot_shoulders
        distance = np.linalg.norm(offset, axis=1)
        projected[:, :3, 3] = self.robot_shoulders+offset*np.minimum(
            1., self.reach/np.maximum(distance, 1e-12))[:, None]
        return projected, bool(np.any(distance > self.reach))

    def controller_poses(self, targets):
        """Inverse mapping for explicit, bounded simulation test fixtures."""
        return (self.human_shoulders+(targets[:, :3, 3]-self.robot_shoulders)/self.scale,
                targets[:, :3, :3]@self.tool_rotations.transpose(0, 2, 1))


class PoseIK:
    """Small bounded nonlinear solve; position dominates soft orientation.

    Residual units are metres: orientation .003 m/rad, continuity .0007 m/rad,
    clutch posture .0002 m/rad. Measured feedback seeds every solve. The last
    accepted solution selects a continuous branch; deterministic bent seeds
    escape a straight arm's zero first derivative for inward translation.
    """
    def __init__(self, model):
        self.model = model
        self.solution = self.posture = None
        self.last_seed_target = None
        self.chain = [(j.origin[:3, :3].copy(), j.origin[:3, 3].copy(),
                       j.axis.copy(), skew(j.axis), j.movable) for j in model.joints]

    def kinematics(self, q):
        """Allocation-light FK/Jacobian for repeated nonlinear evaluations."""
        orient, position, cursor = np.eye(3), np.zeros(3), 0
        axes, origins = np.empty((4, 3)), np.empty((4, 3))
        for origin_r, origin_p, axis, hat, movable in self.chain:
            position = position+orient@origin_p
            orient = orient@origin_r
            if movable:
                axes[cursor], origins[cursor] = orient@axis, position
                angle = q[cursor]
                orient = orient@(np.eye(3)+np.sin(angle)*hat+(1-np.cos(angle))*(hat@hat))
                cursor += 1
        tip = position+orient@self.model.tool_offset
        pose = np.eye(4); pose[:3, :3], pose[:3, 3] = orient, tip
        return pose, np.vstack((np.cross(axes, tip-origins).T, axes.T))

    def reset(self, q):
        self.solution, self.posture = q.copy(), q.copy()
        self.last_seed_target = None

    def solve(self, target, measured):
        model, prior = self.model, self.solution
        cached_q = cached_value = None
        def evaluate(q):
            nonlocal cached_q, cached_value
            if cached_q is not None and np.array_equal(q, cached_q):
                return cached_value
            pose, jac = self.kinematics(q)
            angular = Rotation.from_matrix(pose[:3, :3] @ target[:3, :3].T).as_rotvec()
            theta = np.linalg.norm(angular)
            hat = skew(angular)
            coefficient = 1/12 if theta < 1e-5 else (1-.5*theta/np.tan(.5*theta))/(theta*theta)
            inverse_left = np.eye(3)-.5*hat+coefficient*(hat@hat)
            residual = np.r_[pose[:3, 3]-target[:3, 3], .003*angular,
                             .0007*(q-prior), .0002*(q-self.posture)]
            derivative = np.vstack((jac[:3], .003*inverse_left@jac[3:],
                                    .0007*np.eye(4), .0002*np.eye(4)))
            cached_q, cached_value = q.copy(), (residual, derivative)
            return cached_value
        def optimize(seed):
            return least_squares(lambda q: evaluate(q)[0], np.clip(seed, model.lower+1e-8, model.upper-1e-8),
                                 jac=lambda q: evaluate(q)[1], bounds=(model.lower, model.upper),
                                 max_nfev=18, ftol=1e-5, xtol=1e-5, gtol=1e-7).x
        candidates = [optimize(measured)]
        if np.max(np.abs(prior-measured)) > .06:
            candidates.append(optimize(prior))
        score = lambda q: float(evaluate(q)[0] @ evaluate(q)[0])
        best = min(candidates, key=score)
        residual = np.linalg.norm(model.fk(best)[:3, 3]-target[:3, 3])
        if residual > .004 and (self.last_seed_target is None or
                np.linalg.norm(target[:3, 3]-self.last_seed_target) > .015):
            self.last_seed_target = target[:3, 3].copy()
            side = 1. if model.tool_offset[1] > 0 else -1.
            for seed in (np.array([-.4, -.6*side, -1.4, -1.5*side]),
                         np.array([.4, -.6*side, 1.4, -1.5*side])):
                candidate = optimize(seed)
                if score(candidate) < score(best):
                    best = candidate
        self.solution = best.copy()
        return best


def segment_distance(a, b, c, d):
    """Closest distance of finite 3D segments, including zero-length links."""
    u, v, w = b-a, d-c, a-c
    aa, bb, cc, dd, ee = u@u, u@v, v@v, u@w, v@w
    if aa < 1e-16:
        t = np.clip(ee / cc, 0., 1.) if cc > 1e-16 else 0.
        return float(np.linalg.norm(a-(c+t*v)))
    if cc < 1e-16:
        return float(np.linalg.norm(a+np.clip(-dd/aa, 0., 1.)*u-c))
    denominator = aa*cc-bb*bb
    s = np.clip((bb*ee-cc*dd)/denominator, 0., 1.) if denominator > 1e-16 else 0.
    t = (bb*s+ee)/cc
    if t < 0:
        t, s = 0., np.clip(-dd/aa, 0., 1.)
    elif t > 1:
        t, s = 1., np.clip((bb-dd)/aa, 0., 1.)
    return float(np.linalg.norm(w+s*u-t*v))


def check_bimanual_geometry(models, joints, table_top, obstacles):
    all_points = []
    torso = [-.08, -.075, -.10, .08, .075, .22]
    for model, q in zip(models, joints):
        if np.any(q < model.lower) or np.any(q > model.upper):
            raise ValueError('teleop_joint_limit')
        _, points = model.fk(q, with_points=True)
        if np.any(points[:, 2] < table_top+.025):
            raise ValueError('teleop_table_clearance')
        for index in range(1, len(points)-1):
            if segment_intersects_box(points[index], points[index+1], torso, .025):
                raise ValueError('teleop_trunk_clearance')
        for bounds in obstacles:
            for a, b in zip(points[:-1], points[1:]):
                if segment_intersects_box(a, b, bounds, .025):
                    raise ValueError('teleop_obstacle_clearance')
        all_points.append(points)
    for a, b in zip(all_points[0][:-1], all_points[0][1:]):
        for c, d in zip(all_points[1][:-1], all_points[1][1:]):
            if segment_distance(a, b, c, d) < .05:
                raise ValueError('teleop_cross_arm_clearance')


class BimanualClutch:
    def __init__(self, models, *, timeout=.25, speed=.35, acceleration=.7,
                 table_top=-.15, obstacles=(), translation_scale=1.,
                 shoulder_forward=-.05, shoulder_half_width=.18, shoulder_down=.20,
                 tool_yaw_degrees=(-90., 90.)):
        if (not all(math.isfinite(v) for v in (timeout, speed, acceleration, table_top, translation_scale))
                or not 0 < timeout <= .25 or speed <= 0 or acceleration <= 0 or not 0 < translation_scale <= 2):
            raise ValueError('invalid_teleop_limits')
        self.models = tuple(models)
        self.timeout, self.speed, self.acceleration = timeout, speed, acceleration
        self.table_top, self.obstacles = table_top, tuple(tuple(box) for box in obstacles)
        self.translation_scale = translation_scale
        self.mapping = AnatomicalArmMapping(self.models, translation_scale, shoulder_forward,
                                            shoulder_half_width, shoulder_down, tool_yaw_degrees)
        self.solvers = [PoseIK(model) for model in self.models]
        self.active = self.armed = False
        self.state, self.reason = 'release_required', 'startup_release_required'
        self.last_fault = ''
        self.session_id, self.sequence, self.input_stamp = None, -1, float('-inf')
        self.retired_sessions = set()
        self.received = self.last_tick = float('-inf')
        self.positions = self.command = None
        self.rotations = self.targets = None
        self.measured_position_errors = self.commanded_position_errors = None
        self.measured_orientation_errors = self.commanded_orientation_errors = None
        self.solve_duration = 0.
        self.velocity = np.zeros((2, 4))

    def stop(self, reason):
        self.active = self.armed = False
        self.state, self.reason = 'release_required', reason
        if reason not in ('clutch_released', 'teleop_session_changed_release_required', 'teleop_release_required'):
            self.last_fault = reason
        self.velocity[:] = 0.

    def ingest(self, payload, now, monotonic, joints, guard_reason=None):
        try:
            value = ClutchInput.parse(payload)
            if value.session_id in self.retired_sessions:
                return 'ignored'
            if value.session_id == self.session_id and value.sequence <= self.sequence:
                return 'ignored'  # Replays never extend the input lease.
            if now-value.stamp > self.timeout or value.stamp-now > .05:
                raise ValueError('teleop_input_stale_or_future')
            if value.session_id != self.session_id:
                if len(self.retired_sessions) >= 64:
                    raise ValueError('teleop_session_limit_restart_controller')
                if self.session_id is not None:
                    self.retired_sessions.add(self.session_id)
                self.stop('teleop_session_changed_release_required')
                self.session_id, self.sequence, self.input_stamp = value.session_id, -1, float('-inf')
            if value.stamp < self.input_stamp:
                raise ValueError('teleop_timestamp_reversed')
            self.sequence, self.input_stamp = value.sequence, value.stamp
            self.received = monotonic
            if not value.left_tracked or not value.right_tracked:
                raise ValueError('teleop_controller_tracking_lost')
            if guard_reason is not None:
                raise ValueError(guard_reason)
            if not value.clutch:
                self.active, self.armed = False, True
                self.state, self.reason = 'ready', 'clutch_released'
                self.velocity[:] = 0.
                return 'released'
            if not self.armed:
                raise ValueError('teleop_release_required')
            self.positions = value.positions.copy()
            self.rotations = value.rotations.copy()
            # The current human pose defines the target immediately, including
            # on first acquisition and re-clutch. Joint commands still start at
            # measured feedback and approach this goal through the rate guards.
            self.targets = self.mapping.targets(self.positions, self.rotations)
            if not self.active:
                check_bimanual_geometry(self.models, joints, self.table_top, self.obstacles)
                self.command = np.array(joints, dtype=float, copy=True)
                for solver, q in zip(self.solvers, joints):
                    solver.reset(q)
                self.last_tick, self.active = monotonic, True
                self.velocity[:] = 0.
                self.state, self.reason = 'active', 'bimanual_pose_ik'
                self.update_errors(joints)
                return 'began'
            return 'updated'
        except (ValueError, TypeError, KeyError, np.linalg.LinAlgError) as error:
            self.stop(str(error))
            return 'rejected'

    def update_errors(self, joints):
        if self.targets is None or joints is None:
            return
        def errors(qs):
            poses = [model.fk(q) for model, q in zip(self.models, qs)]
            return ([float(np.linalg.norm(p[:3, 3]-t[:3, 3])) for p, t in zip(poses, self.targets)],
                    [float(Rotation.from_matrix(p[:3, :3]@t[:3, :3].T).magnitude())
                     for p, t in zip(poses, self.targets)])
        self.measured_position_errors, self.measured_orientation_errors = errors(joints)
        self.commanded_position_errors, self.commanded_orientation_errors = errors(self.command)

    def tick(self, now, monotonic, joints, guard_reason=None):
        if not self.active:
            if self.armed and (monotonic-self.received > self.timeout or now-self.input_stamp > self.timeout):
                self.stop('teleop_input_timeout')
            return None
        try:
            if guard_reason:
                raise ValueError(guard_reason)
            if (monotonic-self.received > self.timeout or now-self.input_stamp > self.timeout
                    or self.input_stamp-now > .05):
                raise ValueError('teleop_input_timeout')
            dt = monotonic-self.last_tick
            if not 0 < dt <= .15:
                raise ValueError('teleop_servo_deadline')
            if np.max(np.abs(joints-self.command)) > .10:
                raise ValueError('teleop_joint_tracking_error')
            projected, outside_reach = self.mapping.project(self.targets)
            limited = 'teleop_workspace_projection' if outside_reach else None
            solutions, started = [], time.perf_counter()
            for solver, target, measured in zip(self.solvers, projected, joints):
                solutions.append(solver.solve(target, measured))
            self.solve_duration = time.perf_counter()-started
            solutions = np.asarray(solutions)
            # Brake before the goal or the measured-feedback lead envelope.
            # The .075-rad envelope prevents command integration running away
            # under lag; the independent .10-rad tracking fault is unchanged.
            displacement = solutions-self.command
            direction = np.sign(displacement)
            lead_distance = np.maximum(0., .075-direction*(self.command-joints))
            braking_distance = np.minimum(np.abs(displacement), lead_distance)
            braking_speed = np.maximum(0., np.sqrt((self.acceleration*dt)**2+
                                       2*self.acceleration*braking_distance)-self.acceleration*dt)
            desired_velocity = direction*np.minimum(self.speed, braking_speed)
            desired_velocity = np.where(np.abs(displacement) < 1e-5, 0., desired_velocity)
            for i, (solver, q) in enumerate(zip(self.solvers, self.command)):
                _, jacobian = solver.kinematics(q)
                cartesian_speed = np.linalg.norm(jacobian[:3]@desired_velocity[i])
                desired_velocity[i] *= min(1., .09/max(cartesian_speed, 1e-12))
            # Reserve braking distance to BOTH fixed joint boundaries and the
            # measured lead envelope, independently of the moving IK goal.
            # Clamping the whole velocity vector after one joint hit a bound
            # would incorrectly brake the other seven joints instantaneously.
            lower = np.maximum(np.array([m.lower for m in self.models]), joints-.075)
            upper = np.minimum(np.array([m.upper for m in self.models]), joints+.075)
            brake = lambda distance: np.maximum(0., np.sqrt((self.acceleration*dt)**2+
                2*self.acceleration*np.maximum(0., distance))-self.acceleration*dt)
            safe_low, safe_high = -brake(self.command-lower), brake(upper-self.command)
            low = np.maximum(self.velocity-self.acceleration*dt, safe_low)
            high = np.minimum(self.velocity+self.acceleration*dt, safe_high)
            if np.any(low > high+1e-10):
                # Abrupt feedback reversal can make the old velocity infeasible;
                # the measured-envelope safety stop takes priority in that case.
                velocity = np.clip(self.velocity, safe_low, safe_high)
                limited = 'teleop_feedback_braking_hold'
            else:
                velocity = np.clip(desired_velocity, low, high)
            command = self.command+velocity*dt
            # Joint bounds, tracking lead and 0.10 m/s commanded tool motion
            # are projection/hold constraints, never a latched workspace fault.
            scale = 1.
            for model, before, after, measured in zip(self.models, self.command, command, joints):
                tool_step = np.linalg.norm(model.fk(after)[:3, 3]-model.fk(before)[:3, 3])
                scale = min(scale, .10*dt/max(tool_step, 1e-12))
            if scale < 1.:
                limited = limited or 'teleop_rate_or_feedback_projection'
                command = self.command+scale*(command-self.command)
                velocity *= scale
            try:
                samples = max(2, int(np.ceil(np.max(np.abs(command-joints))/.01))+1)
                for fraction in np.linspace(0., 1., samples):
                    check_bimanual_geometry(self.models, joints+fraction*(command-joints),
                                             self.table_top, self.obstacles)
            except ValueError as error:
                command, velocity, limited = self.command.copy(), np.zeros_like(velocity), str(error)
            residuals = [np.linalg.norm(m.fk(q)[:3, 3]-t[:3, 3])
                         for m, q, t in zip(self.models, solutions, self.targets)]
            if max(residuals) > .012:
                limited = limited or 'teleop_pose_unreachable_projection'
            angular_residuals = [Rotation.from_matrix(m.fk(q)[:3, :3]@t[:3, :3].T).magnitude()
                                 for m, q, t in zip(self.models, solutions, self.targets)]
            if max(angular_residuals) > .15:
                limited = limited or 'teleop_orientation_limited'
            self.command, self.velocity, self.last_tick = command, velocity, monotonic
            self.state, self.reason = ('limited', limited) if limited else ('active', 'bimanual_pose_ik')
            self.update_errors(joints)
            return command.copy()
        except (ValueError, np.linalg.LinAlgError) as error:
            self.stop(str(error))
            return None
