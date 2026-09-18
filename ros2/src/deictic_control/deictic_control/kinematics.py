"""URDF-derived fixed-base position IK and sampled conservative geometry checks.

K1 has four arm joints. Requested orientation is deliberately NOT solved. These
capsule/AABB checks are a simulation guard, not a complete mesh collision model.
"""
from dataclasses import dataclass
import xml.etree.ElementTree as ET
import numpy as np
from .se3 import rotation


@dataclass
class Joint:
    name: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    movable: bool


class ArmModel:
    def __init__(self, urdf, root='trunk', tip='right_elbow_yaw_link', tool_offset=(0, -.10, 0)):
        robot = ET.parse(urdf).getroot()
        parents = {j.find('child').get('link'): j for j in robot.findall('joint')}
        chain = []
        while tip != root:
            if tip not in parents:
                raise ValueError('No URDF chain from root to tip')
            joint = parents[tip]
            chain.append(joint)
            tip = joint.find('parent').get('link')
        self.joints = []
        for joint in reversed(chain):
            origin = joint.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            t = np.eye(4)
            t[:3, :3] = rotation([0,0,1], rpy[2]) @ rotation([0,1,0], rpy[1]) @ rotation([1,0,0], rpy[0])
            t[:3, 3] = xyz
            movable = joint.get('type') != 'fixed'
            if movable and joint.get('type') != 'revolute':
                raise ValueError('Only bounded revolute arm joints are supported')
            limit, axis = joint.find('limit'), joint.find('axis')
            self.joints.append(Joint(joint.get('name'), t,
                np.fromstring(axis.get('xyz'), sep=' ') if axis is not None else np.array([1.,0,0]),
                float(limit.get('lower')) if movable else 0,
                float(limit.get('upper')) if movable else 0, movable))
        self.names = [j.name for j in self.joints if j.movable]
        self.lower = np.array([j.lower for j in self.joints if j.movable])
        self.upper = np.array([j.upper for j in self.joints if j.movable])
        self.tool_offset = np.asarray(tool_offset, dtype=float)
        if self.tool_offset.shape != (3,) or not np.all(np.isfinite(self.tool_offset)):
            raise ValueError('Invalid tool offset')

    def fk(self, q, with_points=False):
        q = np.asarray(q, dtype=float)
        if q.shape != self.lower.shape or not np.all(np.isfinite(q)):
            raise ValueError('Invalid joint vector')
        t, cursor, points = np.eye(4), 0, []
        for joint in self.joints:
            t = t @ joint.origin
            points.append(t[:3, 3].copy())
            if joint.movable:
                motion = np.eye(4)
                motion[:3, :3] = rotation(joint.axis, q[cursor])
                t = t @ motion
                cursor += 1
        t[:3, 3] += t[:3, :3] @ self.tool_offset
        points.append(t[:3, 3].copy())
        return (t, np.array(points)) if with_points else t

    def inverse_position(self, target, initial, tolerance=.002, iterations=180, cancel=None):
        target, initial = np.asarray(target, dtype=float), np.asarray(initial, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError('Target must be a finite 3D point')
        if initial.shape != self.lower.shape or not np.all(np.isfinite(initial)):
            raise ValueError('Invalid initial joints')
        seeds = [initial, np.clip(initial + np.array([.1, .25, -.2, .3]), self.lower, self.upper),
                 (self.lower+self.upper)/2]
        best, best_error = None, float('inf')
        for seed in seeds:
            q = np.clip(seed, self.lower, self.upper)
            for _ in range(iterations):
                if cancel is not None and cancel():
                    raise ValueError('planning_cancelled')
                position = self.fk(q)[:3, 3]
                error = target-position
                distance = np.linalg.norm(error)
                if distance < best_error:
                    best, best_error = q.copy(), distance
                if distance <= tolerance:
                    return q
                jacobian = np.empty((3, len(q)))
                for j in range(len(q)):
                    step = np.zeros(len(q)); step[j] = 1e-5
                    jacobian[:, j] = (self.fk(q+step)[:3,3]-self.fk(q-step)[:3,3])/(2e-5)
                delta = jacobian.T @ np.linalg.solve(jacobian@jacobian.T + .008**2*np.eye(3), error)
                delta *= min(1, .15/max(1e-12, np.linalg.norm(delta)))
                q = np.clip(q+delta, self.lower, self.upper)
        raise ValueError(f'Unreachable position: best residual {best_error:.4f} m')


def segment_intersects_box(a, b, bounds, padding=0.):
    low, high = np.asarray(bounds[:3])-padding, np.asarray(bounds[3:])+padding
    start, end = 0., 1.
    for axis in range(3):
        delta = b[axis]-a[axis]
        if abs(delta) < 1e-12:
            if a[axis] < low[axis] or a[axis] > high[axis]:
                return False
        else:
            v0, v1 = (low[axis]-a[axis])/delta, (high[axis]-a[axis])/delta
            start, end = max(start, min(v0, v1)), min(end, max(v0, v1))
            if start > end:
                return False
    return True


@dataclass
class Plan:
    names: list
    times: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray


def make_plan(model, current, target, velocity=.35, acceleration=.7, table_top=-.15, obstacles=(), cancel=None):
    goal = model.inverse_position(target, current, cancel=cancel)
    return make_joint_plan(model, current, goal, velocity, acceleration, table_top, obstacles, cancel)


def make_joint_plan(model, current, goal, velocity=.35, acceleration=.7, table_top=-.15, obstacles=(), cancel=None):
    """Use the same limits, timing and geometry guards for an explicit joint-space goal."""
    if cancel is not None and cancel():
        raise ValueError('planning_cancelled')
    if not np.isfinite(velocity+acceleration+table_top) or velocity <= 0 or acceleration <= 0:
        raise ValueError('Speed/acceleration must be finite and positive; table height must be finite')
    current = np.asarray(current, dtype=float)
    goal = np.asarray(goal, dtype=float)
    if (current.shape != model.lower.shape or goal.shape != model.lower.shape
            or not np.all(np.isfinite(current)) or not np.all(np.isfinite(goal))):
        raise ValueError('Invalid current/goal joint vector')
    if np.any(current < model.lower-1e-4) or np.any(current > model.upper+1e-4):
        raise ValueError('Measured joints outside URDF limits')
    if np.any(goal < model.lower) or np.any(goal > model.upper):
        raise ValueError('Goal joints outside URDF limits')
    delta = goal-current
    duration = max(.6, float(np.max(1.875*np.abs(delta)/velocity)),
                   float(np.sqrt(np.max(5.774*np.abs(delta)/acceleration))))
    count = max(3, int(np.ceil(duration*60))+1, int(np.ceil(np.max(np.abs(delta))/.01))+1)
    times = np.linspace(0, duration, count)
    u = times/duration
    blend = 10*u**3-15*u**4+6*u**5
    positions = current+blend[:,None]*delta
    velocities = (30*u**2-60*u**3+30*u**4)[:,None]*delta/duration
    accelerations = (60*u-180*u**2+120*u**3)[:,None]*delta/duration**2
    for q in positions:
        if cancel is not None and cancel():
            raise ValueError('planning_cancelled')
        _, points = model.fk(q, with_points=True)
        if np.any(points[:,2] < table_top+.025):
            raise ValueError('Trajectory intersects table clearance')
        # Conservative trunk approximation; the mounting shoulder segment is excluded.
        torso = [-.08, -.075, -.10, .08, .075, .22]
        for index in range(1, len(points)-1):
            if segment_intersects_box(points[index], points[index+1], torso, .025):
                raise ValueError('Trajectory intersects conservative trunk volume')
        for bounds in obstacles:
            for a, b in zip(points[:-1], points[1:]):
                if segment_intersects_box(a, b, bounds, .025):
                    raise ValueError('Trajectory intersects obstacle clearance')
    return Plan(model.names, times, positions, velocities, accelerations)
