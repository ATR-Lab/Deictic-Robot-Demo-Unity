#!/usr/bin/env python3
"""Independent K1 pose-servo checks. No ROS, network, simulator or motor output.

Six FK-generated cases use seed 38126 and were declared independently of the
solver's own regression fixtures. Three additional cases exercise the original
inward singularity, stalled measured feedback/recovery, and an underactuated
orientation request. This checks emitted setpoints; it does not model physics.
"""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import sys
import time

import numpy as np
import scipy
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2/src/deictic_control'))
from deictic_control.kinematics import ArmModel
from deictic_control.teleop import BimanualClutch, chain_reach, check_bimanual_geometry

DT = .05
REST = np.array([[0., 0., 0., 0.], [0., .5, 0., 0.]])


def declared_cases(models):
    rng, cases = np.random.default_rng(38126), []
    for _ in range(1000):
        start = REST.copy() if not cases else REST+rng.uniform(-.7, .7, (2, 4))
        goal = start+rng.uniform(-.35, .35, (2, 4))
        try:
            for fraction in np.linspace(0., 1., 30):
                check_bimanual_geometry(models, start+fraction*(goal-start), -.15, ())
        except ValueError:
            continue
        if max(np.linalg.norm(m.fk(b)[:3, 3]-m.fk(a)[:3, 3])
               for m, a, b in zip(models, start, goal)) > .25:
            continue
        cases.append((f'fk_pose_{len(cases)+1}', start, goal, 120))
        if len(cases) == 6:
            break
    if len(cases) != 6:
        raise RuntimeError('Could not construct the predeclared six admissible fixtures')
    cases.extend((name, REST.copy(), None, 200) for name in
                 ('inward_5cm', 'stalled_feedback_then_recover', 'twist_180deg'))
    return cases


def run_case(models, name, start, goal, steps, scale):
    servo = BimanualClutch(models, translation_scale=scale)
    measured, previous, previous_velocity = start.copy(), start.copy(), np.zeros((2, 4))
    starts = np.array([m.fk(q) for m, q in zip(models, start)])
    targets = np.array([m.fk(q) for m, q in zip(models, goal)]) if goal is not None else starts.copy()
    if name == 'inward_5cm':
        targets[:, 1, 3] += [-.05, .05]
    elif name == 'stalled_feedback_then_recover':
        targets[:, 0, 3] += .08
    elif name == 'twist_180deg':
        targets[:, :3, :3] = Rotation.from_rotvec([np.pi, 0., 0.]).as_matrix() @ starts[:, :3, :3]
    delta = targets[:, :3, 3]-starts[:, :3, 3]
    rotation_delta = Rotation.from_matrix(targets[:, :3, :3] @ starts[:, :3, :3].transpose(0, 2, 1)).as_rotvec()

    def packet(sequence, stamp, clutch, fraction):
        poses = starts.copy()
        poses[:, :3, 3] += delta*fraction
        poses[:, :3, :3] = Rotation.from_rotvec(rotation_delta*fraction).as_matrix() @ starts[:, :3, :3]
        positions, matrices = servo.mapping.controller_poses(poses)
        rotations = Rotation.from_matrix(matrices).as_quat()
        return json.dumps(dict(schema_version=3, frame_id='teleop_body', session_id='offline_independent',
                               sequence=sequence, stamp=stamp, clutch=clutch,
                               left_tracked=True, right_tracked=True,
                               left_position=positions[0].tolist(), right_position=positions[1].tolist(),
                               left_rotation=rotations[0].tolist(), right_rotation=rotations[1].tolist()))

    if servo.ingest(packet(0, 100., False, 0.), 100., 10., measured) != 'released':
        raise RuntimeError('Fixture failed release handshake')
    if servo.ingest(packet(1, 100.01, True, 0.), 100.01, 10.01, measured) != 'began':
        raise RuntimeError('Fixture failed clutch entry: '+servo.reason)
    peaks = dict(joint_speed_rad_s=0., joint_acceleration_rad_s2=0., command_lead_rad=0., tool_speed_m_s=0.)
    reasons, timings, fault, geometry_valid = {}, [], None, True
    for step in range(1, steps+1):
        elapsed = step*DT
        wall, monotonic = 100.01+elapsed, 10.01+elapsed
        servo.ingest(packet(step+1, wall, True, min(1., elapsed)), wall, monotonic, measured)
        command = servo.tick(wall, monotonic, measured)
        reasons[servo.reason] = reasons.get(servo.reason, 0)+1
        if command is None:
            fault = servo.reason
            break
        velocity = (command-previous)/DT
        peaks['joint_speed_rad_s'] = max(peaks['joint_speed_rad_s'], float(np.max(np.abs(velocity))))
        peaks['joint_acceleration_rad_s2'] = max(peaks['joint_acceleration_rad_s2'],
                                               float(np.max(np.abs(velocity-previous_velocity))/DT))
        peaks['command_lead_rad'] = max(peaks['command_lead_rad'], float(np.max(np.abs(command-measured))))
        peaks['tool_speed_m_s'] = max(peaks['tool_speed_m_s'], max(float(
            np.linalg.norm(m.fk(after)[:3, 3]-m.fk(before)[:3, 3])/DT)
            for m, before, after in zip(models, previous, command)))
        try:
            check_bimanual_geometry(models, command, -.15, ())
        except ValueError:
            geometry_valid = False
        timings.append(servo.solve_duration)
        previous, previous_velocity = command.copy(), velocity
        if name != 'stalled_feedback_then_recover' or step >= 100:
            measured = command.copy()
    final = np.array([m.fk(q) for m, q in zip(models, measured)])
    position_error = np.linalg.norm(final[:, :3, 3]-targets[:, :3, 3], axis=1)
    orientation_error = Rotation.from_matrix(final[:, :3, :3] @ targets[:, :3, :3].transpose(0, 2, 1)).magnitude()
    checks = dict(no_fault=fault is None, valid_geometry=geometry_valid,
                  speed=peaks['joint_speed_rad_s'] <= .35+1e-8,
                  acceleration=peaks['joint_acceleration_rad_s2'] <= .7+1e-8,
                  tracking_lead=peaks['command_lead_rad'] <= .075+1e-8,
                  tool_speed=peaks['tool_speed_m_s'] <= .10+1e-8)
    if goal is not None:
        checks.update(position_accuracy=bool(np.max(position_error) < .002),
                      orientation_accuracy=bool(np.max(orientation_error) < .01))
    elif name == 'inward_5cm':
        checks['singularity_escape'] = bool(np.max(position_error) < .002)
    elif name == 'stalled_feedback_then_recover':
        checks['resumed_progress'] = bool(np.max(np.abs(measured-start)) > .02 and servo.active)
    elif name == 'twist_180deg':
        checks['underactuation_reported'] = bool(servo.state == 'limited' and max(orientation_error) > .15)
        checks['position_priority'] = bool(max(position_error) < .005)
    return dict(name=name, initial_joints=start.tolist(), target_joints=goal.tolist() if goal is not None else None,
                target_poses=targets.tolist(), completed_steps=step, checks=checks, passed=all(checks.values()),
                peak=peaks, position_error_m=position_error.tolist(), orientation_error_rad=orientation_error.tolist(),
                reasons=reasons, fault=fault, final_joints=measured.tolist(),
                solve_seconds=dict(mean=float(np.mean(timings)) if timings else None,
                                   maximum=max(timings, default=None)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    urdf = ROOT / 'models/K1/K1_22dof.urdf'
    models = (ArmModel(urdf, tip='left_elbow_yaw_link', tool_offset=[0., .1, 0.]), ArmModel(urdf))
    scale = min(chain_reach(model) for model in models)/.60
    source_paths = ['sim/benchmark_teleop_offline.py', 'ros2/src/deictic_control/deictic_control/teleop.py',
                    'ros2/src/deictic_control/deictic_control/kinematics.py', 'models/K1/K1_22dof.urdf']
    hashes = {path: hashlib.sha256((ROOT/path).read_bytes()).hexdigest() for path in source_paths}
    started = time.time()
    results = [run_case(models, *case, scale) for case in declared_cases(models)]
    if any(hashlib.sha256((ROOT/path).read_bytes()).hexdigest() != digest for path, digest in hashes.items()):
        raise RuntimeError('Source changed during benchmark; repeat against one fixed source version')
    report = dict(scope='Offline setpoint/URDF validation; no ROS, native input, network or physics simulation',
                  source_sha256=hashes, python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__,
                  started_unix=started, ended_unix=time.time(), dt=DT, translation_scale=scale, seed=38126,
                  exceptions='Abrupt external feedback reversals and geometry safety holds may override nominal acceleration; '
                             'these nine declared cases do not require those emergency exceptions.',
                  cases=results, passed=all(result['passed'] for result in results))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(dict(output=str(args.output), passed=report['passed'],
                         cases=[dict(name=r['name'], passed=r['passed'], checks=r['checks']) for r in results])))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
