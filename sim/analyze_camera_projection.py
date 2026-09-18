#!/usr/bin/env python3
"""Read-only mount design using weakest-axis shared image resolution.

Uses the same isolated geometry dependencies and exact scene meshes as
analyze_camera_occlusion.py. No learned registration data select candidates.
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np
from analyze_camera_occlusion import Geometry, ROOT, PLANE_Z, MOUNT, ArmModel, in_view


def pixel_support(eye, rotation, points):
    """Squared smaller singular value of the plane-to-image Jacobian, px²/m²."""
    camera = (points-eye)@rotation
    z = np.maximum(camera[:, 2], .015)
    x, y = camera[:, 0]/z, camera[:, 1]/z
    a = 320/z*(rotation[0, 0]-x*rotation[0, 2])
    b = 320/z*(rotation[1, 0]-x*rotation[1, 2])
    c = 320/z*(rotation[0, 1]-y*rotation[0, 2])
    d = 320/z*(rotation[1, 1]-y*rotation[1, 2])
    determinant2 = (a*d-b*c)**2
    trace = a*a+b*b+c*c+d*d
    # Equivalent to (trace-sqrt(trace²-4det²))/2, without cancellation.
    smaller = 2*determinant2/np.maximum(trace+np.sqrt(np.maximum(trace*trace-4*determinant2, 0)), 1e-30)
    return smaller, np.sqrt(determinant2)


def fixed_aim(terminal, offset):
    eye = terminal[:3, 3]+terminal[:3, :3]@offset
    forward = terminal[:3, :3].T@(np.array([.28, -.30, PLANE_Z])-eye)
    forward /= np.linalg.norm(forward)
    right = MOUNT[:, 0]-forward*np.dot(MOUNT[:, 0], forward)
    right /= np.linalg.norm(right)
    return np.column_stack((right, np.cross(forward, right), forward))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--grid', type=int, default=120)
    parser.add_argument('--output', type=Path, default=ROOT/'sim/artifacts/projection_mount_comparison.json')
    args = parser.parse_args()
    started = time.monotonic()
    geometry = Geometry()
    model = ArmModel(str(ROOT/'models/K1/K1_22dof.urdf'))
    rest = np.array([0., .5, 0., 0.])
    poses = [('rest', rest)]
    goals = []
    for i, x in enumerate([.08, .10, .12, .14, .16]):
        goal = model.inverse_position([x, -.25, .02], rest)
        goals.append(goal)
        poses.extend((f'target{i+1}_path{step:02d}', rest+(goal-rest)*a)
                     for step, a in enumerate(np.linspace(0, 1, 21)))
    poses.append(('measured_postreach', np.array([-.11820929497480392, 1.0505858659744263,
                                                 -.06283773481845856, 1.2502663135528564])))
    xx, yy = np.meshgrid(.005+(np.arange(args.grid)+.5)*.55/args.grid,
                         -.55+(np.arange(args.grid)+.5)*.5/args.grid)
    points = np.c_[xx.ravel(), yy.ravel(), np.full(xx.size, PLANE_Z)]
    cell_area = .55*.5/len(points)
    head_eye = np.array([.05, -.65, .45])
    forward = np.array([.28, -.30, -.15])-head_eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0, 0, 1]); right /= np.linalg.norm(right)
    head_rotation = np.column_stack((right, np.cross(forward, right), forward))
    head_fov, _ = in_view(head_eye, head_rotation, points)
    head_support, head_area = pixel_support(head_eye, head_rotation, points)
    candidates = []
    for x in [.06, .10, .14]:
        for z in [.08, .12]:
            offset = np.array([x, -.10, z])
            orientations = [('existing', MOUNT)]
            for fraction in [0., .25, .5, .75, 1.]:
                terminal = geometry.frames(rest+(goals[2]-rest)*fraction)['right_elbow_yaw_link']
                orientations.append((f'aim_path_{fraction:.2f}', fixed_aim(terminal, offset)))
            for label, rotation in orientations:
                candidates.append(dict(name=f'x{x:.2f}_z{z:.2f}_{label}', offset=offset,
                                       rotation=rotation, records=[]))
    for index, (label, joints) in enumerate(poses):
        frames = geometry.frames(joints)
        blocked, _, _ = geometry.occluded(head_eye, points, frames)
        head_visible = head_fov & ~blocked
        terminal = frames['right_elbow_yaw_link']
        for offset_index in range(0, len(candidates), 6):
            group = candidates[offset_index:offset_index+6]
            eye = terminal[:3, 3]+terminal[:3, :3]@group[0]['offset']
            blocked, _, _ = geometry.occluded(eye, points, frames)
            unobstructed = head_visible & ~blocked
            incidence = abs(PLANE_Z-eye[2])/np.linalg.norm(points-eye, axis=1)
            for candidate in group:
                rotation = terminal[:3, :3]@candidate['rotation']
                fov, _ = in_view(eye, rotation, points)
                shared = unobstructed & fov
                support, area = pixel_support(eye, rotation, points)
                candidate['records'].append(dict(pose=label, q=joints.tolist(), eye=eye.tolist(),
                    shared_fraction=float(shared.mean()),
                    shared_weak_axis_pixel_support=float(np.minimum(head_support, support)[shared].sum()*cell_area),
                    shared_wrist_pixels=float(area[shared].sum()*cell_area),
                    shared_head_pixels=float(head_area[shared].sum()*cell_area),
                    mean_wrist_incidence_cosine=float(incidence[shared].mean()) if shared.any() else 0.))
        if index % 20 == 0:
            print(f'projection geometry poses {index+1}/{len(poses)}', flush=True)
    summaries = []
    for candidate in candidates:
        records = candidate['records']
        worst = min(records, key=lambda r:r['shared_weak_axis_pixel_support'])
        summaries.append(dict(name=candidate['name'], offset=candidate['offset'].tolist(),
            rotation_optical_to_terminal=candidate['rotation'].tolist(),
            minimum_weak_axis_pixel_support=worst['shared_weak_axis_pixel_support'], worst_pose=worst['pose'],
            minimum_shared_fraction=min(r['shared_fraction'] for r in records),
            minimum_shared_wrist_pixels=min(r['shared_wrist_pixels'] for r in records),
            rest=records[0], endpoints=[r for r in records if r['pose'].endswith('_path20') or r['pose']=='measured_postreach']))
    summaries.sort(key=lambda s:(-s['minimum_weak_axis_pixel_support'], np.linalg.norm(s['offset'])))
    report = dict(scope='Predetermined fixed optical geometry only; no learned scores or estimator truth in selection',
        objective='Maximize worst-pose sum over mutually visible cells of min(head,wrist) squared smaller plane-projection Jacobian singular value times cell area; ties shorter bracket',
        candidate_grid='X=.06,.10,.14;Y=-.10;Z=.08,.12; existing orientation plus fixed target3 path aim fractions0,.25,.5,.75,1; optical-right clocking retained',
        candidate_count=len(candidates), pose_count=len(poses), grid_cell_count=len(points),
        robot_visual_triangles=geometry.triangle_count, geometry_selected=summaries[0], summaries=summaries,
        records={c['name']:c['records'] for c in candidates}, elapsed_seconds=time.monotonic()-started)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps({'top5':summaries[:5], 'elapsed_seconds':report['elapsed_seconds']}, indent=2))


if __name__ == '__main__':
    main()
