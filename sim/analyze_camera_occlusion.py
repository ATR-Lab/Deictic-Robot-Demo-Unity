#!/usr/bin/env python3
"""Offline fixed-mount geometry comparison. No ROS, renderer, or robot commands.

Dependencies used for the recorded analysis: numpy 2.5.3, trimesh 5.1.0,
embreex 4.4.0. Uses the pinned visual STLs, authored scene geometry, and fixed
camera intrinsics only. Learned matches and registration errors are not inputs.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from trimesh.ray.ray_pyembree import RayMeshIntersector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros2/src/deictic_control'))
from deictic_control.kinematics import ArmModel
from deictic_control.se3 import from_pose, rotation

ARM = ('aaright_shoulder_pitch_joint', 'right_shoulder_roll_joint',
       'right_elbow_pitch_joint', 'right_elbow_yaw_joint')
MOUNT = from_pose([0,0,0,.8673395077793156,-.4396883649539208,.0733619553864093,.221391832444005367])[:3,:3]
PLANE_Z = -.149


def origin(element):
    value = np.eye(4)
    if element is not None:
        value[:3,3] = np.fromstring(element.get('xyz','0 0 0'),sep=' ')
        r,p,y = np.fromstring(element.get('rpy','0 0 0'),sep=' ')
        value[:3,:3] = rotation([0,0,1],y)@rotation([0,1,0],p)@rotation([1,0,0],r)
    return value


def transform_points(vertices, matrix):
    return vertices@matrix[:3,:3].T+matrix[:3,3]


class Geometry:
    def __init__(self):
        self.xml = ET.parse(ROOT/'models/K1/K1_22dof.urdf').getroot()
        self.joints = self.xml.findall('joint')
        self.dynamic = set()
        while True:
            previous = set(self.dynamic)
            for j in self.joints:
                if j.get('name') in ARM or j.find('parent').get('link') in self.dynamic:
                    self.dynamic.add(j.find('child').get('link'))
            if previous == self.dynamic: break
        zero = self.frames(np.zeros(4)); fixed, moving = [], {}
        self.triangle_count = 0
        for link in self.xml.findall('link'):
            meshes = []
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                if mesh is None: continue
                path = ROOT/'models/K1'/mesh.get('filename')
                loaded = trimesh.load_mesh(path,process=False)
                scale = np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
                vertices = transform_points(loaded.vertices*scale,origin(visual.find('origin')))
                meshes.append(trimesh.Trimesh(vertices=vertices,faces=loaded.faces,process=False))
                self.triangle_count += len(loaded.faces)
            if not meshes: continue
            merged = trimesh.util.concatenate(meshes)
            name = link.get('name')
            if name in self.dynamic:
                moving[name] = RayMeshIntersector(merged)
            else:
                merged.apply_transform(zero[name]); fixed.append(merged)
        rng = np.random.default_rng(7)
        for i in range(35):
            yaw = rng.uniform(-np.pi,np.pi)
            size = [rng.uniform(.015,.035),rng.uniform(.02,.065),rng.uniform(.008,.028)]
            rng.uniform(.03,.95)  # Preserve the renderer's color RNG draw.
            t = np.eye(4);t[:3,:3] = rotation([0,0,1],yaw)
            t[:3,3] = [.10+(i%7)*.06,-.44+(i//7)*.11,-.139]
            fixed.append(trimesh.creation.box(extents=size,transform=t))
        for i in range(5):
            sphere = trimesh.creation.icosphere(subdivisions=2,radius=.0125)
            sphere.apply_translation([.08+.02*i,-.25,.02]);fixed.append(sphere)
        self.fixed = RayMeshIntersector(trimesh.util.concatenate(fixed))
        self.moving = moving

    def frames(self,q):
        values = dict(zip(ARM,q));frames={'trunk':np.eye(4)};pending=list(self.joints)
        while pending:
            remainder=[]
            for j in pending:
                parent,child=j.find('parent').get('link'),j.find('child').get('link')
                if parent not in frames: remainder.append(j);continue
                t=origin(j.find('origin'));motion=np.eye(4)
                if j.get('name') in values:
                    motion[:3,:3]=rotation(np.fromstring(j.find('axis').get('xyz'),sep=' '),values[j.get('name')])
                frames[child]=frames[parent]@t@motion
            if len(remainder)==len(pending):raise ValueError('Unresolved URDF hierarchy')
            pending=remainder
        return frames

    def occluded(self,eye,points,frames):
        distance=np.linalg.norm(points-eye,axis=1);nearest=np.full(len(points),np.inf);blocker=np.full(len(points),-1,dtype=int)
        names=['fixed robot/scene']+list(self.moving)
        for i,(name,intersector) in enumerate([('fixed robot/scene',self.fixed)]+list(self.moving.items())):
            if i:
                t=frames[name];local_eye=(eye-t[:3,3])@t[:3,:3];local_points=(points-t[:3,3])@t[:3,:3]
            else:local_eye,local_points=eye,points
            origins=np.broadcast_to(local_eye,local_points.shape)
            locations,ray_ids,_=intersector.intersects_location(origins,local_points-local_eye,multiple_hits=False)
            hit_dist=np.linalg.norm(locations-local_eye,axis=1)
            valid=(hit_dist>1e-5)&(hit_dist<distance[ray_ids]-1e-4)&(hit_dist<nearest[ray_ids])
            ids=ray_ids[valid];nearest[ids]=hit_dist[valid];blocker[ids]=i
        return blocker>=0,blocker,names


def in_view(eye,R,points):
    v=(points-eye)@R
    uv=v[:,:2]/v[:,2,None]*320+[320,240]
    valid=(v[:,2]>.015)&(uv[:,0]>=0)&(uv[:,0]<640)&(uv[:,1]>=0)&(uv[:,1]<480)
    return valid,v[:,2]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'sim/artifacts/mesh_mount_comparison.json')
    parser.add_argument('--grid',type=int,default=60)
    parser.add_argument('--measured-end',type=float,nargs=4,
        default=[-.11820929497480392,1.0505858659744263,-.06283773481845856,1.2502663135528564],
        help='Previously measured simulated end joints, included as a fixed additional sample; never commands motion')
    args=parser.parse_args();start=time.monotonic();scene=Geometry()
    # Equal-area cell centers on the printed tabletop, excluding its thin boundary.
    xx,yy=np.meshgrid(.005+(np.arange(args.grid)+.5)*.55/args.grid,-.55+(np.arange(args.grid)+.5)*.5/args.grid)
    points=np.c_[xx.ravel(),yy.ravel(),np.full(xx.size,PLANE_Z)]
    cell_area=.55*.5/len(points)
    head_eye=np.array([.05,-.65,.45]);head_f=np.array([.28,-.30,-.15])-head_eye;head_f/=np.linalg.norm(head_f)
    head_r=np.cross(head_f,[0,0,1]);head_r/=np.linalg.norm(head_r);head_R=np.column_stack((head_r,np.cross(head_f,head_r),head_f))
    head_fov,_=in_view(head_eye,head_R,points)
    rest=np.array([0,.5,0,0]);model=ArmModel(str(ROOT/'models/K1/K1_22dof.urdf'))
    poses=[('rest',rest)]
    for i,x in enumerate([.08,.10,.12,.14,.16]):
        goal=model.inverse_position([x,-.25,.02],rest)
        poses.extend((f'target{i+1}_path{step:02d}',rest+(goal-rest)*a) for step,a in enumerate(np.linspace(0,1,21)))
    # Pin the observed end configuration rather than depending on ignored capture
    # files, so a clean checkout reproduces the same 107 geometry samples.
    poses.append(('measured_postreach',np.array(args.measured_end)))
    candidates={str(x):[] for x in [.06,.10,.14]}
    for index,(label,q) in enumerate(poses):
        frames=scene.frames(q);head_blocked,_,_=scene.occluded(head_eye,points,frames);head_visible=head_fov&~head_blocked
        t=frames['right_elbow_yaw_link']
        for offset_x,records in candidates.items():
            eye=t[:3,3]+t[:3,:3]@np.array([float(offset_x),-.10,.08]);R=t[:3,:3]@MOUNT
            fov,depth=in_view(eye,R,points);blocked,blocker,names=scene.occluded(eye,points,frames)
            geometric=head_visible&fov;shared=geometric&~blocked
            projected_area=320**2*abs(PLANE_Z-eye[2])/np.maximum(depth,.015)**3*cell_area
            records.append(dict(pose=label,q=q.tolist(),eye=eye.tolist(),head_visible_fraction=float(head_visible.mean()),
                shared_fraction=float(shared.mean()),geometric_fraction=float(geometric.mean()),
                shared_projected_wrist_pixels=float(projected_area[shared].sum()),
                occluders={name:int(np.sum(geometric&(blocker==i))) for i,name in enumerate(names)}))
        if index%20==0:print(f'geometry poses {index+1}/{len(poses)}',flush=True)
    summary=[]
    for x,records in candidates.items():
        worst=min(records,key=lambda r:r['shared_fraction'])
        summary.append(dict(offset=[float(x),-.10,.08],minimum_shared_fraction=worst['shared_fraction'],worst_pose=worst['pose'],
            minimum_shared_projected_wrist_pixels=min(r['shared_projected_wrist_pixels'] for r in records),
            rest=records[0],endpoints=[r for r in records if r['pose'].endswith('_path20') or r['pose']=='measured_postreach']))
    selected=max(summary,key=lambda s:(s['minimum_shared_fraction'],-s['offset'][0]))
    report=dict(scope='Offline mesh visibility only; no learned matching or truth-fed estimator; bracket is a virtual camera extrinsic, not a certified physical mechanism',
        objective='Maximize worst jointly visible printed-plane area across fixed pose set; tie-break shorter X bracket; projected image footprint reported separately',
        dependencies={'numpy':np.__version__,'trimesh':trimesh.__version__,'embreex':'4.4.0'},robot_visual_triangles=scene.triangle_count,
        grid_cell_count=len(points),pose_count=len(poses),mount_quaternion_xyzw=[.8673395077793156,-.4396883649539208,.0733619553864093,.221391832444005367],
        candidate_summary=summary,geometry_selected_offset=selected['offset'],records=candidates,elapsed_seconds=time.monotonic()-start)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2))
    print(json.dumps({'summary':summary,'selected_offset':selected['offset'],'seconds':report['elapsed_seconds']},indent=2))


if __name__=='__main__':main()
