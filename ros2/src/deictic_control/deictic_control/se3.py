"""SE(3) helpers. Pose arrays use [x,y,z,qx,qy,qz,qw], SI units."""
import numpy as np


def skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12 or not np.isfinite(norm + angle):
        raise ValueError('Invalid rotation axis/angle')
    k = skew(axis / norm)
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


def from_pose(pose):
    pose = np.asarray(pose, dtype=float)
    if pose.shape != (7,) or not np.all(np.isfinite(pose)):
        raise ValueError('Pose must have seven finite values')
    q = pose[3:]
    if abs(np.linalg.norm(q) - 1) > 0.01:
        raise ValueError('Quaternion must be unit length')
    q = q / np.linalg.norm(q)
    v, w = q[:3], q[3]
    result = np.eye(4)
    result[:3, :3] = (w*w - v@v) * np.eye(3) + 2*np.outer(v, v) + 2*w*skew(v)
    result[:3, 3] = pose[:3]
    return result


def to_pose(transform):
    t = np.asarray(transform, dtype=float)
    r = t[:3, :3]
    # Eigenvector formulation avoids the trace singularity at 180 degrees.
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
                  [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
                  [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]]) / 3
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    if q[3] < 0:
        q = -q
    return np.concatenate((t[:3, 3], q)).tolist()


def inverse(t):
    result = np.eye(4)
    result[:3, :3] = t[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ t[:3, 3]
    return result


def transform_point(t, point):
    return t[:3, :3] @ np.asarray(point) + t[:3, 3]
