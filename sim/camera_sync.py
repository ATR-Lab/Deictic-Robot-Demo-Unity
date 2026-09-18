"""Strict snapshots of Isaac's cached camera frames; no current-stage pose fallback."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class CameraSnapshot:
    rgb: np.ndarray
    depth: np.ndarray | None
    base_from_optical: np.ndarray
    rendering_time: float
    reference: tuple[int,int]


def snapshot(camera, require_depth=False):
    frame = camera.get_current_frame(clone=True)
    rgb,params,reference = frame.get("rgb"),frame.get("CameraParams"),frame.get("rendering_frame")
    if rgb is None or not isinstance(params,dict) or not isinstance(reference,dict):
        return None
    timestamp = float(frame.get("rendering_time",0.))
    if not np.isfinite(timestamp) or timestamp <= 0:
        return None
    try:
        key = (int(reference["referenceTimeNumerator"]),int(reference["referenceTimeDenominator"]))
        width,height = map(int,params["renderProductResolution"])
        view = np.asarray(params["cameraViewTransform"],dtype=float).reshape(4,4)
        units = float(params["metersPerSceneUnit"])
    except (KeyError,ValueError,TypeError):
        return None
    if key[1] <= 0 or (width,height) != tuple(camera.get_resolution()) or abs(units-1.) > 1e-8:
        return None
    if np.shape(rgb) != (height,width,4) or not np.isfinite(view).all():
        return None
    try:
        # Replicator's view is a row-vector world->USD-camera matrix. Optical
        # coordinates flip USD's +Y-up/-Z-forward axes into +Y-down/+Z-forward.
        pose = np.linalg.inv(view.T) @ np.diag([1.,-1.,-1.,1.])
    except np.linalg.LinAlgError:
        return None
    if not np.allclose(pose[3],[0,0,0,1],atol=1e-6) or not np.allclose(pose[:3,:3].T@pose[:3,:3],np.eye(3),atol=1e-5):
        return None
    depth = frame.get("distance_to_image_plane")
    if require_depth and (depth is None or np.shape(depth) != (height,width)):
        return None
    return CameraSnapshot(np.ascontiguousarray(rgb[:,:,:3],dtype=np.uint8),depth,pose,timestamp,key)


def synchronized(wrist,headset):
    return (wrist is not None and headset is not None and wrist.reference == headset.reference
            and abs(wrist.rendering_time-headset.rendering_time) <= 1e-8)
