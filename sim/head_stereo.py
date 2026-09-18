"""Demand-rendered robot-forward stereo display; no physical calibration claim.

The vendor URDF supplies one optical mount, not calibrated stereo extrinsics.
The simulated eyes straddle that origin by an adjustable 64 mm baseline and sit
20 mm farther forward. Both cameras retain the vendor optical orientation. The
default forward offset clears the complete pinned Head_2 mesh by 16.69 mm.
Only this pair's render products are enabled on a display request (maximum 5 Hz).
Wrist/headset registration cameras and their render scheduling are not modified.
"""
import json
import math
from fractions import Fraction
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from camera_sync import snapshot, synchronized
from k1_model import URDF

MOUNT_LINK = "head_booster_stereo_rgb_link"
FORWARD_OFFSET_M = 0.020
MAX_RATE_HZ = 5.0
MAX_CAPTURE_AGE_S = 1.0
CAPTURE_TIMEOUT_S = 2.0


def reference_key(reference):
    try:
        numerator,denominator = int(reference['referenceTimeNumerator']),int(reference['referenceTimeDenominator'])
        return Fraction(numerator,denominator) if numerator > 0 and denominator > 0 else None
    except (KeyError,TypeError,ValueError,OverflowError):
        return None


def frame_diagnostic(camera, valid=None):
    """Metadata only: never serialize or copy full pixel buffers."""
    try:
        raw = camera.get_current_frame(clone=False)
        params = raw.get('CameraParams')
        params = params if isinstance(params,dict) else {}
        return dict(keys=sorted(raw),rgb_shape=list(np.shape(raw.get('rgb'))),
                    rendering_time=raw.get('rendering_time'),reference=raw.get('rendering_frame'),
                    camera_params_resolution=params.get('renderProductResolution'),
                    view_shape=list(np.shape(params.get('cameraViewTransform'))),
                    meters_per_scene_unit=params.get('metersPerSceneUnit'),
                    snapshot_valid=valid,paused=camera.is_paused(),product_path=camera.get_render_product_path())
    except Exception as error:
        return dict(diagnostic_error=f'{type(error).__name__}: {error}')


def rigid_link_prim(stage, link_name, rigid_body_api):
    """URDF visuals/collisions can share the link name; attach to its rigid body."""
    parents = [prim for prim in stage.Traverse()
               if prim.GetName() == link_name and prim.HasAPI(rigid_body_api)]
    if len(parents) != 1:
        paths = [str(prim.GetPath()) for prim in parents]
        raise RuntimeError(f"Expected one rigid head mount parent {link_name}; found {paths}")
    return parents[0]


def stereo_mount(baseline_m=0.064, width=640, urdf=URDF):
    """Return simulated eye poses in the vendor mount's parent-link frame."""
    if not math.isfinite(baseline_m) or not 0 < baseline_m <= 0.20:
        raise ValueError("Simulation head-stereo baseline must be in (0, 0.20] metres")
    if width not in (320, 640):
        raise ValueError("Head-stereo width must be 320 or 640 pixels")
    joints = [joint for joint in ET.parse(urdf).getroot().findall("joint")
              if joint.find("child").get("link") == MOUNT_LINK]
    if len(joints) != 1 or joints[0].get("type") != "fixed":
        raise ValueError("Expected one fixed vendor head-stereo optical mount")
    joint = joints[0]
    origin = joint.find("origin")
    xyz = np.fromstring(origin.get("xyz"), sep=" ")
    r, p, y = np.fromstring(origin.get("rpy"), sep=" ")
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rotation = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                         [-sp, cp*sr, cp*cr]])
    return {
        "simulation_only": True,
        "calibration_source": "vendor single optical origin plus assumed virtual stereo baseline/intrinsics",
        "mount_link": MOUNT_LINK,
        "parent_link": joint.find("parent").get("link"),
        "baseline_m": baseline_m,
        "forward_standoff_m": FORWARD_OFFSET_M,
        "resolution": [width, width*3//4],
        "max_rate_hz": MAX_RATE_HZ,
        "rotation_parent_from_optical": rotation.tolist(),
        "eyes": {side: {
            "position_parent": (xyz+rotation@np.array([offset, 0., FORWARD_OFFSET_M])).tolist(),
            "frame_id": f"k1_head_{side}_camera_optical",
            "image_topic": f"/k1/head_camera/{side}/image_raw",
            "info_topic": f"/k1/head_camera/{side}/camera_info",
        } for side, offset in (("left", -baseline_m/2), ("right", baseline_m/2))},
    }


class StereoSchedule:
    """Bound render attempts and publications; reject unmatched or reused frames."""
    def __init__(self):
        self.next_render = 0.0
        self.next_publish = 0.0
        self.last_reference = None
        self.reference_floor = None
        self.pending = False
        self.deadline = 0.0
        self.requests = 0
        self.timeouts = 0

    def request(self, now, demand, reference_floor=None):
        if not demand:
            self.pending = False
            return False
        if self.pending:
            if now < self.deadline:
                return True
            self.timeouts += 1
            self.pending = False
            self.next_render = now+1/MAX_RATE_HZ
            return False
        if now < max(self.next_render, self.next_publish):
            return False
        self.next_render = now+1/MAX_RATE_HZ
        self.pending = True
        self.deadline = now+CAPTURE_TIMEOUT_S
        self.reference_floor = reference_floor
        self.requests += 1
        return True

    def accept(self, left, right, now):
        if not synchronized(left, right) or now < self.next_publish:
            return False
        if (left.rgb.shape != right.rgb.shape or len(left.rgb.shape) != 3
                or left.rgb.shape[2] != 3 or left.rgb.shape[0] > 480 or left.rgb.shape[1] > 640):
            return False
        reference = Fraction(*left.reference)
        if self.reference_floor is not None and reference <= self.reference_floor:
            return False
        if self.last_reference is not None and reference <= self.last_reference:
            return False
        self.last_reference = reference
        self.next_publish = now+1/MAX_RATE_HZ
        self.pending = False
        return True


class HeadStereoDisplay:
    """Own only the head display cameras/products; imports Isaac after app startup."""
    def __init__(self, stage, output, node, configure_camera, intrinsics,
                 baseline_m=0.064, width=640):
        import omni.replicator.core as rep
        from pxr import UsdGeom, UsdPhysics
        from isaacsim.sensors.camera import Camera
        from isaacsim.core.utils.rotations import rot_matrix_to_quat

        self.spec = stereo_mount(baseline_m, width)
        self.output, self.node, self.intrinsics = Path(output), node, intrinsics
        self.schedule = StereoSchedule()
        self.cameras, self.products, self.publishers = {}, {}, {}
        self.requested = False
        self.saved = False
        self.published_pairs = 0
        self.reference_times = {}
        self.render_opportunities = 0
        self.requested_renders = 0
        self.next_diagnostic = 0.0
        self.subscriber_counts = {}
        self.last_decision = 'initializing'
        parent = rigid_link_prim(stage, self.spec["parent_link"], UsdPhysics.RigidBodyAPI)
        orientation = rot_matrix_to_quat(np.array(self.spec["rotation_parent_from_optical"]))
        for side, eye in self.spec["eyes"].items():
            # Derive the pose from the URDF fixed joint even if the importer omits
            # the empty vendor camera link. The parent remains the actual head.
            path = str(parent.GetPath())+f"/SimHeadStereo_{side}"
            UsdGeom.Camera.Define(stage, path)
            product = rep.create.render_product(path, tuple(self.spec["resolution"]),
                                                name=f"DeicticHeadStereo_{side}")
            product.hydra_texture.set_updates_enabled(False)
            camera = Camera(path, name=f"head_stereo_{side}", frequency=-1,
                            resolution=tuple(self.spec["resolution"]), render_product_path=product.path)
            camera.set_local_pose(translation=np.array(eye["position_parent"]),
                                  orientation=orientation, camera_axes="ros")
            camera.initialize()
            configure_camera(camera)
            camera.attach_annotator("CameraParams")
            product.hydra_texture.set_updates_enabled(False)
            self.cameras[side], self.products[side] = camera, product
            if node:
                from sensor_msgs.msg import Image, CameraInfo
                self.publishers[side] = (
                    node.create_publisher(Image, eye["image_topic"], 2),
                    node.create_publisher(CameraInfo, eye["info_topic"], 2),
                )
        self.output.mkdir(parents=True, exist_ok=True)
        (self.output/"head_stereo_configuration.json").write_text(json.dumps(self.spec, indent=2))
        print("DEICTIC_HEAD_STEREO_READY "+json.dumps(self.spec), flush=True)

    def prepare_render(self, now):
        # No ROS is the explicit offline smoke path: capture one proof pair only.
        self.subscriber_counts = {side:[pub.get_subscription_count() for pub in pair]
                                  for side,pair in self.publishers.items()}
        demand = (not self.saved if self.node is None else
                  any(count for counts in self.subscriber_counts.values() for count in counts))
        references = list(self.reference_times)
        if not self.schedule.pending:
            for camera in self.cameras.values():
                key = reference_key(camera.get_current_frame(clone=False).get('rendering_frame'))
                if key is not None:
                    references.append(key)
        self.requested = self.schedule.request(now,demand,max(references) if references else None)
        self.render_opportunities += 1
        self.requested_renders += int(self.requested)
        for product in self.products.values():
            product.hydra_texture.set_updates_enabled(self.requested)

    def observe_render_reference(self, frame, stamp_ns, now):
        """Keep first-observed UTC times for at most 64 continuous-camera references."""
        reference = reference_key(frame.get('rendering_frame'))
        simulation_time = float(frame.get('rendering_time',0.))
        if reference is None or not math.isfinite(simulation_time) or simulation_time <= 0 or stamp_ns <= 0:
            return
        if reference not in self.reference_times:
            self.reference_times[reference] = (int(stamp_ns),now,simulation_time)
            for old in list(self.reference_times)[:-64]:
                del self.reference_times[old]

    def diagnose(self, now, frames=None):
        if now < self.next_diagnostic:
            return
        self.next_diagnostic = now+5.
        report = dict(subscribers=self.subscriber_counts,render_opportunities=self.render_opportunities,
                      requested_renders=self.requested_renders,requests=self.schedule.requests,
                      timeouts=self.schedule.timeouts,pending=self.schedule.pending,
                      reference_floor=str(self.schedule.reference_floor),
                      reference_history=len(self.reference_times),published_pairs=self.published_pairs,
                      decision=self.last_decision,
                      cameras={side:frame_diagnostic(camera,frames[side] is not None if frames else None)
                               for side,camera in self.cameras.items()})
        print('DEICTIC_HEAD_STEREO_DIAGNOSTIC '+json.dumps(report,default=lambda value:
              value.tolist() if isinstance(value,np.ndarray) else str(value)),flush=True)

    def finish_render(self, now):
        if not self.requested:
            self.last_decision = 'not_requested_or_backoff'
            self.diagnose(now)
            return
        frames = {side: snapshot(camera) for side, camera in self.cameras.items()}
        self.last_decision = 'waiting_for_coherent_pair'
        if not synchronized(frames['left'],frames['right']):
            self.diagnose(now,frames)
            return
        reference = Fraction(*frames['left'].reference)
        capture = self.reference_times.get(reference)
        if capture is None or abs(capture[2]-frames['left'].rendering_time) > 1e-8:
            self.last_decision = 'waiting_for_known_render_reference'
            self.diagnose(now,frames)
            return
        capture_stamp_ns,capture_monotonic,_ = capture
        wall_age = (self.node.get_clock().now().nanoseconds-capture_stamp_ns)*1e-9 if self.node else now-capture_monotonic
        if now-capture_monotonic >= MAX_CAPTURE_AGE_S or not -.05 <= wall_age < MAX_CAPTURE_AGE_S:
            self.last_decision = 'capture_too_old'
            self.diagnose(now,frames)
            return
        if not self.schedule.accept(frames["left"], frames["right"], now):
            self.last_decision = 'reference_watermark_duplicate_or_rate_gate'
            self.diagnose(now,frames)
            return
        self.requested = False
        for product in self.products.values():
            product.hydra_texture.set_updates_enabled(False)
        self.last_decision = 'accepted'
        if self.node:
            from sensor_msgs.msg import Image, CameraInfo
            stamp = self.node.get_clock().now().to_msg()
            stamp.sec,stamp.nanosec = divmod(capture_stamp_ns,1_000_000_000)
            for side, frame in frames.items():
                image = Image()
                image.header.stamp, image.header.frame_id = stamp, self.spec["eyes"][side]["frame_id"]
                image.height, image.width = frame.rgb.shape[:2]
                image.encoding, image.step = "rgb8", image.width*3
                image.data = frame.rgb.tobytes()
                info = CameraInfo()
                info.header, info.height, info.width = image.header, image.height, image.width
                intrinsic = self.intrinsics(self.cameras[side])
                info.k, info.r = intrinsic.reshape(-1).tolist(), np.eye(3).reshape(-1).tolist()
                projection = np.zeros((3, 4)); projection[:, :3] = intrinsic
                if side == "right":
                    projection[0, 3] = -intrinsic[0, 0]*self.spec["baseline_m"]
                info.p = projection.reshape(-1).tolist()
                info.distortion_model, info.d = "plumb_bob", [0.]*5
                self.publishers[side][0].publish(image)
                self.publishers[side][1].publish(info)
            self.published_pairs += 1
        self.diagnose(now,frames)
        if not self.saved:
            from PIL import Image as PILImage
            metadata = dict(self.spec, render_reference=list(frames["left"].reference),
                            rendering_time_seconds=frames["left"].rendering_time,
                            capture_stamp_ns=capture_stamp_ns,
                            stamp_source="first continuous-camera observation of the same renderer reference",
                            pose_source="CameraParams in same cached frame as each RGB image")
            metadata["cameras"] = {}
            for side, frame in frames.items():
                PILImage.fromarray(frame.rgb).save(self.output/f"head_{side}_camera.png")
                metadata["cameras"][side] = {
                    "K": self.intrinsics(self.cameras[side]).tolist(),
                    "T_base_optical": frame.base_from_optical.tolist(),
                }
            if self.node:
                metadata["capture_stamp_ns"] = stamp.sec*1000000000+stamp.nanosec
            (self.output/"head_stereo_capture.json").write_text(json.dumps(metadata, indent=2))
            self.saved = True
            print("DEICTIC_HEAD_STEREO_PAIR_SAVED", flush=True)
