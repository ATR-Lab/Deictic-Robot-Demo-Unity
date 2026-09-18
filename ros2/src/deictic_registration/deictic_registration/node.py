"""ROS 2 camera frontend. Failed, missing, or stale inputs fail closed."""

from concurrent.futures import ThreadPoolExecutor
import json
import time

import cv2
import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from .core import CameraModel, RegistrationConfig, RegistrationError, estimate_registration, matrix_to_pose
from .features import SuperPointMatcher
from .camera_qos import camera_subscription_qos


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def _raw_array(message, dtype, channels):
    dtype = np.dtype(dtype).newbyteorder(">" if message.is_bigendian else "<")
    row_bytes = message.width * channels * dtype.itemsize
    if message.height <= 0 or message.width <= 0 or message.step < row_bytes:
        raise RegistrationError("Invalid image dimensions/stride")
    raw = np.asarray(message.data, dtype=np.uint8)
    if raw.size != message.step * message.height:
        raise RegistrationError("Image payload does not match dimensions/stride")
    rows = raw.reshape(message.height, message.step)[:, :row_bytes].copy()
    return rows.view(dtype).reshape(message.height, message.width, channels).astype(
        dtype.newbyteorder("="), copy=False)


def rgb_image(message):
    if isinstance(message, CompressedImage):
        image = cv2.imdecode(np.asarray(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise RegistrationError("Invalid compressed camera image")
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    if message.encoding in ("rgb8", "bgr8"):
        image = _raw_array(message, "uint8", 3)
        return image if message.encoding == "rgb8" else image[:, :, ::-1].copy()
    if message.encoding == "mono8":
        return np.repeat(_raw_array(message, "uint8", 1), 3, axis=2)
    raise RegistrationError("Unsupported camera encoding: " + message.encoding)


def depth_image(message):
    if message.encoding != "32FC1":
        raise RegistrationError("Depth must be aligned optical-z meters in 32FC1")
    return _raw_array(message, "float32", 1)[:, :, 0]


def camera_model(message):
    if message.distortion_model not in ("", "plumb_bob", "rational_polynomial"):
        raise RegistrationError("Unsupported camera distortion model")
    if message.binning_x > 1 or message.binning_y > 1 or message.roi.width or message.roi.height:
        raise RegistrationError("Publish effective image intrinsics without CameraInfo ROI/binning")
    return CameraModel(message.width, message.height,
                       np.asarray(message.k).reshape(3, 3), np.asarray(message.d))


def pose_array(message):
    p, q = message.pose.position, message.pose.orientation
    pose = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    if not np.all(np.isfinite(pose)) or not np.isclose(np.linalg.norm(pose[3:]), 1, atol=1e-3):
        raise RegistrationError("Invalid/non-unit capture pose")
    return pose.tolist()


class RegistrationNode(Node):
    def __init__(self):
        super().__init__("deictic_registration")
        defaults = {
            "headset_image_topic": "/deictic/headset/image/compressed",
            "headset_info_topic": "/deictic/headset/camera_info",
            "headset_depth_topic": "/deictic/headset/depth",
            "headset_pose_topic": "/deictic/headset/camera_pose",
            "wrist_image_topic": "/k1/wrist_camera/image_raw",
            "wrist_info_topic": "/k1/wrist_camera/camera_info",
            "wrist_pose_topic": "/k1/wrist_camera/pose",
            "headset_compressed": True,
            "wrist_compressed": False,
            "camera_qos": "sensor_data",
            "headset_world_frame": "headset_world",
            "robot_base_frame": "base_link",
            "sync_slop_s": 0.02,
            "max_age_s": 2.0,
            "input_timeout_s": 2.0,
            "registration_period_s": 1.0,
            "recovery_period_s": 0.25,
            "confidence_threshold": 0.7,
            "device": "cpu",
            "cpu_threads": 4,
            "cuda_memory_fraction": 0.25,
            "max_keypoints": 512,
            "match_filter_threshold": 0.1,
            "min_inliers": 12,
            "min_inlier_ratio": 0.35,
            "ransac_pixels": 3.0,
            "max_median_pixels": 2.0,
            "min_depth_m": 0.2,
            "max_depth_m": 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        camera_qos = camera_subscription_qos(self.settings["camera_qos"])
        self.config = RegistrationConfig(**{
            name: self.settings[name] for name in (
                "min_inliers", "min_inlier_ratio", "ransac_pixels",
                "max_median_pixels", "min_depth_m", "max_depth_m")})
        for name in ("sync_slop_s", "max_age_s", "input_timeout_s", "registration_period_s", "recovery_period_s"):
            if self.settings[name] <= 0:
                raise ValueError(name + " must be positive")
        if not 0 <= self.settings["confidence_threshold"] <= 1:
            raise ValueError("confidence_threshold must be in [0, 1]")
        memory_fraction = self.settings["cuda_memory_fraction"]
        if not np.isfinite(memory_fraction) or not 0 < memory_fraction <= 1:
            raise ValueError("cuda_memory_fraction must be finite and in (0, 1]")
        self.publisher = self.create_publisher(String, "/deictic/registration", 10)
        self.failures = self.create_publisher(String, "/deictic/registration_failure", 10)
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="superpoint")
        self.frontend = None
        self.pending = self.pool.submit(SuperPointMatcher, self.settings["device"],
                                        self.settings["max_keypoints"], self.settings["cpu_threads"],
                                        self.settings["match_filter_threshold"],
                                        self.settings["cuda_memory_fraction"])
        self.pending_kind = "model"
        self.latest = None
        self.last_input = time.monotonic()
        self.last_attempt = -float("inf")
        self.last_failure = -float("inf")
        self.model_error = None
        self.graph_confident_until = -float("inf")
        self.status_subscriber = self.create_subscription(
            String, "/deictic/status", self._status, 10)
        kinds = [CompressedImage if self.settings["headset_compressed"] else Image,
                 CameraInfo, Image, PoseStamped,
                 CompressedImage if self.settings["wrist_compressed"] else Image,
                 CameraInfo, PoseStamped]
        names = ["headset_image_topic", "headset_info_topic", "headset_depth_topic",
                 "headset_pose_topic", "wrist_image_topic", "wrist_info_topic", "wrist_pose_topic"]
        self.subscribers = [message_filters.Subscriber(
            self, kind, self.settings[name], qos_profile=camera_qos)
            for kind, name in zip(kinds, names)]
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            self.subscribers, queue_size=8, slop=self.settings["sync_slop_s"],
            allow_headerless=False)
        self.synchronizer.registerCallback(self._receive)
        self.timer = self.create_timer(0.05, self._poll)
        self.get_logger().info("Awaiting calibrated camera streams and SuperPoint model; no synthetic fallback")

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _failure(self, reason, force=False):
        now = time.monotonic()
        if force or now - self.last_failure >= 1.0:
            self.failures.publish(String(data=json.dumps({
                "schema_version": 1, "stamp": self.now_seconds(),
                "reason": str(reason), "source": "superpoint_pnp"})))
            self.last_failure = now

    def _receive(self, *messages):
        # Keep only the latest synchronized set, so a slow model cannot create a backlog.
        self.latest = messages
        self.last_input = time.monotonic()

    def _status(self, message):
        try:
            data = json.loads(message.data)
            fresh = abs(self.now_seconds() - float(data["stamp"])) <= self.settings["max_age_s"]
            confidence = float(data["confidence"])
            confident = self.settings["confidence_threshold"] <= confidence <= 1.0
            self.graph_confident_until = (time.monotonic() + self.settings["input_timeout_s"]
                                           if fresh and confident and data.get("can_commit") is True
                                           else -float("inf"))
        except (ValueError, TypeError, KeyError):
            self.graph_confident_until = -float("inf")

    def _validate_frames(self, messages):
        hi, hc, hd, hp, wi, wc, wp = messages
        if (not hi.header.frame_id or not wi.header.frame_id
                or hi.header.frame_id != hc.header.frame_id
                or hi.header.frame_id != hd.header.frame_id
                or wi.header.frame_id != wc.header.frame_id):
            raise RegistrationError("RGB/depth/CameraInfo optical frame IDs disagree or are empty")
        if hp.header.frame_id != self.settings["headset_world_frame"]:
            raise RegistrationError("Headset optical capture pose has wrong parent frame")
        if wp.header.frame_id != self.settings["robot_base_frame"]:
            raise RegistrationError("Wrist optical capture pose has wrong parent frame")
        stamps = [stamp_seconds(m) for m in messages]
        if min(stamps) <= 0 or max(stamps) - min(stamps) > self.settings["sync_slop_s"]:
            raise RegistrationError("Camera measurements are not timestamp-synchronized")
        age = self.now_seconds() - min(stamps)
        if age < -self.settings["sync_slop_s"] or age > self.settings["max_age_s"]:
            raise RegistrationError("Camera data is stale or clocks disagree")

    def _estimate(self, messages):
        hi, hc, hd, hp, wi, wc, wp = messages
        headset, wrist = rgb_image(hi), rgb_image(wi)
        head_model, wrist_model = camera_model(hc), camera_model(wc)
        if (headset.shape[:2] != (head_model.height, head_model.width)
                or wrist.shape[:2] != (wrist_model.height, wrist_model.width)):
            raise RegistrationError("CameraInfo resolution does not match RGB images")
        head_pose, wrist_pose = pose_array(hp), pose_array(wp)
        source, target = self.frontend.match(headset, wrist)
        result = estimate_registration(source, target, depth_image(hd), head_model,
                                       wrist_model, self.config)
        return {
            "schema_version": 1,
            "stamp": min(stamp_seconds(hi), stamp_seconds(wi)),
            "headset_pose": head_pose,
            "camera_pose": wrist_pose,
            "camera_from_headset": matrix_to_pose(result.camera_from_headset),
            "inliers": result.inliers,
            "matches": result.matches,
            "median_reprojection_error": result.median_reprojection_error,
            "source": "superpoint_pnp",
        }

    def _poll(self):
        if self.pending is not None and self.pending.done():
            kind = self.pending_kind
            try:
                result = self.pending.result()
                if kind == "model":
                    self.frontend = result
                    self.get_logger().info("SuperPoint + LightGlue ready")
                elif not -self.settings["sync_slop_s"] <= self.now_seconds() - result["stamp"] <= self.settings["max_age_s"]:
                    self._failure("Registration result expired during processing", force=True)
                else:
                    self.publisher.publish(String(data=json.dumps(result, allow_nan=False)))
            except Exception as exc:
                if kind == "model":
                    self.model_error = str(exc)
                    self.get_logger().error(self.model_error)
                self._failure(str(exc), force=True)
            self.pending = None
        if self.model_error:
            self._failure(self.model_error)
            return
        if self.frontend is None:
            self._failure("SuperPoint provider is still loading")
            return
        if time.monotonic() - self.last_input > self.settings["input_timeout_s"]:
            self._failure("Missing synchronized calibrated camera observations")
        if self.pending is not None or self.latest is None:
            return
        period = (self.settings["registration_period_s"]
                  if time.monotonic() < self.graph_confident_until
                  else min(self.settings["registration_period_s"], self.settings["recovery_period_s"]))
        if time.monotonic() - self.last_attempt < period:
            return
        messages, self.latest = self.latest, None
        self.last_attempt = time.monotonic()
        try:
            self._validate_frames(messages)
            self.pending = self.pool.submit(self._estimate, messages)
            self.pending_kind = "registration"
        except Exception as exc:
            self._failure(str(exc), force=True)

    def destroy_node(self):
        self.pool.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RegistrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # ROS signal handlers may invalidate the context before spin unwinds.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
