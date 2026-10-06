"""Read-only ROS visualization of the reference worker's measured right arm.

No control subscription, service or action is created here. The worker HTTP owner
remains the only command path. JointState includes only actual articulated DOFs;
fixed left-arm/head/leg joints are never fabricated as sensor measurements.
"""
from __future__ import annotations

import json
import math


def measurement_payload(names, joints, derived_velocities, native_velocities, *,
                        boot_id, sequence, sim_time, received_monotonic, stamp_ns):
    if (len(names) != 4 or len(set(names)) != 4 or len(joints) != 4
            or len(native_velocities) != 4
            or derived_velocities is not None and len(derived_velocities) != 4):
        raise ValueError("expected exactly four measured reference-arm DOFs")
    numbers = list(joints) + list(native_velocities) + list(derived_velocities or [])
    numbers += [sim_time, received_monotonic]
    if not all(type(value) in (int, float) and math.isfinite(value) for value in numbers):
        raise ValueError("nonfinite or untyped physics telemetry")
    if sim_time < 0 or type(stamp_ns) is not int or stamp_ns <= 0 or type(sequence) is not int or sequence < 1:
        raise ValueError("invalid source time or sample identity")
    return {
        "schema_version": 1, "source": "isaac_fixed_right_arm", "simulation_only": True,
        "boot_id": boot_id, "sample_sequence": sequence, "sim_time_seconds": sim_time,
        "sample_received_monotonic": received_monotonic, "stamp_ns": stamp_ns,
        "stamp_source": "control-host ROS system clock after physics sample",
        "joint_names": list(names), "positions_rad": list(joints),
        "position_difference_velocity_rad_s": None if derived_velocities is None else list(derived_velocities),
        "native_velocity_rad_s": list(native_velocities),
        "joint_state_velocity_source": "position_difference_per_sim_second" if derived_velocities is not None else "unavailable_initial_sample",
        "fixed_model_parts": ["trunk", "left_arm", "head", "legs"],
        "fixed_model_parts_are_measured": False,
        "control_transport": "exclusive_loopback_worker_http",
        "ros_command_inputs": [],
    }


def configure_camera(camera):
    """Same calibrated virtual pinhole profile as the Unity demo cameras."""
    from pxr import Gf
    camera.set_focal_length(.012)
    camera.set_horizontal_aperture(.024)
    camera.set_vertical_aperture(.018)
    camera.set_focus_distance(.5)
    camera.set_lens_aperture(0.)
    camera.set_clipping_range(.015, 20.)
    width, height = camera.get_resolution()
    camera.set_opencv_pinhole_properties(cx=width/2, cy=height/2,
                                         fx=width/2, fy=width/2, pinhole=[0.]*8)
    image_size = camera.prim.GetAttribute("omni:lensdistortion:opencvPinhole:imageSize")
    if not image_size:
        raise RuntimeError("renderer lacks calibrated OpenCV imageSize")
    image_size.Set(Gf.Vec2i(width, height))


def camera_intrinsics(camera):
    import numpy as np
    if tuple(camera.prim.GetAttribute("omni:lensdistortion:opencvPinhole:imageSize").Get()) != tuple(camera.get_resolution()):
        raise RuntimeError("renderer calibration resolution mismatch")
    cx, cy, fx, fy, _ = camera.get_opencv_pinhole_properties()
    return np.array([[fx, 0., cx], [0., fy, cy], [0., 0., 1.]], dtype=float)


class SimulationTelemetry:
    def __init__(self, names, boot_id):
        import rclpy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        rclpy.init()
        self.rclpy, self.JointState, self.String = rclpy, JointState, String
        self.node = rclpy.create_node("transition_k1_isaac_observation")
        self.joints = self.node.create_publisher(JointState, "/joint_states", 1)
        self.metadata = self.node.create_publisher(String, "/transition/simulation/telemetry", 1)
        self.names, self.boot_id = tuple(names), boot_id
        self.sequence, self.next_publish, self.last_source_time = 0, 0., None

    def spin(self):
        self.rclpy.spin_once(self.node, timeout_sec=0.)

    def publish(self, joints, derived, native, now, sim_time):
        # This gate cannot refresh an old physics sample's timestamp.
        if self.last_source_time is not None and sim_time <= self.last_source_time:
            return False
        self.last_source_time = sim_time
        if now < self.next_publish:
            return False
        self.sequence += 1
        stamp = self.node.get_clock().now()
        payload = measurement_payload(self.names, joints, derived, native,
            boot_id=self.boot_id, sequence=self.sequence, sim_time=sim_time,
            received_monotonic=now, stamp_ns=stamp.nanoseconds)
        message = self.JointState()
        message.header.stamp = stamp.to_msg()
        message.header.frame_id = "trunk"
        message.name, message.position = list(self.names), list(joints)
        message.velocity = list(derived) if derived is not None else []
        self.joints.publish(message)
        self.metadata.publish(self.String(data=json.dumps(payload, allow_nan=False)))
        self.next_publish = now + .05
        return True

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()
