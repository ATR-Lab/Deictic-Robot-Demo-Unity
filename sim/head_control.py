"""ROS-free, bounded head-pose commands for the fixed-base K1 simulator.

The quaternion rotates a forward-left-up head frame into base_link. The K1
has yaw about +Z followed by pitch about +Y, with no roll joint. Targets follow
the quaternion's forward direction immediately; physics drives the joints.
"""
import math
from collections import OrderedDict
from uuid import UUID

from command_control import valid_stamp
from k1_model import HEAD_JOINTS, joint_limits

HEAD_COMMAND_TIMEOUT = .30
MAX_COMMAND_BYTES = 2048
MAX_SESSIONS = 64


def orientation_angles(orientation):
    """Map normalized XYZW FLU orientation to yaw/pitch, without roll."""
    if (not isinstance(orientation, (list, tuple)) or len(orientation) != 4
            or any(isinstance(value, bool) or not isinstance(value, (int, float))
                   or not math.isfinite(value) for value in orientation)):
        raise ValueError("Head orientation requires four finite XYZW quaternion values")
    norm = math.sqrt(sum(value*value for value in orientation))
    if not .99 <= norm <= 1.01:
        raise ValueError("Head orientation quaternion must have unit length")
    x, y, z, w = (value/norm for value in orientation)
    # First column of Rz(yaw) Ry(pitch) Rx(roll): head forward in base_link.
    forward = (1.-2.*(y*y+z*z), 2.*(x*y+w*z), 2.*(x*z-w*y))
    yaw = math.atan2(forward[1], forward[0])
    pitch = math.atan2(-forward[2], math.hypot(forward[0], forward[1]))
    return [yaw, pitch]


def orientation_targets(orientation, limits=None):
    """Map normalized XYZW FLU orientation to limited yaw/pitch radians."""
    limits = limits or joint_limits()
    return [min(limits[name][1], max(limits[name][0], value))
            for name, value in zip(HEAD_JOINTS, orientation_angles(orientation))]


class HeadSetpoints:
    """Latest-pose actuator lease; stale/inactive input holds measured neck pose.

    Both source UTC age and monotonic receipt age bound the command lifetime.
    Invalid or replayed input never refreshes that lease. A new publisher can
    replace an expired/inactive one, retiring the previous session so queued
    packets from it cannot reclaim the neck.
    """
    def __init__(self, initial=(0., 0.), limits=None):
        self.limits = limits or joint_limits()
        self._measured(initial)
        self.desired = list(map(float, initial))
        self.session = None
        self.sessions = OrderedDict()
        self.retired = OrderedDict()
        self.last_stamp = 0.
        self.received = None
        self.source_to_receipt_s = None
        self.active = False
        self.holding = True
        self.limited = False
        self.reason = "waiting_for_head_pose"

    @staticmethod
    def _measured(measured):
        if len(measured) != 2 or any(isinstance(value, bool) or not math.isfinite(value)
                                     for value in measured):
            raise ValueError("Head feedback requires two finite measured joint positions")

    def _fresh(self, now, wall_now):
        return (self.received is not None and 0 <= now-self.received <= HEAD_COMMAND_TIMEOUT
                and valid_stamp(self.last_stamp, wall_now, HEAD_COMMAND_TIMEOUT))

    def command(self, payload, now, wall_now):
        if (not isinstance(payload, dict) or type(payload.get("schema_version")) is not int
                or payload["schema_version"] != 1 or payload.get("frame_id") != "base_link"):
            raise ValueError("Head command requires schema 1 and frame_id base_link")
        session = payload.get("session_id")
        if not isinstance(session, str) or len(session) > 36:
            raise ValueError("Head session_id must be a UUID string")
        session = str(UUID(session))
        sequence, stamp = payload.get("sequence"), payload.get("stamp")
        active, tracked = payload.get("active"), payload.get("tracked")
        if (type(sequence) is not int or not 0 <= sequence <= 9007199254740991
                or type(active) is not bool or type(tracked) is not bool):
            raise ValueError("Head command sequence/active/tracked fields are invalid")
        if not valid_stamp(stamp, wall_now, HEAD_COMMAND_TIMEOUT) or stamp < self.last_stamp:
            raise ValueError("Head command timestamp is stale, future, missing or reordered")
        target = orientation_targets(payload.get("orientation"), self.limits)
        previous = self.sessions.get(session)
        if session in self.retired or (previous and (sequence <= previous[0] or stamp < previous[1])):
            raise ValueError("Head command is replayed or from a retired session")
        if self.session is not None and session != self.session:
            if self.active and self._fresh(now, wall_now):
                raise ValueError("Another active head session owns the simulator")
            self.retired[self.session] = None
            self.retired.move_to_end(self.session)
            while len(self.retired) > MAX_SESSIONS:
                self.retired.popitem(last=False)
        # Capture once at first acquisition or when leaving active motion.
        # Repeated inactive heartbeats must preserve that captured hold target;
        # rebasing it to each slightly sagged measurement ratchets the neck down.
        capture_hold = self.active or self.session is None
        self.session = session
        self.sessions[session] = (sequence, stamp)
        self.sessions.move_to_end(session)
        while len(self.sessions) > MAX_SESSIONS:
            self.sessions.popitem(last=False)
        self.last_stamp, self.received = stamp, now
        self.source_to_receipt_s = max(0., wall_now-stamp)
        self.active = active and tracked
        if self.active or capture_hold:
            self.holding = False
        if self.active:
            self.desired = target
            self.limited = any(abs(a-b) > 1e-6 for a, b in
                               zip(target, orientation_angles(payload["orientation"])))
            self.reason = "head_pose_limited" if self.limited else "head_pose_active"
        else:
            self.limited = False
            self.reason = "head_tracking_lost" if active else "head_view_inactive"

    def sample(self, measured, now, wall_now):
        self._measured(measured)
        if (not self.active or not self._fresh(now, wall_now)) and not self.holding:
            if self.active:
                self.reason = "head_pose_expired"
            self.desired = list(map(float, measured))
            self.holding = True
            self.active = False
            self.limited = False
        return list(self.desired)

    def status(self, measured, now, wall_now):
        """Diagnostic acknowledgement of accepted target and measured joint pose."""
        targets = self.sample(measured, now, wall_now)
        sequence = self.sessions.get(self.session)
        return dict(schema_version=1, stamp=wall_now, frame_id="base_link", ready=True,
                    active=self.active, reason=self.reason, limited=self.limited,
                    command_age=self.command_age(now, wall_now), source_to_receipt_s=self.source_to_receipt_s,
                    session_id=self.session, sequence=sequence[0] if sequence else None,
                    names=list(HEAD_JOINTS), measured=list(map(float, measured)), targets=targets)

    def command_age(self, now, wall_now):
        return None if self.received is None else max(0., now-self.received, wall_now-self.last_stamp)
