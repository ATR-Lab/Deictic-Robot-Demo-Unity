"""ROS-free source arbitration and stamped actuator watchdog for the K1 simulator."""
import math
from collections import OrderedDict
from uuid import UUID

from k1_model import ARM_JOINTS, BOTH_ARM_JOINTS, Trajectory, joint_limits, merge_arm_positions, validate_positions

TELEOP_LEASE = .30
FEEDBACK_TIMEOUT = .50
COMMAND_TIMEOUT = .50
FUTURE_TOLERANCE = .05


def valid_stamp(stamp, wall_now, max_age):
    return (not isinstance(stamp, bool) and isinstance(stamp, (int, float))
            and math.isfinite(stamp) and stamp > 0
            and -FUTURE_TOLERANCE <= wall_now-stamp <= max_age)


class ArmSetpoints:
    """Actuator boundary: queued/stale commands cannot renew the measured hold timer."""
    def __init__(self, initial, limits=None):
        self.limits = limits or joint_limits()
        self.desired = validate_positions(BOTH_ARM_JOINTS, initial, self.limits, BOTH_ARM_JOINTS)
        self.last_stamp = 0.
        self.received = None
        self.holding = True

    def command(self, names, positions, stamp, now, wall_now):
        if not valid_stamp(stamp, wall_now, COMMAND_TIMEOUT) or stamp <= self.last_stamp:
            raise ValueError("Simulator command timestamp is stale, future, missing or replayed")
        desired = merge_arm_positions(names, positions, self.desired, self.limits)
        self.desired, self.last_stamp, self.received = desired, stamp, now
        self.holding = False

    def sample(self, measured, now, wall_now):
        expired = (self.received is None or now-self.received > COMMAND_TIMEOUT
                   or not valid_stamp(self.last_stamp, wall_now, COMMAND_TIMEOUT))
        if expired and not self.holding:
            self.desired = list(measured)
            self.holding = True
        return list(self.desired)


class RelayControl:
    """One owner at a time; expired teleoperation never revives a reaching trajectory."""
    def __init__(self, limits=None):
        self.limits = limits or joint_limits()
        self.current = None
        self.state_received = None
        self.trajectory = None
        self.started = 0.
        self.left_hold = None
        self.hold = None
        self.owner = None
        self.latched = False
        self.teleop_target = None
        self.teleop_received = None
        self.teleop_stamp = 0.
        self.sessions = OrderedDict()
        self.retired = set()
        self.release_stamp = 0.
        self.reason = "waiting_for_feedback"

    def feedback(self, names, positions, now):
        if len(names) != len(positions) or len(set(names)) != len(names):
            raise ValueError("Simulator feedback has duplicate/mismatched joints")
        values = dict(zip(names, positions))
        if not all(name in values and not isinstance(values[name], bool)
                   and isinstance(values[name], (int, float)) and math.isfinite(values[name])
                   and self.limits[name][0]-.03 <= values[name] <= self.limits[name][1]+.03
                   for name in BOTH_ARM_JOINTS):
            raise ValueError("Simulator feedback requires all eight finite arm positions within tracking limits")
        self.current = [values[name] for name in BOTH_ARM_JOINTS]
        self.state_received = now

    def fresh_feedback(self, now):
        return (self.current is not None and self.state_received is not None
                and 0 <= now-self.state_received <= FEEDBACK_TIMEOUT)

    def _hold_measured(self, now, reason):
        self.trajectory = None
        self.teleop_target = None
        self.hold = list(self.current) if self.fresh_feedback(now) else None
        self.reason = reason

    def _expire(self, now, wall_now):
        if not self.fresh_feedback(now):
            self._hold_measured(now, "joint_feedback_stale")
            if self.owner is not None:
                self.latched = True
        elif self.owner is not None and not self.latched and (
                self.teleop_received is None or now-self.teleop_received > TELEOP_LEASE
                or not valid_stamp(self.teleop_stamp, wall_now, TELEOP_LEASE)):
            self._hold_measured(now, "teleop_lease_expired")
            self.latched = True

    def legacy(self, names, points, now, wall_now):
        self._expire(now, wall_now)
        if self.owner is not None:
            raise ValueError("Teleoperation owns the simulator; reaching trajectory rejected")
        if not self.fresh_feedback(now):
            raise ValueError("Simulator joint state unavailable/stale")
        trajectory = Trajectory(names, points, self.current[4:], self.limits)
        self.trajectory, self.started = trajectory, now
        self.left_hold, self.hold = list(self.current[:4]), None
        self.reason = "legacy_trajectory"

    def teleop(self, payload, now, wall_now):
        self._expire(now, wall_now)
        if not isinstance(payload, dict) or type(payload.get("schema_version")) is not int or payload["schema_version"] != 1:
            raise ValueError("Teleoperation command requires schema 1")
        session = payload.get("session_id")
        if not isinstance(session, str):
            raise ValueError("Teleoperation session_id must be a UUID string")
        session = str(UUID(session))
        sequence, stamp, active = payload.get("sequence"), payload.get("stamp"), payload.get("active")
        if type(sequence) is not int or sequence < 0 or type(active) is not bool:
            raise ValueError("Teleoperation sequence/active fields are invalid")
        if not valid_stamp(stamp, wall_now, TELEOP_LEASE):
            raise ValueError("Teleoperation command timestamp is stale, future or missing")
        command_names, positions = payload.get("names", []), payload.get("positions", [])
        if active:
            target = validate_positions(command_names, positions, self.limits, BOTH_ARM_JOINTS)
        else:
            # A measured hold may lie microscopically outside an authored joint
            # limit after physics integration. Release never applies these
            # payload positions, so require their structure/finiteness only.
            if (len(command_names) != 8 or set(command_names) != set(BOTH_ARM_JOINTS)
                    or len(positions) != 8 or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                                or not math.isfinite(v) for v in positions)):
                raise ValueError("Inactive command requires all eight finite arm positions")
            target = None
        previous = self.sessions.get(session)
        if session in self.retired or (previous and (sequence <= previous[0] or stamp < previous[1])):
            raise ValueError("Teleoperation command is replayed or from a retired session")
        if self.owner is not None and session != self.owner:
            if active or not self.latched:
                raise ValueError("Another teleoperation session owns the simulator")
            # A new backend may explicitly release a dead owner's lease; delayed
            # packets from the replaced backend can never reacquire ownership.
            self.retired.add(self.owner)
        if active:
            if self.latched:
                raise ValueError("Expired teleoperation requires an explicit inactive release")
            if stamp < self.release_stamp:
                raise ValueError("Teleoperation target predates the latest release")
            if not self.fresh_feedback(now):
                raise ValueError("Simulator joint state unavailable/stale")
            self.trajectory = None
            self.hold = None
            self.owner = session
            self.teleop_target = target
            self.teleop_received, self.teleop_stamp = now, stamp
            self.reason = "teleop_active"
        else:
            self._hold_measured(now, "teleop_released_hold")
            self.owner, self.latched = None, False
            self.release_stamp = max(self.release_stamp, stamp)
        self.sessions[session] = (sequence, stamp)
        self.sessions.move_to_end(session)
        # At 0.3 s freshness, packets from evicted inactive sessions are already
        # old. Never evict the current owner or a replacement-session tombstone.
        while len(self.sessions) > 64:
            candidate = next(key for key in self.sessions if key != self.owner)
            del self.sessions[candidate]

    def sample(self, now, wall_now):
        self._expire(now, wall_now)
        if not self.fresh_feedback(now):
            return None
        if self.owner is not None:
            return list(self.hold) if self.latched and self.hold is not None else (
                list(self.teleop_target) if not self.latched and self.teleop_target is not None else None)
        if self.trajectory is not None:
            return self.left_hold + self.trajectory.sample(now-self.started)
        return list(self.hold) if self.hold is not None else None

    def status(self, now, wall_now, command_publisher_live):
        """Acknowledge relay ownership, not merely a subscribed command topic.

        Expiry is evaluated even when the output timer has not run yet. A
        retained, expired owner is explicitly inactive and cannot revive until
        a fresh release. Readiness means feedback and actuator connectivity;
        it does not clear that latch or acknowledge physical target attainment.
        Command age is the larger of source-clock and monotonic receipt age.
        """
        self._expire(now, wall_now)
        feedback_ready = self.fresh_feedback(now)
        ready = feedback_ready and bool(command_publisher_live)
        active = (ready and self.owner is not None and not self.latched
                  and self.teleop_target is not None)
        owner_sequence = self.sessions.get(self.owner)
        age = None
        if self.owner is not None and self.teleop_received is not None:
            age = max(0., wall_now-self.teleop_stamp, now-self.teleop_received)
        reason = self.reason
        if not feedback_ready:
            reason = "joint_feedback_stale"
        elif not command_publisher_live:
            reason = "simulation_command_subscriber_missing"
        return dict(schema_version=1, stamp=wall_now, ready=ready, active=active,
                    owner_session_id=self.owner,
                    last_sequence=owner_sequence[0] if owner_sequence is not None else None,
                    reason=reason, command_age=age)
