"""Neck mapping, source ownership and actuator holds; no ROS or Isaac imports."""
import json
import math
import unittest

from head_control import HeadSetpoints, orientation_targets
from command_control import RelayControl
from k1_model import BOTH_ARM_JOINTS, HEAD_JOINTS, joint_limits, validate_positions

A = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa"
B = "bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb"


def quaternion(yaw=0., pitch=0., roll=0.):
    cy, sy, cp, sp, cr, sr = (math.cos(yaw/2), math.sin(yaw/2),
                             math.cos(pitch/2), math.sin(pitch/2),
                             math.cos(roll/2), math.sin(roll/2))
    return [cy*cp*sr-sy*sp*cr, cy*sp*cr+sy*cp*sr,
            sy*cp*cr-cy*sp*sr, cy*cp*cr+sy*sp*sr]


def packet(sequence=1, stamp=100., session=A, active=True, tracked=True, orientation=None):
    return dict(schema_version=1, frame_id="base_link", session_id=session,
                sequence=sequence, stamp=stamp, active=active, tracked=tracked,
                orientation=orientation if orientation is not None else quaternion(.4, .2))


class HeadOrientationTests(unittest.TestCase):
    def assertAngles(self, actual, expected):
        self.assertEqual(len(actual), 2)
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=7)

    def test_forward_zero_and_positive_flu_axes(self):
        for yaw, pitch in ((0., 0.), (.3, 0.), (-.3, 0.), (0., .25), (0., -.25), (.4, -.2)):
            self.assertAngles(orientation_targets(quaternion(yaw, pitch)), (yaw, pitch))

    def test_roll_never_creates_a_fictitious_joint_or_changes_forward_direction(self):
        for roll in (-2., -.3, .4, 2.):
            self.assertAngles(orientation_targets(quaternion(.4, -.2, roll)), (.4, -.2))

    def test_quaternion_sign_and_small_normalization_error_are_equivalent(self):
        value = quaternion(.3, .2)
        self.assertAngles(orientation_targets([-v for v in value]), (.3, .2))
        self.assertAngles(orientation_targets([v*1.001 for v in value]), (.3, .2))

    def test_authored_joint_limits_are_applied_in_yaw_pitch_order(self):
        limits = joint_limits()
        for sign in (-1, 1):
            target = orientation_targets(quaternion(sign*1.4, sign*1.))
            self.assertAngles(target, [limits[name][0 if sign < 0 else 1] for name in HEAD_JOINTS])

    def test_invalid_quaternions_are_rejected(self):
        for invalid in (None, [0]*3, [0]*4, [0, 0, 0, 2.], [True, 0, 0, 1],
                        [0, 0, math.nan, 1], [0, 0, math.inf, 1], "0001", [0, 0, "0", 1]):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                orientation_targets(invalid)


class HeadCommandTests(unittest.TestCase):
    def setUp(self):
        self.control = HeadSetpoints()

    def test_first_active_packet_targets_current_head_rotation_immediately(self):
        self.control.command(packet(), 0., 100.)
        target = self.control.sample([0., 0.], 0., 100.)
        self.assertAlmostEqual(target[0], .4)
        self.assertAlmostEqual(target[1], .2)

    def test_inactive_or_tracking_lost_takes_measured_hold_not_payload_or_zero(self):
        for active, tracked in ((False, True), (True, False), (False, False)):
            control = HeadSetpoints()
            control.command(packet(), 0., 100.)
            control.command(packet(sequence=2, stamp=100.1, active=active, tracked=tracked), .1, 100.1)
            self.assertEqual(control.sample([.12, -.08], .1, 100.1), [.12, -.08])
            self.assertEqual(control.sample([.2, -.1], .11, 100.11), [.12, -.08])
            self.assertFalse(control.active)

    def test_timeout_latches_measured_pose_until_new_fresh_head_packet(self):
        self.control.command(packet(), 0., 100.)
        self.assertEqual(self.control.sample([.12, -.08], .31, 100.31), [.12, -.08])
        self.assertEqual(self.control.sample([.2, -.1], .4, 100.4), [.12, -.08])
        self.control.command(packet(sequence=2, stamp=100.41), .41, 100.41)
        self.assertAlmostEqual(self.control.sample([.12, -.08], .41, 100.41)[0], .4)

    def test_repeated_inactive_and_untracked_heartbeats_cannot_ratchet_measured_hold(self):
        for active, tracked in ((False, True), (True, False), (False, False)):
            control = HeadSetpoints()
            control.command(packet(), 0., 100.)
            for sequence in range(2, 7):
                elapsed = .05*sequence
                control.command(packet(sequence=sequence, stamp=100.+elapsed,
                                       active=active, tracked=tracked), elapsed, 100.+elapsed)
                # A loaded joint can settle below its drive target. Subsequent
                # release heartbeats must not continually adopt that sagged pose.
                measured = [.12-.01*(sequence-2), -.08-.005*(sequence-2)]
                self.assertEqual(control.sample(measured, elapsed, 100.+elapsed), [.12, -.08])

    def test_inactive_heartbeat_after_watchdog_preserves_existing_hold(self):
        self.control.command(packet(), 0., 100.)
        self.assertEqual(self.control.sample([.12, -.08], .31, 100.31), [.12, -.08])
        self.control.command(packet(sequence=2, stamp=100.4, active=False), .4, 100.4)
        self.assertEqual(self.control.sample([.10, -.10], .4, 100.4), [.12, -.08])

    def test_delayed_receipt_cannot_extend_source_age(self):
        self.control.command(packet(), .29, 100.29)
        self.assertEqual(self.control.sample([.12, -.08], .31, 100.31), [.12, -.08])

    def test_monotonic_age_still_expires_when_source_clock_stops(self):
        self.control.command(packet(), 0., 100.)
        self.assertEqual(self.control.sample([.12, -.08], .31, 100.1), [.12, -.08])

    def test_clock_jump_holds_measured(self):
        self.control.command(packet(), 0., 100.)
        self.assertEqual(self.control.sample([.12, -.08], .1, 99.), [.12, -.08])

    def test_duplicate_or_out_of_order_packet_cannot_refresh_lease(self):
        self.control.command(packet(sequence=4), 0., 100.)
        for duplicate in (packet(sequence=4), packet(sequence=3, stamp=100.1),
                          packet(sequence=5, stamp=99.99)):
            with self.assertRaises(ValueError):
                self.control.command(duplicate, .1, 100.1)
        self.assertEqual(self.control.sample([.12, -.08], .31, 100.31), [.12, -.08])

    def test_new_session_cannot_steal_active_neck_but_can_replace_expired_session(self):
        self.control.command(packet(), 0., 100.)
        with self.assertRaises(ValueError):
            self.control.command(packet(session=B, stamp=100.1), .1, 100.1)
        self.control.command(packet(session=B, stamp=100.31), .31, 100.31)
        self.assertEqual(self.control.session, B)
        with self.assertRaises(ValueError):
            self.control.command(packet(sequence=100, stamp=100.32), .32, 100.32)

    def test_new_session_can_replace_released_old_one_without_waiting(self):
        self.control.command(packet(active=False), 0., 100.)
        self.control.command(packet(session=B, stamp=100.1), .1, 100.1)
        self.assertEqual(self.control.session, B)
        self.assertIn(A, self.control.retired)

    def test_release_rejects_delayed_active_packets(self):
        self.control.command(packet(), 0., 100.)
        self.control.command(packet(sequence=3, stamp=100.1, active=False), .1, 100.1)
        for old in (packet(sequence=2, stamp=100.05), packet(sequence=4, stamp=100.05),
                    packet(session=B, stamp=100.05)):
            with self.assertRaises(ValueError):
                self.control.command(old, .15, 100.15)
        self.assertEqual(self.control.sample([.12, -.08], .15, 100.15), [.12, -.08])

    def test_invalid_contract_and_timestamps_cannot_claim(self):
        for key, value in (("schema_version", True), ("schema_version", 2), ("frame_id", "world"),
                           ("session_id", "bogus"), ("sequence", True), ("sequence", -1),
                           ("sequence", 2**54), ("active", 1), ("tracked", 1),
                           ("stamp", math.nan), ("stamp", 99.), ("stamp", 100.1), ("stamp", True)):
            invalid = packet(); invalid[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.control.command(invalid, 0., 100.)
            self.assertIsNone(self.control.session)

    def test_status_exposes_limits_measured_pose_and_expiration(self):
        self.control.command(packet(orientation=quaternion(1.4, 1.)), 0., 100.)
        status = self.control.status([.3, .2], .1, 100.1)
        self.assertTrue(status["active"])
        self.assertTrue(status["limited"])
        self.assertEqual(status["names"], list(HEAD_JOINTS))
        self.assertEqual(status["targets"], [1.012, .794])
        self.assertEqual(status["measured"], [.3, .2])
        self.assertEqual(json.loads(json.dumps(status, allow_nan=False)), status)
        expired = self.control.status([.31, .21], .31, 100.31)
        self.assertFalse(expired["active"])
        self.assertFalse(expired["limited"])
        self.assertEqual(expired["reason"], "head_pose_expired")
        self.assertEqual(expired["targets"], [.31, .21])

    def test_timing_diagnostics_distinguish_source_receipt_and_current_command_age(self):
        self.assertIsNone(self.control.command_age(0., 100.))
        self.control.command(packet(), .1, 100.2)
        self.assertAlmostEqual(self.control.source_to_receipt_s, .2)
        self.assertAlmostEqual(self.control.command_age(.15, 100.25), .25)
        # A stalled source clock still exposes monotonic command staleness.
        self.assertAlmostEqual(self.control.command_age(.5, 100.25), .4)

    def test_neck_feedback_is_accepted_without_widening_arm_command_contract(self):
        arms = [0., 0., 0., 0., 0., .5, 0., 0.]
        names = list(BOTH_ARM_JOINTS + HEAD_JOINTS)
        full_measured = arms + [.3, .2]
        relay = RelayControl()
        relay.feedback(names, full_measured, 0.)
        self.assertEqual(relay.current, arms)
        with self.assertRaises(ValueError):
            validate_positions(names, full_measured, joint_order=BOTH_ARM_JOINTS)


if __name__ == "__main__":
    unittest.main()
