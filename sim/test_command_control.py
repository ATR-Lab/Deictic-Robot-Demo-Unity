"""Command ownership, replay prevention and measured watchdog holds; no ROS/GPU."""
import math
import json
import unittest

from command_control import ArmSetpoints, RelayControl
from k1_model import ARM_JOINTS, BOTH_ARM_JOINTS

A = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa"
B = "bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb"
REST = [0.,0.,0.,0.,0.,.5,0.,0.]


def packet(seq=1, stamp=100., active=True, session=A, positions=None):
    return dict(schema_version=1,session_id=session,sequence=seq,stamp=stamp,active=active,
                names=list(BOTH_ARM_JOINTS),positions=positions or [.1,.1,.1,.1,.1,.6,.1,.1])


class RelayControlTests(unittest.TestCase):
    def setUp(self):
        self.control = RelayControl()
        self.control.feedback(BOTH_ARM_JOINTS, REST, 0.)

    def test_legacy_right_trajectory_keeps_measured_left_hold(self):
        measured = [.1,-.2,.3,-.4]+REST[4:]
        self.control.feedback(BOTH_ARM_JOINTS,measured,0.)
        self.control.legacy(ARM_JOINTS,[(1.,[.2,.7,.2,.2])],0.,100.)
        self.assertEqual(self.control.sample(.25,100.25)[:4],measured[:4])
        self.assertAlmostEqual(self.control.sample(.25,100.25)[4],.05)

    def test_teleop_claim_discards_legacy_and_rejects_new_legacy(self):
        self.control.legacy(ARM_JOINTS,[(1.,[.2,.7,.2,.2])],0.,100.)
        command = packet()
        self.control.teleop(command,0.,100.)
        self.assertIsNone(self.control.trajectory)
        self.assertEqual(self.control.sample(.1,100.1),command['positions'])
        with self.assertRaises(ValueError):
            self.control.legacy(ARM_JOINTS,[(1.,REST[4:])],.1,100.1)

    def test_timeout_holds_both_measured_and_requires_fresh_release(self):
        self.control.teleop(packet(),0.,100.)
        measured = [.02]*4+[.02,.52,.02,.02]
        self.control.feedback(BOTH_ARM_JOINTS,measured,.31)
        self.assertEqual(self.control.sample(.31,100.31),measured)
        self.assertTrue(self.control.latched)
        with self.assertRaises(ValueError):
            self.control.teleop(packet(seq=2,stamp=100.32),.32,100.32)
        self.control.teleop(packet(seq=3,stamp=100.33,active=False),.33,100.33)
        self.assertIsNone(self.control.owner)
        self.assertEqual(self.control.sample(.34,100.34),measured)
        self.control.teleop(packet(seq=4,stamp=100.35),.35,100.35)
        self.assertFalse(self.control.latched)

    def test_release_does_not_resume_old_trajectory_or_queued_active(self):
        self.control.legacy(ARM_JOINTS,[(1.,[.2,.7,.2,.2])],0.,100.)
        self.control.teleop(packet(),0.,100.)
        self.control.teleop(packet(seq=3,stamp=100.1,active=False),.1,100.1)
        self.assertEqual(self.control.sample(.2,100.2),REST)
        self.assertIsNone(self.control.trajectory)
        for queued in (packet(seq=2,stamp=100.05),packet(seq=4,stamp=100.05),
                       packet(seq=1,stamp=100.05,session=B)):
            with self.assertRaises(ValueError):
                self.control.teleop(queued,.2,100.2)

    def test_duplicates_do_not_renew_lease(self):
        self.control.teleop(packet(),0.,100.)
        with self.assertRaises(ValueError):
            self.control.teleop(packet(),.2,100.2)
        self.control.feedback(BOTH_ARM_JOINTS,REST,.31)
        self.assertEqual(self.control.sample(.31,100.31),REST)
        self.assertTrue(self.control.latched)

    def test_restart_can_release_expired_owner_but_not_live_owner(self):
        self.control.teleop(packet(),0.,100.)
        with self.assertRaises(ValueError):
            self.control.teleop(packet(stamp=100.1,active=False,session=B),.1,100.1)
        self.control.feedback(BOTH_ARM_JOINTS,REST,.31)
        self.control.teleop(packet(stamp=100.31,active=False,session=B),.31,100.31)
        self.assertIsNone(self.control.owner)
        self.assertIn(A,self.control.retired)
        with self.assertRaises(ValueError):
            self.control.teleop(packet(seq=2,stamp=100.32),.32,100.32)
        self.control.teleop(packet(seq=2,stamp=100.33,session=B),.33,100.33)
        self.assertEqual(self.control.owner,B)

    def test_joint_feedback_loss_stops_publishing_and_cannot_resume_old_target(self):
        self.control.teleop(packet(),0.,100.)
        self.assertIsNone(self.control.sample(.6,100.6))
        self.assertTrue(self.control.latched)
        self.control.feedback(BOTH_ARM_JOINTS,REST,.61)
        self.assertIsNone(self.control.sample(.61,100.61))
        self.control.teleop(packet(seq=2,stamp=100.62,active=False),.62,100.62)
        self.assertEqual(self.control.sample(.62,100.62),REST)

    def test_bad_targets_timestamps_and_schema_cannot_claim(self):
        bad=[]
        for key,value in (('schema_version',True),('sequence',True),('active',1),('stamp',math.nan),
                          ('stamp',99.),('stamp',100.1),('session_id','not-uuid')):
            p=packet();p[key]=value;bad.append(p)
        for values in ([0]*7,[0]*7+[math.nan],[0]*7+[99.]):
            bad.append(packet(positions=values))
        duplicate=packet();duplicate['names'][-1]=duplicate['names'][0];bad.append(duplicate)
        for command in bad:
            with self.subTest(command=command),self.assertRaises((ValueError,TypeError)):
                self.control.teleop(command,0.,100.)
            self.assertIsNone(self.control.owner)

    def test_reversed_joint_names_are_mapped(self):
        command=packet();command['names'].reverse();command['positions'].reverse()
        self.control.teleop(command,0.,100.)
        self.assertEqual(self.control.sample(.1,100.1),packet()['positions'])

    def test_future_clock_jump_closes_lease(self):
        self.control.teleop(packet(),0.,100.)
        self.assertEqual(self.control.sample(.1,99.),REST)
        self.assertTrue(self.control.latched)

    def test_inactive_ignores_finite_out_of_range_payload_and_holds_measured(self):
        self.control.teleop(packet(),0.,100.)
        self.control.teleop(packet(seq=2,stamp=100.1,active=False,positions=[99.]*8),.1,100.1)
        self.assertEqual(self.control.sample(.1,100.1),REST)


class SimulatorBoundaryTests(unittest.TestCase):
    def test_both_arm_command_then_measured_watchdog_hold(self):
        controller=ArmSetpoints(REST)
        target=packet()['positions'];measured=[v*.5 for v in target]
        controller.command(BOTH_ARM_JOINTS,target,100.,0.,100.)
        self.assertEqual(controller.sample(REST,.1,100.1),target)
        self.assertEqual(controller.sample(measured,.51,100.51),measured)
        self.assertEqual(controller.sample(REST,.6,100.6),measured)

    def test_right_only_command_does_not_reset_left(self):
        initial=[.1,-.2,.3,-.4]+REST[4:]
        controller=ArmSetpoints(initial)
        controller.command(ARM_JOINTS,[.1,.6,.1,.1],100.,0.,100.)
        self.assertEqual(controller.sample(initial,.1,100.1)[:4],initial[:4])

    def test_old_buffered_or_invalid_packet_cannot_renew_watchdog(self):
        controller=ArmSetpoints(REST)
        controller.command(BOTH_ARM_JOINTS,packet()['positions'],100.,0.,100.)
        for stamp in (100.,99.9,99.,100.7,math.nan,0.):
            with self.assertRaises(ValueError):
                controller.command(BOTH_ARM_JOINTS,REST,stamp,.4,100.4)
        with self.assertRaises(ValueError):
            controller.command(BOTH_ARM_JOINTS,[math.nan]*8,100.4,.4,100.4)
        self.assertEqual(controller.sample(REST,.51,100.51),REST)
        self.assertTrue(controller.holding)

    def test_source_timestamp_age_limits_receipt_refresh(self):
        controller=ArmSetpoints(REST)
        controller.command(BOTH_ARM_JOINTS,packet()['positions'],100.,.49,100.49)
        self.assertEqual(controller.sample(REST,.51,100.51),REST)
        self.assertTrue(controller.holding)


class RelayStatusTests(unittest.TestCase):
    def setUp(self):
        self.control = RelayControl()

    def test_missing_feedback_is_not_ready_and_serializes_explicit_nulls(self):
        status = self.control.status(0., 100., True)
        self.assertEqual(json.loads(json.dumps(status, allow_nan=False)), status)
        self.assertEqual(set(status), {'schema_version', 'stamp', 'ready', 'active',
                         'owner_session_id', 'last_sequence', 'reason', 'command_age'})
        self.assertFalse(status['ready'])
        self.assertFalse(status['active'])
        for key in ('owner_session_id', 'last_sequence', 'command_age'):
            self.assertIsNone(status[key])
        self.assertEqual(status['reason'], 'joint_feedback_stale')

    def test_ready_requires_valid_feedback_and_actuator_subscriber(self):
        for values in (REST[:-1], [math.nan]+REST[1:], [99.]+REST[1:], [True]+REST[1:]):
            with self.assertRaises(ValueError):
                self.control.feedback(BOTH_ARM_JOINTS, values, 0.)
            self.assertFalse(self.control.status(0., 100., True)['ready'])
        self.control.feedback(BOTH_ARM_JOINTS, REST, 0.)
        unavailable = self.control.status(.1, 100.1, False)
        self.assertFalse(unavailable['ready'])
        self.assertEqual(unavailable['reason'], 'simulation_command_subscriber_missing')
        self.assertTrue(self.control.status(.1, 100.1, True)['ready'])

    def test_active_ack_identifies_canonical_owner_and_accepted_sequence(self):
        self.control.feedback(BOTH_ARM_JOINTS, REST, 0.)
        self.control.teleop(packet(session=A.upper()), 0., 100.)
        status = self.control.status(.1, 100.1, True)
        self.assertTrue(status['ready'])
        self.assertTrue(status['active'])
        self.assertEqual(status['owner_session_id'], A)
        self.assertEqual(status['last_sequence'], 1)
        self.assertAlmostEqual(status['command_age'], .1)
        self.assertEqual(status['reason'], 'teleop_active')

    def test_status_detects_expiry_without_output_tick_and_retains_owner(self):
        self.control.feedback(BOTH_ARM_JOINTS, REST, 0.)
        self.control.teleop(packet(), 0., 100.)
        status = self.control.status(.31, 100.31, True)
        self.assertTrue(status['ready'])
        self.assertFalse(status['active'])
        self.assertEqual(status['owner_session_id'], A)
        self.assertEqual(status['last_sequence'], 1)
        self.assertEqual(status['reason'], 'teleop_lease_expired')
        self.assertAlmostEqual(status['command_age'], .31)
        with self.assertRaises(ValueError):
            self.control.teleop(packet(seq=2, stamp=100.32), .32, 100.32)
        self.assertEqual(self.control.status(.32, 100.32, True)['last_sequence'], 1)

    def test_explicit_release_ack_has_no_active_owner_or_old_command_age(self):
        self.control.feedback(BOTH_ARM_JOINTS, REST, 0.)
        self.control.teleop(packet(), 0., 100.)
        self.control.teleop(packet(seq=2, stamp=100.1, active=False), .1, 100.1)
        status = self.control.status(.1, 100.1, True)
        self.assertTrue(status['ready'])
        self.assertFalse(status['active'])
        self.assertEqual(status['reason'], 'teleop_released_hold')
        for key in ('owner_session_id', 'last_sequence', 'command_age'):
            self.assertIsNone(status[key])
        self.assertEqual(self.control.sample(.1, 100.1), REST)

    def test_feedback_loss_ack_latches_even_after_feedback_recovers(self):
        self.control.feedback(BOTH_ARM_JOINTS, REST, 0.)
        self.control.teleop(packet(), 0., 100.)
        lost = self.control.status(.6, 100.6, True)
        self.assertFalse(lost['ready'])
        self.assertFalse(lost['active'])
        self.assertEqual(lost['owner_session_id'], A)
        self.control.feedback(BOTH_ARM_JOINTS, REST, .61)
        recovered = self.control.status(.61, 100.61, True)
        self.assertTrue(recovered['ready'])
        self.assertFalse(recovered['active'])
        self.assertEqual(recovered['reason'], 'joint_feedback_stale')

    def test_rejected_stale_replayed_or_retired_inputs_cannot_change_ack(self):
        self.control.feedback(BOTH_ARM_JOINTS, REST, 0.)
        self.control.teleop(packet(), 0., 100.)
        for invalid in (packet(), packet(seq=2, stamp=99.5)):
            with self.assertRaises(ValueError):
                self.control.teleop(invalid, .1, 100.1)
        status = self.control.status(.1, 100.1, True)
        self.assertEqual(status['last_sequence'], 1)
        self.assertAlmostEqual(status['command_age'], .1)
        self.control.feedback(BOTH_ARM_JOINTS, REST, .31)
        self.control.teleop(packet(stamp=100.31, active=False, session=B), .31, 100.31)
        self.control.teleop(packet(seq=2, stamp=100.32, session=B), .32, 100.32)
        with self.assertRaises(ValueError):
            self.control.teleop(packet(seq=10, stamp=100.33), .33, 100.33)
        status = self.control.status(.33, 100.33, True)
        self.assertEqual(status['owner_session_id'], B)
        self.assertEqual(status['last_sequence'], 2)

    def test_command_age_covers_source_and_monotonic_lease_clocks(self):
        self.control.feedback(BOTH_ARM_JOINTS, REST, 0.)
        self.control.teleop(packet(), .1, 100.2)
        self.assertAlmostEqual(self.control.status(.15, 100.25, True)['command_age'], .25)
        # Source clock is slow, but monotonic receipt age still expires ownership.
        self.assertFalse(self.control.status(.41, 100.25, True)['active'])
        self.assertAlmostEqual(self.control.status(.41, 100.25, True)['command_age'], .31)

    def test_disconnected_actuator_never_reports_active(self):
        self.control.feedback(BOTH_ARM_JOINTS, REST, 0.)
        self.control.teleop(packet(), 0., 100.)
        status = self.control.status(.1, 100.1, False)
        self.assertFalse(status['ready'])
        self.assertFalse(status['active'])
        self.assertEqual(status['owner_session_id'], A)


if __name__ == '__main__':
    unittest.main()
