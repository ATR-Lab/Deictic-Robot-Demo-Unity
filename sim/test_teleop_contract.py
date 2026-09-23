"""Actual controller JSON emission -> pure simulator arbiter (isolated ROS domain)."""
import copy
import json
from pathlib import Path
import sys
import unittest
from uuid import UUID

import numpy as np
from command_control import RelayControl
from k1_model import BOTH_ARM_JOINTS, URDF

try:
    import rclpy
except ImportError:
    rclpy = None


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))

    def get_subscription_count(self):
        return 1


@unittest.skipIf(rclpy is None, "Source the installed ROS environment for the emitted-wire contract test")
class TeleopWireContractTests(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2/src/deictic_control'))
        from deictic_control.node import DeicticControl, duration_msg
        from sensor_msgs.msg import JointState
        rclpy.init(args=['--ros-args','-p',f'urdf:={URDF}',
                        '-p','synthetic_test:=true','-p','synthetic_reference_backend:=true',
                        '-p','allow_execution:=true','-p','allow_teleoperation:=true'],domain_id=211)
        self.node = DeicticControl()
        self.clock = [100.]
        self.node.now = lambda:self.clock[0]
        self.node.monotonic = lambda:self.clock[0]-90.
        self.node.teleop_pub = Recorder()
        self.current = [0.,0.,0.,0.,0.,.5,0.,0.]
        state=JointState(name=list(BOTH_ARM_JOINTS),position=self.current)
        duration_msg(self.clock[0],state.header.stamp)
        self.node.on_joints(state)  # Real startup code emits the inactive handshake.
        self.relay = RelayControl()
        self.relay.feedback(BOTH_ARM_JOINTS,self.current,0.)
        self.deliver_ack(0.)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def emitted(self):
        return json.loads(self.node.teleop_pub.messages[-1].data)

    def deliver_ack(self, monotonic):
        from std_msgs.msg import String
        self.node.on_teleop_relay_status(String(data=json.dumps(
            self.relay.status(monotonic, self.clock[0], True), allow_nan=False)))

    def test_actual_startup_active_release_packets_are_accepted_and_replay_rejected(self):
        startup=self.emitted()
        self.assertEqual(startup['schema_version'],1)
        self.assertFalse(startup['active'])
        self.assertEqual(startup['names'],list(BOTH_ARM_JOINTS))
        self.relay.teleop(startup,0.,100.)
        self.node.teleop.command=np.array(self.current).reshape(2,4)+.01
        self.clock[0]=100.05
        self.node.send_teleop_command(True)
        active=self.emitted()
        self.relay.teleop(active,.05,100.05)
        self.assertEqual(self.relay.sample(.1,100.1),active['positions'])
        self.assertEqual(self.relay.owner,str(UUID(self.node.teleop_session_id)))
        self.clock[0]=100.15
        self.node.send_teleop_command(False)
        self.relay.teleop(self.emitted(),.15,100.15)
        self.assertEqual(self.relay.sample(.2,100.2),self.current)
        with self.assertRaises(ValueError):
            self.relay.teleop(active,.2,100.2)

    def test_actual_output_timeout_holds_then_requires_explicit_release(self):
        self.relay.teleop(self.emitted(),0.,100.)
        self.node.teleop.command=np.array(self.current).reshape(2,4)+.01
        self.clock[0]=100.05
        self.node.send_teleop_command(True)
        self.relay.teleop(self.emitted(),.05,100.05)
        self.relay.feedback(BOTH_ARM_JOINTS,self.current,.36)
        self.assertEqual(self.relay.sample(.36,100.36),self.current)
        self.clock[0]=100.37
        self.node.send_teleop_command(True)
        with self.assertRaises(ValueError):
            self.relay.teleop(self.emitted(),.37,100.37)
        self.clock[0]=100.38
        self.node.send_teleop_command(False)
        self.relay.teleop(self.emitted(),.38,100.38)
        self.assertIsNone(self.relay.owner)

    def test_real_relay_ack_reports_expiry_to_actual_controller_guard(self):
        self.assertIsNone(self.node.teleop_guard())
        self.node.teleop.command=np.array(self.current).reshape(2,4)+.01
        self.clock[0]=100.05
        self.node.send_teleop_command(True)
        active = self.emitted()
        self.relay.teleop(active,.05,100.05)
        self.node.teleop.active=True
        self.node.teleop_acquired=self.node.monotonic()
        self.node.teleop_acquire_sequence=active['sequence']
        self.deliver_ack(.05)
        self.assertIsNone(self.node.teleop_guard())
        # The real relay still has an owner/subscriber, but its lease expired.
        self.clock[0]=100.36
        self.relay.feedback(BOTH_ARM_JOINTS,self.current,.36)
        self.node.teleop_joint_time=self.clock[0]
        self.deliver_ack(.36)
        self.assertIn('teleop_relay_lease_lost', self.node.teleop_guard())
        self.assertFalse(self.relay.status(.36,self.clock[0],True)['active'])
        self.node.stop_teleop('relay_ack_contract_test')
        self.relay.teleop(self.emitted(),.36,self.clock[0])
        self.clock[0]=100.37
        self.deliver_ack(.37)
        self.assertIsNone(self.node.teleop_guard())
        self.assertIsNone(self.relay.owner)

    def test_repeated_release_and_timeout_recovery_keep_reclutch_available(self):
        from scipy.spatial.transform import Rotation
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from deictic_control.node import duration_msg

        self.node.status_pub = Recorder()
        delivered = 0
        sequence = 0
        targets = np.array([model.fk(q) for model, q in zip(
            self.node.teleop.models, np.array(self.current).reshape(2, 4))])
        targets[:, 0, 3] += .01
        positions, rotations = self.node.teleop.mapping.controller_poses(targets)
        quaternions = Rotation.from_matrix(rotations).as_quat()

        def advance(seconds):
            self.clock[0] += seconds
            state = JointState(name=list(BOTH_ARM_JOINTS), position=self.current)
            duration_msg(self.clock[0], state.header.stamp)
            self.node.on_joints(state)
            self.relay.feedback(BOTH_ARM_JOINTS, self.current, self.node.monotonic())
            self.deliver_ack(self.node.monotonic())

        def send(held):
            nonlocal sequence
            message = dict(schema_version=3, session_id='reclutch_client', sequence=sequence,
                           stamp=self.clock[0], frame_id='teleop_body', clutch=held,
                           left_tracked=True, right_tracked=True,
                           left_position=positions[0].tolist(), right_position=positions[1].tolist(),
                           left_rotation=quaternions[0].tolist(), right_rotation=quaternions[1].tolist())
            sequence += 1
            self.node.on_teleop(String(data=json.dumps(message)))

        def forward_commands():
            nonlocal delivered
            for message in self.node.teleop_pub.messages[delivered:]:
                self.relay.teleop(json.loads(message.data), self.node.monotonic(), self.clock[0])
            delivered = len(self.node.teleop_pub.messages)

        # Use the actual controller output and RelayControl acknowledgement in
        # both directions, including delayed active acknowledgements at release.
        advance(.01)
        forward_commands()
        for cycle, stop in enumerate(('release', 'release', 'timeout', 'release', 'timeout', 'release')):
            with self.subTest(cycle=cycle, stop=stop):
                advance(.01)
                send(False)
                self.assertTrue(json.loads(self.node.status_pub.messages[-1].data)['teleop_ready'])
                advance(.01)
                send(True)
                self.assertTrue(self.node.teleop.active, self.node.teleop.reason)
                forward_commands()
                self.assertTrue(self.relay.status(self.node.monotonic(), self.clock[0], True)['active'])
                # Remain active beyond the acquisition acknowledgement grace.
                for _ in range(6):
                    advance(.05)
                    send(True)
                    self.node.teleop_tick()
                    self.assertTrue(self.node.teleop.active, self.node.teleop.reason)
                    forward_commands()
                    self.current = self.relay.sample(self.node.monotonic(), self.clock[0])
                if stop == 'release':
                    advance(.01)
                    send(False)
                    self.assertTrue(json.loads(self.node.status_pub.messages[-1].data)['teleop_ready'])
                else:
                    advance(.31)  # Fresh feedback; input/relay command lease expires.
                    self.node.teleop_tick()
                    self.assertFalse(self.node.teleop.armed)
                self.assertFalse(self.node.teleop.active)
                forward_commands()
                self.assertIsNone(self.relay.owner)
                advance(.01)
                if stop == 'timeout':
                    send(True)
                    self.assertFalse(self.node.teleop.active, 'Held input must not recover a fault')
                send(False)
                self.assertTrue(json.loads(self.node.status_pub.messages[-1].data)['teleop_ready'])
                # A normal released heartbeat keeps readiness available through
                # an idle interval, rather than requiring a process restart.
                for _ in range(6):
                    advance(.1)
                    send(False)
                    self.node.teleop_tick()
                    self.node.publish_status()
                    self.assertTrue(json.loads(self.node.status_pub.messages[-1].data)['teleop_ready'])


if __name__ == '__main__':
    unittest.main()
