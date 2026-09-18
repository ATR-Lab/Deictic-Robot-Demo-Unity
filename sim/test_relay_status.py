"""Exercise real ROS message adapters without starting ROS nodes or services."""
import json
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from command_control import RelayControl
from k1_model import BOTH_ARM_JOINTS

try:
    from sensor_msgs.msg import JointState
    from trajectory_relay import Relay
except ImportError:
    Relay = None


@unittest.skipIf(Relay is None, 'Source ROS to test relay message adapters')
class RelayStatusAdapterTests(unittest.TestCase):
    def setUp(self):
        self.messages, self.errors = [], []
        self.node = SimpleNamespace(
            control=RelayControl(), last_state_stamp=0., wall_now=lambda: 100.,
            publisher=SimpleNamespace(get_subscription_count=lambda: 1),
            status_publisher=SimpleNamespace(publish=self.messages.append),
            get_logger=lambda: SimpleNamespace(error=self.errors.append))

    def state(self, stamp):
        message = JointState(name=list(BOTH_ARM_JOINTS), position=[0., 0., 0., 0., 0., .5, 0., 0.])
        message.header.stamp.sec = int(stamp)
        message.header.stamp.nanosec = int(round((stamp-int(stamp))*1e9))
        return message

    def test_publishes_actual_string_with_nulls_and_ready_state(self):
        with patch('trajectory_relay.time.monotonic', return_value=10.):
            Relay.publish_status(self.node)
            self.assertFalse(json.loads(self.messages[-1].data)['ready'])
            Relay.state(self.node, self.state(100.))
            Relay.publish_status(self.node)
        value = json.loads(self.messages[-1].data)
        self.assertEqual(value['schema_version'], 1)
        self.assertEqual(value['stamp'], 100.)
        self.assertTrue(value['ready'])
        self.assertFalse(value['active'])
        self.assertIsNone(value['owner_session_id'])
        self.assertIsNone(value['last_sequence'])
        self.assertIsNone(value['command_age'])

    def test_delayed_feedback_preserves_capture_age_and_replays_cannot_refresh_it(self):
        with patch('trajectory_relay.time.monotonic', return_value=10.):
            Relay.state(self.node, self.state(99.6))
        self.assertAlmostEqual(self.node.control.state_received, 9.6)
        self.node.wall_now = lambda: 100.05
        with patch('trajectory_relay.time.monotonic', return_value=10.05):
            Relay.state(self.node, self.state(99.6))
        self.assertEqual(len(self.errors), 1)
        self.assertAlmostEqual(self.node.control.state_received, 9.6)
        self.node.wall_now = lambda: 100.11
        with patch('trajectory_relay.time.monotonic', return_value=10.11):
            Relay.publish_status(self.node)
        self.assertFalse(json.loads(self.messages[-1].data)['ready'])


if __name__ == '__main__':
    unittest.main()
