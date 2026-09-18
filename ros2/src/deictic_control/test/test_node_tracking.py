"""Tracking lifecycle regression in an isolated ROS domain; never drives Isaac."""
import json
from pathlib import Path
import pytest

rclpy = pytest.importorskip('rclpy')
from std_msgs.msg import String
from deictic_control.node import DeicticControl


def test_pause_resume_and_recenter_require_fresh_registration():
    root = Path(__file__).resolve().parents[4]
    rclpy.init(args=['--ros-args', '-p', f'urdf:={root / "models/K1/K1_22dof.urdf"}',
                    '-p', 'synthetic_test:=true', '-p', 'synthetic_reference_backend:=true'], domain_id=201)
    node = DeicticControl()
    try:
        node.synthetic_observation()
        assert node.fusion.healthy
        original = node.fusion.transform.copy()

        def event(kind, reason):
            node.on_failure(String(data=json.dumps(dict(schema_version=1, stamp=node.now(),
                source='quest_tracking', event_type=kind, reason=reason))))

        event('paused', 'Application paused')
        assert node.tracking_inhibited and not node.fusion.healthy
        first_epoch = node.fusion.epoch
        node.synthetic_observation()
        assert not node.fusion.healthy
        assert (node.fusion.transform == original).all()

        event('resumed', 'Application resumed')
        assert not node.tracking_inhibited and not node.fusion.healthy
        assert node.fusion.epoch > first_epoch
        node.synthetic_observation()
        assert node.fusion.healthy

        event('recentered', 'Headset recentered')
        assert not node.tracking_inhibited and not node.fusion.healthy
        node.synthetic_observation()
        assert node.fusion.healthy
    finally:
        node.destroy_node()
        rclpy.shutdown()
