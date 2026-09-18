#!/usr/bin/env python3
"""Record the live Unity/ROS/Isaac teleoperation path without sending commands."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from k1_model import URDF
from deictic_control.kinematics import ArmModel


class Observer(Node):
    def __init__(self):
        super().__init__('deictic_teleop_observer')
        self.models = {
            'left': ArmModel(URDF, tip='left_elbow_yaw_link', tool_offset=(0., .10, 0.)),
            'right': ArmModel(URDF),
        }
        self.records = []
        self.latest = {}
        self.active = False
        self.clutches = 0
        self.releases = 0
        self.tips = {'left': [], 'right': []}
        self.create_subscription(String, '/deictic/teleop/input', self.input, 20)
        self.create_subscription(String, '/k1/teleop/command', self.command, 20)
        self.create_subscription(String, '/k1/teleop/relay_status', self.relay, 20)
        self.create_subscription(String, '/deictic/status', self.status, 20)
        self.create_subscription(JointState, '/joint_states', self.joints, 5)

    def append(self, kind, value):
        self.records.append({'received_unix': time.time(), 'kind': kind, 'value': value})

    def parse(self, message, kind):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError('expected object')
            return data
        except (ValueError, TypeError) as error:
            self.append('parse_error', {'topic': kind, 'error': str(error)})
            return None

    def input(self, message):
        data = self.parse(message, 'input')
        if data is None:
            return
        active = data.get('clutch') is True
        if active and not self.active:
            self.clutches += 1
        if not active and self.active:
            self.releases += 1
        self.active = active
        self.append('input', data)

    def command(self, message):
        data = self.parse(message, 'command')
        if data is not None:
            self.append('command', data)

    def status(self, message):
        data = self.parse(message, 'status')
        if data is not None:
            self.append('status', {key: data.get(key) for key in (
                'stamp', 'teleop_active', 'teleop_ready', 'teleop_state', 'teleop_reason',
                'teleop_last_fault', 'teleop_limited', 'teleop_position_error',
                'teleop_orientation_error', 'teleop_protocol_version', 'teleop_translation_scale',
                'can_commit', 'reason', 'operation', 'executing')})

    def relay(self, message):
        data = self.parse(message, 'relay')
        if data is not None:
            self.append('relay', data)

    def joints(self, message):
        values = dict(zip(message.name, message.position))
        positions, tips = {}, {}
        for side, model in self.models.items():
            if not all(name in values for name in model.names):
                return
            q = np.asarray([values[name] for name in model.names])
            if not np.all(np.isfinite(q)):
                return
            positions[side] = q.tolist()
            tips[side] = model.fk(q)[:3, 3].tolist()
            self.tips[side].append(tips[side])
        self.latest = tips
        self.append('joints', {'stamp': message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                               'positions': positions, 'tips_base': tips})

    def summary(self):
        counts = {kind: sum(record['kind'] == kind for record in self.records)
                  for kind in ('input', 'command', 'status', 'relay', 'joints', 'parse_error')}
        # Total displacement from the first measured sample is evidence of
        # motion only; it is not an assertion that IK followed a specific path.
        motion = {}
        for side, points in self.tips.items():
            a = np.asarray(points)
            motion[side] = float(np.max(np.linalg.norm(a-a[0], axis=1))) if len(a) else 0.
        return {'counts': counts, 'clutch_entries': self.clutches,
                'clutch_releases': self.releases, 'max_tool_displacement_m': motion,
                'both_arms_moved': all(distance > .01 for distance in motion.values()),
                'scope': 'Read-only observation; no goals or motion commands published'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60.)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--require-motion', action='store_true')
    args = parser.parse_args()
    if not np.isfinite(args.duration) or not 1 <= args.duration <= 600:
        parser.error('duration must be 1..600 seconds')
    rclpy.init()
    node = Observer()
    started = time.time()
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        summary = node.summary()
        report = {'started_unix': started, 'ended_unix': time.time(),
                  'summary': summary, 'records': node.records}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print(json.dumps({'output': str(args.output), **summary}), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return int(args.require_motion and not summary['both_arms_moved'])


if __name__ == '__main__':
    raise SystemExit(main())
