"""Read-only live scoring for the declared static Isaac fixture, never physical data."""
import argparse
import json
import math
import sys
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'ros2/src/deictic_registration'))
sys.path.insert(0, str(root / 'ros2/src/deictic_control'))
from deictic_registration.core import pose_to_matrix
from deictic_control.fusion import FusionConfig

cfg = FusionConfig()
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--synthetic-scene', required=True, choices=['coarse_fixture'])
parser.add_argument('--duration', type=float, default=60.)
parser.add_argument('--max-age-s', type=float, default=3.)
parser.add_argument('--output', type=Path, default=root / 'sim/artifacts/live_candidate_registration.json')
args = parser.parse_args()
if not math.isfinite(args.duration) or args.duration <= 0:
    parser.error('--duration must be finite and positive')
if not math.isfinite(args.max_age_s) or args.max_age_s <= 0:
    parser.error('--max-age-s must be finite and positive')
observations, failures = [], []
truth = np.eye(4)
truth[:3, 3] = [-1.2, 0, -.9]
targets = np.array([[x, -.25, .02, 1] for x in (.08, .10, .12, .14, .16)])
rclpy.init()
node = Node('registration_candidate_observer')


def observation(message):
    value = json.loads(message.data)
    now = node.get_clock().now().nanoseconds * 1e-9
    logit = cfg.alpha * value['inliers'] / value['matches'] - cfg.beta * value['median_reprojection_error'] - cfg.gamma
    confidence = float(1 / (1 + np.exp(-np.clip(logit, -60, 60))))
    x = pose_to_matrix(value['camera_pose']) @ pose_to_matrix(value['camera_from_headset']) @ np.linalg.inv(pose_to_matrix(value['headset_pose']))
    # Independent known simulator frame used only to score the already published estimate.
    predicted = targets @ np.linalg.inv(truth).T @ x.T
    errors = np.linalg.norm(predicted[:, :3] - targets[:, :3], axis=1)
    record = {'received_stamp': now, 'capture_age_s': now - value['stamp'],
              'confidence': confidence, 'max_target_error_m': float(errors.max()),
              'target_errors_m': errors.tolist(), 'observation': value,
              'passes': bool(confidence >= cfg.threshold and errors.max() < .03
                             and 0 <= now - value['stamp'] < args.max_age_s)}
    observations.append(record)
    print(json.dumps({k: v for k, v in record.items() if k not in ('observation', 'target_errors_m')}), flush=True)


def failure(message):
    value = json.loads(message.data)
    failures.append(value)
    print(json.dumps({'failure': value}), flush=True)


subscriptions = [node.create_subscription(String, '/deictic/registration_candidate', observation, 20),
                 node.create_subscription(String, '/deictic/registration_candidate_failure', failure, 20)]
start = time.monotonic()
while time.monotonic() - start < args.duration:
    rclpy.spin_once(node, timeout_sec=.1)
report = {'scope': 'static simulation candidate stream; read-only scoring',
          'duration_s': args.duration, 'synthetic_scene': args.synthetic_scene,
          'maximum_capture_age_s': args.max_age_s,
          'observations': observations, 'failures': failures,
          'passing_observations': sum(o['passes'] for o in observations),
          'total_observations': len(observations)}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({k: v for k, v in report.items() if k not in ('observations', 'failures')}), flush=True)
node.destroy_node()
rclpy.shutdown()
