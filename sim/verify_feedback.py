#!/usr/bin/env python3
"""Read-only check that Isaac measured joints and reported tool pose agree with URDF FK."""
import json
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"ros2/src/deictic_control"))
from deictic_control.kinematics import ArmModel
from k1_model import ARM_JOINTS,URDF


def main():
    model = ArmModel(str(URDF))
    states,errors = {},[]
    rclpy.init()
    node = rclpy.create_node("k1_readonly_feedback_verifier")
    def key(header):
        return header.stamp.sec,header.stamp.nanosec
    def state(message):
        states[key(message.header)] = dict(zip(message.name,message.position))
        if len(states)>120:
            states.pop(next(iter(states)))
    def pose(message):
        values = states.get(key(message.header))
        if values is not None and all(name in values for name in ARM_JOINTS):
            actual = np.array([message.pose.position.x,message.pose.position.y,message.pose.position.z])
            expected = model.fk([values[name] for name in ARM_JOINTS])[:3,3]
            errors.append(float(np.linalg.norm(actual-expected)))
    node.create_subscription(JointState,"/joint_states",state,10)
    node.create_subscription(PoseStamped,"/k1/end_effector_pose",pose,10)
    deadline = time.monotonic()+15
    while len(errors)<20 and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
    result = {"matched_samples":len(errors),"max_fk_error_m":max(errors) if errors else None}
    print(json.dumps(result))
    node.destroy_node()
    rclpy.shutdown()
    if len(errors)<5 or max(errors)>.002:
        raise SystemExit("Measured feedback did not agree with source URDF within 2 mm")


if __name__ == "__main__":
    main()
