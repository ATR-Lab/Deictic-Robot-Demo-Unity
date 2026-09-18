#!/usr/bin/env python3
"""Explicit checked simulator joint maintenance; separate from learned goal control."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from deictic_control.kinematics import ArmModel, make_joint_plan
from deictic_control.node import duration_msg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Send the checked maintenance trajectory to the live simulator')
    parser.add_argument('--joint-goal', type=float, nargs=4, default=[0., .5, 0., 0.],
                        metavar=('SHOULDER_PITCH', 'SHOULDER_ROLL', 'ELBOW_PITCH', 'ELBOW_YAW'),
                        help='Explicit simulation-only joint goal in radians; defaults to the rest pose')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    arm = ArmModel(root/'models/K1/K1_22dof.urdf')
    goal = np.asarray(args.joint_goal, dtype=float)
    if not np.all(np.isfinite(goal)):
        parser.error('--joint-goal must contain four finite angles')
    rclpy.init()
    node = Node('deictic_sim_maintenance_return')
    state = {}

    def receive(message):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        lookup = dict(zip(message.name, message.position))
        if -.05 <= time.time()-stamp <= .5 and all(n in lookup for n in arm.names):
            q = np.array([lookup[n] for n in arm.names])
            if np.all(np.isfinite(q)):
                state.update(q=q, received=time.monotonic())

    sub = node.create_subscription(JointState, '/joint_states', receive, 10)
    publisher = node.create_publisher(JointTrajectory, '/k1/arm_controller/joint_trajectory', 10)
    until = time.monotonic()+8
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.1)
        if state and publisher.get_subscription_count() and 'k1_fixed_base_isaac' in node.get_node_names():
            break
    try:
        if not state or time.monotonic()-state['received'] > .5:
            raise RuntimeError('Fresh measured simulator joints unavailable')
        if 'k1_fixed_base_isaac' not in node.get_node_names() or not publisher.get_subscription_count():
            raise RuntimeError('Expected Isaac simulator and trajectory relay unavailable')
        plan = make_joint_plan(arm, state['q'], goal)
        print(json.dumps({'mode': 'simulation_maintenance_joint_move', 'execute': args.execute,
                          'start': state['q'].tolist(), 'goal': goal.tolist(),
                          'duration_s': float(plan.times[-1]), 'points': len(plan.times)}), flush=True)
        if not args.execute:
            return
        message = JointTrajectory()
        message.joint_names = plan.names
        for t, q, v, a in zip(plan.times, plan.positions, plan.velocities, plan.accelerations):
            point = JointTrajectoryPoint(positions=q.tolist(), velocities=v.tolist(), accelerations=a.tolist())
            duration_msg(t, point.time_from_start)
            message.points.append(point)
        publisher.publish(message)
        started = time.monotonic()
        while time.monotonic()-started < plan.times[-1]+5:
            rclpy.spin_once(node, timeout_sec=.05)
            if time.monotonic()-state['received'] > .5:
                raise RuntimeError('Simulator feedback became stale during maintenance return')
            error = float(np.max(np.abs(state['q']-goal)))
            if time.monotonic()-started >= plan.times[-1] and error <= .025:
                print(json.dumps({'completed': True, 'measured': state['q'].tolist(), 'max_joint_error_rad': error}), flush=True)
                return
        raise RuntimeError('Maintenance return did not converge within .025 rad')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
