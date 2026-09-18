"""Preview/execution contract in an isolated ROS domain; no live controller."""
import copy
import json
import threading
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from deictic_control.node import DeicticControl, duration_msg


class Recorder:
    def __init__(self):
        self.messages = []
        self.subscribers = 1

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))

    def get_subscription_count(self):
        return self.subscribers


@pytest.fixture
def controller():
    root = Path(__file__).resolve().parents[4]
    rclpy.init(args=['--ros-args', '-p', f'urdf:={root / "models/K1/K1_22dof.urdf"}',
                    '-p', 'synthetic_test:=true', '-p', 'synthetic_reference_backend:=true',
                    '-p', 'allow_execution:=true'], domain_id=201)
    node = DeicticControl()
    clock = [100.]
    node.now = lambda: clock[0]
    node.command_pub, node.preview_pub = Recorder(), Recorder()
    feedback = JointState(name=node.arm.names, position=[0., .5, 0., 0.])
    duration_msg(clock[0], feedback.header.stamp)
    node.on_joints(feedback)
    node.synthetic_observation()
    try:
        yield node, clock
    finally:
        node.destroy_node()
        rclpy.shutdown()


def begin_goal(node, x=.12):
    message = PoseStamped()
    message.header.frame_id = 'base_link'
    duration_msg(node.now(), message.header.stamp)
    message.pose.position.x, message.pose.position.y, message.pose.position.z = x, -.25, .02
    message.pose.orientation.w = 1.
    node.on_goal(message)
    return (message.header.stamp.sec, message.header.stamp.nanosec)


def finish_planning(node, timeout=5.):
    deadline = time.monotonic()+timeout
    while node.planner_future is not None or node.pending_request is not None:
        assert time.monotonic() < deadline, 'Planning worker did not finish'
        node.poll_planner()
        time.sleep(.001)


def goal(node, x=.12):
    begin_goal(node, x)
    finish_planning(node)
    assert node.plan is not None, node.operation
    return node.plan_goal_stamp


def execute(node, token):
    node.on_execute_request(String(data=json.dumps(dict(schema_version=1,
        goal_stamp_sec=token[0], goal_stamp_nanosec=token[1]))))


def test_delayed_execute_cannot_execute_or_erase_newer_preview(controller):
    node, clock = controller
    token_a = goal(node, .10)
    clock[0] += .01
    token_b = goal(node, .12)
    preview = node.preview_pub.messages[-1]
    assert (preview.header.stamp.sec, preview.header.stamp.nanosec) == token_b
    planned = node.plan.positions.copy()
    execute(node, token_a)
    assert not node.command_pub.messages and node.plan_goal_stamp == token_b
    node.on_execute(Empty())  # Legacy Empty cannot bypass exact correlation.
    assert not node.command_pub.messages and node.plan_goal_stamp == token_b
    execute(node, token_b)
    assert len(node.command_pub.messages) == 1 and node.plan is None
    command = node.command_pub.messages[0]
    assert command.header.stamp.sec == command.header.stamp.nanosec == 0
    np.testing.assert_array_equal([point.positions for point in command.points], planned)
    execute(node, token_b)  # The consumed preview cannot be replayed.
    assert len(node.command_pub.messages) == 1


@pytest.mark.parametrize('guard', ['stale_registration', 'stale_feedback', 'expired_preview',
                                  'changed_alignment', 'changed_epoch', 'quality_lost',
                                  'moved_robot', 'missing_controller'])
def test_matching_token_still_requires_unchanged_fresh_preview(controller, guard):
    node, clock = controller
    token = goal(node)
    if guard == 'stale_registration':
        clock[0] += 1.1
        node.joint_time = clock[0]
    elif guard == 'stale_feedback':
        clock[0] += .6
        node.synthetic_observation()
    elif guard == 'expired_preview':
        node.plan_time -= 11.
    elif guard == 'changed_alignment':
        node.fusion.transform[0, 3] += .006
    elif guard == 'changed_epoch':
        node.fusion.epoch += 1
    elif guard == 'quality_lost':
        node.fusion.invalidate('registration_quality_low')
    elif guard == 'moved_robot':
        node.joints[0] += .04
    elif guard == 'missing_controller':
        node.command_pub.subscribers = 0
    execute(node, token)
    assert not node.command_pub.messages
    assert node.plan is None
    assert node.operation.startswith('execute_rejected:'), node.operation


def test_small_alignment_update_preserves_exact_preview_and_token(controller):
    node, clock = controller
    token = goal(node)
    version = node.fusion.version
    planned = node.plan.positions.copy()
    original = node.plan_transform.copy()
    clock[0] += .1
    node.synthetic_transform[0, 3] += .004
    node.synthetic_observation()
    assert node.fusion.version > version and node.plan_goal_stamp == token
    np.testing.assert_array_equal(node.plan_transform, original)
    np.testing.assert_array_equal(node.plan.positions, planned)
    execute(node, token)
    assert len(node.command_pub.messages) == 1
    np.testing.assert_array_equal([p.positions for p in node.command_pub.messages[0].points], planned)


def test_preview_binding_rejects_cumulative_drift_from_original_transform(controller):
    node, clock = controller
    goal(node)
    clock[0] += .1
    node.synthetic_transform[0, 3] += .003
    node.synthetic_observation()
    assert node.plan is not None
    clock[0] += .1
    node.synthetic_transform[0, 3] += .003
    node.synthetic_observation()
    assert node.fusion.healthy and node.plan is None
    assert node.operation == 'preview_invalidated: alignment_moved_target_recommit_goal'
    assert not node.preview_pub.messages[-1].points
    assert not node.command_pub.messages


def test_rotation_can_move_preview_target_outside_translation_tolerance(controller):
    from deictic_control.se3 import rotation
    node, clock = controller
    goal(node)
    original_translation = node.synthetic_transform[:3, 3].copy()
    clock[0] += .1
    node.synthetic_transform[:3, :3] = rotation([0, 0, 1], .006)
    node.synthetic_observation()
    np.testing.assert_array_equal(node.synthetic_transform[:3, 3], original_translation)
    assert node.fusion.healthy and node.plan is None
    assert node.operation == 'preview_invalidated: alignment_moved_target_recommit_goal'


@pytest.mark.parametrize('invalid_tolerance', [float('nan'), float('inf')])
def test_runtime_nonfinite_tolerance_cannot_disable_preview_binding(controller, invalid_tolerance):
    from rclpy.parameter import Parameter
    node, _ = controller
    token = goal(node)
    node.fusion.transform[0, 3] += .02
    result = node.set_parameters([Parameter('preview_target_tolerance', value=invalid_tolerance)])
    assert result[0].successful  # Exercise a value set after constructor validation.
    execute(node, token)
    assert not node.command_pub.messages and node.plan is None
    assert node.operation == 'execute_rejected: invalid_preview_target_tolerance'


def test_out_of_order_observation_freezes_transform_and_clears_preview(controller):
    node, _ = controller
    goal(node)
    frozen = node.fusion.transform.copy()
    identity = [0, 0, 0, 0, 0, 0, 1]
    node.on_registration(String(data=json.dumps(dict(schema_version=1, stamp=99.9,
        headset_pose=identity, camera_pose=identity, camera_from_headset=identity,
        inliers=100, matches=100, median_reprojection_error=0., source='simulation_ground_truth'))))
    assert node.plan is None and not node.fusion.healthy
    assert node.fusion.reason == 'out_of_order_registration'
    np.testing.assert_array_equal(node.fusion.transform, frozen)
    assert not node.command_pub.messages


@pytest.fixture
def blocked_planner(controller, monkeypatch):
    import deictic_control.node as module
    node, clock = controller
    entered, release = threading.Event(), threading.Event()
    real_solve = module.solve_request
    calls = []

    def solve(request):
        calls.append(request)
        entered.set()
        assert release.wait(3), 'Test did not release worker'
        return real_solve(request)

    monkeypatch.setattr(module, 'solve_request', solve)
    try:
        yield node, clock, entered, release, calls
    finally:
        release.set()


def test_feedback_callbacks_are_serviced_while_planner_is_blocked(blocked_planner):
    node, clock, entered, release, _ = blocked_planner
    token = begin_goal(node)
    assert entered.wait(1) and node.operation == 'planning_position_only'
    publisher = node.create_publisher(JointState, '/joint_states', 10)
    initial = node.joints.copy()
    feedback = JointState(name=node.arm.names, position=(initial+np.array([.005, 0, 0, 0])).tolist())
    duration_msg(node.now(), feedback.header.stamp)
    deadline = time.monotonic()+1
    while time.monotonic() < deadline and np.max(np.abs(node.joints-initial)) < .004:
        publisher.publish(feedback)
        rclpy.spin_once(node, timeout_sec=.01)
    assert np.max(np.abs(node.joints-initial)) > .004
    assert node.planner_future is not None and not node.planner_future.done()
    assert node.plan is None and not node.command_pub.messages
    release.set()
    finish_planning(node)
    assert node.plan_goal_stamp == token and node.plan is not None


def test_cancelled_worker_cannot_restore_preview(blocked_planner):
    node, _, entered, release, calls = blocked_planner
    begin_goal(node)
    assert entered.wait(1)
    node.on_cancel(None)
    assert calls[0].cancelled.is_set()
    sent_before = len(node.command_pub.messages)
    release.set()
    finish_planning(node)
    assert node.plan is None and node.operation == 'cancel_hold_sent'
    assert len(node.command_pub.messages) == sent_before
    assert not any(m.points for m in node.preview_pub.messages)


def test_superseded_jobs_keep_only_latest_request_and_exact_token(blocked_planner):
    node, clock, entered, release, calls = blocked_planner
    begin_goal(node, .10)
    assert entered.wait(1)
    for x in (.11, .12, .13):
        clock[0] += .001
        latest = begin_goal(node, x)
    assert len(calls) == 1 and node.pending_request.goal_stamp == latest
    assert not any(m.points for m in node.preview_pub.messages)
    release.set()
    finish_planning(node)
    assert len(calls) == 2 and node.plan_goal_stamp == latest
    previews = [m for m in node.preview_pub.messages if m.points]
    assert len(previews) == 1
    assert (previews[0].header.stamp.sec, previews[0].header.stamp.nanosec) == latest


@pytest.mark.parametrize('guard', ['joint_stale', 'registration_stale', 'epoch', 'target_drift',
                                  'start_drift', 'deadline', 'failure_then_recovery'])
def test_worker_completion_rechecks_original_context(blocked_planner, guard):
    from std_msgs.msg import String
    node, clock, entered, release, calls = blocked_planner
    begin_goal(node)
    assert entered.wait(1)
    original = calls[0].transform.copy()
    assert not calls[0].joints.flags.writeable and not calls[0].target.flags.writeable
    if guard == 'joint_stale':
        node.joint_time -= .6
    elif guard == 'registration_stale':
        clock[0] += 1.1
        node.joint_time = clock[0]
    elif guard == 'epoch':
        node.fusion.epoch += 1
    elif guard == 'target_drift':
        node.fusion.transform[0, 3] += .006
    elif guard == 'start_drift':
        node.joints[0] += .04
    elif guard == 'deadline':
        node.planner_request = replace(node.planner_request, deadline=time.monotonic()-1)
    elif guard == 'failure_then_recovery':
        node.on_failure(String(data=json.dumps({'reason': 'temporary_visual_failure'})))
        clock[0] += .01
        node.synthetic_observation()
        assert node.fusion.healthy
    np.testing.assert_array_equal(calls[0].transform, original)
    release.set()
    finish_planning(node)
    assert node.plan is None and not any(m.points for m in node.preview_pub.messages)
    assert node.operation != 'preview_ready_position_only'
    assert not node.command_pub.messages


def test_stale_status_reports_preview_invalidation(controller):
    node, clock = controller
    goal(node)
    node.joint_time -= .6
    node.publish_status()
    assert node.plan is None
    assert node.operation == 'preview_invalidated: joint_feedback_stale'


@pytest.mark.parametrize('invalid_timeout', [float('nan'), float('inf'), -float('inf'), 0., -1.])
def test_invalid_planning_timeout_rejects_before_worker_submission(controller, invalid_timeout):
    from rclpy.parameter import Parameter
    node, _ = controller
    assert node.set_parameters([Parameter('plan_timeout', value=invalid_timeout)])[0].successful
    begin_goal(node)
    assert node.planner_future is None and node.pending_request is None and node.plan is None
    assert node.operation == 'goal_rejected: invalid_plan_timeout'
    assert not node.command_pub.messages


def test_timeout_changed_to_nan_cannot_disable_execution_expiry(controller):
    from rclpy.parameter import Parameter
    node, _ = controller
    token = goal(node)
    assert node.set_parameters([Parameter('plan_timeout', value=float('nan'))])[0].successful
    execute(node, token)
    assert not node.command_pub.messages and node.plan is None
    assert node.operation == 'execute_rejected: invalid_plan_timeout'
