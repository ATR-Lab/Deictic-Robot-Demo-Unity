"""Executor-only teleop arbitration and holds in an isolated ROS domain."""
import copy
import json
import time
from pathlib import Path
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from deictic_control.node import DeicticControl, duration_msg


class Recorder:
    def __init__(self):
        self.messages, self.subscribers = [], 1
    def publish(self, message): self.messages.append(copy.deepcopy(message))
    def get_subscription_count(self): return self.subscribers


@pytest.fixture
def node():
    root = Path(__file__).resolve().parents[4]
    rclpy.init(args=['--ros-args', '-p', f'urdf:={root / "models/K1/K1_22dof.urdf"}',
                    '-p', 'synthetic_test:=true', '-p', 'synthetic_reference_backend:=true',
                    '-p', 'allow_execution:=true', '-p', 'allow_teleoperation:=true'], domain_id=203)
    result = DeicticControl()
    clock = [100.]
    result.now = result.monotonic = lambda: clock[0]
    result.command_pub, result.teleop_pub = Recorder(), Recorder()
    result.preview_pub, result.status_pub = Recorder(), Recorder()
    feedback(result)
    result.teleop_pub.messages.clear()
    try:
        yield result, clock
    finally:
        result.destroy_node()
        rclpy.shutdown()


def feedback(node, values=None):
    values = [0., 0., 0., 0., 0., .5, 0., 0.] if values is None else values
    message = JointState(name=node.both_arm_names, position=values)
    duration_msg(node.now(), message.header.stamp)
    node.on_joints(message)
    relay_ack(node)


def relay_ack(node, **extra):
    data = dict(schema_version=1, stamp=node.now(), ready=True, active=node.teleop.active,
                owner_session_id=node.teleop_session_id if node.teleop.active else None,
                last_sequence=node.teleop_sequence-1 if node.teleop.active else None,
                reason='active' if node.teleop.active else 'idle', command_age=0.)
    data.update(extra)
    node.on_teleop_relay_status(String(data=json.dumps(data)))

def input_message(node, sequence, held, **extra):
    data = dict(schema_version=2, session_id='client', sequence=sequence, stamp=node.now(),
                frame_id='teleop_head', clutch=held, left_tracked=True, right_tracked=True,
                left_position=[0., .2, .1], right_position=[0., -.2, .1],
                left_rotation=[0.,0.,0.,1.], right_rotation=[0.,0.,0.,1.])
    data.update(extra)
    node.on_teleop(String(data=json.dumps(data)))


def begin(node, clock):
    input_message(node, 0, False)
    assert json.loads(node.status_pub.messages[-1].data)['teleop_ready']
    clock[0] += .01
    input_message(node, 1, True)
    assert node.teleop.active, node.teleop.reason


def test_both_arm_teleop_runs_without_learned_alignment_and_preserves_deictic_gate(node):
    controller, clock = node
    assert not controller.fusion.healthy
    begin(controller, clock)
    transitions = [json.loads(message.data) for message in controller.teleop_pub.messages]
    assert [message['active'] for message in transitions] == [False, True]
    assert transitions[0]['sequence'] < transitions[1]['sequence']
    np.testing.assert_allclose(transitions[1]['positions'], controller.teleop_joints.reshape(-1))
    clock[0] += .05
    input_message(controller, 2, True, left_position=[.01, .2, .1], right_position=[.01, -.2, .1])
    controller.teleop_tick()
    controller.publish_status()
    status = json.loads(controller.status_pub.messages[-1].data)
    assert status['teleop_ready'] and status['teleop_active'] and not status['can_commit']
    assert status['teleop_state'] == 'active'
    command = json.loads(controller.teleop_pub.messages[-1].data)
    assert command['schema_version'] == 1 and command['active']
    assert command['names'] == controller.left_arm.names+controller.arm.names
    assert len(command['positions']) == 8 and command['stamp'] == controller.now()
    assert not controller.command_pub.messages


def test_clutch_cancels_preview_and_execution_and_blocks_goal_and_execute(node):
    controller, clock = node
    controller.synthetic_observation()
    controller.plan = object()
    controller.active_goal = np.zeros(4)
    controller.active_started, controller.active_duration = controller.now(), 10.
    generation = controller.planning_generation
    begin(controller, clock)
    assert controller.plan is None and controller.active_goal is None
    assert controller.planning_generation > generation and not controller.preview_pub.messages[-1].points
    goal = PoseStamped()
    goal.header.frame_id = 'base_link'; duration_msg(controller.now(), goal.header.stamp)
    goal.pose.orientation.w = 1.
    controller.on_goal(goal)
    controller.execute_current_plan()
    assert controller.teleop.active and not controller.command_pub.messages
    assert controller.pending_request is controller.planner_future is None


@pytest.mark.parametrize('event', ['release', 'timeout', 'tracking', 'feedback', 'relay', 'disabled', 'cancel'])
def test_each_stop_holds_both_once_and_requires_release_before_reclutch(node, event):
    controller, clock = node
    begin(controller, clock)
    controller.teleop_pub.messages.clear()
    if event == 'release': input_message(controller, 2, False)
    elif event == 'tracking': input_message(controller, 2, True, right_tracked=False)
    elif event == 'cancel': controller.on_cancel(None)
    else:
        if event == 'timeout': clock[0] += .26
        if event == 'feedback': controller.teleop_joint_time -= 1.
        if event == 'relay': controller.teleop_pub.subscribers = 0
        if event == 'disabled': controller.set_parameters([Parameter('allow_execution', value=False)])
        controller.teleop_tick()
    assert not controller.teleop.active
    messages = [json.loads(message.data) for message in controller.teleop_pub.messages]
    assert len(messages) == 1 and messages[0]['active'] is False
    assert messages[0]['names'] == controller.both_arm_names
    np.testing.assert_allclose(messages[0]['positions'], controller.teleop_joints.reshape(-1))
    assert not controller.command_pub.messages
    controller.teleop_tick()
    assert len(controller.teleop_pub.messages) == 1
    if event != 'release':
        clock[0] += .01
        feedback(controller)
        controller.teleop_pub.subscribers = 1
        controller.set_parameters([Parameter('allow_execution', value=True)])
        input_message(controller, 3, True)
        assert not controller.teleop.active
        input_message(controller, 4, False)
        input_message(controller, 5, True)
        assert controller.teleop.active


def test_idle_false_heartbeats_do_not_cancel_deictic_trajectory(node):
    controller, _ = node
    controller.active_goal = np.ones(4)
    controller.active_started, controller.active_duration = controller.now(), 10.
    for sequence in range(5):
        input_message(controller, sequence, False)
    assert not controller.teleop_pub.messages and not controller.command_pub.messages
    assert controller.active_goal is not None


def test_missing_one_arm_or_zero_feedback_stamp_never_enables_teleop(node):
    controller, _ = node
    controller.teleop_joints = controller.teleop_joint_time = None
    controller.on_joints(JointState(name=controller.both_arm_names, position=[0., 0., 0., 0., 0., .5, 0., 0.]))
    assert controller.teleop_joint_time is None
    message = JointState(name=controller.arm.names, position=[0., .5, 0., 0.])
    duration_msg(controller.now(), message.header.stamp)
    controller.on_joints(message)
    input_message(controller, 0, False)
    assert not json.loads(controller.status_pub.messages[-1].data)['teleop_ready']
    input_message(controller, 1, True)
    assert not controller.teleop.active and not controller.teleop_pub.messages


def test_headset_epoch_event_holds_teleop_even_though_registration_failure_does_not(node):
    controller, clock = node
    begin(controller, clock)
    controller.on_failure(String(data=json.dumps(dict(reason='too_few_matches', source='superpoint_pnp'))))
    assert controller.teleop.active
    controller.teleop_pub.messages.clear()
    controller.on_failure(String(data=json.dumps(dict(schema_version=1, source='quest_tracking',
        stamp=controller.now(), reason='Head tracking lost', event_type='lost'))))
    assert not controller.teleop.active
    assert json.loads(controller.teleop_pub.messages[-1].data)['active'] is False


def test_shutdown_sends_both_arm_release(node):
    controller, clock = node
    begin(controller, clock)
    # Exercise shutdown's explicit stop separately; fixture then destroys resources.
    controller.stop_teleop('controller_shutdown')
    assert not controller.teleop.active
    assert json.loads(controller.teleop_pub.messages[-1].data)['active'] is False


def test_right_only_mock_remains_plannable_if_teleop_flag_is_accidentally_enabled(node):
    controller, _ = node
    controller.set_parameters([Parameter('mock_joint_states', value=True)])
    controller.teleop_joints = controller.teleop_joint_time = None
    controller.synthetic_observation()
    assert controller.param('allow_teleoperation')
    assert controller.teleop_guard() == 'teleop_mock_bimanual_unsupported'
    controller.can_plan()  # Right-only mock still supports the original workflow.
    goal = PoseStamped()
    goal.header.frame_id = 'base_link'; duration_msg(controller.now(), goal.header.stamp)
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = .12, -.25, .02
    goal.pose.orientation.w = 1.
    controller.on_goal(goal)
    deadline = time.monotonic()+5.
    while controller.planner_future is not None:
        assert time.monotonic() < deadline
        controller.poll_planner()
        time.sleep(.001)
    assert controller.plan is not None, controller.operation


@pytest.mark.parametrize('when', ['before_preview', 'before_execute'])
def test_legacy_preview_is_bound_to_held_left_arm_and_cannot_ignore_its_motion(node, when):
    controller, _ = node
    controller.synthetic_observation()
    goal = PoseStamped()
    goal.header.frame_id = 'base_link'; duration_msg(controller.now(), goal.header.stamp)
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = .12, -.25, .02
    goal.pose.orientation.w = 1.
    controller.on_goal(goal)
    request = controller.planner_request
    assert request.held_model is not None and request.held_joints is not None
    deadline = time.monotonic()+5.
    while not controller.planner_future.done():
        assert time.monotonic() < deadline
        time.sleep(.001)
    if when == 'before_preview': controller.teleop_joints[0, 0] += .04
    controller.poll_planner()
    if when == 'before_execute':
        assert controller.plan is not None, controller.operation
        controller.teleop_joints[0, 0] += .04
        controller.execute_current_plan()
    assert controller.plan is None and not controller.command_pub.messages
    assert 'held_left_arm_changed' in controller.operation


def test_v1_protocol_fault_survives_fresh_release_and_v2_status_exposes_scale(node):
    controller, clock = node
    input_message(controller, 0, False, schema_version=1)
    assert controller.teleop.last_fault == 'protocol_version_mismatch'
    input_message(controller, 1, False)
    status = json.loads(controller.status_pub.messages[-1].data)
    assert status['teleop_ready'] and status['teleop_relay_ready']
    assert status['teleop_protocol_version'] == 2
    assert status['teleop_translation_scale'] == pytest.approx(.558833641)
    assert status['teleop_last_fault'] == 'protocol_version_mismatch'
    assert status['teleop_position_error'] is None


@pytest.mark.parametrize('cause', ['missing', 'old_source', 'old_receipt', 'not_ready', 'foreign_owner'])
def test_relay_ack_required_before_acquisition(node, cause):
    controller, clock = node
    if cause == 'missing': controller.teleop_relay_status = None
    if cause == 'old_source': controller.teleop_relay_status['stamp'] -= .31
    if cause == 'old_receipt': controller.teleop_relay_received -= .31
    if cause == 'not_ready': controller.teleop_relay_status['ready'] = False
    if cause == 'foreign_owner':
        controller.teleop_relay_status.update(active=True,
            owner_session_id='11111111-1111-4111-8111-111111111111', last_sequence=1)
    input_message(controller, 0, False)
    input_message(controller, 1, True)
    assert not controller.teleop.active
    assert 'relay' in controller.teleop.reason
    assert not controller.teleop_pub.messages


def test_relay_lease_loss_after_acquisition_grace_holds_once_and_requires_release(node):
    controller, clock = node
    begin(controller, clock)
    clock[0] += .15
    feedback(controller)
    relay_ack(controller, active=False, owner_session_id=controller.teleop_session_id,
              last_sequence=controller.teleop_sequence-1, reason='expired')
    assert controller.teleop_guard() is None  # Acquisition grace only.
    clock[0] += .06
    feedback(controller)
    controller.teleop_relay_status.update(active=False, reason='expired')
    input_message(controller, 2, True)
    assert not controller.teleop.active
    assert controller.teleop.last_fault == 'teleop_relay_lease_lost: expired'
    assert json.loads(controller.teleop_pub.messages[-1].data)['active'] is False
    input_message(controller, 3, True)
    assert not controller.teleop.active
    input_message(controller, 4, False)
    input_message(controller, 5, True)
    assert controller.teleop.active
    assert controller.teleop.last_fault == 'teleop_relay_lease_lost: expired'


def test_ack_replay_cannot_renew_lease_and_measured_errors_are_reported(node):
    controller, clock = node
    begin(controller, clock)
    old = copy.deepcopy(controller.teleop_relay_status)
    received = controller.teleop_relay_received
    clock[0] += .1
    controller.on_teleop_relay_status(String(data=json.dumps(old)))
    assert controller.teleop_relay_received == received
    input_message(controller, 2, True, left_position=[.05,.2,.1])
    controller.teleop_tick(); controller.publish_status()
    status = json.loads(controller.status_pub.messages[-1].data)
    assert status['teleop_position_error'] == pytest.approx(.05*status['teleop_translation_scale'])
    assert len(status['teleop_measured_position_errors']) == 2
    assert len(status['teleop_commanded_orientation_errors']) == 2


def test_recoverable_projection_has_explicit_limited_flag_and_does_not_remove_ready(node):
    controller, clock = node
    begin(controller, clock)
    clock[0] += .05
    input_message(controller, 2, True, left_position=[0., 1.2, .1])
    controller.teleop_tick(); controller.publish_status()
    status = json.loads(controller.status_pub.messages[-1].data)
    assert status['teleop_limited'] and status['teleop_state'] == 'limited'
    assert status['teleop_ready'] and status['teleop_active']
    assert status['teleop_last_fault'] == ''


@pytest.mark.parametrize('cause', ['deadline', 'input', 'joints', 'relay'])
def test_slow_solve_cannot_publish_a_fresh_active_stamp_for_expired_state(node, monkeypatch, cause):
    controller, clock = node
    begin(controller, clock)
    clock[0] += .05
    feedback(controller)
    input_message(controller, 2, True)
    if cause == 'input':
        controller.teleop.received -= .20
        controller.teleop.input_stamp -= .20
    if cause == 'joints': controller.teleop_joint_time -= .45
    if cause == 'relay':
        controller.teleop_relay_status['stamp'] -= .25
        controller.teleop_relay_received -= .25
    original_tick = controller.teleop.tick
    def slow_tick(*args, **kwargs):
        result = original_tick(*args, **kwargs)
        assert result is not None, controller.teleop.reason
        clock[0] += .16 if cause == 'deadline' else .06
        return result
    monkeypatch.setattr(controller.teleop, 'tick', slow_tick)
    controller.teleop_pub.messages.clear()
    controller.teleop_tick()
    emitted = [json.loads(message.data) for message in controller.teleop_pub.messages]
    assert len(emitted) == 1 and emitted[0]['active'] is False
    np.testing.assert_array_equal(emitted[0]['positions'], controller.teleop_joints.reshape(-1))
    assert not controller.teleop.active and not controller.teleop.armed
    expected = dict(deadline='teleop_servo_deadline', input='teleop_input_timeout',
                    joints='teleop_both_joint_feedback_stale', relay='teleop_relay_ack_stale')
    assert controller.teleop.last_fault == expected[cause]
