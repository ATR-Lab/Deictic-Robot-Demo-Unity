"""Bimanual clutch guards and actual URDF position servo; no simulator needed."""
import json
from pathlib import Path
import time
import numpy as np
import pytest
from deictic_control.kinematics import ArmModel
from deictic_control.teleop import (BimanualClutch, ClutchInput, position_jacobian,
                                   check_bimanual_geometry, segment_distance)

ROOT = Path(__file__).resolve().parents[4]


@pytest.fixture
def clutch():
    urdf = ROOT / 'models/K1/K1_22dof.urdf'
    left = ArmModel(urdf, tip='left_elbow_yaw_link', tool_offset=[0., .1, 0.])
    right = ArmModel(urdf)
    return BimanualClutch((left, right))


def joints():
    return np.array([[0., 0., 0., 0.], [0., .5, 0., 0.]])


def packet(sequence=0, stamp=100., clutch=False, **extra):
    value = dict(schema_version=2, session_id='client_a', sequence=sequence, stamp=stamp,
                 frame_id='teleop_head', clutch=clutch, left_tracked=True, right_tracked=True,
                 left_position=[0., .2, .1], right_position=[0., -.2, .1],
                 left_rotation=[0.,0.,0.,1.], right_rotation=[0.,0.,0.,1.])
    value.update(extra)
    return json.dumps(value)


def start(clutch):
    assert clutch.ingest(packet(), 100., 10., joints()) == 'released'
    assert clutch.ingest(packet(1, 100.01, True), 100.01, 10.01, joints()) == 'began'


@pytest.mark.parametrize('field,value', [('sequence', True), ('stamp', float('nan')),
    ('frame_id', 'headset_world'), ('clutch', 1), ('left_position', [0., 0., float('inf')]),
    ('right_position', [0., 0.]), ('session_id', ''), ('left_tracked', 'true')])
def test_malformed_input_rejected(field, value):
    with pytest.raises((ValueError, TypeError)):
        ClutchInput.parse(packet(**{field:value}))


def test_initial_held_requires_fresh_release_and_clutch_start_is_zero_motion(clutch):
    assert clutch.ingest(packet(0, clutch=True), 100., 10., joints()) == 'rejected'
    assert not clutch.active
    assert clutch.ingest(packet(1), 100., 10., joints()) == 'released'
    assert clutch.ingest(packet(2, 100.01, True), 100.01, 10.01, joints()) == 'began'
    np.testing.assert_allclose(clutch.tick(100.06, 10.06, joints()), joints(), atol=1e-12)


def test_analytic_jacobians_match_actual_both_arm_kinematics(clutch):
    for model, q in zip(clutch.models, joints()+.05):
        position, jacobian = position_jacobian(model, q)
        np.testing.assert_allclose(position, model.fk(q)[:3, 3], atol=1e-12)
        numeric = np.column_stack([(model.fk(q+np.eye(4)[i]*1e-5)[:3, 3] -
                                    model.fk(q-np.eye(4)[i]*1e-5)[:3, 3]) / 2e-5 for i in range(4)])
        np.testing.assert_allclose(jacobian, numeric, atol=1e-9)


def test_relative_targets_reduce_both_fk_errors_with_rate_and_acceleration_bounds(clutch):
    start(clutch)
    q = joints()
    target_offsets = np.array([[.025, -.005, -.01], [.025, .005, .01]])
    positions = np.array([[0., .2, .1], [0., -.2, .1]])+target_offsets
    initial_error = np.linalg.norm(target_offsets, axis=1)
    previous_velocity = np.zeros((2, 4))
    elapsed = time.monotonic()
    for i in range(1, 61):
        now = 100.01+i*.05
        assert clutch.ingest(packet(i+1, now, True, left_position=positions[0].tolist(),
                                   right_position=positions[1].tolist()), now, 10.01+i*.05, q) == 'updated'
        command = clutch.tick(now, 10.01+i*.05, q)
        assert command is not None, clutch.reason
        velocity = (command-q)/.05
        assert np.max(np.abs(velocity)) <= .35+1e-10
        assert np.max(np.abs(velocity-previous_velocity)) <= .7*.05+1e-10
        previous_velocity, q = velocity, command
    errors = np.array([np.linalg.norm(model.fk(arm)[:3, 3]-target) for model, arm, target in
                       zip(clutch.models, q, clutch.tool_start+target_offsets)])
    assert np.all(errors < initial_error*.15), errors
    assert time.monotonic()-elapsed < 3., 'Bounded servo must not stall the ROS executor'


@pytest.mark.parametrize('cause', ['timeout', 'tracking', 'future', 'stale', 'feedback', 'deadline'])
def test_fault_holds_both_and_held_packets_cannot_resume(clutch, cause):
    start(clutch)
    if cause == 'tracking':
        clutch.ingest(packet(2, 100.02, True, left_tracked=False), 100.02, 10.02, joints())
    elif cause in ('future', 'stale'):
        stamp = 101. if cause == 'future' else 99.
        clutch.ingest(packet(2, stamp, True), 100.02, 10.02, joints())
    elif cause == 'limit':
        clutch.ingest(packet(2, 100.02, True, left_position=[1., .2, .1]), 100.02, 10.02, joints())
        clutch.tick(100.06, 10.06, joints())
    elif cause == 'deadline':
        clutch.tick(100.21, 10.21, joints())
    else:
        clutch.tick(100.31 if cause == 'timeout' else 100.06,
                    10.31 if cause == 'timeout' else 10.06, joints(),
                    'teleop_both_joint_feedback_stale' if cause == 'feedback' else None)
    assert not clutch.active and not clutch.armed
    assert clutch.ingest(packet(3, 100.32, True), 100.32, 10.32, joints()) == 'rejected'
    assert clutch.ingest(packet(4, 100.33, False), 100.33, 10.33, joints()) == 'released'
    assert clutch.ingest(packet(5, 100.34, True), 100.34, 10.34, joints()) == 'began'


def test_replay_does_not_extend_lease_and_retired_session_cannot_steal(clutch):
    start(clutch)
    assert clutch.ingest(packet(1, 100.20, True), 100.20, 10.20, joints()) == 'ignored'
    assert clutch.received == 10.01
    assert clutch.ingest(packet(0, 100.21, False, session_id='client_b'), 100.21, 10.21, joints()) == 'released'
    assert clutch.ingest(packet(1, 100.22, True, session_id='client_b'), 100.22, 10.22, joints()) == 'began'
    assert clutch.ingest(packet(100, 100.23, False), 100.23, 10.23, joints()) == 'ignored'
    assert clutch.active


def test_actual_geometry_rejects_table_obstacle_and_joint_limit(clutch):
    check_bimanual_geometry(clutch.models, joints(), -.15, ())
    with pytest.raises(ValueError, match='table'):
        check_bimanual_geometry(clutch.models, joints(), .2, ())
    with pytest.raises(ValueError, match='obstacle'):
        check_bimanual_geometry(clutch.models, joints(), -.15, [[-1, -1, -1, 1, 1, 1]])
    q = joints(); q[0, 3] = 1.
    with pytest.raises(ValueError, match='joint_limit'):
        check_bimanual_geometry(clutch.models, q, -.15, ())


def test_cross_arm_segment_distance_includes_crossing_parallel_and_degenerate():
    v = lambda *p: np.array(p, dtype=float)
    assert segment_distance(v(-1, 0, 0), v(1, 0, 0), v(0, -1, 0), v(0, 1, 0)) == 0
    assert segment_distance(v(-1, 0, 0), v(1, 0, 0), v(-1, .2, 0), v(1, .2, 0)) == pytest.approx(.2)
    assert segment_distance(v(0, 0, 0), v(0, 0, 0), v(0, .2, 0), v(0, .2, 0)) == pytest.approx(.2)


def test_right_planning_worker_rejects_crossing_the_captured_held_left_arm(monkeypatch):
    from deictic_control import planning
    from deictic_control.kinematics import Plan

    class Model:
        lower, upper = np.full(4, -1.), np.full(4, 1.)
        def __init__(self, points): self.points = np.array(points)
        def fk(self, q, with_points=False): return np.eye(4), self.points

    left = Model([[.3, 0., .4], [.3, .2, .4]])
    right = Model([[.2, .1, .4], [.4, .1, .4]])
    left_joints = np.zeros(4)
    request = planning.PlanningRequest.capture(1, (100, 0), 100., time.monotonic()+2,
        0, np.eye(4), np.zeros(3), np.zeros(4), right, .35, .7, -.15, (), left, left_joints)
    left_joints[:] = .5
    np.testing.assert_array_equal(request.held_joints, np.zeros(4))
    plan = Plan(['r']*4, np.array([0., .1]), np.zeros((2, 4)), np.zeros((2, 4)), np.zeros((2, 4)))
    monkeypatch.setattr(planning, 'make_plan', lambda *args, **kwargs: plan)
    with pytest.raises(ValueError, match='cross_arm'):
        planning.solve_request(request)
