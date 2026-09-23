"""Offline metric checks; these tests never initialize ROS or move the simulator."""
import numpy as np
import pytest
from verify_teleop_live import (ArmModel, URDF, REST, OFFSETS, INWARD_OFFSETS, tool_positions,
                                movement_metrics, hold_metrics, preflight_motion, graph_ready,
                                check_start_pose, timeout_release_metrics, orientation_metrics,
                                preflight_orientation, ORIENTATION_ANGLE, wait_for_fresh_feedback,
                                mapping_from_status, absolute_controller_fields)
from deictic_control.teleop import AnatomicalArmMapping, chain_reach
from scipy.spatial.transform import Rotation


def models():
    return (ArmModel(URDF, tip='left_elbow_yaw_link', tool_offset=[0., .1, 0.]), ArmModel(URDF))


def test_absolute_fixture_uses_declared_calibration_and_no_relative_clutch_anchor():
    model = models()
    mapping = AnatomicalArmMapping(model, chain_reach(model[0])/.60)
    status = dict(teleop_protocol_version=3, teleop_translation_scale=mapping.scale,
                  teleop_human_shoulders=mapping.human_shoulders.tolist(),
                  teleop_robot_shoulders=mapping.robot_shoulders.tolist(), teleop_tool_yaw_degrees=[-90., 90.])
    checked = mapping_from_status(model, status)
    reference = np.array([m.fk(q) for m, q in zip(model, REST)])
    fields = absolute_controller_fields(checked, reference, OFFSETS)
    rotations = Rotation.from_quat([fields['left_rotation'], fields['right_rotation']]).as_matrix()
    actual = checked.targets(np.array([fields['left_position'], fields['right_position']]), rotations)
    np.testing.assert_allclose(actual[:, :3, 3], reference[:, :3, 3]+OFFSETS)
    np.testing.assert_allclose(actual[:, :3, :3], reference[:, :3, :3], atol=1e-12)
    for old in (1, 2):
        with pytest.raises(ValueError, match='protocol version 3'):
            mapping_from_status(model, dict(status, teleop_protocol_version=old))
    with pytest.raises(ValueError): mapping_from_status(model, dict(status, teleop_human_shoulders=None))
    with pytest.raises(ValueError): mapping_from_status(model, dict(status, teleop_robot_shoulders=[[0,0,0]]*2))


def test_no_motion_has_zero_progress_and_full_requested_error():
    result = movement_metrics(models(), REST, REST, OFFSETS)
    np.testing.assert_allclose(result['progress_ratio'], [0., 0.])
    np.testing.assert_allclose(result['tool_error_m'], np.linalg.norm(OFFSETS, axis=1))


def test_fk_derived_actual_movement_scores_each_arm_independently():
    model = models()
    final = REST.copy(); final[0, 0] -= .04; final[1, 0] -= .03
    requested = tool_positions(model, final)-tool_positions(model, REST)
    result = movement_metrics(model, REST, final, requested)
    np.testing.assert_allclose(result['progress_ratio'], [1., 1.])
    np.testing.assert_allclose(result['tool_error_m'], [0., 0.], atol=1e-12)
    np.testing.assert_allclose(result['joint_movement_max_rad'], [.04, .03])


def test_hold_metric_detects_excursion_even_if_final_returns_to_start():
    moved = REST.copy(); moved[1, 2] += .1
    result = hold_metrics([REST, moved, REST])
    np.testing.assert_allclose(result['joint_span_max_rad'], [0., .1])
    with pytest.raises(ValueError): hold_metrics([REST])


def test_declared_small_rest_movements_pass_urdf_and_geometry_preflight():
    preflight_motion(models(), REST)


def test_inward_regression_targets_pass_independent_multiseed_geometry_preflight():
    preflight_motion(models(), REST, INWARD_OFFSETS)


def test_resolved_graph_permits_readonly_observers_but_requires_one_controller():
    graph = dict(isaac_feedback=True, isaac_command_consumer=True, competing_input_publishers=[],
                 controller_subscribers=['deictic_control', 'readonly_observer', '_NODE_NAME_UNKNOWN_'],
                 status_publishers=['deictic_control'], controller_node_count=1)
    assert graph_ready(graph)
    assert not graph_ready(dict(graph, controller_subscribers=['_NODE_NAME_UNKNOWN_']))
    assert not graph_ready(dict(graph, status_publishers=['_NODE_NAME_UNKNOWN_']))
    assert not graph_ready(dict(graph, status_publishers=['deictic_control', 'deictic_control']))
    assert not graph_ready(dict(graph, controller_node_count=2))
    assert not graph_ready(dict(graph, competing_input_publishers=['unity_endpoint']))


def test_current_pose_requires_explicit_opt_in_and_still_passes_real_geometry():
    current = REST.copy(); current[0, 3] = -.10
    with pytest.raises(ValueError, match='from-current'): check_start_pose(current, False)
    check_start_pose(current, True)
    preflight_motion(models(), current)


def test_timeout_uses_actual_backend_release_separately_from_status_delivery():
    commands = [dict(active=False, stamp=99.9, received_unix=99.91),
                dict(active=True, stamp=100.15, received_unix=100.16),
                dict(active=False, stamp=100.28, received_unix=100.31)]
    result = timeout_release_metrics(commands, 100.05, 100.)
    assert result['source_delay_s'] == pytest.approx(.28)
    assert result['arrival_delay_s'] == pytest.approx(.31)
    with pytest.raises(ValueError, match='No actual'): timeout_release_metrics(commands[:2], 100.05, 100.)
    late = [dict(active=False, stamp=100.41, received_unix=100.42)]
    with pytest.raises(ValueError, match='budget'): timeout_release_metrics(late, 100.05, 100.)
    late_delivery = [dict(active=False, stamp=100.28, received_unix=100.56)]
    with pytest.raises(ValueError, match='budget'): timeout_release_metrics(late_delivery, 100.05, 100.)


def test_orientation_metric_rejects_no_response_and_wrong_direction():
    model = models()
    final = REST.copy(); final[0, 3] += .1
    delta = model[0].fk(final[0])[:3, :3] @ model[0].fk(REST[0])[:3, :3].T
    requested = np.zeros((2, 3)); requested[0] = Rotation.from_matrix(delta).as_rotvec()
    exact = orientation_metrics(model, REST, final, requested)
    assert exact['progress_ratio'][0] == pytest.approx(1.)
    assert exact['orientation_error_rad'][0] < 1e-12
    assert exact['position_drift_m'][0] > .005, 'Rotation accuracy must not hide tool translation'
    assert exact['measured_angle_rad'][1] == pytest.approx(0.)
    unmoved = orientation_metrics(model, REST, REST, requested)
    assert unmoved['progress_ratio'][0] == pytest.approx(0.)
    assert unmoved['orientation_error_rad'][0] == pytest.approx(.1)
    opposite = orientation_metrics(model, REST, final, -requested)
    assert opposite['progress_ratio'][0] == pytest.approx(-1.)
    assert opposite['orientation_error_rad'][0] == pytest.approx(.2)


def test_orientation_preflight_selects_nontrivial_bounded_rotation_and_safe_geometry():
    vectors, report = preflight_orientation(models(), REST)
    np.testing.assert_allclose(np.linalg.norm(vectors, axis=1), ORIENTATION_ANGLE)
    assert min(report['predicted_metrics']['progress_ratio']) >= .5
    assert max(report['predicted_metrics']['position_drift_m']) < .003


def test_preflight_recovery_drains_status_and_joints_independently_without_relaxing_age():
    now, stamps, calls = [10.], {'status': 8., 'joints': 8.}, []
    def spin(timeout):
        now[0] += timeout
        calls.append(timeout)
        # First callback updates status only; the next queued joint is old.
        if len(calls) == 1: stamps['status'] = now[0]
        elif len(calls) == 2: stamps['joints'] = 9.
        else: stamps['joints'] = now[0]
    fresh = lambda: all(-.05 <= now[0]-stamp <= .5 for stamp in stamps.values())
    elapsed = wait_for_fresh_feedback(fresh, spin, monotonic=lambda: now[0])
    assert len(calls) == 3
    assert elapsed == pytest.approx(.06)
    assert fresh()


def test_preflight_recovery_fails_closed_when_joint_source_stays_stale():
    now = [10.]
    def spin(timeout): now[0] += timeout
    with pytest.raises(ValueError, match='did not recover'):
        wait_for_fresh_feedback(lambda: now[0]-8. <= .5, spin, timeout=.1, monotonic=lambda: now[0])
    assert now[0] == pytest.approx(10.1)
