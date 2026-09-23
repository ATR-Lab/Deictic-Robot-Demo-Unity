"""Offline measurement-contract checks; never initialize ROS or command motion."""
import sys
import pytest

import benchmark_head_latency as benchmark


def sample(received, measured, **extra):
    value = dict(received_monotonic=10.+received, stamp=100.+received,
                 session_id='current', sequence=20, active=True, targets=[.20, -.10], measured=measured)
    value.update(extra)
    return value


def metrics(samples, initial=None, target=None):
    return benchmark.summarize_phase(samples, started=10., start_stamp=100., first_sequence=10,
        session='current', initial=initial or [0., 0.], target=target or [.20, -.10])


def test_latency_uses_owned_matching_receipt_and_90_percent_measured_response():
    records = [sample(.01, [.2, -.1], session_id='other'), sample(.02, [.2, -.1], sequence=9),
               sample(.03, [.2, -.1], targets=[-.2, .1]), sample(.05, [0., 0.]),
               sample(.10, [.03, -.02]), sample(.30, [.17, -.08]),
               sample(.40, [.182, -.091]), sample(.50, [.199, -.099])]
    result = metrics(records)
    assert result['accepted_status_samples'] == 5
    assert result['first_matching_status_s'] == pytest.approx(.05)
    assert result['first_measured_movement_s'] == pytest.approx(.10)
    assert result['reached_90_percent_s'] == pytest.approx(.40)
    assert result['final_max_error_rad'] == pytest.approx(.001)
    assert result['converged']
    assert result['observed_status_interval_s']['count'] == 4


def test_no_movement_and_transient_goal_crossing_do_not_claim_convergence():
    unmoved = metrics([sample(.1, [0., 0.]), sample(3.9, [0., 0.])])
    assert unmoved['first_matching_status_s'] == pytest.approx(.1)
    assert unmoved['first_measured_movement_s'] is None
    assert unmoved['reached_90_percent_s'] is None
    assert not unmoved['converged']
    reversed_pose = metrics([sample(.1, [.195, -.095]), sample(3.9, [.08, -.04])])
    assert reversed_pose['reached_90_percent_s'] == pytest.approx(.1)
    assert not reversed_pose['converged']
    missing = metrics([])
    assert missing['final_max_error_rad'] is None and not missing['converged']


def test_neutral_already_at_goal_is_not_a_zero_latency_90_percent_step():
    result = metrics([sample(.1, [.20, -.10])], initial=[.20, -.10])
    assert result['converged'] and result['already_in_90_percent_band']
    assert not result['meaningful_step'] and result['reached_90_percent_s'] is None


def test_camera_rate_age_payload_and_empty_window_are_reported_separately():
    samples = [dict(received_monotonic=10.+i*.2, acquisition_age_s=.10+i*.01,
                    payload_bytes=100+20*i) for i in range(4)]
    result = benchmark.camera_summary(samples, 10., 11.)
    assert result['frames'] == 4
    assert result['received_rate_hz'] == pytest.approx(5.)
    assert result['acquisition_age_s']['mean'] == pytest.approx(.115)
    assert result['payload_bytes']['mean'] == pytest.approx(130.)
    empty = benchmark.camera_summary(samples, 20., 21.)
    assert empty['frames'] == 0 and empty['received_rate_hz'] is None
    assert empty['acquisition_age_s']['mean'] is None


def test_graph_identity_rejects_competing_publisher_and_unknown_camera_source():
    graph = dict(isaac_node_count=1, head_consumers=[benchmark.ISAAC_NODE],
        head_publishers=[benchmark.NODE_NAME], status_publishers=[benchmark.ISAAC_NODE],
        joint_publishers=[benchmark.ISAAC_NODE], camera_publishers=['deictic_camera_view_relay'])
    assert benchmark.graph_ready(graph)
    assert not benchmark.graph_ready(dict(graph, head_publishers=[benchmark.NODE_NAME, 'UnityEndpoint']))
    assert not benchmark.graph_ready(dict(graph, head_consumers=['physical_robot']))
    assert not benchmark.graph_ready(dict(graph, isaac_node_count=2))
    assert not benchmark.graph_ready(dict(graph, camera_publishers=['unknown_camera']))


def test_missing_motion_opt_in_does_not_create_output_or_initialize_ros(tmp_path, monkeypatch):
    output = tmp_path / 'not-created.json'
    monkeypatch.setattr(sys, 'argv', ['benchmark_head_latency.py', '--output', str(output)])
    with pytest.raises(SystemExit) as stopped:
        benchmark.main()
    assert stopped.value.code == 2
    assert not output.exists()


def test_unwritable_output_is_rejected_before_ros_initialization(tmp_path, monkeypatch):
    monkeypatch.setattr(sys, 'argv', ['benchmark_head_latency.py', '--allow-sim-motion',
                                    '--output', str(tmp_path / 'refused.json')])
    def fail(_): raise PermissionError('test destination denied')
    monkeypatch.setattr(benchmark, 'prepare_output', fail)
    with pytest.raises(SystemExit) as stopped:
        benchmark.main()
    assert stopped.value.code == 2
