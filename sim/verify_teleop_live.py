#!/usr/bin/env python3
"""Opt-in, bounded bimanual Isaac test driven by SYNTHETIC ROS controller input.

Run on the ROS workstation with Unity stopped/disconnected. This does not test
Meta trigger input. Restart Unity afterward for a fresh frontend session UUID.
It never writes direct joint targets, overrides a gate, or invokes a reset.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import uuid
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2/src/deictic_control'))
from deictic_control.kinematics import ArmModel, make_joint_plan
from deictic_control.teleop import AnatomicalArmMapping, check_bimanual_geometry, pose_jacobian, PoseIK
from k1_model import BOTH_ARM_JOINTS, URDF

REST = np.array([[0., 0., 0., 0.], [0., .5, 0., 0.]])
OFFSETS = np.array([[.020, -.003, .005], [.020, .003, .005]])
INWARD_OFFSETS = np.array([[0., -.05, 0.], [0., .05, 0.]])
STATUS_TRANSITION_GRACE = .50  # Observation only; production input TTL stays .25 s.
RELEASE_SOURCE_BUDGET = .40  # .25 s input TTL + .05 s servo + .10 s scheduling budget.
RELEASE_DELIVERY_BUDGET = .55  # Additional .15 s for observing the release over DDS.
ORIENTATION_ANGLE = .15  # 8.59 degrees, selected before collecting live outcomes.
ORIENTATION_MAX_ERROR = .09
ORIENTATION_MAX_DRIFT = .01


def tool_positions(models, q):
    return np.array([model.fk(arm)[:3, 3] for model, arm in zip(models, np.asarray(q).reshape(2, 4))])


def mapping_from_status(models, status):
    """Refuse old/unknown calibration before constructing absolute input."""
    if status.get('teleop_protocol_version') != 3:
        raise ValueError('This verifier requires the absolute-pose protocol version 3 controller')
    scale = float(status.get('teleop_translation_scale', 0.))
    shoulders = np.asarray(status.get('teleop_human_shoulders'), dtype=float)
    robot = np.asarray(status.get('teleop_robot_shoulders'), dtype=float)
    yaw = np.asarray(status.get('teleop_tool_yaw_degrees'), dtype=float)
    if (not np.isfinite(scale) or not .1 <= scale <= 2. or shoulders.shape != (2, 3)
            or not np.all(np.isfinite(shoulders)) or robot.shape != (2, 3)
            or not np.all(np.isfinite(robot))):
        raise ValueError('Missing or invalid declared teleoperation anatomical mapping')
    mapping = AnatomicalArmMapping(models, scale, shoulders[0, 0], shoulders[0, 1], -shoulders[0, 2], yaw)
    if (not np.allclose(mapping.human_shoulders, shoulders, atol=1e-10)
            or not np.allclose(mapping.robot_shoulders, robot, atol=1e-10)):
        raise ValueError('Declared shoulder mapping does not match the pinned robot model')
    return mapping


def absolute_controller_fields(mapping, reference, offsets, rotation_vectors=None):
    """Explicitly invert a known robot fixture; never rely on clutch anchors."""
    poses = reference.copy()
    poses[:, :3, 3] += offsets
    if rotation_vectors is not None:
        poses[:, :3, :3] = Rotation.from_rotvec(rotation_vectors).as_matrix() @ reference[:, :3, :3]
    positions, matrices = mapping.controller_poses(poses)
    rotations = Rotation.from_matrix(matrices).as_quat()
    return dict(left_position=positions[0].tolist(), right_position=positions[1].tolist(),
                left_rotation=rotations[0].tolist(), right_rotation=rotations[1].tolist())


def movement_metrics(models, initial, final, offsets):
    initial, final, offsets = np.asarray(initial).reshape(2, 4), np.asarray(final).reshape(2, 4), np.asarray(offsets)
    actual = tool_positions(models, final)-tool_positions(models, initial)
    ratios = [float(a @ requested / (requested @ requested)) if requested @ requested > 0 else None
              for a, requested in zip(actual, offsets)]
    return dict(actual_tool_delta_m=actual.tolist(), requested_tool_delta_m=offsets.tolist(),
                joint_movement_max_rad=np.max(np.abs(final-initial), axis=1).tolist(),
                tool_error_m=np.linalg.norm(actual-offsets, axis=1).tolist(), progress_ratio=ratios)


def orientation_metrics(models, initial, final, requested_rotvecs):
    """Score measured FK against requested world-frame rotation deltas, not IK output."""
    initial, final = np.asarray(initial).reshape(2, 4), np.asarray(final).reshape(2, 4)
    requested = np.asarray(requested_rotvecs, dtype=float).reshape(2, 3)
    actual, errors, drift = [], [], []
    for model, start, end, vector in zip(models, initial, final, requested):
        before, after = model.fk(start), model.fk(end)
        delta = after[:3, :3] @ before[:3, :3].T
        actual.append(Rotation.from_matrix(delta).as_rotvec())
        errors.append(float(Rotation.from_matrix(delta @ Rotation.from_rotvec(vector).as_matrix().T).magnitude()))
        drift.append(float(np.linalg.norm(after[:3, 3]-before[:3, 3])))
    actual = np.asarray(actual)
    return dict(requested_rotation_vector_rad=requested.tolist(), actual_rotation_vector_rad=actual.tolist(),
                requested_angle_rad=np.linalg.norm(requested, axis=1).tolist(),
                measured_angle_rad=np.linalg.norm(actual, axis=1).tolist(),
                orientation_error_rad=errors, position_drift_m=drift,
                progress_ratio=[float(a @ r / (r @ r)) if r @ r > 0 else None for a, r in zip(actual, requested)],
                joint_movement_max_rad=np.max(np.abs(final-initial), axis=1).tolist())


def preflight_orientation(models, current):
    """Choose a small rotation about each arm's position-null direction.

    Four joints cannot satisfy arbitrary six-dimensional requests. This selects
    a locally feasible angular direction from measured FK/Jacobians before the
    test, keeps the requested position fixed, and uses the production solver
    only for a feasibility/geometry precheck. Live scoring uses measured FK.
    """
    current = np.asarray(current, dtype=float).reshape(2, 4)
    vectors, predicted = [], []
    for model, q in zip(models, current):
        pose, jacobian = pose_jacobian(model, q)
        null = np.linalg.svd(jacobian[:3], full_matrices=True)[2][-1]
        axis = jacobian[3:] @ null
        if np.linalg.norm(axis) < .1:
            raise ValueError('No useful orientation direction at the measured arm pose')
        axis /= np.linalg.norm(axis)
        if axis[np.argmax(np.abs(axis))] < 0: axis = -axis
        vector = axis*ORIENTATION_ANGLE
        target = pose.copy()
        target[:3, :3] = Rotation.from_rotvec(vector).as_matrix() @ pose[:3, :3]
        solver = PoseIK(model); solver.reset(q)
        goal = solver.solve(target, q)
        make_joint_plan(model, q, goal, table_top=-.15)
        vectors.append(vector); predicted.append(goal)
    vectors, predicted = np.asarray(vectors), np.asarray(predicted)
    for fraction in np.linspace(0., 1., 41):
        check_bimanual_geometry(models, [current[0]+fraction*(predicted[0]-current[0]), current[1]], -.15, ())
        check_bimanual_geometry(models, [predicted[0], current[1]+fraction*(predicted[1]-current[1])], -.15, ())
    metrics = orientation_metrics(models, current, predicted, vectors)
    if (min(metrics['progress_ratio']) < .5 or max(metrics['orientation_error_rad']) > ORIENTATION_MAX_ERROR
            or max(metrics['position_drift_m']) > .003):
        raise ValueError('The fixed modest orientation request is not feasible at this start pose: '+json.dumps(metrics))
    return vectors, dict(rotation_vectors_rad=vectors.tolist(), predicted_joints=predicted.tolist(),
                         predicted_metrics=metrics, selection='position-Jacobian null direction; no live outcome selection')


def hold_metrics(samples):
    if len(samples) < 3:
        raise ValueError('Hold check requires at least three fresh joint samples')
    q = np.asarray(samples, dtype=float).reshape(-1, 2, 4)
    return dict(sample_count=len(samples), joint_span_max_rad=np.max(np.ptp(q, axis=0), axis=1).tolist())


def graph_ready(graph):
    """Read-only input observers are allowed; control/publishing stays unique."""
    return (graph['isaac_feedback'] and graph['isaac_command_consumer']
            and not graph['competing_input_publishers']
            and graph['controller_subscribers'].count('deictic_control') == 1
            and graph['status_publishers'] == ['deictic_control']
            and graph['controller_node_count'] == 1)


def wait_for_fresh_feedback(fresh, spin_once, timeout=5., monotonic=time.monotonic):
    """Resume callback processing after synchronous preflight while still held.

    A queued status callback does not imply that joint feedback was dispatched.
    Keep spinning until the unchanged caller freshness predicate covers both,
    or fail without authorizing motion when live feedback does not recover.
    """
    started = monotonic()
    deadline = started+timeout
    while monotonic() < deadline:
        spin_once(min(.02, max(0., deadline-monotonic())))
        if fresh():
            return monotonic()-started
    raise ValueError('Fresh status and joint feedback did not recover after offline preflight')


def check_start_pose(current, from_current):
    if not from_current and np.max(np.abs(np.asarray(current)-REST)) > .08:
        raise ValueError('Start near declared rest (left zeros; right 0,.5,0,0), or explicitly use --from-current')


def timeout_release_metrics(commands, silence_started, last_input_stamp):
    releases = [value for value in commands if value.get('active') is False
                and value.get('stamp', 0) >= silence_started]
    if not releases:
        raise ValueError('No actual backend inactive command observed after input silence')
    release = min(releases, key=lambda value: value['stamp'])
    source_delay = release['stamp']-last_input_stamp
    arrival_delay = release['received_unix']-last_input_stamp
    if not 0 <= source_delay <= RELEASE_SOURCE_BUDGET or not 0 <= arrival_delay <= RELEASE_DELIVERY_BUDGET:
        raise ValueError(f'Backend timeout release exceeded its test budget: source={source_delay:.3f}s, arrival={arrival_delay:.3f}s')
    return dict(last_input_stamp=last_input_stamp, release_source_stamp=release['stamp'],
                release_received_unix=release['received_unix'], source_delay_s=source_delay,
                arrival_delay_s=arrival_delay, source_budget_s=RELEASE_SOURCE_BUDGET,
                delivery_budget_s=RELEASE_DELIVERY_BUDGET)


def preflight_motion(models, current, offsets=OFFSETS):
    """Check small declared endpoints/paths without sending any trajectory."""
    deadline = time.monotonic()+5.
    cancelled = lambda: time.monotonic() > deadline
    goals = []
    for model, q, delta in zip(models, current, offsets):
        goal = model.inverse_position(model.fk(q)[:3, 3]+delta, q, cancel=cancelled)
        make_joint_plan(model, q, goal, table_top=-.15, cancel=cancelled)
        goals.append(goal)
    for fraction in np.linspace(0., 1., 41):
        if cancelled():
            raise ValueError('Offline geometry preflight deadline exceeded')
        # Test the independent left stage, followed by the right stage.
        check_bimanual_geometry(models, [current[0]+fraction*(goals[0]-current[0]), current[1]], -.15, ())
        check_bimanual_geometry(models, [goals[0], current[1]+fraction*(goals[1]-current[1])], -.15, ())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--allow-sim-motion', action='store_true',
                        help='Explicitly permit the bounded small movements in the identified Isaac instance')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--from-current', action='store_true',
                        help='Explicitly allow a non-rest measured start; all path/geometry prechecks still run')
    parser.add_argument('--motion-profile', choices=('forward', 'inward'), default='inward',
                        help='Inward 5 cm exercises the initial straight-arm singularity; forward retains the earlier 2 cm check')
    parser.add_argument('--orientation-check', action='store_true',
                        help='After position tests, preflight and measure small orientation-only requests for each arm')
    args = parser.parse_args()
    if not args.allow_sim_motion:
        parser.error('No motion authorized; add --allow-sim-motion only for the isolated Isaac simulator')
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    args.output.parent.mkdir(parents=True, exist_ok=True)
    offsets = INWARD_OFFSETS if args.motion_profile == 'inward' else OFFSETS
    motion_duration = 8. if args.motion_profile == 'inward' else 4.
    report = dict(scope='Synthetic ROS input into the real Isaac simulation; NOT native Meta trigger verification',
                  started_unix=time.time(), passed=False, phases=[], statuses=[], joints=[], commands=[], errors=[],
                  requested_offsets_m=offsets.tolist(), motion_profile=args.motion_profile, input_timeout_s=.25,
                  orientation_check=args.orientation_check,
                  start_mode='current_measured_pose' if args.from_current else 'declared_rest',
                  note='No return-to-rest command is issued; final state is a measured hold after small motions.')
    models = (ArmModel(URDF, tip='left_elbow_yaw_link', tool_offset=[0., .1, 0.]), ArmModel(URDF))
    rclpy.init()
    node = rclpy.create_node('synthetic_isaac_teleop_verifier')
    publisher = node.create_publisher(String, '/deictic/teleop/input', 10)
    latest = dict(status=None, joints=None)
    session, sequence, authorized = str(uuid.uuid4()), 0, False
    last_input_stamp = None
    mapping = reference = None

    def status(message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict): return
            sample = dict(received_unix=time.time(), **value)
            latest['status'] = sample
            report['statuses'].append(sample)
        except (ValueError, TypeError):
            report['errors'].append('Malformed status received')

    def joints(message):
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name): return
        values = dict(zip(message.name, message.position))
        if not all(name in values for name in BOTH_ARM_JOINTS): return
        q = np.array([values[name] for name in BOTH_ARM_JOINTS]).reshape(2, 4)
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if not np.all(np.isfinite(q)) or not -.05 <= time.time()-stamp <= .5: return
        if latest['joints'] is not None and stamp <= latest['joints']['stamp']: return
        sample = dict(received_unix=time.time(), stamp=stamp, positions=q.tolist())
        latest['joints'] = sample
        report['joints'].append(sample)

    def command(message):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict): report['commands'].append(dict(received_unix=time.time(), **value))
        except (ValueError, TypeError):
            report['errors'].append('Malformed backend teleop command observed')

    node.create_subscription(String, '/deictic/status', status, 20)
    node.create_subscription(JointState, '/joint_states', joints, 10)
    node.create_subscription(String, '/k1/teleop/command', command, 10)

    def fresh():
        now = time.time()
        state, measured = latest['status'], latest['joints']
        return (state is not None and measured is not None and
                -.05 <= now-float(state.get('stamp', 0)) <= .5 and
                -.05 <= now-measured['stamp'] <= .5)

    def simulation_graph():
        state_sources = node.get_publishers_info_by_topic('/joint_states')
        sim_consumers = node.get_subscriptions_info_by_topic('/k1/sim/joint_commands')
        input_sources = node.get_publishers_info_by_topic('/deictic/teleop/input')
        controllers = node.get_subscriptions_info_by_topic('/deictic/teleop/input')
        status_publishers = node.get_publishers_info_by_topic('/deictic/status')
        own = node.get_name()
        return dict(isaac_feedback=any(v.node_name == 'k1_fixed_base_isaac' for v in state_sources),
                    isaac_command_consumer=any(v.node_name == 'k1_fixed_base_isaac' for v in sim_consumers),
                    competing_input_publishers=[v.node_name for v in input_sources if v.node_name != own],
                    controller_subscribers=[v.node_name for v in controllers],
                    status_publishers=[v.node_name for v in status_publishers],
                    controller_node_count=node.get_node_names().count('deictic_control'))

    def send(held, offsets, rotation_vectors=None):
        nonlocal sequence, last_input_stamp
        value = dict(schema_version=3, session_id=session, sequence=sequence, stamp=time.time(),
                     frame_id='teleop_body', clutch=held, left_tracked=True, right_tracked=True,
                     **absolute_controller_fields(mapping, reference, offsets, rotation_vectors))
        sequence += 1
        last_input_stamp = value['stamp']
        publisher.publish(String(data=json.dumps(value, allow_nan=False)))

    def assert_inactive():
        if not fresh() or latest['status'].get('teleop_active') or latest['status'].get('executing'):
            raise ValueError('A fresh inactive controller is required before the next phase')

    def phase(name, duration, held=None, offsets=None, ramp_from=None, expect_active=None,
              rotations=None, rotation_ramp_from=None):
        started, wall_start = time.monotonic(), time.time()
        end, next_send, next_graph = started+duration, started, started
        row = dict(name=name, started_unix=wall_start, duration_s=duration)
        if held is None:
            row['last_input_stamp_at_start'] = last_input_stamp
        report['phases'].append(row)
        print(json.dumps(dict(phase=name, started_unix=wall_start)), flush=True)
        while time.monotonic() < end:
            now = time.monotonic()
            if held is not None and now >= next_send:
                target = np.zeros((2, 3)) if offsets is None else offsets
                if ramp_from is not None:
                    fraction = min(1., (now-started)/.5)
                    target = ramp_from+fraction*(target-ramp_from)
                angle_target = np.zeros((2, 3)) if rotations is None else rotations
                if rotation_ramp_from is not None:
                    fraction = min(1., (now-started)/.5)
                    angle_target = rotation_ramp_from+fraction*(angle_target-rotation_ramp_from)
                send(held, target, angle_target)
                next_send = now+.05
            rclpy.spin_once(node, timeout_sec=.005)
            if not fresh():
                raise ValueError('Controller status or actual joint feedback went stale')
            if now >= next_graph:
                graph = simulation_graph()
                if (not graph['isaac_feedback'] or not graph['isaac_command_consumer'] or
                        graph['competing_input_publishers']):
                    raise ValueError('Simulator graph changed or another teleop publisher appeared')
                next_graph = now+.5
            if expect_active is True and now-started > STATUS_TRANSITION_GRACE and not latest['status'].get('teleop_active'):
                raise ValueError('Teleoperation stopped: '+str(latest['status'].get('teleop_reason')))
            if expect_active is False and now-started > STATUS_TRANSITION_GRACE and latest['status'].get('teleop_active'):
                raise ValueError('Teleoperation remained active when a hold was required')
        row['ended_unix'] = time.time()
        row['final_status'] = latest['status']
        row['final_joints'] = latest['joints']['positions']
        row['observed_joint_samples'] = sum(v['received_unix'] >= wall_start for v in report['joints'])
        print(json.dumps(dict(phase=name, teleop_active=latest['status'].get('teleop_active'),
                              reason=latest['status'].get('teleop_reason'))), flush=True)
        return row

    def verify_hold(row):
        samples = [v['positions'] for v in report['joints']
                   if row['ended_unix']-.5 <= v['received_unix'] <= row['ended_unix']]
        row['hold'] = hold_metrics(samples)
        if max(row['hold']['joint_span_max_rad']) > .015:
            raise ValueError('Measured arms did not settle into the bounded hold')

    try:
        deadline = time.monotonic()+10.
        graph = simulation_graph()
        # Fast DDS may expose endpoint GIDs before their node names resolve.
        # Wait for the complete graph rather than treating its first sample as
        # authoritative. Extra read-only input subscribers never own control.
        while (not fresh() or not graph_ready(graph)) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            graph = simulation_graph()
        if not fresh(): raise ValueError('No fresh controller/eight-joint feedback within 10 seconds')
        report['preflight_graph'] = graph
        if not graph_ready(graph):
            raise ValueError('Resolved graph requires one simulation controller/status publisher and no competing teleop publisher')
        assert_inactive()
        state = latest['status']
        mapping = mapping_from_status(models, state)
        report['translation_scale'] = mapping.scale
        report['human_shoulders'] = mapping.human_shoulders.tolist()
        report['tool_yaw_degrees'] = state['teleop_tool_yaw_degrees']
        if (state.get('robot_feedback') != 'external' or not state.get('execution_enabled')
                or not state.get('headset_tracking_ready') or state.get('preview_ready')
                or state.get('teleop_reason') in ('teleop_execution_disabled', 'teleop_mock_bimanual_unsupported')):
            raise ValueError('Controller execution/tracking/idle preconditions are not met')
        initial = np.asarray(latest['joints']['positions'])
        reference = np.array([m.fk(q) for m, q in zip(models, initial)])
        report['initial_joints'] = initial.tolist()
        check_start_pose(initial, args.from_current)
        preflight_motion(models, initial, offsets)
        report['position_preflight_feedback_wait_s'] = wait_for_fresh_feedback(
            fresh, lambda wait: rclpy.spin_once(node, timeout_sec=wait))
        if not graph_ready(simulation_graph()):
            raise ValueError('Simulator ownership changed during offline preflight')
        assert_inactive()
        authorized = True
        phase('fresh_release_handshake', .8, False, expect_active=False)
        if not latest['status'].get('teleop_ready'): raise ValueError('Fresh release did not arm the backend')
        assert_inactive()
        phase('explicit_measured_pose_clutch', .6, True, expect_active=True)
        start_q = np.asarray(latest['joints']['positions'])
        if np.max(np.abs(start_q-initial)) > .025: raise ValueError('Inverse-mapped measured pose unexpectedly moved the arms')
        left_only = offsets.copy(); left_only[1] = 0.
        left = phase('left_only_'+args.motion_profile, motion_duration, True, left_only, np.zeros((2, 3)), True)
        left['movement'] = movement_metrics(models, start_q, latest['joints']['positions'], left_only)
        lm = left['movement']
        if (lm['progress_ratio'][0] < .4 or lm['tool_error_m'][0] > .015
                or lm['joint_movement_max_rad'][0] < .002 or lm['tool_error_m'][1] > .01):
            raise ValueError('Actual left-arm movement/progress or independent right hold failed')
        before_right = np.asarray(latest['joints']['positions'])
        right = phase('right_'+args.motion_profile+'_left_held', motion_duration, True, offsets, left_only, True)
        right_delta = np.zeros((2, 3)); right_delta[1] = offsets[1]
        right['movement'] = movement_metrics(models, before_right, latest['joints']['positions'], right_delta)
        rm = right['movement']
        if (rm['progress_ratio'][1] < .4 or rm['tool_error_m'][1] > .015
                or rm['joint_movement_max_rad'][1] < .002 or rm['tool_error_m'][0] > .01):
            raise ValueError('Actual right-arm movement/progress or independent left hold failed')
        release = phase('release_both_hold', 1.2, False, expect_active=False)
        assert_inactive(); verify_hold(release)
        if args.orientation_check:
            orientation_start = np.asarray(latest['joints']['positions'])
            reference = np.array([m.fk(q) for m, q in zip(models, orientation_start)])
            rotation_vectors, orientation_preflight = preflight_orientation(models, orientation_start)
            report['orientation_preflight'] = orientation_preflight
            report['orientation_limits'] = dict(angle_rad=ORIENTATION_ANGLE, minimum_progress=.4,
                                                maximum_error_rad=ORIENTATION_MAX_ERROR,
                                                maximum_position_drift_m=ORIENTATION_MAX_DRIFT)
            report['orientation_preflight_feedback_wait_s'] = wait_for_fresh_feedback(
                fresh, lambda wait: rclpy.spin_once(node, timeout_sec=wait))
            if not graph_ready(simulation_graph()):
                raise ValueError('Simulator ownership changed during orientation preflight')
            assert_inactive()
            # Feasibility work is done while held. Renew the release handshake
            # afterward; its elapsed CPU time cannot create an active input gap.
            phase('orientation_fresh_release', .8, False, expect_active=False)
            phase('orientation_measured_pose_clutch', .6, True, expect_active=True)
            orientation_start = np.asarray(latest['joints']['positions'])
            previous_vectors = np.zeros((2, 3))
            for side, name in enumerate(('left', 'right')):
                active_vectors = previous_vectors.copy(); active_vectors[side] = rotation_vectors[side]
                row = phase(name+'_orientation_only', 6., True, expect_active=True,
                            rotations=active_vectors, rotation_ramp_from=previous_vectors)
                metric = orientation_metrics(models, orientation_start, latest['joints']['positions'], active_vectors)
                row['orientation'] = metric
                if (metric['progress_ratio'][side] < .4 or metric['orientation_error_rad'][side] > ORIENTATION_MAX_ERROR
                        or max(metric['position_drift_m']) > ORIENTATION_MAX_DRIFT
                        or metric['joint_movement_max_rad'][side] < .002):
                    raise ValueError('Measured '+name+' orientation response/progress or position hold failed')
                if side == 0 and metric['measured_angle_rad'][1] > .03:
                    raise ValueError('Left-only orientation input moved the other arm')
                if side == 1 and metric['orientation_error_rad'][0] > ORIENTATION_MAX_ERROR:
                    raise ValueError('Right orientation input did not preserve the achieved left orientation')
                previous_vectors = active_vectors
            held = phase('orientation_release_both_hold', 1.2, False, rotations=rotation_vectors, expect_active=False)
            assert_inactive(); verify_hold(held)
        # A release does not recalibrate absolute goals. Explicitly map the
        # measured pose for timeout/rearm hold phases to avoid returning to the
        # earlier fixture when either orientation or position has changed.
        reference = np.array([m.fk(q) for m, q in zip(models, latest['joints']['positions'])])
        phase('timeout_measured_pose_clutch', .6, True, expect_active=True)
        timeout = phase('input_silence_timeout', .9, expect_active=False)
        timeout['backend_release'] = timeout_release_metrics(report['commands'], timeout['started_unix'],
                                                             timeout['last_input_stamp_at_start'])
        assert_inactive(); verify_hold(timeout)
        if latest['status'].get('teleop_reason') != 'teleop_input_timeout':
            raise ValueError('Silence did not report the expected input timeout')
        phase('held_without_release_must_not_resume', .5, True, expect_active=False)
        assert_inactive()
        phase('explicit_rearm_release', .6, False, expect_active=False)
        if not latest['status'].get('teleop_ready'): raise ValueError('Release failed to rearm after timeout')
        phase('rearmed_measured_pose_clutch', .6, True, expect_active=True)
        final = phase('final_both_hold', 1., False, expect_active=False)
        assert_inactive(); verify_hold(final)
        report['passed'] = True
    except (Exception, KeyboardInterrupt) as error:
        report['errors'].append(type(error).__name__+': '+str(error))
    finally:
        # Only send a release after all simulation/ownership preconditions passed.
        # This is the same public input protocol, never a direct actuator command.
        if authorized and rclpy.ok():
            cleanup_end = time.monotonic()+.4
            next_release = time.monotonic()
            while time.monotonic() < cleanup_end:
                now = time.monotonic()
                if now >= next_release:
                    send(False, np.zeros((2, 3)))
                    next_release = now+.05
                rclpy.spin_once(node, timeout_sec=min(.005, max(0., cleanup_end-time.monotonic())))
            report['cleanup_status'] = latest['status']
        report['ended_unix'] = time.time()
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps(dict(output=str(args.output), passed=report['passed'], errors=report['errors'])), flush=True)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
