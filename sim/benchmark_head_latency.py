#!/usr/bin/env python3
"""Opt-in small head-step and display-camera transport measurement in Isaac.

Run on the ROS workstation with Unity disconnected. This measures command-to-
status/physics feedback and received ROS camera age, never sensor-to-eye latency.
It leaves markerless registration running and requests only the selected display
camera topic. The final action is an acknowledged inactive measured hold.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import time
import uuid

from k1_model import HEAD_JOINTS
from verify_head_live import prepare_output, quaternion

ROOT = Path(__file__).resolve().parents[1]
NODE_NAME = 'synthetic_isaac_head_benchmark'
ISAAC_NODE = 'k1_fixed_base_isaac'
PHASES = [('neutral', 0., 0.), ('left_up', .20, -.10), ('right_down', -.20, .10),
          ('left_down', .20, .10), ('right_up', -.20, -.10), ('neutral_return', 0., 0.)]
PHASE_SECONDS = 4.
COMMAND_PERIOD = .02


def finite_pair(value):
    return (isinstance(value, list) and len(value) == 2 and
            all(type(v) in (int, float) and math.isfinite(v) for v in value))


def distribution(values):
    values = sorted(float(value) for value in values)
    if not values:
        return dict(count=0, mean=None, median=None, p95=None, maximum=None)
    at = .95*(len(values)-1)
    low, high = math.floor(at), math.ceil(at)
    return dict(count=len(values), mean=statistics.fmean(values), median=statistics.median(values),
                p95=values[low]+(values[high]-values[low])*(at-low), maximum=values[-1])


def graph_ready(value):
    return (value['isaac_node_count'] == 1 and value['head_consumers'] == [ISAAC_NODE]
            and value['head_publishers'] == [NODE_NAME]
            and value['status_publishers'] == value['joint_publishers'] == [ISAAC_NODE]
            and value['camera_publishers'] in ([ISAAC_NODE], ['deictic_camera_view_relay']))


def summarize_phase(samples, *, started, start_stamp, first_sequence, session, initial, target):
    """All latencies use monotonic callback receipt; the status is sampled data."""
    accepted = [sample for sample in samples if sample['received_monotonic'] >= started
                and sample['stamp'] >= start_stamp and sample.get('session_id') == session
                and sample.get('active') is True and type(sample.get('sequence')) is int
                and sample['sequence'] >= first_sequence and finite_pair(sample.get('targets'))
                and finite_pair(sample.get('measured'))
                and max(abs(a-b) for a, b in zip(sample['targets'], target)) < 1e-5]
    step = [abs(goal-before) for goal, before in zip(target, initial)]
    threshold = [max(.005, .1*distance) for distance in step]
    moved = [s for s in accepted if max(abs(a-b) for a, b in zip(s['measured'], initial)) >= .005]
    reached = [s for s in accepted if all(abs(a-b) <= limit for a, b, limit in
                                        zip(s['measured'], target, threshold))]
    meaningful = max(step) >= .01
    errors = [max(abs(a-b) for a, b in zip(s['measured'], target)) for s in accepted]
    intervals = [b['received_monotonic']-a['received_monotonic'] for a, b in zip(accepted, accepted[1:])]
    latency = lambda records: records[0]['received_monotonic']-started if records else None
    return dict(accepted_status_samples=len(accepted), requested_step_rad=step,
                first_matching_status_s=latency(accepted), first_measured_movement_s=latency(moved),
                reached_90_percent_s=latency(reached) if meaningful else None,
                meaningful_step=meaningful, already_in_90_percent_band=not meaningful,
                ninety_percent_band_rad=threshold, converged=bool(errors and errors[-1] <= .02),
                final_measured_rad=accepted[-1]['measured'] if accepted else None,
                final_max_error_rad=errors[-1] if errors else None,
                tracking_error_rad=distribution(errors), observed_status_interval_s=distribution(intervals))


def camera_summary(samples, started, ended):
    frames = [s for s in samples if started <= s['received_monotonic'] <= ended]
    span = frames[-1]['received_monotonic']-frames[0]['received_monotonic'] if len(frames) > 1 else 0.
    return dict(frames=len(frames), window_s=max(0., ended-started),
                received_rate_hz=(len(frames)-1)/span if span > 0 else None,
                acquisition_age_s=distribution([s['acquisition_age_s'] for s in frames]),
                payload_bytes=distribution([s['payload_bytes'] for s in frames]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--allow-sim-motion', action='store_true')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--label', default='measurement', help='For example baseline or updated')
    parser.add_argument('--camera-topic', default='/deictic/camera_view/stereo/image_raw')
    parser.add_argument('--timeout', type=float, default=45., help='Whole-run bound including final hold, 35..60s')
    args = parser.parse_args()
    if not args.allow_sim_motion:
        parser.error('Add --allow-sim-motion only for the isolated Isaac simulator')
    if not math.isfinite(args.timeout) or not 35 <= args.timeout <= 60:
        parser.error('Timeout must be 35..60 seconds')
    if not args.camera_topic.startswith('/') or any(c.isspace() for c in args.camera_topic):
        parser.error('Camera topic must be an absolute ROS topic without whitespace')
    try:
        prepare_output(args.output)
    except OSError as error:
        parser.error(f'Evidence output is not writable; no commands sent: {error}')
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, CompressedImage
    from std_msgs.msg import String

    paths = ['sim/benchmark_head_latency.py', 'sim/verify_head_live.py', 'sim/head_control.py',
             'sim/k1_isaac.py', 'sim/loop_timing.py', 'sim/head_stereo.py',
             'ros2/scripts/camera_view_relay.py', 'ros2/scripts/run_endpoint.py']
    hashes = {path: hashlib.sha256((ROOT/path).read_bytes()).hexdigest() for path in paths}
    report = dict(schema_version=1, label=args.label, passed=False, started_unix=time.time(),
        source_sha256=hashes, scope='Synthetic command-to-ROS-feedback and camera transport; NOT sensor-to-eye latency',
        measurement_note='Head movement and 90% crossing use measured joints in head status. '
                         'Status is normally 10Hz; receipt sampling and scheduling limit latency precision.',
        camera_topic=args.camera_topic, camera_type='CompressedImage' if args.camera_topic.endswith('/compressed') else 'Image',
        command_period_s=COMMAND_PERIOD, phase_duration_s=PHASE_SECONDS, timeout_s=args.timeout,
        phases=[], statuses=[], cameras=[], commands=[], errors=[], rejected_camera_samples=0)
    rclpy.init()
    node = rclpy.create_node(NODE_NAME)
    publisher = node.create_publisher(String, '/k1/head/command', 1)
    session, sequence, authorized = str(uuid.uuid4()), 0, False
    started = time.monotonic(); deadline = started+args.timeout
    latest = {'status': None, 'camera': None}
    next_send, next_graph = 0., 0.

    def wall(): return node.get_clock().now().nanoseconds*1e-9

    def on_status(message):
        try:
            value = json.loads(message.data)
            if (value.get('schema_version') != 1 or value.get('names') != list(HEAD_JOINTS)
                    or not finite_pair(value.get('measured')) or not finite_pair(value.get('targets'))
                    or not -.05 <= wall()-value.get('stamp', 0.) <= .75
                    or (latest['status'] and value['stamp'] <= latest['status']['stamp'])):
                return
            sample = dict(value, received_monotonic=time.monotonic(), received_unix=time.time())
            latest['status'] = sample; report['statuses'].append(sample)
        except (ValueError, TypeError, AttributeError):
            pass

    def on_camera(message):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        age = wall()-stamp
        size = len(message.data)
        if (stamp <= 0 or not math.isfinite(age) or age < -.05 or size <= 0 or size > 16_000_000
                or (latest['camera'] and stamp <= latest['camera']['stamp'])):
            report['rejected_camera_samples'] += 1
            return
        sample = dict(stamp=stamp, received_monotonic=time.monotonic(), received_unix=time.time(),
                      acquisition_age_s=age, payload_bytes=size, frame_id=message.header.frame_id)
        latest['camera'] = sample; report['cameras'].append(sample)

    node.create_subscription(String, '/k1/head/status', on_status, 10)
    image_type = CompressedImage if args.camera_topic.endswith('/compressed') else Image
    node.create_subscription(image_type, args.camera_topic, on_camera,
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def graph():
        pubs = lambda topic: sorted(i.node_name for i in node.get_publishers_info_by_topic(topic))
        return dict(isaac_node_count=node.get_node_names().count(ISAAC_NODE),
            head_consumers=sorted(i.node_name for i in node.get_subscriptions_info_by_topic('/k1/head/command')),
            head_publishers=pubs('/k1/head/command'), status_publishers=pubs('/k1/head/status'),
            joint_publishers=pubs('/joint_states'), camera_publishers=pubs(args.camera_topic))

    def fresh():
        status = latest['status']
        return status is not None and -.05 <= wall()-status['stamp'] <= .75 and time.monotonic()-status['received_monotonic'] <= .75

    def send(yaw=0., pitch=0., active=False, force=False):
        nonlocal sequence, next_send
        now = time.monotonic()
        if not authorized: raise ValueError('Read-only preflight did not authorize motion')
        if not force and now < next_send: return None
        sequence += 1
        command = dict(schema_version=1, frame_id='base_link', session_id=session, sequence=sequence,
            stamp=wall(), active=active, tracked=True, orientation=quaternion(yaw, pitch))
        publisher.publish(String(data=json.dumps(command, allow_nan=False)))
        report['commands'].append(dict(command, sent_monotonic=now))
        next_send = now+COMMAND_PERIOD
        return report['commands'][-1]

    def tick(check_graph=True):
        nonlocal next_graph
        if time.monotonic() >= deadline-3.:
            raise TimeoutError('Measurement deadline reached; reserving final hold time')
        if authorized and check_graph and time.monotonic() >= next_graph:
            current = graph(); next_graph = time.monotonic()+.5
            if not graph_ready(current): raise ValueError('Simulator ownership/publisher graph changed')
        rclpy.spin_once(node, timeout_sec=.005)

    def hold(seconds, final=False):
        first = send(active=False, force=True)
        until = min(deadline, time.monotonic()+seconds)
        while time.monotonic() < until:
            send(active=False)
            rclpy.spin_once(node, timeout_sec=.005)
            status = latest['status']
            if (fresh() and status.get('session_id') == session and status.get('active') is False
                    and status.get('reason') == 'head_view_inactive' and status['stamp'] >= first['stamp']
                    and type(status.get('sequence')) is int and status['sequence'] >= first['sequence']):
                return dict(acknowledged=True, latency_s=status['received_monotonic']-first['sent_monotonic'],
                            status=status)
        return dict(acknowledged=False, status=latest['status'])

    try:
        preflight_end = min(deadline-3., started+10.)
        while time.monotonic() < preflight_end:
            tick(False)
            camera = latest['camera']
            if (graph_ready(graph()) and fresh() and latest['status'].get('active') is False
                    and camera and time.monotonic()-camera['received_monotonic'] <= 2. and camera['acquisition_age_s'] <= 2.):
                break
        else:
            raise ValueError('Unique idle Isaac graph, fresh head feedback and selected camera unavailable: '+json.dumps(graph()))
        report['preflight_graph'] = graph()
        authorized = True
        report['startup_hold'] = hold(3.)
        if not report['startup_hold']['acknowledged']: raise TimeoutError('Startup inactive hold was not acknowledged')
        for name, yaw, pitch in PHASES:
            if not fresh(): raise ValueError('Head feedback stale before phase')
            initial = list(latest['status']['measured'])
            first = send(yaw, pitch, True, True)
            phase = dict(name=name, target_rad=[yaw, pitch], initial_measured_rad=initial,
                         started_monotonic=first['sent_monotonic'], first_sequence=first['sequence'],
                         started_source_stamp=first['stamp'])
            report['phases'].append(phase)
            until = min(deadline-3., first['sent_monotonic']+PHASE_SECONDS)
            try:
                while time.monotonic() < until:
                    send(yaw, pitch, True); tick()
                    if not fresh(): raise ValueError('Head feedback stale during phase')
            finally:
                ended = time.monotonic()
                phase.update(ended_monotonic=ended, elapsed_s=ended-first['sent_monotonic'])
                phase['head'] = summarize_phase(report['statuses'], started=first['sent_monotonic'],
                    start_stamp=first['stamp'], first_sequence=first['sequence'], session=session, initial=initial, target=[yaw, pitch])
                phase['camera'] = camera_summary(report['cameras'], first['sent_monotonic'], ended)
                print('DEICTIC_HEAD_LATENCY_PHASE '+json.dumps(phase), flush=True)
        report['passed'] = all(p['head']['converged'] and p['camera']['frames'] > 0 and
                               p['elapsed_s'] >= PHASE_SECONDS-.02 for p in report['phases'])
        if not report['passed']: report['errors'].append('One or more steps did not converge or lacked camera delivery')
    except (Exception, KeyboardInterrupt) as error:
        report['errors'].append(f'{type(error).__name__}: {error}')
    finally:
        if authorized:
            try:
                report['final_hold'] = hold(3., final=True)
            except Exception as error:
                report['final_hold'] = dict(acknowledged=False, error=str(error))
            if not report['final_hold']['acknowledged']:
                report['passed'] = False; report['errors'].append('Final inactive measured hold was not acknowledged')
        report['ended_unix'] = time.time()
        report['elapsed_s'] = time.monotonic()-started
        report['summary'] = dict(phases_completed=len(report['phases']),
            phases_converged=sum(p.get('head', {}).get('converged', False) for p in report['phases']),
            first_matching_status_s=distribution([p['head']['first_matching_status_s'] for p in report['phases'] if p['head']['first_matching_status_s'] is not None]),
            first_measured_movement_s=distribution([p['head']['first_measured_movement_s'] for p in report['phases'] if p['head']['first_measured_movement_s'] is not None]),
            reached_90_percent_s=distribution([p['head']['reached_90_percent_s'] for p in report['phases'] if p['head']['reached_90_percent_s'] is not None]),
            camera=camera_summary(report['cameras'], started, time.monotonic()))
        changed = [path for path, digest in hashes.items() if hashlib.sha256((ROOT/path).read_bytes()).hexdigest() != digest]
        if changed:
            report['passed'] = False; report['errors'].append('Source changed during benchmark: '+', '.join(changed))
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        node.destroy_node(); rclpy.shutdown()
    print('DEICTIC_HEAD_LATENCY_RESULT '+json.dumps(dict(passed=report['passed'], output=str(args.output),
                                                        summary=report['summary'], errors=report['errors'])), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
