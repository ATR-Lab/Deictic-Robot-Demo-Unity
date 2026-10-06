#!/usr/bin/env python3
"""Observe/shadow K1 ROS gateway; physical mode needs a trusted site assembly.

No SDK is imported or constructed in observe/shadow modes. Start requests are
named task profiles, never raw poses. Physical arming is local CLI-only and the
independent stop callback bypasses the serialized dispatcher.
"""
from __future__ import annotations

import argparse
import importlib
import json
from pathlib import Path
import time

from transition_autonomy.journal import canonical
from transition_autonomy.physical.gateway import FileObservation, PhysicalGateway
from transition_autonomy.physical.manifest import Manifest, strict_json
from transition_autonomy.physical.deployment import device_ledger_path


def handle(gateway, raw, clock_id):
    """ROS-independent strict wrapper; no external operation can arm or grant."""
    value = strict_json(raw)
    if (not isinstance(value, dict) or type(value.get('schema_version')) is not int
            or value['schema_version'] != 1 or value.get('clock_id') != clock_id):
        raise ValueError('Invalid gateway schema or same-host clock identity')
    operation = value.get('operation')
    field = 'command' if operation == 'start' else 'command_id' if operation == 'status' else None
    if field is None or set(value) != {'schema_version', 'clock_id', 'operation', field}:
        raise ValueError('Only exact named start/status requests are exposed')
    if operation == 'start':
        return gateway.submit(value['command'])
    return gateway.command_status(value['command_id'])


def make_node(gateway, *, context=None):
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String

    clock_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    class GatewayNode(Node):
        def __init__(self):
            super().__init__('transition_physical_gateway', context=context)
            # Multiple executor threads allow stop handling while command intake
            # is waiting for fsync. The independent process handles owner hangs.
            group = ReentrantCallbackGroup()
            qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            self.state_pub = self.create_publisher(String, '/transition/physical/state', qos)
            self.event_pub = self.create_publisher(String, '/transition/physical/event', qos)
            self.timer = self.create_timer(.1, self.publish_state, callback_group=group)
            if gateway.mode != 'observe':
                self.command = self.create_subscription(String, '/transition/physical/command', self.receive, qos,
                                                         callback_group=group)
            if gateway.mode == 'physical':
                self.stop = self.create_subscription(String, '/transition/physical/stop', self.receive_stop, qos,
                                                      callback_group=group)

        def publish_state(self):
            self.state_pub.publish(String(data=canonical({**gateway.snapshot(), 'clock_id': clock_id})))

        def receive(self, message):
            try:
                receipt = handle(gateway, message.data, clock_id)
            except Exception as error:
                receipt = {'error': str(error), 'motion_dispatched': None}
            self.event_pub.publish(String(data=canonical(receipt)))

        def receive_stop(self, message):
            # A stop request cannot grant or resume anything. Bound only its
            # diagnostic label; arbitrary command payloads are never evaluated.
            reason = message.data[:160] or 'operator_stop'
            self.event_pub.publish(String(data=canonical(gateway.request_stop(reason))))
    return GatewayNode()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('observe', 'shadow', 'physical'), default='observe')
    parser.add_argument('--domain', type=int, default=175, help='Isolated workstation command domain, never vendor domain0')
    parser.add_argument('--observation-file', default='.runtime/physical-observation/latest.json')
    parser.add_argument('--ledger', help='Absolute diagnostic ledger path; never share observe/shadow ledgers')
    parser.add_argument('--manifest', help='Complete immutable commissioned manifest for physical mode')
    parser.add_argument('--registry-root', help='Persistent absolute per-device registry for physical mode')
    parser.add_argument('--site-factory', help='Trusted installed module:function, physical mode only')
    parser.add_argument('--arm', action='store_true', help='Explicit local startup arm after all live release checks')
    args = parser.parse_args()
    if args.mode == 'physical' and (Path(__file__).resolve().parents[1]/'.runtime/k1-diagnostics-hold.json').exists():
        parser.error('K1 connection held after the 2026-09-24 memory/reset incident; see docs/K1_RESET_INCIDENT.md')
    if not 1 <= args.domain <= 232 or args.domain == 174:
        parser.error('Use an isolated nonzero command domain distinct from visualization domain174')
    observation = FileObservation(args.observation_file)
    if args.mode == 'physical':
        if not args.manifest or not args.registry_root or not args.site_factory or args.ledger:
            parser.error('Physical mode requires --manifest, --registry-root and --site-factory, without --ledger')
        manifest = Manifest.load(args.manifest)
        manifest.require_complete()
        if not manifest.payload['physical_actuation_enabled']:
            parser.error('Reviewed manifest leaves physical actuation disabled')
        ledger = device_ledger_path(args.registry_root, manifest.payload['robot']['serial']).with_name('gateway.sqlite')
        module, separator, function = args.site_factory.partition(':')
        if not separator or not module or not function:
            parser.error('Site factory must be an installed module:function')
        factory = getattr(importlib.import_module(module), function)
        # Factory verifies the trusted monitor/approval service and constructs
        # the three capability-limited ports. It may not manufacture evidence.
        components = factory(manifest=manifest, registry_root=Path(args.registry_root).resolve(), clock=time.monotonic)
        if set(components) != {'backend', 'transport', 'stop_worker'}:
            raise ValueError('Site factory must return exactly backend, transport and stop_worker')
        gateway = PhysicalGateway(mode='physical', ledger=ledger, identity=manifest.payload['robot']['serial'],
                                  observation=observation, **components)
        if args.arm:
            try:
                gateway.arm(manifest.payload['commissioning']['receipt_id'])
            except Exception:
                # A failed local arm must not leave an idle stop child waiting
                # forever during interpreter shutdown.
                gateway.close()
                raise
    else:
        if args.arm or args.site_factory or args.manifest or args.registry_root or not args.ledger:
            parser.error('Observe/shadow require --ledger and cannot construct physical components or arm')
        gateway = PhysicalGateway(mode=args.mode, ledger=Path(args.ledger), identity='read-only-diagnostic-source',
                                  observation=observation)
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor
    context = Context()
    rclpy.init(context=context, domain_id=args.domain)
    node = make_node(gateway, context=context)
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        receipt = gateway.request_stop('gateway_shutdown')
        executor.shutdown(timeout_sec=2)
        node.destroy_node()
        rclpy.shutdown(context=context)
        receipt.update(gateway.close())
        print(json.dumps(receipt, allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
