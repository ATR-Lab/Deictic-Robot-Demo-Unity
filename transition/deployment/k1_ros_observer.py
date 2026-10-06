#!/usr/bin/env python3
"""Read-only vendor ROS telemetry and allowlisted RPC snapshots for commissioning.

No motor publisher, mode change, controller wrapper or movement RPC exists in
this process. DDS source timestamps describe publication, not sensor acquisition.
Snapshots deliberately cannot satisfy the physical release evidence contract.
"""
from __future__ import annotations
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import time
import uuid

from observation_codec import mapped_joints, stamp_ns

READ_QUERIES = {
    'robot_info': (2022, None),
    'robot_status': (2018, None),
    'body_to_left_hand': (2011, {'src': 0, 'dst': 2}),
    'body_to_right_hand': (2011, {'src': 0, 'dst': 3}),
}


def read_request(query):
    if query not in READ_QUERIES:
        raise ValueError('Query is not on the read-only allowlist')
    api_id, parameters = READ_QUERIES[query]
    return api_id, '' if parameters is None else json.dumps(parameters, separators=(',', ':'))


def strict_object(raw):
    if not isinstance(raw, str) or len(raw.encode()) > 65536:
        raise ValueError('Invalid/bounded RPC response')
    def pairs(items):
        out = {}
        for key, value in items:
            if key in out: raise ValueError('Duplicate RPC JSON key')
            out[key] = value
        return out
    value = json.loads(raw, object_pairs_hook=pairs,
                       parse_constant=lambda _: (_ for _ in ()).throw(ValueError('Nonfinite RPC response')))
    if not isinstance(value, dict): raise ValueError('Expected RPC object')
    json.dumps(value, allow_nan=False)  # Reject overflow-to-infinity numbers too.
    return value


def motor_payload(message):
    motors = list(message.motor_state_serial)
    if len(motors) != 22: raise ValueError('Expected 22 K1 serial motors')
    result = []
    for motor in motors:
        values = [float(getattr(motor, key)) for key in ('q','dq','ddq','tau_est')]
        if any(not math.isfinite(x) for x in values): raise ValueError('Nonfinite motor state')
        reserve = list(motor.reserve)
        if len(reserve) != 2: raise ValueError('Invalid motor reserve vector')
        result.append(dict(zip(('q','dq','ddq','tau_est'),values), mode=int(motor.mode),
                           temperature=int(motor.temperature), lost=int(motor.lost),
                           error_code=int(reserve[0]), communication_frequency=int(reserve[1])))
    return {'motors_serial':result,
            'arm_indices':list(range(2,10)),
            'arm_motor_health_clear':all(m['lost']==0 and m['error_code']==0 for m in result[2:10]),
            'health_interpretation':'Reported bits/counters only; no commissioning inference'}


def atomic_snapshot(path, value):
    encoded=json.dumps(value,allow_nan=False,separators=(',',':'))
    tmp=path.with_name(path.name+'.'+str(os.getpid())+'.tmp')
    with tmp.open('w',encoding='utf-8') as stream:
        stream.write(encoded);stream.flush();os.fsync(stream.fileno())
    os.replace(tmp,path)


def publication_stamp(info):
    # Jazzy supplies MessageInfo as a TypedDict; older clients may expose attributes.
    value=info.get('source_timestamp') if isinstance(info,dict) else getattr(info,'source_timestamp',None)
    return value if type(value) is int and value>0 else None


def main():
    if (Path(__file__).resolve().parents[1]/'.runtime/k1-diagnostics-hold.json').exists():
        raise SystemExit('K1 diagnostics held after the 2026-09-24 memory/reset incident; see docs/K1_RESET_INCIDENT.md')
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from booster_interface.msg import LowState,RobotStatesMsg,FallDownState
    from booster_interface.srv import RpcService
    from rosidl_runtime_py.convert import message_to_ordereddict
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',required=True,type=Path)
    parser.add_argument('--seconds',type=float,default=30,help='0 keeps observing until interrupted')
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds<0:parser.error('seconds must be finite/nonnegative')
    args.output_dir.mkdir(parents=True,exist_ok=True)
    # A second writer must never replace a live observation stream.
    import fcntl
    lock=(args.output_dir/'observer.lock').open('a+')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    boot=str(uuid.uuid4()); clock_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    evidence=(args.output_dir/('observations-'+boot+'.jsonl')).open('x',encoding='utf-8')
    rclpy.init()
    class Observer(Node):
        def __init__(self):
            super().__init__('transition_physical_observer',start_parameter_services=False,enable_rosout=False)
            self.values={};self.raw_values={};self.decoded_receipts={};self.counts={};self.errors={};self.sequence=0;self.query_index=0
            self.pending=None;self.next_query=0.;self.rpc_counts={};self.graph={};self.running=True
            self.client=self.create_client(RpcService,'/booster_rpc_service')
            qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
            self.subscriptions_owned=[]
            for topic,cls in [('/joint_states',JointState),('/low_state',LowState),('/robot_states',RobotStatesMsg),('/fall_down',FallDownState)]:
                self.subscriptions_owned.append(self.create_subscription(cls,topic,
                    lambda msg,info,t=topic:self.receive(t,msg,info),qos))
            self.create_timer(.05,self.poll_rpc)
            self.create_timer(.2,self.publish_snapshot)
            self.create_timer(5.,self.inspect_graph)
        def receive(self,topic,msg,info):
            # Vendor joint/motor streams can each exceed 400 Hz. Capture the
            # newest message and its actual receipt time; validate/serialize at
            # the snapshot cadence so RPC responses can share this executor.
            self.raw_values[topic]=(msg,time.monotonic(),publication_stamp(info))
            self.counts[topic]=self.counts.get(topic,0)+1
        def decode_latest(self,topic,raw):
            msg,received,publication=raw
            if self.decoded_receipts.get(topic)==received:return
            self.decoded_receipts[topic]=received
            try:
                if topic=='/joint_states':
                    names,positions,velocities=mapped_joints(list(msg.name),list(msg.position),list(msg.velocity))
                    payload={'names':names,'positions':positions,'velocities':velocities,'source_stamp_ns':stamp_ns(msg.header)}
                elif topic=='/low_state':payload=motor_payload(msg)
                else:payload=dict(message_to_ordereddict(msg))
                json.dumps(payload,allow_nan=False)
                self.values[topic]={'payload':payload,'received_at_monotonic':received,
                    'dds_source_timestamp_ns':publication,'acquisition_timestamp_known':False}
                self.errors.pop(topic,None)
            except (ValueError,TypeError) as exc:self.errors[topic]=str(exc)
        def poll_rpc(self):
            now=time.monotonic()
            if self.pending:
                name,future,started=self.pending
                if future.done():
                    try:
                        response=future.result().msg
                        if response.status!=0:raise ValueError('Vendor RPC status '+str(response.status))
                        payload=strict_object(response.body)
                        self.values[name]={'payload':payload,'request_started_at_monotonic':started,
                            'received_at_monotonic':now,'acquisition_timestamp_known':False}
                        self.rpc_counts[name]=self.rpc_counts.get(name,0)+1;self.errors.pop(name,None)
                    except Exception as exc:self.errors[name]=str(exc)[:1024]
                    self.pending=None;self.next_query=now+.1
                elif now-started>1.5:
                    future.cancel();self.client.remove_pending_request(future)
                    self.errors[name]='Read-only RPC timeout; acquisition unknown'
                    self.pending=None;self.next_query=now+1.
                return
            if now<self.next_query or not self.client.service_is_ready():return
            name=list(READ_QUERIES)[self.query_index%len(READ_QUERIES)];self.query_index+=1
            api,body=read_request(name)
            request=RpcService.Request();request.msg.api_id=api;request.msg.body=body
            self.pending=(name,self.client.call_async(request),now)
        def inspect_graph(self):
            self.graph={topic:[{'node':p.node_name,'namespace':p.node_namespace,'endpoint_gid':list(p.endpoint_gid)}
                              for p in self.get_publishers_info_by_topic(topic)]
                        for topic in ('/joint_ctrl','/low_cmd','/low_state','/joint_states','/robot_states')}
        def snapshot(self):
            for topic,raw in self.raw_values.items():self.decode_latest(topic,raw)
            now=time.monotonic();self.sequence+=1
            values=copy.deepcopy(self.values)
            for value in values.values():value['receipt_age_s']=now-value['received_at_monotonic']
            return {'schema_version':1,'mode':'hardware_observation','motion_capability':False,
                'boot_id':boot,'clock_id':clock_id,'sequence':self.sequence,'observer_running':self.running,
                'generated_at_monotonic':now,'generated_at_utc':time.time(),
                'acquisition_freshness_verified':False,'device_fencing_proven':False,
                'observations':values,'sample_counts':dict(self.counts),'rpc_counts':dict(self.rpc_counts),
                'sample_counts_semantics':'Received ROS messages; only the latest message per topic is validated for each snapshot',
                'errors':dict(self.errors),'publisher_inventory':self.graph,
                'read_only_api_ids':sorted({q[0] for q in READ_QUERIES.values()}),
                'limitations':['DDS publication stamps are not sensor acquisition bounds',
                               'RPC response time does not identify measurement acquisition time',
                               'Publisher discovery is not exclusive command-owner enforcement']}
        def publish_snapshot(self):
            value=self.snapshot();atomic_snapshot(args.output_dir/'latest.json',value)
            evidence.write(json.dumps(value,allow_nan=False,separators=(',',':'))+'\n');evidence.flush()
    node=Observer();started=time.monotonic()
    # Keep the node registered across spins instead of repeatedly adding/removing
    # it through the rclpy.spin_once convenience function.
    executor=SingleThreadedExecutor();executor.add_node(node)
    try:
        while rclpy.ok() and (not args.seconds or time.monotonic()-started<args.seconds):executor.spin_once(timeout_sec=.05)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.running=False;node.publish_snapshot()
        summary={'schema_version':1,'boot_id':boot,'duration_s':time.monotonic()-started,
                 'sample_counts':node.counts,'rpc_counts':node.rpc_counts,'errors':node.errors,
                 'motion_capability':False,'latest_snapshot_sha256':hashlib.sha256((args.output_dir/'latest.json').read_bytes()).hexdigest()}
        with (args.output_dir/('summary-'+boot+'.json')).open('x') as stream:json.dump(summary,stream,indent=2)
        print(json.dumps(summary));executor.shutdown(timeout_sec=2.);node.destroy_node();rclpy.try_shutdown();evidence.close();lock.close()


if __name__=='__main__':main()
