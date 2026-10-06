#!/usr/bin/env python3
"""One-way vendor domain 0 -> Unity domain 174 diagnostic bridge.

The vendor context creates subscriptions only. The Unity context has no
subscription, service, action, SDK or command path back into the vendor domain.
"""
from __future__ import annotations
import argparse
import copy
import json
from pathlib import Path
import threading
import time
import uuid

from observation_codec import mapped_joints, stamp_ns, checked_jpeg


def main():
    if (Path(__file__).resolve().parents[1]/'.runtime/k1-diagnostics-hold.json').exists():
        raise SystemExit('K1 diagnostics held after the 2026-09-24 memory/reset incident; see docs/K1_RESET_INCIDENT.md')
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState, CompressedImage, CameraInfo, Imu
    from std_msgs.msg import String
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vendor-domain", type=int, default=0)
    parser.add_argument("--unity-domain", type=int, default=174)
    args = parser.parse_args()
    if args.vendor_domain == args.unity_domain: parser.error("domains must differ")
    vendor, public = Context(), Context()
    rclpy.init(context=vendor, domain_id=args.vendor_domain)
    rclpy.init(context=public, domain_id=args.unity_domain)
    source = Node("transition_k1_receive_only", context=vendor, start_parameter_services=False,
                  enable_rosout=False)
    output = Node("transition_k1_observation", context=public)
    executor = SingleThreadedExecutor(context=vendor);executor.add_node(source)
    lock=threading.Lock();latest={};counts={};errors={};boot=str(uuid.uuid4());last_sent={};source_stamps={}
    q=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    routes=[
        ("/joint_states", "/joint_states", JointState),
        ("/transition/k1/stereo/compressed", "/deictic/camera_view/stereo/image_raw/compressed", CompressedImage),
        ("/transition/k1/head/compressed", "/transition/hardware/head/image_raw/compressed", CompressedImage),
        ("/transition/k1/camera_status", "/transition/hardware/camera_status", String),
        ("/boostercamera/head/raw/rgb/camera_info", "/k1/head_camera/left/camera_info", CameraInfo),
        ("/boostercamera/head/raw/right/rgb/camera_info", "/k1/head_camera/right/camera_info", CameraInfo),
        ("/send_imu", "/transition/hardware/imu", Imu),
    ]
    publishers={destination:output.create_publisher(typ,destination,1) for _,destination,typ in routes}
    status=output.create_publisher(String,"/transition/hardware/status",1)
    def receive(origin, destination, msg):
        try:
            if isinstance(msg,JointState):
                names,positions,velocities=mapped_joints(list(msg.name),list(msg.position),list(msg.velocity))
                mapped=JointState();mapped.header=copy.deepcopy(msg.header)
                mapped.name=names;mapped.position=positions;mapped.velocity=velocities;msg=mapped
            stamp = stamp_ns(msg.header) if hasattr(msg,"header") else None
            if isinstance(msg,CompressedImage):
                checked_jpeg(msg)
                if not msg.header.frame_id:
                    raise ValueError('JPEG source frame missing')
                msg=copy.deepcopy(msg)
                msg.format='rgb8; jpeg compressed bgr8'
            with lock:
                if stamp is not None:
                    if stamp <= source_stamps.get(origin,0):
                        errors[origin]='duplicate_or_regressing_source_stamp';return
                    source_stamps[origin]=stamp
                latest[destination]=(msg,time.monotonic());counts[origin]=counts.get(origin,0)+1
                errors.pop(origin,None)
        except (ValueError,TypeError) as exc:
            with lock:errors[origin]=str(exc)
    subscriptions=[source.create_subscription(typ,origin,
        lambda msg,o=origin,d=destination:receive(o,d,msg),q) for origin,destination,typ in routes]
    def forward():
        with lock:values=dict(latest)
        for destination,(message,receipt) in values.items():
            if time.monotonic()-receipt>.5 or last_sent.get(destination)==receipt: continue
            publishers[destination].publish(message);last_sent[destination]=receipt
    def report():
        now=time.monotonic()
        with lock:
            data={"schema_version":1,"boot_id":boot,"mode":"hardware_observation","motion_capability":False,
                  "vendor_domain":args.vendor_domain,"unity_domain":args.unity_domain,
                  "sample_counts":dict(counts),"receipt_age_s":{k:now-v[1] for k,v in latest.items()},
                  "validation_errors":dict(errors),"acquisition_freshness_verified":False,
                  "source_stamps_ns":dict(source_stamps),
                  "clock_offset_verified":False,"physical_release":"blocked",
                  "blockers":["identity_and_sdk_release","source_timing","commissioned_profiles",
                              "support_and_independent_stop","enforced_command_owner"]}
        status.publish(String(data=json.dumps(data,allow_nan=False)))
    output.create_timer(1/30,forward);output.create_timer(1,report)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    print(json.dumps({"ready":True,"mode":"observation_only","vendor_domain":args.vendor_domain,
                      "unity_domain":args.unity_domain,"motion_capability":False}),flush=True)
    public_executor=SingleThreadedExecutor(context=public);public_executor.add_node(output)
    try:public_executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown(timeout_sec=2);public_executor.shutdown(timeout_sec=2)
        source.destroy_node();output.destroy_node()
        rclpy.shutdown(context=vendor);rclpy.shutdown(context=public);thread.join(timeout=2)

if __name__ == "__main__": main()
