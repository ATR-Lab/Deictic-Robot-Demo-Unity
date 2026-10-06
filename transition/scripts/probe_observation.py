#!/usr/bin/env python3
"""Bounded read-only ROS inventory/sample receipt. Never invokes a service/action."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState, CompressedImage
    from std_msgs.msg import String
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=15)
    parser.add_argument('--output',required=True)
    parser.add_argument('--vendor',action='store_true')
    args=parser.parse_args()
    if not 1<=args.seconds<=120:parser.error('seconds must be in 1..120')
    destination=Path(args.output)
    if destination.exists():parser.error('output exists; choose a fresh evidence path')
    rclpy.init();node=rclpy.create_node('transition_readonly_evidence_probe')
    stats={};start=time.monotonic();subs=[]
    topics=[('/joint_states',JointState),
        ('/transition/k1/stereo/compressed' if args.vendor else '/deictic/camera_view/stereo/image_raw/compressed',CompressedImage),
        ('/transition/k1/head/compressed' if args.vendor else '/transition/hardware/head/image_raw/compressed',CompressedImage),
        ('/transition/k1/camera_status' if args.vendor else '/transition/hardware/status',String)]
    def receive(topic,msg):
        now=time.monotonic();record=stats.setdefault(topic,{'count':0,'first_receipt_s':now-start})
        record['count']+=1;record['last_receipt_s']=now-start
        if isinstance(msg,String):record['latest_status']=json.loads(msg.data)
        else:
            record['last_source_stamp_ns']=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            record['frame_id']=msg.header.frame_id
            if isinstance(msg,JointState):record['names']=list(msg.name);record['positions']=list(msg.position);record['velocities']=list(msg.velocity)
            else:record['bytes']=len(msg.data);record['format']=msg.format
    for topic,typ in topics:subs.append(node.create_subscription(typ,topic,lambda m,t=topic:receive(t,m),QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)))
    try:
        while time.monotonic()-start<args.seconds:rclpy.spin_once(node,timeout_sec=.05)
        elapsed=time.monotonic()-start
        report={'schema_version':1,'created_at_utc':datetime.now(timezone.utc).isoformat(),'scope':'read_only_ros_diagnostics',
            'duration_s':elapsed,'topics':stats,'graph':node.get_topic_names_and_types(),
            'services_discovered_not_invoked':node.get_service_names_and_types(),'motion_commands':0,
            'acquisition_freshness_verified':False,'controller_identity_verified':False,
            'physical_release':'blocked'}
        for item in stats.values():item['last_receipt_age_s']=elapsed-item['last_receipt_s']
        destination.parent.mkdir(parents=True,exist_ok=True)
        with destination.open('x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
        print(json.dumps({'output':str(destination),'counts':{k:v['count'] for k,v in stats.items()},'physical_release':'blocked'}))
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
