#!/usr/bin/env python3
"""Bounded receive-only check of measured simulation joints and real stereo pixels."""
import argparse
import json
from pathlib import Path
import time


def main():
    import numpy as np
    import rclpy
    from sensor_msgs.msg import Image,JointState
    from std_msgs.msg import String
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=25)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    if not 5<=args.seconds<=120:parser.error('seconds must be in 5..120')
    path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x') as f:f.write('{"passed":false,"status":"started"}\n')
    rclpy.init();node=rclpy.create_node('transition_visualization_validation')
    counts={};metadata={};stamps={'left':set(),'right':set()};subs=[]
    def callback(topic,msg):
        counts[topic]=counts.get(topic,0)+1
        if isinstance(msg,Image):
            eye='left' if '/left/' in topic else 'right'
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if len(stamps[eye])<3000:stamps[eye].add(stamp)
            data=np.frombuffer(msg.data,dtype=np.uint8)
            metadata[topic]={'source_stamp_ns':stamp,'frame':msg.header.frame_id,'width':msg.width,
                'height':msg.height,'encoding':msg.encoding,'bytes':len(data),
                'pixel_stddev':float(data.std()) if len(data) else 0}
        elif isinstance(msg,JointState):metadata[topic]={'names':list(msg.name),'positions':list(msg.position)}
        else:metadata[topic]=json.loads(msg.data)
    topics=[('/joint_states',JointState),('/transition/simulation/telemetry',String),
        ('/k1/head_camera/left/image_raw',Image),('/k1/head_camera/right/image_raw',Image)]
    for topic,typ in topics:subs.append(node.create_subscription(typ,topic,lambda m,t=topic:callback(t,m),1))
    started=time.monotonic()
    try:
        while time.monotonic()-started<args.seconds:rclpy.spin_once(node,timeout_sec=.05)
        matches=len(stamps['left']&stamps['right'])
        expected=['aaright_shoulder_pitch_joint','right_shoulder_roll_joint','right_elbow_pitch_joint','right_elbow_yaw_joint']
        passed=(all(counts.get(t,0)>10 for t,_ in topics) and matches>10
            and metadata.get('/joint_states',{}).get('names')==expected
            and all(metadata.get(t,{}).get('pixel_stddev',0)>1 for t,_ in topics if '/image_raw' in t)
            and metadata.get('/transition/simulation/telemetry',{}).get('simulation_only') is True
            and metadata.get('/transition/simulation/telemetry',{}).get('ros_command_inputs')==[])
        report={'scope':'native_simulation_receive_only','passed':passed,'duration_s':time.monotonic()-started,
                'counts':counts,'metadata':metadata,'matched_stereo_stamps':matches,'motion_commands':0}
        path.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print(json.dumps({'passed':passed,'counts':counts,'matched_stereo_stamps':matches,'output':str(path)}))
        return 0 if passed else 1
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':raise SystemExit(main())
