"""Bounded read-only wall-clock transport/cadence measurement of native images."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=30.)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not np.isfinite(args.seconds) or not 1<=args.seconds<=300:
        parser.error('--seconds must be within1..300')
    topics={
        '/deictic/headset/image_raw':Image,
        '/deictic/headset/depth':Image,
        '/k1/wrist_camera/image_raw':Image,
        '/joint_states':JointState,
    }
    records={topic:[] for topic in topics}
    dimensions={}
    rclpy.init();node=Node('deictic_camera_transport_measurement')
    def callback(topic,message):
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        row={'received_monotonic':time.monotonic(),'stamp':stamp,'age':time.time()-stamp}
        if isinstance(message,Image):
            row['bytes']=len(message.data)
            dimensions[topic]=dict(width=message.width,height=message.height,step=message.step,encoding=message.encoding)
        records[topic].append(row)
    subscriptions=[node.create_subscription(kind,topic,lambda message,t=topic:callback(t,message),qos_profile_sensor_data)
                   for topic,kind in topics.items()]
    started=time.monotonic()
    try:
        while time.monotonic()-started<args.seconds:
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        node.destroy_node();rclpy.shutdown()
    summary={}
    for topic,rows in records.items():
        item={'messages':len(rows),'image':dimensions.get(topic)}
        if rows:
            ages=np.array([r['age'] for r in rows]); stamps=np.array([r['stamp'] for r in rows])
            elapsed=rows[-1]['received_monotonic']-rows[0]['received_monotonic']
            item.update(age_min_median_p95_max_s=np.quantile(ages,[0,.5,.95,1]).tolist(),
                        receipt_rate_hz=(len(rows)-1)/elapsed if elapsed>0 else None,
                        nonpositive_stamps=int(np.sum(stamps<=0)),
                        duplicate_or_reordered_stamps=int(np.sum(np.diff(stamps)<=0)),
                        payload_bytes=sum(r.get('bytes',0) for r in rows))
            if len(rows)>1:
                item['capture_intervals_median_p95_max_s']=np.quantile(np.diff(stamps),[.5,.95,1]).tolist()
        summary[topic]=item
    report={'scope':'Read-only subscriber measurement; DDS deserialization load is included; no model inference',
            'seconds':args.seconds,'topics':summary,'samples':records}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(summary),flush=True)


if __name__=='__main__':
    main()
