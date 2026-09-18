#!/usr/bin/env python3
"""Capture an exact-timestamp synchronized pair from the live simulated cameras."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image as PILImage
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo


def transform(msg):
    p,q = msg.pose.position,msg.pose.orientation
    x,y,z,w = q.x,q.y,q.z,q.w
    matrix = np.eye(4)
    matrix[:3,:3] = [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    matrix[:3,3] = [p.x,p.y,p.z]
    return matrix.tolist()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output",type=Path,default=Path(__file__).parent/"artifacts")
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("capture_k1_camera_pair")
    frames, result = {},[]
    specs = {
        "head_rgb":(Image,"/deictic/headset/image_raw"),
        "head_depth":(Image,"/deictic/headset/depth"),
        "head_info":(CameraInfo,"/deictic/headset/camera_info"),
        "head_pose":(PoseStamped,"/deictic/headset/camera_pose"),
        "wrist_rgb":(Image,"/k1/wrist_camera/image_raw"),
        "wrist_info":(CameraInfo,"/k1/wrist_camera/camera_info"),
        "wrist_pose":(PoseStamped,"/k1/wrist_camera/pose"),
    }
    def receive(key,msg):
        stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if stamp <= 0:
            return
        frames.setdefault(stamp,{})[key] = msg
        if len(frames[stamp]) == len(specs):
            result[:] = [(stamp,frames[stamp])]
        for old in sorted(frames)[:-12]:
            del frames[old]
    for key,(kind,topic) in specs.items():
        node.create_subscription(kind,topic,lambda msg,key=key:receive(key,msg),10)
    end = time.monotonic()+20
    while not result and time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.1)
    if not result:
        raise RuntimeError("No exact-stamp seven-message camera pair in 20 seconds")
    stamp,frame = result[0]
    args.output.mkdir(parents=True,exist_ok=True)
    for name,key in (("headset","head_rgb"),("wrist","wrist_rgb")):
        msg = frame[key]
        assert msg.encoding == "rgb8"
        image = np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        PILImage.fromarray(image).save(args.output/f"{name}_camera.png")
    msg = frame["head_depth"]
    assert msg.encoding == "32FC1" and not msg.is_bigendian
    depth = np.frombuffer(msg.data,dtype="<f4").reshape(msg.height,msg.step//4)[:,:msg.width]
    np.save(args.output/"headset_depth_z.npy",depth)
    metadata = {"headset_K":np.array(frame["head_info"].k).reshape(3,3).tolist(),
                "wrist_K":np.array(frame["wrist_info"].k).reshape(3,3).tolist(),
                "headset_resolution":[frame["head_info"].width,frame["head_info"].height],
                "wrist_resolution":[frame["wrist_info"].width,frame["wrist_info"].height],
                "T_world_headOptical":transform(frame["head_pose"]),
                "T_base_wristOptical":transform(frame["wrist_pose"]),
                "T_base_headsetWorld":[[1,0,0,-1.2],[0,1,0,0],[0,0,1,-.9],[0,0,0,1]],
                "depth_encoding":"32FC1 optical Z meters","synthetic":True,
                "capture_stamp_ns":stamp,
                "message_headers":{key:{"frame_id":msg.header.frame_id,
                    "stamp_ns":msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec}
                    for key,msg in frame.items()},
                "capture_wall_time_unix":stamp/1e9,"source":"live_exact_stamp_ROS_capture"}
    (args.output/"camera_pair.json").write_text(json.dumps(metadata,indent=2))
    print(json.dumps({"captured_stamp":stamp,"output":str(args.output)}))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
