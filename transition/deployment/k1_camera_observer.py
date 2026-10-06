#!/usr/bin/env python3
"""Robot-side receive-only camera compressor. No SDK, RPC, or command topics.

Run with the robot's existing Humble/vendor domain configuration. Only derived
diagnostic images and provenance are published. No vendor service is changed.
"""
from __future__ import annotations
import argparse
import copy
import json
from pathlib import Path
import time
import uuid

from observation_codec import (
    camera_bgr, pair_metadata, stamp_ns, pair_advances, display_size,
    checked_jpeg, transcode_mono, checked_raw_image, checked_frame_id,
    raw_camera_sources,
)


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", type=float, default=10)
    parser.add_argument('--mono-source', choices=('left', 'native'), default='left',
                        help='Use the raw left head camera, or the vendor compressed stream; no automatic source switching')
    parser.add_argument('--stereo', action='store_true',
                        help='Also subscribe to both raw eyes and publish diagnostic stereo (disabled by default)')
    args = parser.parse_args(argv)
    if not 1 <= args.rate <= 15:
        parser.error("rate must be 1..15")
    return args


def main(argv=None):
    if (Path(__file__).resolve().parents[1]/'.runtime/k1-diagnostics-hold.json').exists():
        raise SystemExit('K1 camera diagnostics held after the 2026-09-24 memory/reset incident')
    args = parse_arguments(argv)
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image, CompressedImage
    from std_msgs.msg import String
    cv2.setNumThreads(1)
    rclpy.init()
    class Observer(Node):
        def __init__(self):
            super().__init__("transition_k1_camera_observer", start_parameter_services=False, enable_rosout=False)
            self.eyes = {}; self.last_pair = None; self.sequence = 0; self.boot = str(uuid.uuid4()); self.error = None
            self.mono = None; self.last_mono = None; self.mono_sequence = 0; self.mono_error = None
            q = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                           reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
            self.out = self.create_publisher(CompressedImage, "/transition/k1/stereo/compressed", q) if args.stereo else None
            self.mono_out = self.create_publisher(CompressedImage, "/transition/k1/head/compressed", q)
            self.status = self.create_publisher(String, "/transition/k1/camera_status", q)
            self.subs = [self.create_subscription(Image, topic, lambda m, eye=eye: self.receive(eye,m), q)
                         for eye,topic in raw_camera_sources(args.mono_source, args.stereo)]
            self.mono_sub = self.create_subscription(CompressedImage, '/booster_video_stream', self.receive_mono,
                q) if args.mono_source == 'native' else None
            self.create_timer(1/args.rate, self.publish_latest)
            self.create_timer(1, self.publish_status)
        def receive(self, eye, message):
            try:
                checked_raw_image(message)
                stamp = stamp_ns(message.header)
                old = self.eyes.get(eye)
                if old and stamp <= stamp_ns(old[0].header):
                    self.error = "duplicate_or_regressing_camera_stamp"; return
                self.eyes[eye] = (message, time.monotonic())
            except (ValueError, TypeError) as exc: self.error = str(exc)
        def publish_latest(self):
            self.publish_mono()
            if not args.stereo or len(self.eyes) != 2: return
            left,a = self.eyes["left"]; right,b = self.eyes["right"]
            if time.monotonic() - min(a,b) > .5: return
            key = (stamp_ns(left.header), stamp_ns(right.header))
            if not pair_advances(key, self.last_pair): return
            try:
                provenance = pair_metadata(left,right)
                size = display_size(left.width, left.height)
                images = [cv2.resize(camera_bgr(m), size, interpolation=cv2.INTER_AREA) for m in (left,right)]
                ok, encoded = cv2.imencode('.jpg', np.concatenate(images,axis=1), [cv2.IMWRITE_JPEG_QUALITY,75])
                if not ok: raise ValueError("JPEG encode failed")
                out = CompressedImage();out.header=copy.deepcopy(left.header)
                out.header.frame_id="k1_diagnostic_stereo_left_right";out.format="rgb8; jpeg compressed bgr8";out.data=encoded.tobytes()
                checked_jpeg(out)
                if time.monotonic() - min(a,b) > .5:
                    raise ValueError('Camera pair expired during compression')
                self.out.publish(out);self.last_pair=key;self.sequence+=1;self.error=None;self.provenance=provenance
            except (ValueError,cv2.error) as exc: self.error=str(exc)
        def receive_mono(self, message):
            try:
                stamp = stamp_ns(message.header)
                checked_frame_id(message.header)
                checked_jpeg(message)
                if self.mono and stamp <= stamp_ns(self.mono[0].header):
                    raise ValueError('duplicate_or_regressing_mono_stamp')
                self.mono = (message,time.monotonic())
            except (ValueError,TypeError) as exc: self.mono_error=str(exc)
        def publish_mono(self):
            sample = self.eyes.get('left') if args.mono_source == 'left' else self.mono
            if sample is None: return
            source,received=sample
            stamp=stamp_ns(source.header)
            if stamp == self.last_mono or time.monotonic()-received > .5: return
            try:
                out=CompressedImage();out.header=copy.deepcopy(source.header)
                out.format='rgb8; jpeg compressed bgr8'
                if args.mono_source == 'left':
                    bgr = camera_bgr(source)
                    bgr = cv2.resize(bgr, display_size(source.width, source.height), interpolation=cv2.INTER_AREA)
                    ok, encoded = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY,65])
                    if not ok: raise ValueError('Left camera JPEG encode failed')
                    out.data = encoded.tobytes()
                else:
                    out.data=transcode_mono(source)
                checked_jpeg(out)
                if time.monotonic()-received > .5:
                    raise ValueError('Mono frame expired during transcoding')
                self.mono_out.publish(out);self.last_mono=stamp;self.mono_sequence+=1;self.mono_error=None
            except (ValueError,cv2.error) as exc: self.mono_error=str(exc)
        def publish_status(self):
            now=time.monotonic()
            sample = self.eyes.get('left') if args.mono_source == 'left' else self.mono
            data={"schema_version":1,"boot_id":self.boot,"sequence":self.sequence,"motion_capability":False,
                  "stereo_enabled":args.stereo,
                  "camera_receipt_age_s":{k:now-v[1] for k,v in self.eyes.items()},"error":self.error,
                  "pair":getattr(self,"provenance",None),"source_clock_offset_verified":False,
                  "mono_sequence":self.mono_sequence,"mono_receipt_age_s":None if sample is None else now-sample[1],
                  "mono_error":self.mono_error,"mono_source_stamp_ns":self.last_mono,
                  "mono_source_topic":"/boostercamera/head/raw/rgb" if args.mono_source == 'left' else '/booster_video_stream',
                  "mono_use":"diagnostic_display_only"}
            self.status.publish(String(data=json.dumps(data,allow_nan=False)))
    node=Observer()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()

if __name__ == "__main__": main()
