#!/usr/bin/env python3
"""Read-only 15-second ROS stereo/relay check; no robot command publications.

Subscriptions activate the demand-rendered cameras during the observation.
Saved pixels come from received ROS messages, never generated test fixtures.
Image checks do not establish physical stereo calibration.
"""
import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
from PIL import Image as PILImage


def stamp_ns(message):
    stamp = message.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000 or not message.header.frame_id:
        raise ValueError('Invalid acquisition header')
    value = stamp.sec*1_000_000_000+stamp.nanosec
    if value <= 0:
        raise ValueError('Acquisition stamp must be positive')
    return value


def pixels(message, stereo=False):
    stamp_ns(message)
    width,height,step = int(message.width),int(message.height),int(message.step)
    maximum = (960,360) if stereo else (640,480)
    if (message.encoding != 'rgb8' or not 0 < width <= maximum[0]
            or not 0 < height <= maximum[1] or step < width*3
            or step*height > 4_194_304 or len(message.data) != step*height):
        raise ValueError('Invalid RGB8 dimensions, stride, or payload')
    return np.frombuffer(message.data,np.uint8).reshape(height,step)[:,:width*3].reshape(height,width,3)


def image_statistics(rgb):
    gray = cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
    return dict(mean=float(rgb.mean()), std=float(rgb.std()), spatial_gray_std=float(gray.std()),
                minimum=int(rgb.min()),maximum=int(rgb.max()),
                black_fraction=float(np.mean(np.max(rgb,axis=2) <= 2)))


def pair_metrics(left, right, left_info, right_info):
    stamps = [stamp_ns(message) for message in (left,right,left_info,right_info)]
    if len(set(stamps)) != 1 or left.header.frame_id == right.header.frame_id:
        raise ValueError('Pair requires one exact stamp and distinct optical frames')
    images = [pixels(left),pixels(right)]
    if images[0].shape != images[1].shape:
        raise ValueError('Eye sizes differ')
    for message,info in ((left,left_info),(right,right_info)):
        k,p = np.asarray(info.k,dtype=float).reshape(3,3),np.asarray(info.p,dtype=float).reshape(3,4)
        if (message.header.frame_id != info.header.frame_id
                or (message.width,message.height) != (info.width,info.height)
                or not np.isfinite(k).all() or not np.isfinite(p).all()
                or k[0,0] <= 0 or k[1,1] <= 0 or abs(k[2,2]-1) > 1e-8):
            raise ValueError('CameraInfo differs from RGB geometry or is invalid')
    if abs(left_info.p[3]) > 1e-8 or right_info.p[3] >= 0:
        raise ValueError('Expected rectified virtual stereo projection baseline')
    difference = np.abs(images[0].astype(np.int16)-images[1].astype(np.int16))
    return dict(stamp_ns=stamps[0],width=left.width,height=left.height,
                frame_ids=[left.header.frame_id,right.header.frame_id],
                left=image_statistics(images[0]),right=image_statistics(images[1]),
                mean_absolute_eye_difference=float(difference.mean()),
                changed_eye_pixel_fraction=float(np.mean(np.any(difference != 0,axis=2))),
                inferred_simulation_baseline_m=float(-right_info.p[3]/right_info.k[0]))


def relay_metrics(left, right, relay):
    if len({stamp_ns(message) for message in (left,right,relay)}) != 1:
        raise ValueError('Relay did not preserve the exact source stamp')
    actual = pixels(relay,stereo=True)
    if relay.header.frame_id != 'k1_head_stereo_optical' or relay.width % 2:
        raise ValueError('Invalid side-by-side optical frame or width')
    eyes = [pixels(message) for message in (left,right)]
    width,height = relay.width//2,relay.height
    expected = np.concatenate([cv2.resize(eye,(width,height),interpolation=cv2.INTER_AREA)
                               if eye.shape[:2] != (height,width) else eye for eye in eyes],axis=1)
    delta = np.abs(expected.astype(np.int16)-actual.astype(np.int16))
    return dict(stamp_ns=stamp_ns(relay),width=relay.width,height=relay.height,
                maximum_pixel_error=int(delta.max()),mean_pixel_error=float(delta.mean()),
                exact_source_halves=bool(np.array_equal(expected,actual)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True,help='New directory for received PNGs/report')
    parser.add_argument('--duration',type=float,default=15.,help='Observation seconds, 1..60')
    parser.add_argument('--min-pairs',type=int,default=3)
    parser.add_argument('--max-age',type=float,default=1.,help='Maximum age at receipt/end, seconds')
    args = parser.parse_args()
    if (not math.isfinite(args.duration) or not 1 <= args.duration <= 60 or args.min_pairs < 1
            or not math.isfinite(args.max_age) or not 0 < args.max_age <= 5):
        parser.error('Invalid duration, pair count, or receipt-age bound')
    args.output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image, CameraInfo
    rclpy.init()
    node = rclpy.create_node('validate_k1_head_stereo')
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=2,reliability=ReliabilityPolicy.RELIABLE)
    topics = {
        'left':(Image,'/k1/head_camera/left/image_raw'),
        'right':(Image,'/k1/head_camera/right/image_raw'),
        'left_info':(CameraInfo,'/k1/head_camera/left/camera_info'),
        'right_info':(CameraInfo,'/k1/head_camera/right/camera_info'),
        'relay':(Image,'/deictic/camera_view/stereo/image_raw'),
    }
    received = {key:[] for key in topics}
    frames,pairs,relays,errors = {},{},{},[]
    source_saved,relay_saved = False,False
    start = time.time()
    deadline = time.monotonic()+args.duration

    def save_received(current,include_relay):
        for eye in ('left','right'):
            PILImage.fromarray(pixels(current[eye])).save(args.output/f'head_{eye}_camera.png')
        if include_relay:
            PILImage.fromarray(pixels(current['relay'],True)).save(args.output/'head_stereo_relay.png')
        metadata = {name:{'frame_id':msg.header.frame_id,'stamp_ns':stamp_ns(msg)}
                    for name,msg in current.items()}
        for eye in ('left','right'):
            info=current[eye+'_info']
            metadata[eye+'_info'].update(K=list(info.k),P=list(info.p),D=list(info.d),
                                        width=info.width,height=info.height)
        (args.output/'received_headers.json').write_text(json.dumps(metadata,indent=2))

    def receive(key,message):
        nonlocal source_saved,relay_saved
        try:
            stamp = stamp_ns(message)
            receipt = node.get_clock().now().nanoseconds*1e-9
            age = receipt-stamp*1e-9
            received[key].append(dict(stamp_ns=stamp,receipt_unix=receipt,age_s=age))
            if not -.05 <= age <= args.max_age:
                raise ValueError(f'{key}: age {age:.6f}s outside [-.05,{args.max_age}]')
            frames.setdefault(stamp,{})[key] = message
            current = frames[stamp]
            if all(name in current for name in ('left','right','left_info','right_info')) and stamp not in pairs:
                pairs[stamp] = pair_metrics(current['left'],current['right'],current['left_info'],current['right_info'])
                if not source_saved:
                    save_received(current,False)
                    source_saved = True
            if stamp in pairs and 'relay' in current and stamp not in relays:
                relays[stamp] = relay_metrics(current['left'],current['right'],current['relay'])
                if not relay_saved:
                    # Keep all PNGs at one exact stamp; retain raw evidence even
                    # when the relay never produces a matching composite.
                    save_received(current,True)
                    relay_saved = True
        except (ValueError,TypeError,AttributeError,cv2.error) as error:
            if len(errors) < 50: errors.append(str(error))
        finally:
            for old in sorted(frames)[:-16]: del frames[old]

    try:
        for key,(kind,topic) in topics.items():
            node.create_subscription(kind,topic,lambda message,key=key:receive(key,message),qos)
        while time.monotonic() < deadline:
            rclpy.spin_once(node,timeout_sec=min(.1,max(0.,deadline-time.monotonic())))
        end = node.get_clock().now().nanoseconds*1e-9
    finally:
        node.destroy_node()
        rclpy.shutdown()
    stats = {}
    for key,samples in received.items():
        stamps = sorted(set(sample['stamp_ns'] for sample in samples))
        intervals = np.diff(stamps)*1e-9
        ages = [sample['age_s'] for sample in samples]
        stats[key] = dict(topic=topics[key][1],messages=len(samples),distinct_stamps=len(stamps),
                          first_stamp_ns=stamps[0] if stamps else None,last_stamp_ns=stamps[-1] if stamps else None,
                          median_capture_interval_s=float(np.median(intervals)) if len(intervals) else None,
                          min_capture_interval_s=float(intervals.min()) if len(intervals) else None,
                          max_capture_interval_s=float(intervals.max()) if len(intervals) else None,
                          receipt_age_min_s=min(ages) if ages else None,receipt_age_max_s=max(ages) if ages else None,
                          final_age_s=end-stamps[-1]*1e-9 if stamps else None)
    nonblank = bool(pairs) and all(result[eye]['mean'] > 1 and result[eye]['spatial_gray_std'] > 1
                                    for result in pairs.values() for eye in ('left','right'))
    checks = dict(enough_exact_pairs=len(pairs) >= args.min_pairs,
                  enough_exact_relay_frames=len(relays) >= args.min_pairs,
                  relay_matches_source_halves=bool(relays) and all(r['exact_source_halves'] for r in relays.values()),
                  nonblank_spatial_image_content=nonblank,valid_fresh_messages=not errors,
                  advancing_pair_stamps=len(pairs) >= 2,
                  fresh_at_end=all(s['final_age_s'] is not None and -.05 <= s['final_age_s'] <= args.max_age
                                   for s in stats.values()),
                  source_at_most_5hz=all(stats[key]['min_capture_interval_s'] is not None
                                        and stats[key]['min_capture_interval_s'] >= .19 for key in ('left','right')))
    report = dict(source='live ROS simulated head-stereo pixels; no generated test images',
                  physical_calibration_validated=False,start_unix=start,end_unix=end,
                  configured_duration_s=args.duration,checks=checks,passed=all(checks.values()),
                  topics=stats,pairs=list(pairs.values()),relays=list(relays.values()),errors=errors,
                  limits='Static pixels need not change over time; differing eyes do not prove physical calibration.')
    (args.output/'validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(dict(passed=report['passed'],checks=checks,pairs=len(pairs),relay_frames=len(relays),output=str(args.output))))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
