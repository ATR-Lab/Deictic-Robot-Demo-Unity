#!/usr/bin/env python3
"""Demand-driven atomic head-stereo display stream; never an estimator input."""
import copy
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image


def acquisition_stamp_ns(message):
    stamp = message.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('Invalid acquisition timestamp')
    value = stamp.sec * 1_000_000_000 + stamp.nanosec
    if value <= 0 or not message.header.frame_id:
        raise ValueError('Positive acquisition timestamp and optical frame are required')
    return value


def acquisition_stamp(message):
    return acquisition_stamp_ns(message) * 1e-9


def make_view_frame(message, max_width=480, max_height=360):
    """Validate bounded packed RGB(A), preserve aspect and the original header."""
    if not 1 <= max_width <= 480 or not 1 <= max_height <= 360:
        raise ValueError('Display bounds must be at most 480 by 360')
    acquisition_stamp(message)
    width, height, stride = int(message.width), int(message.height), int(message.step)
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(message.encoding)
    if channels is None:
        raise ValueError('Unsupported packed camera encoding')
    if (not 1 <= width <= 2048 or not 1 <= height <= 2048 or width * height > 2_097_152
            or stride < width * channels or stride * height > 16_777_216
            or message.is_bigendian not in (0, 1) or len(message.data) != stride * height):
        raise ValueError('Invalid or oversized image dimensions, stride, or payload')
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(height, stride)
    rgb = rows[:, :width * channels].reshape(height, width, channels)[:, :, :3]
    if message.encoding.startswith('bgr'):
        rgb = rgb[:, :, ::-1]
    scale = min(1., max_width / width, max_height / height)
    output_width, output_height = max(1, int(width * scale)), max(1, int(height * scale))
    if (output_width, output_height) != (width, height):
        rgb = cv2.resize(rgb, (output_width, output_height), interpolation=cv2.INTER_AREA)
    output = Image()
    output.header = copy.deepcopy(message.header)
    output.width, output.height = output_width, output_height
    output.encoding, output.is_bigendian, output.step = 'rgb8', 0, output_width * 3
    output.data = np.ascontiguousarray(rgb).tobytes()
    return output


def validate_stereo_eye(message):
    """Each source is bounded RGB8; row padding is allowed within a 2 MiB cap."""
    stamp = acquisition_stamp_ns(message)
    width, height, stride = int(message.width), int(message.height), int(message.step)
    if (message.encoding != 'rgb8' or not 1 <= width <= 640 or not 1 <= height <= 480
            or stride < width * 3 or stride * height > 2_097_152
            or message.is_bigendian not in (0, 1) or len(message.data) != stride * height):
        raise ValueError('Stereo eyes require bounded RGB8 up to 640 by 480 with valid stride/payload')
    return stamp


def make_stereo_frame(left, right, max_width=480, max_height=360):
    """Pack left/right halves after exact acquisition and geometry validation."""
    if validate_stereo_eye(left) != validate_stereo_eye(right):
        raise ValueError('Stereo acquisition timestamps must match exactly')
    if (left.width, left.height) != (right.width, right.height):
        raise ValueError('Stereo eye dimensions must match')
    if left.header.frame_id == right.header.frame_id:
        raise ValueError('Stereo eyes require distinct optical frames')
    left_view = make_view_frame(left, max_width, max_height)
    right_view = make_view_frame(right, max_width, max_height)
    shape = (left_view.height, left_view.width, 3)
    pixels = np.concatenate((np.frombuffer(left_view.data, np.uint8).reshape(shape),
                             np.frombuffer(right_view.data, np.uint8).reshape(shape)), axis=1)
    output = Image()
    output.header = copy.deepcopy(left.header)
    output.header.frame_id = 'k1_head_stereo_optical'
    output.width, output.height = 2 * left_view.width, left_view.height
    output.encoding, output.is_bigendian, output.step = 'rgb8', 0, output.width * 3
    output.data = pixels.tobytes()
    return output


class CameraViewRelay(Node):
    def __init__(self):
        super().__init__('deictic_camera_view_relay')
        defaults = dict(left_source_topic='/k1/head_camera/left/image_raw',
                        right_source_topic='/k1/head_camera/right/image_raw',
                        output_topic='/deictic/camera_view/stereo/image_raw',
                        max_width=480, max_height=360, display_hz=5., max_age_s=1.)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        cfg = self.settings
        if (not 1 <= cfg['max_width'] <= 480 or not 1 <= cfg['max_height'] <= 360
                or not math.isfinite(cfg['display_hz']) or not 0 < cfg['display_hz'] <= 5
                or not math.isfinite(cfg['max_age_s']) or cfg['max_age_s'] <= 0
                or not all(cfg[key] for key in ('left_source_topic', 'right_source_topic', 'output_topic'))
                or len({cfg['left_source_topic'], cfg['right_source_topic'], cfg['output_topic']}) != 3):
            raise ValueError('Invalid bounded display relay parameters')
        self.publisher = self.create_publisher(Image, cfg['output_topic'], 1)
        # Large fragmented RGB needs retransmission on the host UDP transport.
        # Bound completed DDS history to one image; capture age is still checked
        # at receipt and publication, so delayed delivery cannot freshen a frame.
        self.source_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                     reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.VOLATILE)
        self.sources = dict(left=None, right=None)
        self.pending = dict(left=None, right=None)
        self.ready = None
        self.last_received = dict(left=0, right=0)
        self.last_published = 0
        self.last_warning = -float('inf')
        self.create_timer(.25, self.refresh_demand)
        self.create_timer(1 / cfg['display_hz'], self.publish_latest)
        self.get_logger().info('Head stereo display relay idle; estimator inputs remain separate')

    def refresh_demand(self):
        wanted = self.publisher.get_subscription_count() > 0
        if wanted and self.sources['left'] is None:
            for eye in self.sources:
                self.sources[eye] = self.create_subscription(
                    Image, self.settings[eye + '_source_topic'],
                    lambda message, eye=eye: self.receive(eye, message), self.source_qos)
            self.get_logger().info('Stereo viewer present; subscribed to both head cameras')
        elif not wanted and self.sources['left'] is not None:
            for eye, subscription in self.sources.items():
                self.destroy_subscription(subscription)
                self.sources[eye] = None
            self.clear_frames()
            self.get_logger().info('Stereo viewer absent; both head cameras unsubscribed')

    def clear_frames(self):
        self.pending = dict(left=None, right=None)
        self.ready = None

    def fresh(self, stamp):
        return -.25 <= time.time() - stamp * 1e-9 <= self.settings['max_age_s']

    def warn_rejected(self, error):
        if time.monotonic() - self.last_warning > 5:
            self.get_logger().warning('Rejected stereo display frame: ' + str(error))
            self.last_warning = time.monotonic()

    def receive(self, eye, message):
        if self.sources[eye] is None or self.publisher.get_subscription_count() == 0:
            return
        try:
            stamp = validate_stereo_eye(message)
            if stamp <= self.last_received[eye] or stamp <= self.last_published or not self.fresh(stamp):
                return
            self.last_received[eye] = stamp
            self.pending[eye] = message
            other = 'right' if eye == 'left' else 'left'
            if self.pending[other] is not None:
                other_stamp = acquisition_stamp_ns(self.pending[other])
                if stamp == other_stamp:
                    left, right = self.pending['left'], self.pending['right']
                    self.pending = dict(left=None, right=None)
                    if (left.width, left.height) != (right.width, right.height):
                        raise ValueError('Stereo eye dimensions must match')
                    if left.header.frame_id == right.header.frame_id:
                        raise ValueError('Stereo eyes require distinct optical frames')
                    # One latest complete pair, plus at most one unmatched image per eye.
                    self.ready = (left, right)
                elif stamp > other_stamp:
                    self.pending[other] = None
                else:
                    self.pending[eye] = None
        except (ValueError, TypeError, AttributeError) as error:
            self.warn_rejected(error)

    def publish_latest(self):
        if self.publisher.get_subscription_count() == 0:
            self.clear_frames()
            return
        for eye, message in self.pending.items():
            if message is not None and not self.fresh(acquisition_stamp_ns(message)):
                self.pending[eye] = None
        pair, self.ready = self.ready, None
        if pair is None:
            return
        try:
            stamp = acquisition_stamp_ns(pair[0])
            if stamp <= self.last_published or not self.fresh(stamp):
                return
            output = make_stereo_frame(*pair, self.settings['max_width'], self.settings['max_height'])
            self.publisher.publish(output)
            self.last_published = stamp
        except (ValueError, TypeError, AttributeError, cv2.error) as error:
            self.warn_rejected(error)


def main():
    rclpy.init()
    node = CameraViewRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
