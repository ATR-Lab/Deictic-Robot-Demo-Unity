import importlib.util
from pathlib import Path
import time

import numpy as np
import pytest

pytest.importorskip('cv2')
rclpy = pytest.importorskip('rclpy')
from sensor_msgs.msg import Image

ROOT = Path(__file__).resolve().parents[4]
spec = importlib.util.spec_from_file_location('camera_view_relay', ROOT / 'ros2/scripts/camera_view_relay.py')
relay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(relay)


def frame(width=2, height=2, padding=2, encoding='rgb8'):
    message = Image()
    message.header.frame_id = 'k1_wrist_camera_optical'
    message.header.stamp.sec = 100
    message.header.stamp.nanosec = 123456789
    message.width, message.height, message.encoding = width, height, encoding
    channels = 4 if 'a' in encoding else 3
    message.step = width * channels + padding
    pixels = np.arange(width * height * channels, dtype=np.uint8).reshape(height, width * channels)
    data = np.full((height, message.step), 222, dtype=np.uint8)
    data[:, :width * channels] = pixels
    message.data = data.tobytes()
    return message, pixels.reshape(height, width, channels)


@pytest.mark.parametrize('encoding', ['rgb8', 'bgr8', 'rgba8', 'bgra8'])
def test_padding_channels_and_exact_acquisition_header(encoding):
    message, original = frame(encoding=encoding)
    source_data = bytes(message.data)
    output = relay.make_view_frame(message)
    expected = original[:, :, :3]
    if encoding.startswith('bgr'):
        expected = expected[:, :, ::-1]
    np.testing.assert_array_equal(np.frombuffer(output.data, np.uint8).reshape(2, 2, 3), expected)
    assert output.header == message.header and output.header is not message.header
    assert output.step == 6 and output.encoding == 'rgb8'
    assert bytes(message.data) == source_data


@pytest.mark.parametrize('width,height,expected', [(640, 480, (480, 360)), (1200, 600, (480, 240)), (2, 3, (2, 3))])
def test_resize_is_bounded_aspect_preserving_and_never_upscales(width, height, expected):
    output = relay.make_view_frame(frame(width, height)[0])
    assert (output.width, output.height) == expected
    assert len(output.data) == output.width * output.height * 3


@pytest.mark.parametrize('fault', ['zero', 'oversize', 'pixels', 'stride', 'payload', 'payload_bound', 'stamp'])
def test_invalid_frame_bounds_and_stamps_are_rejected(fault):
    message, _ = frame()
    if fault == 'zero': message.width = 0
    elif fault == 'oversize': message.width = 2**32-1
    elif fault == 'pixels': message.width = message.height = 2048
    elif fault == 'stride': message.step = 1
    elif fault == 'payload': message.data = b'\x00'
    elif fault == 'payload_bound': message.step = 2**32-1
    elif fault == 'stamp': message.header.stamp.sec = message.header.stamp.nanosec = 0
    with pytest.raises(ValueError):
        relay.make_view_frame(message)


def stereo_pair(width=2, height=2, nanosec=123456789):
    left, _ = frame(width, height, padding=2)
    right, _ = frame(width, height, padding=5)
    left.header.frame_id = 'k1_head_left_optical'
    right.header.frame_id = 'k1_head_right_optical'
    left.header.stamp.nanosec = right.header.stamp.nanosec = nanosec
    right.data = (np.frombuffer(right.data, np.uint8).astype(np.uint16) + 40).astype(np.uint8).tobytes()
    return left, right


def test_stereo_packs_left_then_right_preserving_exact_common_stamp_and_padding():
    left, right = stereo_pair()
    source_bytes = bytes(left.data), bytes(right.data)
    output = relay.make_stereo_frame(left, right)
    expected = np.concatenate([
        np.frombuffer(relay.make_view_frame(eye).data, np.uint8).reshape(2, 2, 3)
        for eye in (left, right)], axis=1)
    np.testing.assert_array_equal(np.frombuffer(output.data, np.uint8).reshape(2, 4, 3), expected)
    assert (output.width, output.height, output.step, output.encoding) == (4, 2, 12, 'rgb8')
    assert output.header.stamp == left.header.stamp
    assert output.header.frame_id == 'k1_head_stereo_optical'
    assert left.header.frame_id == 'k1_head_left_optical'
    assert source_bytes == (bytes(left.data), bytes(right.data))


@pytest.mark.parametrize('width,height,expected', [(640, 480, (960, 360)), (320, 240, (640, 240)),
                                                (640, 320, (960, 240))])
def test_stereo_bounds_apply_per_eye_without_upscaling(width, height, expected):
    output = relay.make_stereo_frame(*stereo_pair(width, height))
    assert (output.width, output.height) == expected
    assert len(output.data) == expected[0] * expected[1] * 3 <= 1_036_800


@pytest.mark.parametrize('fault', ['stamp_ns', 'dimensions', 'same_frame', 'large_source',
                                  'encoding', 'payload', 'padded_payload_bound'])
def test_stereo_rejects_mixed_or_malformed_eyes(fault):
    left, right = stereo_pair()
    # At real UTC magnitudes a one-nanosecond difference vanishes in float seconds.
    left.header.stamp.sec = right.header.stamp.sec = 1_800_000_000
    if fault == 'stamp_ns': right.header.stamp.nanosec += 1
    elif fault == 'dimensions':
        right = stereo_pair(3, 2)[1]
        right.header.stamp.sec = left.header.stamp.sec
    elif fault == 'same_frame': right.header.frame_id = left.header.frame_id
    elif fault == 'large_source': right.width = 641
    elif fault == 'encoding': right.encoding = 'bgr8'
    elif fault == 'payload': right.data = b'\x00'
    elif fault == 'padded_payload_bound': right.step = 2_097_153
    with pytest.raises(ValueError):
        relay.make_stereo_frame(left, right)


@pytest.fixture
def stereo_node(monkeypatch):
    rclpy.init(domain_id=202)
    node = relay.CameraViewRelay()

    class Publisher:
        def __init__(self):
            self.count = 0
            self.messages = []
        def get_subscription_count(self): return self.count
        def publish(self, message): self.messages.append(message)

    node.publisher = Publisher()
    node.compressed_publisher = Publisher()
    clock = [100.5]
    monkeypatch.setattr(relay.time, 'time', lambda: clock[0])
    monkeypatch.setattr(relay.time, 'monotonic', lambda: clock[0])
    try:
        yield node, clock
    finally:
        node.destroy_node()
        rclpy.shutdown()


def enable_viewer(node):
    node.publisher.count = 1
    node.refresh_demand()
    assert all(source is not None for source in node.sources.values())


def receive_pair(node, pair):
    node.receive('left', pair[0])
    node.receive('right', pair[1])


def test_both_actual_source_subscriptions_are_reliable_latest_and_volatile(stereo_node):
    node, _ = stereo_node
    enable_viewer(node)
    for subscription in node.sources.values():
        qos = subscription.qos_profile
        assert qos.reliability == relay.ReliabilityPolicy.RELIABLE
        assert qos.history == relay.HistoryPolicy.KEEP_LAST and qos.depth == 1
        assert qos.durability == relay.DurabilityPolicy.VOLATILE


def test_both_subscriptions_follow_demand_and_each_pair_is_published_once(stereo_node):
    node, _ = stereo_node
    node.refresh_demand()
    assert all(source is None for source in node.sources.values())
    assert node.settings['output_topic'] == '/deictic/camera_view/stereo/image_raw'
    assert node.settings['left_source_topic'] == '/k1/head_camera/left/image_raw'
    assert node.settings['right_source_topic'] == '/k1/head_camera/right/image_raw'
    enable_viewer(node)
    pair = stereo_pair()
    receive_pair(node, pair)
    node.publish_latest()
    node.publish_latest()
    assert len(node.publisher.messages) == 1
    assert node.publisher.messages[0].header.stamp == pair[0].header.stamp
    receive_pair(node, pair)
    assert node.ready is None and not any(node.pending.values())
    node.receive('left', stereo_pair(nanosec=223456789)[0])
    node.publisher.count = 0
    node.refresh_demand()
    assert all(source is None for source in node.sources.values())
    assert node.ready is None and not any(node.pending.values())
    # Queued callbacks after unsubscribe cannot restore either eye.
    receive_pair(node, stereo_pair(nanosec=323456789))
    assert node.ready is None and not any(node.pending.values())


def test_one_eye_or_different_stamps_never_publishes(stereo_node):
    node, _ = stereo_node
    enable_viewer(node)
    earlier, later = stereo_pair(), stereo_pair(nanosec=223456789)
    node.receive('left', earlier[0])
    node.publish_latest()
    node.receive('right', later[1])
    node.publish_latest()
    assert not node.publisher.messages and node.pending['left'] is None
    node.receive('left', later[0])
    node.publish_latest()
    assert len(node.publisher.messages) == 1
    assert node.publisher.messages[0].header.stamp.nanosec == 223456789


def test_latest_complete_pair_replaces_prior_and_reordered_images_do_not_revive_it(stereo_node):
    node, _ = stereo_node
    enable_viewer(node)
    for stamp in range(123456789, 123456839):
        receive_pair(node, stereo_pair(nanosec=stamp))
    assert node.ready[0].header.stamp.nanosec == 123456838
    assert not any(node.pending.values())
    receive_pair(node, stereo_pair())
    node.publish_latest()
    assert len(node.publisher.messages) == 1
    assert node.publisher.messages[0].header.stamp.nanosec == 123456838


@pytest.mark.parametrize('when', ['before_receive', 'before_publish', 'future'])
def test_stale_or_future_pair_is_not_relabelled_or_published(stereo_node, when):
    node, clock = stereo_node
    enable_viewer(node)
    if when == 'before_receive': clock[0] += 2
    elif when == 'future': clock[0] -= 1
    receive_pair(node, stereo_pair())
    if when == 'before_publish': clock[0] += 2
    node.publish_latest()
    assert not node.publisher.messages and node.ready is None


def test_delayed_eye_retransmission_is_dropped_then_fresh_pair_recovers(stereo_node):
    node, clock = stereo_node
    enable_viewer(node)
    old_pair = stereo_pair()
    node.receive('left', old_pair[0])
    clock[0] += 2
    node.receive('right', old_pair[1])
    node.publish_latest()
    assert not node.publisher.messages and node.ready is None and not any(node.pending.values())
    fresh_pair = stereo_pair()
    for eye in fresh_pair:
        eye.header.stamp.sec = 102
    receive_pair(node, fresh_pair)
    node.publish_latest()
    assert len(node.publisher.messages) == 1
    assert node.publisher.messages[0].header.stamp == fresh_pair[0].header.stamp


def test_demand_loss_before_publish_clears_complete_pair(stereo_node):
    node, _ = stereo_node
    enable_viewer(node)
    receive_pair(node, stereo_pair())
    node.publisher.count = 0
    node.publish_latest()
    assert not node.publisher.messages and node.ready is None
    node.refresh_demand()
    enable_viewer(node)
    node.publish_latest()
    assert not node.publisher.messages


def test_malformed_received_eye_does_not_enter_pair_buffer(stereo_node):
    node, _ = stereo_node
    enable_viewer(node)
    left, right = stereo_pair()
    left.data = b'\x00'
    node.receive('left', left)
    node.receive('right', right)
    node.publish_latest()
    assert node.pending['left'] is None and node.ready is None and not node.publisher.messages


def test_jpeg_keeps_acquisition_pair_eye_order_and_rgb_colors():
    left, right = stereo_pair(320, 240)
    for eye, color in ((left, (230, 30, 10)), (right, (15, 40, 220))):
        eye.step = eye.width * 3
        eye.data = np.full((eye.height, eye.width, 3), color, np.uint8).tobytes()
    packed = relay.make_stereo_frame(left, right)
    message = relay.compress_stereo_frame(packed)
    assert message.header == packed.header and message.header is not packed.header
    assert message.format == 'rgb8; jpeg compressed bgr8'
    assert len(message.data) < len(packed.data) // 10
    decoded = relay.cv2.imdecode(np.frombuffer(message.data, np.uint8), relay.cv2.IMREAD_COLOR)
    rgb = relay.cv2.cvtColor(decoded, relay.cv2.COLOR_BGR2RGB)
    assert rgb.shape == (240, 640, 3)
    np.testing.assert_allclose(rgb[100, 100], [230, 30, 10], atol=3)
    np.testing.assert_allclose(rgb[100, 500], [15, 40, 220], atol=3)


@pytest.mark.parametrize('quality', [0, 96, 80.5, True])
def test_jpeg_rejects_invalid_quality(quality):
    with pytest.raises(ValueError):
        relay.compress_stereo_frame(relay.make_stereo_frame(*stereo_pair()), quality)


def test_compressed_only_demand_starts_both_eyes_without_raw_output(stereo_node):
    node, clock = stereo_node
    node.compressed_publisher.count = 1
    node.refresh_demand()
    assert all(source is not None for source in node.sources.values())
    receive_pair(node, stereo_pair())
    node.publish_latest()
    assert not node.publisher.messages
    assert len(node.compressed_publisher.messages) == 1
    assert node.compressed_publisher.messages[0].header.frame_id == 'k1_head_stereo_optical'
    # Raw consumers can join without creating another pair or changing stamps.
    node.publisher.count = 1
    clock[0] += .04
    pair = stereo_pair(nanosec=223456789)
    receive_pair(node, pair)
    node.publish_latest()
    assert len(node.publisher.messages) == 1 and len(node.compressed_publisher.messages) == 2
    assert node.publisher.messages[-1].header == node.compressed_publisher.messages[-1].header
    node.compressed_publisher.count = node.publisher.count = 0
    node.refresh_demand()
    assert all(source is None for source in node.sources.values())


def test_rate_cap_retains_only_newest_pair_without_a_second_camera_period(stereo_node):
    node, clock = stereo_node
    enable_viewer(node)
    receive_pair(node, stereo_pair())
    node.publish_latest()
    clock[0] += .01
    receive_pair(node, stereo_pair(nanosec=223456789))
    node.publish_latest()
    clock[0] += .01
    receive_pair(node, stereo_pair(nanosec=323456789))
    node.publish_latest()
    assert len(node.publisher.messages) == 1
    clock[0] += .015
    node.publish_latest()
    assert len(node.publisher.messages) == 2
    assert node.publisher.messages[-1].header.stamp.nanosec == 323456789


def test_encoding_cannot_freshen_a_pair_which_expires_during_compression(stereo_node, monkeypatch):
    node, clock = stereo_node
    node.compressed_publisher.count = 1
    node.refresh_demand()
    encode = relay.compress_stereo_frame
    def slow_encode(*args):
        message = encode(*args)
        clock[0] += 1.1
        return message
    monkeypatch.setattr(relay, 'compress_stereo_frame', slow_encode)
    receive_pair(node, stereo_pair())
    node.publish_latest()
    assert not node.compressed_publisher.messages and node.last_published == 0


def test_raw_packing_cannot_publish_a_pair_which_expires_during_resize(stereo_node, monkeypatch):
    node, clock = stereo_node
    enable_viewer(node)
    pack = relay.make_stereo_frame
    def slow_pack(*args):
        message = pack(*args)
        clock[0] += 1.1
        return message
    monkeypatch.setattr(relay, 'make_stereo_frame', slow_pack)
    receive_pair(node, stereo_pair())
    node.publish_latest()
    assert not node.publisher.messages
