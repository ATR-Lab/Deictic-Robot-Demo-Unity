"""Diagnostic conversion and command-isolation tests; no robot connection."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

DEPLOYMENT=Path(__file__).resolve().parents[1]/"deployment"
def module(name):
    spec=importlib.util.spec_from_file_location(name,DEPLOYMENT/(name+".py"))
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result
codec=module("observation_codec")

def image(stamp=1_000_000_000,frame="left"):
    return SimpleNamespace(width=4,height=2,step=4,encoding="nv12",data=bytes([16]*8+[128]*4),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp//10**9,nanosec=stamp%10**9),frame_id=frame))

def test_reordered_joints_map_by_name():
    names=list(reversed(codec.VENDOR_NAMES));positions=list(range(22))
    actual,p,v=codec.mapped_joints(names,positions,[])
    assert actual==list(codec.URDF_NAMES) and p==list(reversed(positions)) and v==[]

@pytest.mark.parametrize("damage",["duplicate","unknown","missing","nan","short_velocity"])
def test_invalid_joint_telemetry_rejected(damage):
    names=list(codec.VENDOR_NAMES);p=[0.]*22;v=[0.]*22
    if damage=="duplicate":names[-1]=names[0]
    if damage=="unknown":names[-1]="made_up"
    if damage=="missing":names.pop()
    if damage=="nan":p[1]=float("nan")
    if damage=="short_velocity":v.pop()
    with pytest.raises(ValueError):codec.mapped_joints(names,p,v)

def test_nv12_black_conversion_and_stride_validation():
    pytest.importorskip("cv2")
    m=image();rgb=codec.camera_bgr(m)
    assert rgb.shape==(2,4,3) and not rgb.any()
    m.data=m.data[:-1]
    with pytest.raises(ValueError):codec.camera_bgr(m)


def test_oversized_raw_payload_rejected_without_copying_or_importing_decoder(monkeypatch):
    class OversizedPayload:
        def __len__(self): return codec.MAX_RAW_IMAGE_BYTES + 1
        def __bytes__(self): raise AssertionError('Oversized payload copied')
    message = image()
    message.data = OversizedPayload()
    monkeypatch.setitem(sys.modules, 'cv2', None)
    with pytest.raises(ValueError, match='byte limit'):
        codec.camera_bgr(message)


@pytest.mark.parametrize('encoding,step', [('nv12', -1), ('nv12', 3), ('nv12', 5),
                                          ('nv12', 262), ('rgb8', 11), ('bgr8', 269)])
def test_raw_stride_rejected_even_when_payload_matches_claimed_stride(encoding, step):
    message = image()
    message.encoding = encoding
    message.step = step
    message.data = bytes(max(0, step * message.height * 3 // 2 if encoding == 'nv12' else step * message.height))
    with pytest.raises(ValueError): codec.checked_raw_image(message)


@pytest.mark.parametrize('frame', ['', '   ', 'left\nframe', 'x'*257, '\N{LATIN SMALL LETTER E WITH ACUTE}'*129])
def test_raw_frame_id_rejected_before_caching(frame):
    with pytest.raises(ValueError, match='frame ID'):
        codec.checked_raw_image(image(frame=frame))


@pytest.mark.parametrize('field,value', [('width', True), ('height', 2.0), ('step', 4.0),
                                        ('width', 2049), ('height', 2049)])
def test_raw_shape_requires_bounded_integer_dimensions(field, value):
    message = image()
    setattr(message, field, value)
    with pytest.raises(ValueError): codec.checked_raw_image(message)


def test_bounded_padded_nv12_and_packed_rgb_remain_supported():
    pytest.importorskip('cv2')
    message = image()
    message.step = 6
    message.data = bytes(([16]*4+[77]*2)*2 + [128]*4+[77]*2)
    assert not codec.camera_bgr(message).any()
    message.width = 2
    message.step = 8
    message.encoding = 'bgr8'
    message.data = bytes(([1,2,3]*2+[77]*2)*2)
    assert codec.camera_bgr(message).tolist() == [[[1,2,3],[1,2,3]], [[1,2,3],[1,2,3]]]


@pytest.fixture
def camera_runtime(monkeypatch):
    """Run only the Python observer wiring; every ROS/native module is a fake."""
    monkeypatch.setitem(sys.modules, 'observation_codec', codec)
    observer = module('k1_camera_observer')
    captured = SimpleNamespace(publishers=[], subscriptions=[], threads=[], shutdown=False)
    class Node:
        def __init__(self, name, **kwargs):
            captured.node = self
            captured.node_options = kwargs
        def create_publisher(self, kind, topic, qos):
            publisher = SimpleNamespace(topic=topic, qos=qos, messages=[])
            publisher.publish = publisher.messages.append
            captured.publishers.append(publisher)
            return publisher
        def create_subscription(self, kind, topic, callback, qos):
            subscription = SimpleNamespace(topic=topic, callback=callback, qos=qos)
            captured.subscriptions.append(subscription)
            return subscription
        def create_timer(self, interval, callback): pass
        def destroy_node(self): captured.destroyed = True
    monkeypatch.setitem(sys.modules, 'cv2', SimpleNamespace(setNumThreads=captured.threads.append, error=RuntimeError))
    monkeypatch.setitem(sys.modules, 'numpy', SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'rclpy', SimpleNamespace(init=lambda: None, spin=lambda node: None,
                                                          shutdown=lambda: setattr(captured, 'shutdown', True)))
    monkeypatch.setitem(sys.modules, 'rclpy.node', SimpleNamespace(Node=Node))
    monkeypatch.setitem(sys.modules, 'rclpy.qos', SimpleNamespace(QoSProfile=lambda **kw: SimpleNamespace(**kw),
        ReliabilityPolicy=SimpleNamespace(BEST_EFFORT='best_effort'), HistoryPolicy=SimpleNamespace(KEEP_LAST='keep_last'),
        DurabilityPolicy=SimpleNamespace(VOLATILE='volatile')))
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', SimpleNamespace(Image=SimpleNamespace, CompressedImage=SimpleNamespace))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', SimpleNamespace(String=SimpleNamespace))
    # Ignore the real hold only inside this completely fake runtime fixture.
    original_exists = Path.exists
    monkeypatch.setattr(Path, 'exists', lambda path: False if path.name == 'k1-diagnostics-hold.json' else original_exists(path))
    return observer, captured


@pytest.mark.parametrize('source,stereo,expected', [
    ('left', False, ['/boostercamera/head/raw/rgb']),
    ('native', False, ['/booster_video_stream']),
    ('left', True, ['/boostercamera/head/raw/rgb', '/boostercamera/head/raw/right/rgb']),
    ('native', True, ['/boostercamera/head/raw/rgb', '/boostercamera/head/raw/right/rgb', '/booster_video_stream']),
])
def test_only_requested_camera_sources_and_latest_frame_qos(camera_runtime, source, stereo, expected):
    observer, captured = camera_runtime
    observer.main(['--mono-source', source] + (['--stereo'] if stereo else []))
    assert [s.topic for s in captured.subscriptions] == expected
    assert bool(captured.node.out) is stereo
    assert ('/transition/k1/stereo/compressed' in [p.topic for p in captured.publishers]) is stereo
    for endpoint in captured.subscriptions + captured.publishers:
        assert vars(endpoint.qos) == dict(history='keep_last', depth=1, reliability='best_effort', durability='volatile')
    assert captured.node_options == dict(start_parameter_services=False, enable_rosout=False)
    assert captured.threads == [1] and captured.destroyed and captured.shutdown
    captured.node.publish_status()
    import json
    assert json.loads(captured.node.status.messages[-1].data)['stereo_enabled'] is stereo


def test_mono_default_has_no_stereo_work_even_if_both_frames_exist(camera_runtime, monkeypatch):
    observer, captured = camera_runtime
    assert observer.parse_arguments([]).stereo is False
    observer.main([])
    node = captured.node
    node.receive('left', image())
    node.receive('right', image(frame='right'))
    calls = []
    monkeypatch.setattr(node, 'publish_mono', lambda: calls.append('mono'))
    monkeypatch.setattr(observer, 'pair_metadata', lambda *args: pytest.fail('Unrequested stereo processing'))
    node.publish_latest()
    assert calls == ['mono'] and node.sequence == 0


def test_callback_keeps_last_valid_frame_when_new_raw_frame_fails_validation(camera_runtime):
    observer, captured = camera_runtime
    observer.main([])
    receive = captured.subscriptions[0].callback
    valid = image()
    receive(valid)
    for damage in ('stride', 'frame', 'payload'):
        invalid = image(stamp=2_000_000_000)
        if damage == 'stride': invalid.step = 1_000_000; invalid.data = bytes(3_000_000)
        if damage == 'frame': invalid.header.frame_id = 'x'*257
        if damage == 'payload': invalid.data = bytes(codec.MAX_RAW_IMAGE_BYTES + 1)
        receive(invalid)
        assert captured.node.eyes['left'][0] is valid and captured.node.error


def test_incident_hold_precedes_native_imports(monkeypatch):
    monkeypatch.setitem(sys.modules, 'observation_codec', codec)
    observer = module('k1_camera_observer')
    original_exists = Path.exists
    monkeypatch.setattr(Path, 'exists', lambda path: True if path.name == 'k1-diagnostics-hold.json' else original_exists(path))
    monkeypatch.setitem(sys.modules, 'cv2', None)
    monkeypatch.setitem(sys.modules, 'rclpy', None)
    with pytest.raises(SystemExit, match='diagnostics held'):
        observer.main([])

def test_stereo_keeps_both_source_stamps_without_claiming_sync():
    result=codec.pair_metadata(image(),image(1_020_000_000,"right"))
    assert result["source_stamp_skew_ns"]==20_000_000
    assert result["acquisition_synchronization_verified"] is False
    with pytest.raises(ValueError):codec.pair_metadata(image(),image(1_100_000_000,"right"))
    with pytest.raises(ValueError):codec.pair_metadata(image(),image())


def test_both_camera_sources_must_advance_and_resize_retains_aspect():
    assert codec.pair_advances((11, 12), (10, 11))
    assert not codec.pair_advances((10, 12), (10, 11))
    assert not codec.pair_advances((12, 11), (10, 11))
    assert codec.display_size(640, 480) == (480, 360)
    w, h = codec.display_size(544, 448)
    assert w <= 480 and h <= 360 and abs(w/h - 544/448) < .003


def test_native_mono_jpeg_is_bounded_without_relabeling_frame_or_stamp():
    cv2 = pytest.importorskip('cv2')
    import numpy as np
    ok, encoded = cv2.imencode('.jpg', np.zeros((306,544,3), np.uint8))
    message = SimpleNamespace(format='jpeg', data=encoded.tobytes(),
                              header=image().header)
    assert codec.checked_jpeg(message) == (544, 306)
    assert message.format == 'jpeg' and message.header.frame_id == 'left'
    for damaged in (b'', encoded.tobytes()[:-2], b'x'*(512*1024+1)):
        message.data=damaged
        with pytest.raises(ValueError): codec.checked_jpeg(message)
    _, huge = cv2.imencode('.jpg', np.zeros((400,544,3), np.uint8))
    message.data=huge.tobytes()
    with pytest.raises(ValueError, match='dimensions'): codec.checked_jpeg(message)


def test_mono_transcode_reduces_pixels_without_touching_source_provenance():
    cv2=pytest.importorskip('cv2')
    import numpy as np
    pixels=np.random.default_rng(7).integers(0,256,(306,544,3),dtype=np.uint8)
    _,encoded=cv2.imencode('.jpg',pixels,[cv2.IMWRITE_JPEG_QUALITY,95])
    original=encoded.tobytes();header=image().header
    source=SimpleNamespace(format='jpeg',data=original,header=header)
    output=codec.transcode_mono(source)
    compressed=SimpleNamespace(format='rgb8; jpeg compressed bgr8',data=output)
    assert codec.checked_jpeg(compressed)==(432,243)
    assert len(output)<len(original)
    assert source.data==original and source.header is header

def test_receive_only_endpoint_rejects_every_write_route():
    calls=[]
    class Commands:
        tcp_server=SimpleNamespace(send_unity_error=calls.append)
    module("readonly_endpoint").install_readonly(Commands)
    instance=Commands()
    for name in ("publish","ros_service","unity_service","request","response"):
        getattr(instance,name)("/joint_ctrl","any",queue_size=1)
    assert len(calls)==5 and all("receive-only" in x for x in calls)
