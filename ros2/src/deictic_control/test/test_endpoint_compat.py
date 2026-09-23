"""Missing unsubscribe compatibility, including an isolated real TCP endpoint."""
import importlib.util
import json
import os
from pathlib import Path
import queue
import socket
import struct
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[4]
WRAPPER = ROOT / 'ros2/scripts/run_endpoint.py'
spec = importlib.util.spec_from_file_location('deictic_run_endpoint', WRAPPER)
compat = importlib.util.module_from_spec(spec)
spec.loader.exec_module(compat)


def test_remove_only_requested_subscriber_and_duplicate_is_harmless():
    class Commands:
        pass
    camera, status = object(), object()
    removed, errors = [], []
    server = SimpleNamespace(subscribers_table={'/camera': camera, '/status': status},
        publishers_table={'/control': object()}, unregister_node=removed.append,
        loginfo=lambda _: None, send_unity_error=errors.append)
    assert compat.install_compatibility(Commands)
    assert not compat.install_compatibility(Commands)
    commands = Commands(); commands.tcp_server = server
    commands.remove_subscriber('/camera')
    commands.remove_subscriber('/camera')
    commands.remove_subscriber('/unknown')
    assert removed == [camera]
    assert server.subscribers_table == {'/status': status}
    assert '/control' in server.publishers_table
    assert errors == []
    commands.remove_subscriber(None)
    assert len(errors) == 1 and removed == [camera]


def test_compatibility_preserves_native_command():
    original = lambda self, topic: None
    class Commands:
        remove_subscriber = original
    assert not compat.install_compatibility(Commands)
    assert Commands.remove_subscriber is original


def wire_packet(topic, payload):
    topic = topic.encode()
    return struct.pack('<I', len(topic)) + topic + struct.pack('<I', len(payload)) + payload


def test_slow_viewer_keeps_only_latest_image_without_dropping_control_or_clock():
    messages = compat.CameraLatestQueue()
    camera = '/deictic/camera_view/stereo/image_raw/compressed'
    messages.put(wire_packet('__handshake', b'hello'))
    messages.put(wire_packet(camera, b'old'))
    controls = [wire_packet('/deictic/time_sync/reply', str(i).encode()) for i in range(10)]
    for control in controls:
        messages.put(control)
        for i in range(100):
            messages.put(wire_packet(camera, str(i).encode()))
    assert messages.qsize() == 12 and messages.unfinished_tasks == 12
    actual = []
    while not messages.empty():
        actual.append(messages.get_nowait())
        messages.task_done()
    assert actual == [wire_packet('__handshake', b'hello'), wire_packet(camera, b'99')] + controls
    assert messages.unfinished_tasks == 0 and not messages.images
    messages.join()


def test_camera_queue_does_not_coalesce_service_bundles_or_arbitrary_payloads():
    messages = compat.CameraLatestQueue()
    camera = '/deictic/camera_view/stereo/image_raw'
    packet = wire_packet(camera, b'pixels')
    bundle = packet + wire_packet('__response', b'service')
    values = [bundle, bundle, bytearray(b'unframed'), camera.encode(), None]
    for value in values:
        messages.put(value)
    for value in values:
        assert messages.get_nowait() == value


def test_raw_and_compressed_images_have_independent_latest_slots():
    messages = compat.CameraLatestQueue()
    raw = '/deictic/camera_view/stereo/image_raw'
    jpeg = raw + '/compressed'
    for i in range(20):
        messages.put(wire_packet(raw, bytes([i])))
        messages.put(wire_packet(jpeg, bytes([i + 1])))
    assert messages.qsize() == 2
    assert messages.get_nowait() == wire_packet(raw, bytes([19]))
    assert messages.get_nowait() == wire_packet(jpeg, bytes([20]))


def test_actual_tcp_unsubscribe_clears_camera_demand_and_preserves_connection(tmp_path, monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.serialization import deserialize_message, serialize_message
    from std_msgs.msg import String

    vendor = ROOT / '.codex/ros2/vendor/ROS-TCP-Endpoint'
    if not (vendor / 'ros_tcp_endpoint/server.py').is_file():
        pytest.skip('Run ros2/scripts/setup.sh to provision the pinned endpoint')
    with socket.socket() as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    # Same transport on both fixture and endpoint, including WSL installations
    # where Fast DDS discovers peers but cannot deliver via shared memory.
    monkeypatch.setenv('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')
    env = dict(os.environ, ROS_DOMAIN_ID='211')
    env['PYTHONPATH'] = str(vendor)+os.pathsep+env.get('PYTHONPATH', '')
    log = (tmp_path / 'endpoint.log').open('w+', encoding='utf-8')
    process = subprocess.Popen([sys.executable, str(WRAPPER), '--ros-args',
        '-p', 'ROS_IP:=127.0.0.1', '-p', f'ROS_TCP_PORT:={port}'], env=env, stdout=log, stderr=log)
    context, node, connection, executor = Context(), None, None, None
    stop, received, failures = threading.Event(), queue.Queue(), queue.Queue()
    reader = None
    try:
        rclpy.init(context=context, domain_id=211)
        node = rclpy.create_node('endpoint_compat_fixture', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        camera = node.create_publisher(String, '/compat/camera', 10)
        status = node.create_publisher(String, '/compat/status', 10)
        deadline = time.monotonic()+10.
        while connection is None and time.monotonic() < deadline:
            if process.poll() is not None:
                log.seek(0); pytest.fail('Endpoint exited: '+log.read())
            try:
                connection = socket.create_connection(('127.0.0.1', port), timeout=.1)
            except OSError:
                time.sleep(.02)
        assert connection is not None, 'Endpoint did not listen'
        connection.settimeout(.1)

        def read():
            buffer = bytearray()
            try:
                while not stop.is_set():
                    try:
                        chunk = connection.recv(65536)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise EOFError('Endpoint closed the shared connection')
                    buffer.extend(chunk)
                    while len(buffer) >= 4:
                        name_size = struct.unpack_from('<I', buffer)[0]
                        if len(buffer) < 8+name_size:
                            break
                        size = struct.unpack_from('<I', buffer, 4+name_size)[0]
                        total = 8+name_size+size
                        if len(buffer) < total:
                            break
                        name = bytes(buffer[4:4+name_size]).decode()
                        payload = bytes(buffer[8+name_size:total])
                        del buffer[:total]
                        if name == '__error':
                            raise RuntimeError(payload.decode())
                        if name.startswith('/compat/'):
                            received.put((name, deserialize_message(payload, String).data))
            except Exception as error:
                if not stop.is_set(): failures.put(error)

        reader = threading.Thread(target=read, daemon=True); reader.start()

        def send(command, **fields):
            name = command.encode(); payload = json.dumps(fields).encode()+b'\0'
            connection.sendall(struct.pack('<I', len(name))+name+struct.pack('<I', len(payload))+payload)

        def wait(predicate, description):
            until = time.monotonic()+8.
            while time.monotonic() < until:
                if not failures.empty(): raise failures.get()
                assert process.poll() is None, 'Endpoint process stopped'
                executor.spin_once(timeout_sec=.01)
                if predicate(): return
            pytest.fail(description)

        seen = set()
        def forwarded(topic, marker):
            # Round-trip through the endpoint's real ROS publisher/subscriber
            # nodes. The separate fixture publishers observe DDS demand only;
            # test transport does not depend on WSL cross-process delivery.
            name = topic.encode(); payload = serialize_message(String(data=marker))
            connection.sendall(struct.pack('<I', len(name))+name+struct.pack('<I', len(payload))+payload)
            while not received.empty(): seen.add(received.get_nowait())
            return (topic, marker) in seen

        send('__subscribe', topic='/compat/camera', message_name='std_msgs/String')
        send('__subscribe', topic='/compat/status', message_name='std_msgs/String')
        send('__publish', topic='/compat/camera', message_name='std_msgs/String')
        send('__publish', topic='/compat/status', message_name='std_msgs/String')
        wait(lambda: camera.get_subscription_count() == status.get_subscription_count() == 1,
             'Both ROS subscriber demands were not created')
        wait(lambda: forwarded('/compat/camera', 'before'), 'Initial camera forwarding failed')
        send('__remove_subscriber', topic='/compat/camera')
        send('__remove_subscriber', topic='/compat/camera')
        wait(lambda: camera.get_subscription_count() == 0, 'Camera subscriber demand was not released')
        assert status.get_subscription_count() == 1
        wait(lambda: forwarded('/compat/status', 'after-remove'), 'Unrelated status stopped after removal')
        send('__subscribe', topic='/compat/camera', message_name='std_msgs/String')
        wait(lambda: camera.get_subscription_count() == 1, 'Camera could not resubscribe on the same socket')
        wait(lambda: forwarded('/compat/camera', 'after-resubscribe'), 'Resubscribed camera forwarding failed')
        assert failures.empty()
    finally:
        stop.set()
        if connection is not None: connection.close()
        if reader is not None: reader.join(timeout=1.)
        if executor is not None: executor.shutdown()
        if node is not None: node.destroy_node()
        if context.ok(): context.shutdown()
        process.terminate()
        try: process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait(timeout=5.)
        log.close()
