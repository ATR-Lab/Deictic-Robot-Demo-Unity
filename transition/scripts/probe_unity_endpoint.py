#!/usr/bin/env python3
"""Receive-only ROS-TCP probe, usable through the launcher's SSH tunnel.

This checks the wire path Unity uses without starting Unity or registering any
publisher/service. Results describe receipt and source progression, not certified
physical acquisition freshness.
"""
import argparse
import hashlib
import json
from pathlib import Path
import socket
import struct
import sys
import time
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'deployment'))
from observation_codec import checked_jpeg

TOPICS = {
    '/joint_states': 'sensor_msgs/JointState',
    '/transition/hardware/status': 'std_msgs/String',
    '/transition/hardware/camera_status': 'std_msgs/String',
    '/transition/hardware/head/image_raw/compressed': 'sensor_msgs/CompressedImage',
}


def probe(connection, seconds):
    connection.settimeout(1)
    for topic, name in TOPICS.items():
        destination = b'__subscribe'
        data = json.dumps({'topic': topic, 'message_name': name}).encode() + b'\0'
        connection.sendall(struct.pack('<I', len(destination)) + destination + struct.pack('<I', len(data)) + data)
    counts = dict.fromkeys(TOPICS, 0)
    statuses, images, errors = {}, [], []
    buffer = bytearray()
    start = time.monotonic()
    while time.monotonic() - start < seconds:
        try:
            chunk = connection.recv(65536)
        except socket.timeout:
            continue
        if not chunk:
            raise IOError('Endpoint disconnected')
        buffer.extend(chunk)
        while len(buffer) >= 4:
            size = struct.unpack_from('<I', buffer)[0]
            if size > 1024:
                raise ValueError('Invalid topic length')
            if len(buffer) < size + 8:
                break
            length = struct.unpack_from('<I', buffer, size + 4)[0]
            if length > 3 * 1024 * 1024:
                raise ValueError('Invalid packet size')
            if len(buffer) < size + 8 + length:
                break
            topic = bytes(buffer[4:4 + size]).decode()
            data = bytes(buffer[size + 8:size + 8 + length])
            del buffer[:size + 8 + length]
            if topic == '__error':
                errors.append(data.decode(errors='replace'))
            if topic not in counts:
                continue
            counts[topic] += 1
            if data[:4] != b'\x00\x01\x00\x00':
                raise ValueError('Expected little-endian CDR encapsulation')
            if TOPICS[topic] == 'std_msgs/String':
                n = struct.unpack_from('<I', data, 4)[0]
                statuses[topic] = json.loads(data[8:8 + n - 1])
            elif TOPICS[topic] == 'sensor_msgs/CompressedImage':
                sec, nsec = struct.unpack_from('<iI', data, 4)
                offset = 12
                strings = []
                for _ in range(2):
                    offset = 4 + ((offset - 4 + 3) // 4) * 4
                    n = struct.unpack_from('<I', data, offset)[0]
                    strings.append(data[offset + 4:offset + 4 + n - 1].decode())
                    offset += 4 + n
                offset = 4 + ((offset - 4 + 3) // 4) * 4
                n = struct.unpack_from('<I', data, offset)[0]
                jpeg = data[offset + 4:offset + 4 + n]
                if len(jpeg) != n:
                    raise ValueError('Truncated JPEG sequence')
                dimensions = checked_jpeg(SimpleNamespace(format=strings[1], data=jpeg))
                images.append({'stamp_ns': sec * 10**9 + nsec, 'frame': strings[0],
                               'dimensions': dimensions, 'bytes': n,
                               'sha256': hashlib.sha256(jpeg).hexdigest()})
    return {'schema_version': 1, 'receive_only': True, 'motion_capability': False,
            'acquisition_freshness_verified': False, 'duration_s': time.monotonic() - start,
            'sample_counts': counts, 'latest_status': statuses, 'images': images, 'errors': errors,
            'telemetry_received': counts['/joint_states'] > 0,
            'camera_received': len(images) > 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=10000)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 120:
        parser.error('seconds must be 1..120')
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', encoding='utf-8') as receipt:
        with socket.create_connection((args.host, args.port), timeout=5) as connection:
            result = probe(connection, args.seconds)
        json.dump(result, receipt, indent=2, allow_nan=False)
    print(json.dumps({k: result[k] for k in ('sample_counts', 'telemetry_received', 'camera_received', 'errors')}))
    return 0 if result['telemetry_received'] and not result['errors'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
