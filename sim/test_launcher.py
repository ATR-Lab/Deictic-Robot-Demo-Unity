"""Linux socket checks for the launcher; Docker is replaced by a harmless stub."""
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest


@unittest.skipUnless(sys.platform.startswith('linux') and shutil.which('bash'),
                     'Launcher socket behavior is tested on Linux')
class LauncherPortTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='isaac-launcher-test-')
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        (self.root / 'sim').mkdir()
        (self.root / 'bin').mkdir()
        self.marker = self.root / 'docker-invoked'
        docker = self.root / 'bin' / 'docker'
        docker.write_text('#!/bin/sh\n: > "$LAUNCHER_TEST_MARKER"\n')
        docker.chmod(0o755)
        self.env = dict(os.environ, PATH=str(self.root / 'bin') + os.pathsep + os.environ['PATH'],
                        LAUNCHER_TEST_MARKER=str(self.marker))
        self.source = Path(__file__).with_name('run_isaac_container.sh').read_text()

    @staticmethod
    def free_port(kind):
        with socket.socket(socket.AF_INET, kind) as probe:
            probe.bind(('127.0.0.1', 0))
            return probe.getsockname()[1]

    def launch(self, tcp_port, udp_port):
        # Only remap the two fixed ports to isolated ephemeral ports. The full
        # launcher's actual preflight and failure/child ordering run unchanged.
        source = self.source.replace('socket.SOCK_STREAM, 49100', f'socket.SOCK_STREAM, {tcp_port}')
        source = source.replace('socket.SOCK_DGRAM, 47998', f'socket.SOCK_DGRAM, {udp_port}')
        launcher = self.root / 'sim' / 'run_isaac_container.sh'
        launcher.write_text(source)
        return subprocess.run(['bash', str(launcher), '--webrtc'], env=self.env,
                              capture_output=True, text=True, timeout=10)

    def assert_blocked(self, result, protocol):
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('Refusing --webrtc launch', result.stderr)
        self.assertIn(protocol, result.stderr)
        self.assertFalse(self.marker.exists(), 'Docker must not run after a failed port probe')

    def test_active_tcp_listener_still_blocks_even_with_reuse(self):
        for reuse in (False, True):
            with self.subTest(reuse=reuse), socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, int(reuse))
                server.bind(('127.0.0.1', 0))
                server.listen(1)
                result = self.launch(server.getsockname()[1], self.free_port(socket.SOCK_DGRAM))
                self.assert_blocked(result, 'TCP')

    def test_udp_binding_remains_exclusive_even_with_reuse(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('127.0.0.1', 0))
            result = self.launch(self.free_port(socket.SOCK_STREAM), server.getsockname()[1])
            self.assert_blocked(result, 'UDP')

    def test_ipv6_listener_still_blocks(self):
        try:
            server = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
            server.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('::1', 0))
        except OSError as error:
            if 'server' in locals():
                server.close()
            self.skipTest(f'IPv6 unavailable: {error}')
        with server:
            server.listen(1)
            self.assert_blocked(self.launch(server.getsockname()[1], self.free_port(socket.SOCK_DGRAM)), 'TCP ::')

    def test_closed_server_time_wait_allows_restart(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('127.0.0.1', 0))
            server.listen(1)
            port = server.getsockname()[1]
            with socket.create_connection(('127.0.0.1', port), timeout=2) as client:
                connection, _ = server.accept()
                # Close the server side first, so its port enters TIME_WAIT.
                connection.close()
                self.assertEqual(client.recv(1), b'')
        deadline = time.monotonic() + 2
        while True:
            entries = [line.split() for line in Path('/proc/net/tcp').read_text().splitlines()[1:]]
            time_wait = any(int(entry[1].split(':')[1], 16) == port and entry[3] == '06'
                            for entry in entries)
            if time_wait or time.monotonic() >= deadline:
                break
            time.sleep(.01)
        self.assertTrue(time_wait, 'Test must exercise a real server-port TIME_WAIT entry')
        # Demonstrate the original no-reuse probe would reject this exact port.
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as original_probe:
            with self.assertRaises(OSError):
                original_probe.bind(('0.0.0.0', port))
        result = self.launch(port, self.free_port(socket.SOCK_DGRAM))
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue(self.marker.exists(), 'Free listening port must allow the Docker launch')


if __name__ == '__main__':
    unittest.main()
