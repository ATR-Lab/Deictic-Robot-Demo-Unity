#!/usr/bin/env python3
"""Reconnectable localhost ROS tunnel. Password lives only in process memory.

The launcher prompts without echo and sends the password through an anonymous
stdin pipe to a hidden detached worker. No credential file, environment value,
or process argument is created. Install paramiko in .codex/tunnel-deps first.
"""
import argparse
import getpass
import os
from pathlib import Path
import socket
import socketserver
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
STATE = ROOT/'.codex'
sys.path.insert(0, str(STATE/'tunnel-deps'))


def worker(args, password):
    import paramiko
    known_hosts = Path.home()/'.ssh/known_hosts'
    if not known_hosts.is_file():
        raise RuntimeError('First verify this host interactively with OpenSSH to populate known_hosts')
    transport = [None]
    lock = threading.Lock()

    def connect_loop():
        backoff = 1
        while True:
            client = paramiko.SSHClient()
            client.load_host_keys(str(known_hosts))
            try:
                client.connect(args.host, username=args.user, password=password,
                               allow_agent=False, look_for_keys=False,
                               timeout=10, auth_timeout=15, banner_timeout=15)
                session = client.get_transport()
                session.set_keepalive(15)
                with lock:
                    transport[0] = session
                backoff = 1
                print(time.strftime('%Y-%m-%d %H:%M:%S'), 'SSH connected; forwarding localhost', args.port, flush=True)
                while session.is_active():
                    time.sleep(1)
            except Exception as error:
                print(time.strftime('%Y-%m-%d %H:%M:%S'), 'SSH reconnect:', type(error).__name__, str(error), flush=True)
            finally:
                with lock:
                    transport[0] = None
                client.close()
            time.sleep(backoff)
            backoff = min(30, backoff*2)

    class Handler(socketserver.BaseRequestHandler):
        def handle(self):
            with lock:
                session = transport[0]
            if session is None or not session.is_active():
                return
            channel = None
            try:
                channel = session.open_channel('direct-tcpip', ('127.0.0.1', args.remote_port), self.client_address, timeout=10)

                def upstream():
                    try:
                        while True:
                            data = self.request.recv(65536)
                            if not data:
                                break
                            channel.sendall(data)
                    except (OSError, EOFError):
                        pass
                    finally:
                        channel.close()

                threading.Thread(target=upstream, daemon=True).start()
                while True:
                    data = channel.recv(65536)
                    if not data:
                        break
                    self.request.sendall(data)
            except Exception as error:
                print('Forwarding channel closed:', type(error).__name__, str(error), flush=True)
            finally:
                if channel is not None:
                    channel.close()

    class Server(socketserver.ThreadingTCPServer):
        allow_reuse_address = True
        daemon_threads = True

    with Server(('127.0.0.1', args.port), Handler) as server:
        (STATE/'tunnel.pid').write_text(str(os.getpid()), encoding='ascii')
        threading.Thread(target=connect_loop, daemon=True).start()
        server.serve_forever(poll_interval=.5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True, help='Verified SSH workstation hostname or IP address')
    parser.add_argument('--user', required=True, help='SSH account on the workstation')
    parser.add_argument('--port', type=int, default=10000)
    parser.add_argument('--remote-port', type=int, default=10000)
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not args.host.strip() or not args.user.strip():
        parser.error('--host and --user must be nonblank')
    if not 1 <= args.port <= 65535 or not 1 <= args.remote_port <= 65535:
        parser.error('--port and --remote-port must be between 1 and 65535')
    STATE.mkdir(exist_ok=True)
    if args.worker:
        password = sys.stdin.buffer.readline().decode('utf-8').rstrip('\r\n')
        if not password:
            raise RuntimeError('No password received on private stdin pipe')
        worker(args, password)
        return
    password = getpass.getpass('SSH password (held in memory only): ')
    command = [sys.executable, '-u', str(Path(__file__).resolve()), '--worker', '--host', args.host,
               '--user', args.user, '--port', str(args.port), '--remote-port', str(args.remote_port)]
    kwargs = dict(stdin=subprocess.PIPE, close_fds=True)
    if os.name == 'nt':
        kwargs['creationflags'] = subprocess.CREATE_NO_WINDOW | subprocess.DETACHED_PROCESS
    else:
        kwargs['start_new_session'] = True
    with (STATE/'tunnel.log').open('ab') as log:
        child = subprocess.Popen(command, stdout=log, stderr=log, **kwargs)
        child.stdin.write((password+'\n').encode())
        child.stdin.close()
    del password
    print(f'Tunnel worker PID {child.pid}; log {STATE / "tunnel.log"}')


if __name__ == '__main__':
    main()
