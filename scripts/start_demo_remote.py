#!/usr/bin/env python3
"""Supervise the installed native-Ubuntu demo, without commanding robot motion.

This file may be streamed through SSH with ``python3 -u -``; --repo is explicit.
The Windows Start-Demo.ps1 launcher owns the SSH connection and ROS tunnel.
"""

import argparse
import errno
import ipaddress
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import time
import uuid


NODE_NAMES = {"k1_fixed_base_isaac", "k1_sim_trajectory_relay", "deictic_control",
              "deictic_registration", "deictic_camera_view_relay", "UnityEndpoint"}
PARAMS = "ros2/src/deictic_registration/config/isaac_head1280_cuda.yaml"
# This is an installed-file check, not a deployment list. The launcher is
# transferred alone; the matching project revision must already be present.
REQUIRED_SOURCES = (
    "sim/run_isaac_container.sh", "sim/k1_isaac.py", "sim/k1_model.py",
    "sim/head_control.py", "sim/head_stereo.py", "sim/loop_timing.py",
    "sim/trajectory_relay.py",
    "ros2/scripts/run_bridge.sh", "ros2/scripts/run_endpoint.py",
    "ros2/scripts/camera_view_relay.py",
    "ros2/src/deictic_control/deictic_control/node.py",
    "ros2/src/deictic_control/deictic_control/teleop.py",
    "ros2/src/deictic_registration/deictic_registration/node.py", PARAMS,
)


def log(message):
    try:
        print(f"[demo] {message}", flush=True)
    except OSError:
        # A disconnected SSH PTY may return EIO/BrokenPipe even during cleanup.
        pass


def service_plan(repo, public_ip):
    """Keep the four commands aligned with docs/start-demo.md."""
    return [
        ("isaac", ["bash", str(repo / "sim/run_isaac_container.sh"),
                   "--synthetic-headset", "--headset-resolution-scale", "2",
                   "--textured-table", "--floor-profile", "matte",
                   "--camera-mount-profile", "reach-projection-balanced",
                   "--webrtc", "--public-ip", public_ip, "--output",
                   "/workspace/deictic/sim/artifacts/reach_head1280_matte_live"]),
        ("relay", ["/usr/bin/python3", "sim/trajectory_relay.py"]),
        ("bridge", ["bash", "ros2/scripts/run_bridge.sh", "markerless"]),
        ("registration", [str(repo / ".venv-registration/bin/python"), "-m",
                          "deictic_registration.node", "--ros-args", "--params-file", PARAMS]),
    ]


def sourced_environment(repo):
    # An SSH login may auto-activate Conda. Start from an allowlist instead of
    # inheriting its Python paths, library paths, ROS overlays or shell hooks.
    base = {key: os.environ[key] for key in
            ("HOME", "USER", "LOGNAME", "LANG", "LC_ALL", "TERM", "TZ",
             "SSH_AUTH_SOCK", "XDG_RUNTIME_DIR", "DOCKER_HOST", "DOCKER_CONTEXT")
            if key in os.environ}
    base["PATH"] = "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    result = subprocess.run(
        ["/bin/bash", "--noprofile", "--norc", "-c",
         "source /opt/ros/jazzy/setup.bash >&2 && /usr/bin/env -0"],
        env=base, check=True, stdout=subprocess.PIPE, timeout=30)
    env = dict(item.decode().split("=", 1) for item in result.stdout.split(b"\0") if item)
    env.update(ROS_DOMAIN_ID="42", RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
               FASTDDS_BUILTIN_TRANSPORTS="UDPv4", PYTHONUNBUFFERED="1",
               ROS_BIND_IP="127.0.0.1", ROS_TCP_PORT="10000",
               DEICTIC_PYTHON=str(repo / ".venv-control/bin/python"),
               DEICTIC_CONTROL_PARAMS=str(repo / PARAMS))
    env["PYTHONPATH"] = ":".join([str(repo / "ros2/src/deictic_control"),
                                 str(repo / "ros2/src/deictic_registration"),
                                 str(repo / ".codex/ros2/vendor/ROS-TCP-Endpoint"),
                                 env.get("PYTHONPATH", "")])
    return env


def run_checked(command, env, repo, timeout=30):
    result = subprocess.run(command, env=env, cwd=repo, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, timeout=timeout)
    if result.returncode:
        raise RuntimeError(f"Preflight failed: {shlex.join(command)}\n{result.stdout[-6000:]}")
    return result.stdout


def probe_ports():
    probes = []
    try:
        for kind, port in ((socket.SOCK_STREAM, 10000),
                           (socket.SOCK_STREAM, 49100), (socket.SOCK_DGRAM, 47998)):
            for family, address in ((socket.AF_INET, "0.0.0.0"), (socket.AF_INET6, "::")):
                try:
                    probe = socket.socket(family, kind)
                    probes.append(probe)
                    if kind == socket.SOCK_STREAM:
                        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    if family == socket.AF_INET6:
                        probe.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
                    probe.bind((address, port))
                except OSError as error:
                    if family == socket.AF_INET6 and error.errno in (
                            errno.EAFNOSUPPORT, errno.EPROTONOSUPPORT, errno.EADDRNOTAVAIL):
                        continue
                    protocol = "TCP" if kind == socket.SOCK_STREAM else "UDP"
                    raise RuntimeError(f"{protocol} {address}:{port} is unavailable. "
                                       "Reuse or stop its existing service first; nothing was stopped.") from error
    finally:
        for probe in probes:
            probe.close()


GRAPH_PROBE = """
import json, time, rclpy
rclpy.init()
node = rclpy.create_node('deictic_launcher_preflight')
try:
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    print('DEICTIC_GRAPH=' + json.dumps(node.get_node_names()))
finally:
    node.destroy_node()
    rclpy.shutdown()
"""


def preflight(repo, env):
    required = (*REQUIRED_SOURCES, ".venv-control/bin/python",
                ".venv-registration/bin/python", ".codex/ros2/vendor/ROS-TCP-Endpoint")
    missing = [str(repo / item) for item in required if not (repo / item).exists()]
    if missing:
        raise RuntimeError("Missing installed sources/dependencies:\n" + "\n".join(missing)
                           + "\nDeploy the matching project revision first; this launcher transfers only itself.")
    for executable in ("bash", "docker", "nvidia-smi"):
        if not shutil.which(executable, path=env["PATH"]):
            raise RuntimeError(f"Required executable is missing: {executable}")
    probe_ports()
    run_checked(["docker", "info", "--format", "{{.ServerVersion}}"], env, repo)
    # Fail before waiting through a several-minute simulator boot for a missing
    # installed image. This quick-start helper does not install dependencies.
    run_checked(["docker", "image", "inspect", "nvcr.io/nvidia/isaac-sim:5.0.0"], env, repo)
    run_checked(["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"], env, repo)
    containers = run_checked(["docker", "ps", "--format", "{{.Names}}"], env, repo)
    conflicts = [name for name in containers.splitlines() if name.startswith("deictic-k1-")]
    if conflicts:
        raise RuntimeError("Existing demo containers must be reused or stopped in their own session: "
                           + ", ".join(conflicts))
    imports = [
        ("/usr/bin/python3", "import rclpy; import sensor_msgs.msg; import trajectory_msgs.msg"),
        (str(repo / ".venv-control/bin/python"),
         "import rclpy, numpy, scipy, gtsam, cv2; import ros_tcp_endpoint; import deictic_control.node"),
        (str(repo / ".venv-registration/bin/python"),
         "import rclpy, cv2, torch, lightglue; import deictic_registration.node; "
         "assert torch.cuda.is_available(), 'Registration profile requires CUDA'"),
    ]
    for executable, code in imports:
        run_checked([executable, "-c", code], env, repo, timeout=90)
    output = run_checked(["/usr/bin/python3", "-c", GRAPH_PROBE], env, repo)
    names = next(json.loads(line[len("DEICTIC_GRAPH="):]) for line in output.splitlines()
                 if line.startswith("DEICTIC_GRAPH="))
    conflicts = NODE_NAMES.intersection(names)
    if conflicts:
        raise RuntimeError("Existing demo ROS nodes in domain 42: " + ", ".join(sorted(conflicts))
                           + ". Stop/reuse their original session first; nothing was stopped.")


class Supervisor:
    def __init__(self, repo, env, logs):
        self.repo, self.env, self.logs = repo, env, logs
        self.children = []
        self.container_name = None
        self.stopping = False

    def start(self, name, command):
        path = self.logs / f"{name}.log"
        child_env = self.env.copy()
        if name == "isaac":
            # The installed shell script names its container from USER and $$.
            # Use an unguessable session token and launch Bash directly so its
            # PID is known. No updated remote shell script is required.
            child_env["USER"] = "session-" + uuid.uuid4().hex[:16]
        # Do not let a terminal signal arrive between spawn and recording its
        # ownership. Restore the mask in the child too (Popen inherits it).
        stop_signals = {signal.SIGINT, signal.SIGTERM, signal.SIGHUP}
        previous_mask = signal.pthread_sigmask(signal.SIG_BLOCK, stop_signals)
        try:
            with path.open("wb") as output:
                process = subprocess.Popen(
                    command, cwd=self.repo, env=child_env, stdin=subprocess.DEVNULL,
                    stdout=output, stderr=subprocess.STDOUT, start_new_session=True,
                    preexec_fn=lambda: signal.pthread_sigmask(signal.SIG_SETMASK, previous_mask))
            self.children.append((name, process, path))
            if name == "isaac":
                self.container_name = f"deictic-k1-{child_env['USER']}-{process.pid}"
        finally:
            signal.pthread_sigmask(signal.SIG_SETMASK, previous_mask)
        log(f"Started {name} (PID {process.pid}); log: {path}")
        return process

    def check_children(self):
        for name, process, path in self.children:
            code = process.poll()
            if code is not None:
                tail = path.read_bytes()[-5000:].decode(errors="replace")
                raise RuntimeError(f"{name} exited ({code}). Log: {path}\n{tail}")

    def wait_for_isaac(self, timeout):
        path = self.logs / "isaac.log"
        deadline, report_at = time.monotonic() + timeout, 0
        pending = b""
        with path.open("rb") as output:
            while time.monotonic() < deadline:
                self.check_children()
                pending += output.read()
                if b"DEICTIC_K1_READY" in pending:
                    log("DEICTIC_K1_READY observed; starting ROS services.")
                    return
                pending = pending[-64:]
                if time.monotonic() >= report_at:
                    log(f"Waiting for Isaac camera scene readiness; log: {path}")
                    report_at = time.monotonic() + 30
                time.sleep(0.5)
        raise RuntimeError(f"Isaac did not report DEICTIC_K1_READY within {timeout:g}s. Log: {path}")

    def wait_for_endpoint(self, timeout=90):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.check_children()
            try:
                with socket.create_connection(("127.0.0.1", 10000), timeout=0.5):
                    return
            except OSError:
                time.sleep(0.5)
        raise RuntimeError("ROS TCP endpoint did not listen on 127.0.0.1:10000 within 90s.")

    def cleanup(self):
        if self.stopping:
            return
        self.stopping = True
        # A second Ctrl+C must not interrupt the cleanup sequence and orphan
        # detached groups. The caller installs these handlers only in main.
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal.SIG_IGN)
        log("Stopping this session's services.")
        for _, process, _ in reversed(self.children):
            self._signal_group(process.pid, signal.SIGINT)
        self._wait_groups(5)
        # docker run may exit before its container. Resolve ONLY our random
        # exact name to an ID; never enumerate and stop unrelated containers.
        if self.container_name:
            try:
                result = subprocess.run(
                    ["docker", "inspect", "--type", "container", "--format", "{{.Id}}",
                     self.container_name], env=self.env, stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL, text=True, timeout=10)
                container_id = result.stdout.strip()
                if result.returncode == 0 and len(container_id) == 64 and all(
                        character in "0123456789abcdef" for character in container_id):
                    stopped = subprocess.run(["docker", "stop", "--time", "10", container_id],
                                             env=self.env, stdout=subprocess.DEVNULL,
                                             stderr=subprocess.DEVNULL, timeout=20)
                    if stopped.returncode:
                        log(f"Docker could not stop this session's container: {self.container_name}")
            except (OSError, subprocess.TimeoutExpired) as error:
                log(f"Container cleanup needs inspection: {self.container_name}: {error}")
        for sig in (signal.SIGTERM, signal.SIGKILL):
            for _, process, _ in reversed(self.children):
                self._signal_group(process.pid, sig)
            self._wait_groups(3 if sig == signal.SIGTERM else 1)
        for _, process, _ in self.children:
            process.poll()
        log(f"Session stopped. Logs retained: {self.logs}")

    @staticmethod
    def _signal_group(pid, sig):
        try:
            os.killpg(pid, sig)
        except ProcessLookupError:
            pass

    def _wait_groups(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            alive = False
            for _, process, _ in self.children:
                process.poll()
                try:
                    os.killpg(process.pid, 0)
                    alive = True
                except ProcessLookupError:
                    pass
            if not alive:
                return
            time.sleep(0.1)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", required=True)
    parser.add_argument("--public-ip", required=True, type=lambda value: str(ipaddress.ip_address(value)))
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--startup-timeout", type=float, default=900)
    args = parser.parse_args(argv)
    if args.startup_timeout <= 0:
        parser.error("--startup-timeout must be positive")
    repo = Path(args.repo).expanduser().resolve()
    plan = service_plan(repo, args.public_ip)
    if args.dry_run:
        log(f"Dry run; repository: {repo}; no services or files will be created.")
        log("Clean Bash environment, source /opt/ros/jazzy/setup.bash; ROS_DOMAIN_ID=42; "
            "RMW_IMPLEMENTATION=rmw_fastrtps_cpp; FASTDDS_BUILTIN_TRANSPORTS=UDPv4")
        log(f"DEICTIC_PYTHON={repo / '.venv-control/bin/python'}")
        log(f"DEICTIC_CONTROL_PARAMS={repo / PARAMS}")
        for name, command in plan:
            log(f"{name}: {shlex.join(command)}")
        log("Wait for DEICTIC_K1_READY before relay/bridge/registration; "
            "Ctrl+C or SSH hangup stops only this session. Unity is untouched.")
        return 0
    if sys.platform != "linux":
        parser.error("Actual startup requires native Ubuntu. Use --dry-run to preview elsewhere.")
    import fcntl
    supervisor = None
    lock = None
    try:
        if not repo.is_dir():
            raise RuntimeError(f"Repository does not exist: {repo}")
        state = repo / ".codex/demo-sessions"
        state.mkdir(parents=True, exist_ok=True)
        lock = (state / "launcher.lock").open("a+")
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("Another one-command demo session already owns this repository.")
        def stop(signum, _frame):
            raise KeyboardInterrupt(f"Signal {signum}")
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, stop)
        env = sourced_environment(repo)
        log("Checking installed dependencies, ports, GPU access and existing ROS services.")
        preflight(repo, env)
        logs = state / (time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8])
        logs.mkdir()
        supervisor = Supervisor(repo, env, logs)
        supervisor.start(*plan[0])
        supervisor.wait_for_isaac(args.startup_timeout)
        for name, command in plan[1:]:
            supervisor.start(name, command)
        supervisor.wait_for_endpoint()
        log("DEICTIC_DEMO_RUNNING: Isaac scene and ROS TCP endpoint are up; "
            "registration may still initialize/download weights. Logs: " + str(logs))
        log("Keep this terminal open. Ctrl+C stops this session. "
            "Confirm fresh /deictic/status and /joint_states before operating Unity.")
        while True:
            supervisor.check_children()
            time.sleep(0.5)
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        log(f"ERROR: {error}")
        return 1
    finally:
        if supervisor:
            supervisor.cleanup()
        if lock:
            lock.close()


if __name__ == "__main__":
    sys.exit(main())
