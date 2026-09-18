#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${ISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim:5.0.0}"
for argument in "$@"; do
  if [[ "$argument" == --webrtc ]]; then
    # Probe both address families without SO_REUSEPORT. A duplicate streaming
    # server can otherwise bind successfully yet leave clients without video.
    /usr/bin/python3 - <<'PY'
import errno
import socket
import sys

probes = []
failures = []
try:
    for protocol, kind, port in (("TCP", socket.SOCK_STREAM, 49100),
                                 ("UDP", socket.SOCK_DGRAM, 47998)):
        for family, address in ((socket.AF_INET, "0.0.0.0"),
                                (socket.AF_INET6, "::")):
            try:
                probe = socket.socket(family, kind)
                probes.append(probe)
                if kind == socket.SOCK_STREAM:
                    # A stopped TCP server can leave accepted connections in
                    # TIME_WAIT. Reuse that address while still rejecting a
                    # live listener; UDP must retain exclusive binding.
                    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                if family == socket.AF_INET6:
                    probe.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
                probe.bind((address, port))
            except OSError as error:
                if family == socket.AF_INET6 and error.errno in (
                    errno.EAFNOSUPPORT, errno.EPROTONOSUPPORT, errno.EADDRNOTAVAIL
                ):
                    continue  # IPv6 is optional on the host.
                failures.append(f"{protocol} {address}:{port}: {error}")
    if failures:
        print("Refusing --webrtc launch: default streaming ports are unavailable.", file=sys.stderr)
        for failure in failures:
            print(f"  {failure}", file=sys.stderr)
        print("Inspect listeners with: sudo ss -lntup | grep -E ':(49100|47998)\\b'", file=sys.stderr)
        print("Inspect containers with: docker ps --format '{{.Names}}\\t{{.Image}}\\t{{.Command}}'", file=sys.stderr)
        print("Reuse the existing stream, or stop only its verified owned container before retrying.", file=sys.stderr)
        print("No containers or listeners were stopped by this preflight.", file=sys.stderr)
        sys.exit(1)
finally:
    for probe in probes:
        probe.close()
PY
    break
  fi
done
mkdir -p "$repo_dir/sim/artifacts/cache"
docker run --rm --name "deictic-k1-${USER:-sim}-$$" --gpus all --network host \
  -e ACCEPT_EULA=Y -e ROS_DISTRO=jazzy -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e PYTHONPATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/rclpy \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib \
  -v "$repo_dir/sim/artifacts/cache:/root/.cache" \
  -v "$repo_dir:/workspace/deictic" -w /workspace/deictic --entrypoint /isaac-sim/python.sh \
  "$image" sim/k1_isaac.py --headless "$@"
