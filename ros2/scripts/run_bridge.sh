#!/usr/bin/env bash
set -euo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
MODE=${1:-mock}
STATE="$REPO/.codex/ros2"
ENDPOINT="$STATE/vendor/ROS-TCP-Endpoint"
[[ -d "$ENDPOINT" ]] || { echo 'Run bash ros2/scripts/setup.sh first'; exit 1; }
[[ -n ${ROS_DISTRO:-} ]] || { echo 'Source /opt/ros/<distro>/setup.bash first'; exit 1; }
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
# This workaround is specific to the supplied WSL network. Applying LARGE_DATA
# to a native host partitions it from Isaac's default UDP discovery/transport.
if grep -qi microsoft /proc/sys/kernel/osrelease; then
  export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}
  export ROS_STATIC_PEERS=${ROS_STATIC_PEERS:-127.0.0.1}
  export FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-LARGE_DATA}
else
  # Isaac is host-networked but has a private IPC namespace. UDP avoids Fast
  # DDS selecting a shared-memory path that cannot carry samples across it.
  export FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}
fi
export PYTHONPATH="$REPO/ros2/src/deictic_control:$REPO/ros2/src/deictic_registration:$ENDPOINT:${PYTHONPATH:-}"
PYTHON=${DEICTIC_PYTHON:-python3}
args=(--ros-args -p "urdf:=$REPO/models/K1/K1_22dof.urdf" -p allow_execution:=true)
case "$MODE" in
  mock) args+=(-p synthetic_test:=true -p synthetic_reference_backend:=true -p mock_joint_states:=true) ;;
  synthetic) args+=(-p synthetic_test:=true -p allow_teleoperation:=true) ;;
  markerless) args+=(-p allow_teleoperation:=true) ;;
  *) echo 'Mode must be mock, synthetic, or markerless'; exit 1 ;;
esac
if [[ -n ${DEICTIC_CONTROL_PARAMS:-} ]]; then
  [[ "$MODE" == markerless ]] || { echo 'DEICTIC_CONTROL_PARAMS is supported only in markerless mode'; exit 1; }
  [[ -f "$DEICTIC_CONTROL_PARAMS" ]] || { echo "Missing controller parameter file: $DEICTIC_CONTROL_PARAMS"; exit 1; }
  args+=(--params-file "$DEICTIC_CONTROL_PARAMS")
fi
BIND_IP=${ROS_BIND_IP:-127.0.0.1}
TCP_PORT=${ROS_TCP_PORT:-10000}
# The endpoint's listener runs in a background thread, so a failed bind may
# leave its process alive. Check before starting any of the three services.
if ! "$PYTHON" - "$BIND_IP" "$TCP_PORT" <<'PY'
import socket
import sys

host, raw_port = sys.argv[1:]
try:
    port = int(raw_port)
    if not 1 <= port <= 65535:
        raise ValueError('port must be between 1 and 65535')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((host, port))
except (OSError, ValueError) as error:
    print(f'ROS TCP endpoint cannot bind {host}:{raw_port}: {error}', file=sys.stderr)
    print('Check existing listeners with ss -ltnp. Stop the intended existing '
          'bridge, or choose an available ROS_BIND_IP/ROS_TCP_PORT.', file=sys.stderr)
    sys.exit(1)
PY
then
  echo 'Bridge preflight failed; no endpoint, controller, or display relay was started.' >&2
  exit 1
fi
endpoint_pid=''
control_pid=''
camera_pid=''
cleanup() {
  [[ -z "$control_pid" ]] || kill -INT "$control_pid" 2>/dev/null || true
  [[ -z "$endpoint_pid" ]] || kill -INT "$endpoint_pid" 2>/dev/null || true
  [[ -z "$camera_pid" ]] || kill -INT "$camera_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
"$PYTHON" -m ros_tcp_endpoint.default_server_endpoint --ros-args -p "ROS_IP:=$BIND_IP" -p "ROS_TCP_PORT:=$TCP_PORT" &
endpoint_pid=$!
"$PYTHON" -m deictic_control.node "${args[@]}" &
control_pid=$!
"$PYTHON" "$REPO/ros2/scripts/camera_view_relay.py" &
camera_pid=$!
echo "Bridge mode=$MODE domain=$ROS_DOMAIN_ID endpoint=$BIND_IP:$TCP_PORT"
wait -n "$endpoint_pid" "$control_pid" "$camera_pid"
