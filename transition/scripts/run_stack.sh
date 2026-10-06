#!/usr/bin/env bash
# Foreground owner for transition services; leaving this session stops only its children.
set -eo pipefail
repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
mode=${1:-isaac}
policy=${2:-consequence}
display=${3:-summary}
stream_address=${4:-131.123.237.31}
[[ $# -le 4 ]] || { echo 'Usage: run_stack.sh [isaac|logical|hardware-observation] [ordinary|consequence] [summary|history] [public_ipv4|off]' >&2; exit 2; }
case "$mode" in logical|isaac|hardware-observation) ;; *) echo 'Expected logical, isaac, or hardware-observation' >&2; exit 2;; esac
case "$policy" in ordinary|consequence) ;; *) echo 'Expected ordinary or consequence policy' >&2; exit 2;; esac
case "$display" in summary|history) ;; *) echo 'Expected summary or history display' >&2; exit 2;; esac
if [[ "$mode" == hardware-observation && -f "$repo_root/transition/.runtime/k1-diagnostics-hold.json" ]]; then
    echo 'K1 diagnostics are held after an out-of-memory/reset incident. See transition/docs/K1_RESET_INCIDENT.md.' >&2
    exit 78
fi
unset PYTHONHOME PYTHONPATH
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=174 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset PYTHONHOME ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_LOCALHOST_ONLY ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export PYTHONPATH="$repo_root/transition/src:$repo_root/.codex/ros2/vendor/ROS-TCP-Endpoint:${PYTHONPATH:-}"
cd "$repo_root/transition"
[[ -d "$repo_root/.codex/ros2/vendor/ROS-TCP-Endpoint" ]] || { echo 'Run ros2/scripts/setup.sh first'; exit 1; }
stream_args=()
port_args=()
if [[ "$mode" == isaac && "$stream_address" != off ]]; then
    /usr/bin/python3 - "$stream_address" <<'PY'
import ipaddress, sys
try:
    ipaddress.IPv4Address(sys.argv[1])
except ipaddress.AddressValueError:
    raise SystemExit('WebRTC requires the workstation public IPv4 address, or off')
PY
    stream_args=(--webrtc --public-ip "$stream_address")
    port_args=(--webrtc)
fi
/usr/bin/python3 -m transition_autonomy.startup_ports "$mode" "${port_args[@]}"
run_dir="runs/$(date -u +%Y%m%dT%H%M%SZ)-$mode-$policy-$display-$$"
mkdir -p "$run_dir"
source scripts/process_owner.sh
cleanup() {
    trap - EXIT
    # Let the owned container launcher finish its bounded CID cleanup even if
    # the terminal/server delivers another termination signal during shutdown.
    trap '' INT TERM HUP
    cleanup_owned
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP
start() { start_owned "$@"; }
if [[ "$mode" == hardware-observation ]]; then
    start bash scripts/run_observation.sh
elif [[ "$mode" == isaac ]]; then
    [[ -f assets/booster_k1/K1_22dof.urdf ]] || /usr/bin/python3 scripts/fetch_k1_assets.py
    start_owned_with_grace 30 bash scripts/run_isaac_worker.sh --ros "${stream_args[@]}" --output "$run_dir/isaac"
    /usr/bin/python3 -m transition_autonomy.isaac_readiness
    start /usr/bin/python3 "$repo_root/ros2/scripts/camera_view_relay.py"
fi
start /usr/bin/python3 deployment/readonly_endpoint.py --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
if [[ "$mode" != hardware-observation ]]; then
    task_args=()
    if [[ "$mode" == logical ]]; then task_args=(--task examples/k1_pointing.json --world examples/k1_logical_world.json); fi
    start /usr/bin/python3 -m transition_autonomy.cli serve --backend "$mode" --policy "$policy" --display "$display" "${task_args[@]}" --run-dir "$run_dir/operator" --port 8766
fi
echo "Transition mode=$mode domain=174. Unity is receive-only; enter Play mode yourself."
echo "Run evidence: $PWD/$run_dir"
if [[ "$mode" != hardware-observation ]]; then echo "Simulation policy=$policy return_display=$display"; fi
if ((${#stream_args[@]})); then echo "WebRTC spectator: $stream_address (TCP 49100 / UDP 47998). Connect the Isaac Sim Streaming Client directly."; fi
wait -n "${pids[@]}"
