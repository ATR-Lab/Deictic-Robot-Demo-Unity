#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "$0")/.." && pwd)"
monorepo_root="$(dirname "$repo_root")"
mkdir -p "$repo_root/.runtime/isaac-cache" "$repo_root/artifacts/isaac"
ros_args=(--ros)
output_path=".runtime/isaac-run-$(date -u +%Y%m%dT%H%M%SZ)-$$"
worker_args=()
expect_output=false
for argument in "$@"; do
  if $expect_output; then output_path="$argument"; expect_output=false; continue; fi
  if [[ "$argument" == --ros || "$argument" == --no-ros ]]; then ros_args=(); fi
  if [[ "$argument" == --output ]]; then expect_output=true; continue; fi
  if [[ "$argument" == --output=* ]]; then output_path="${argument#--output=}"; continue; fi
  worker_args+=("$argument")
done
if $expect_output; then echo '--output requires a path' >&2; exit 2; fi
# Create the destination as the host user before Docker writes scene artifacts.
# Otherwise a first launch creates a root-owned directory that blocks host-side
# validation receipts. Refuse path escapes and directories owned by someone else.
output_path="$(/usr/bin/python3 - "$repo_root" "$output_path" <<'PY'
import os
from pathlib import Path
import sys
root = Path(sys.argv[1]).resolve(strict=True)
output = (root / sys.argv[2]).resolve()
if not output.is_relative_to(root):
    raise SystemExit('--output must remain within the transition project directory')
output.mkdir(parents=True, exist_ok=True)
if output.stat().st_uid != os.getuid() or not os.access(output, os.W_OK):
    raise SystemExit('Output directory is not owned and writable by the launching user; choose a new path')
print(output.relative_to(root).as_posix())
PY
)"
echo "Transition Isaac output: $repo_root/$output_path"
owner_id="transition-isaac-$$-$(date +%s%N)"
cidfile="$repo_root/.runtime/$owner_id.cid"
if [[ -e "$cidfile" ]]; then echo 'Owned container ID file already exists' >&2; exit 2; fi
cleanup() {
  local cid actual_owner
  # The parent may escalate its initial signal while Docker is stopping. Keep
  # the bounded, ownership-checked cleanup intact until its own timeouts expire.
  trap '' INT TERM HUP
  if [[ ! -f "$cidfile" ]]; then return; fi
  cid="$(cat -- "$cidfile")"
  if [[ ! "$cid" =~ ^[0-9a-f]{64}$ ]]; then return; fi
  actual_owner="$(timeout --kill-after=1 5 docker inspect --format '{{ index .Config.Labels "org.atr.transition.owner" }}' "$cid" 2>/dev/null || true)"
  if [[ "$actual_owner" == "$owner_id" ]]; then
    timeout --kill-after=1 12 docker stop --time 5 "$cid" >/dev/null 2>&1 || true
    timeout --kill-after=1 8 docker rm --force "$cid" >/dev/null 2>&1 || true
  fi
  # Keep the tiny CID receipt for ownership audit; never select a container by name.
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
docker run --rm --name transition-k1-isaac --gpus all --network host \
  --cidfile "$cidfile" --label "org.atr.transition.owner=$owner_id" \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -e ROS_DISTRO=jazzy -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-174}" \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e PYTHONPATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/rclpy \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib \
  -v "$repo_root/.runtime/isaac-cache:/root/.cache" \
  -v "$monorepo_root:/workspace/deictic" -w "/workspace/deictic/$(basename "$repo_root")" \
  --entrypoint /isaac-sim/python.sh "${ISAAC_IMAGE:-k1-isaac-sim:5.0.0}" \
  deployment/isaac_worker.py "${ros_args[@]}" "${worker_args[@]}" --output "$output_path" &
docker_pid=$!
wait "$docker_pid"
