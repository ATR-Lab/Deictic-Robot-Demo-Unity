#!/usr/bin/env bash
set -euo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
STATE="$REPO/.codex/ros2"
mkdir -p "$STATE/vendor"
ENDPOINT="$STATE/vendor/ROS-TCP-Endpoint"
if [[ ! -d "$ENDPOINT/.git" ]]; then
  git clone --depth 1 --branch ROS2v0.7.0 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git "$ENDPOINT"
fi
EXPECTED=54c1a64b6d5ef6ffa0a0431570bb74329b79b15b
[[ $(git -C "$ENDPOINT" rev-parse HEAD) == "$EXPECTED" ]] || { echo 'Endpoint revision mismatch'; exit 1; }
echo "Pinned ROS 2 endpoint ready: $ENDPOINT"
echo 'Source your ROS installation, then run bash ros2/scripts/run_bridge.sh mock|synthetic|markerless.'
echo 'Tested native control environment: Python 3.12 + numpy==1.26.4 + scipy==1.11.4 + gtsam==4.2.2; do not install it over ROS Lyrical NumPy 2.'
