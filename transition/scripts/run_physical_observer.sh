#!/usr/bin/env bash
# MLWorkstation: vendor ROS telemetry plus identity/status/transform queries only.
set -eo pipefail
repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
runtime="$repo_root/transition/.runtime"
if [[ -f "$runtime/k1-diagnostics-hold.json" ]]; then
    echo 'K1 diagnostics are held after an out-of-memory/reset incident. See transition/docs/K1_RESET_INCIDENT.md.' >&2
    exit 78
fi
if [[ ! -f "$runtime/vendor-ros2/install/setup.bash" ]]; then
    echo 'First run: bash transition/scripts/setup_booster_ros2.sh' >&2
    exit 1
fi
unset PYTHONHOME PYTHONPATH
source /opt/ros/jazzy/setup.bash
source "$runtime/vendor-ros2/install/setup.bash"
set -u
mkdir -p "$runtime"
/usr/bin/python3 - "$runtime/physical-observer-dds.xml" <<'PY'
import ipaddress, json, subprocess, sys
from pathlib import Path
route = json.loads(subprocess.check_output(['ip', '-j', 'route', 'get', '192.168.10.102']))[0]
address = str(ipaddress.IPv4Address(route['prefsrc']))
Path(sys.argv[1]).write_text('''<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
<transport_descriptors><transport_descriptor><transport_id>observer_udp</transport_id><type>UDPv4</type>
<interfaceWhiteList><address>127.0.0.1</address><address>'''+address+'''</address></interfaceWhiteList>
</transport_descriptor></transport_descriptors><participant profile_name="physical_observer" is_default_profile="true">
<rtps><userTransports><transport_id>observer_udp</transport_id></userTransports><useBuiltinTransports>false</useBuiltinTransports></rtps>
</participant></profiles>''')
PY
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$runtime/physical-observer-dds.xml"
unset FASTDDS_BUILTIN_TRANSPORTS FASTDDS_DEFAULT_PROFILES_FILE ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_LOCALHOST_ONLY ROS_DISCOVERY_SERVER
exec /usr/bin/python3 "$repo_root/transition/deployment/k1_ros_observer.py" \
    --output-dir "$runtime/physical-observation" "$@"
