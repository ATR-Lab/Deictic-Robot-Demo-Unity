#!/usr/bin/env bash
# MLWorkstation: one-way real K1 -> isolated Unity display domain.
set -eo pipefail
repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ -f "$repo_root/transition/.runtime/k1-diagnostics-hold.json" ]]; then
    echo 'K1 diagnostics are held after an out-of-memory/reset incident. See transition/docs/K1_RESET_INCIDENT.md.' >&2
    exit 78
fi
source /opt/ros/jazzy/setup.bash
set -u
runtime="$repo_root/transition/.runtime"
mkdir -p "$runtime"
local_address=$(/usr/bin/python3 - <<'PY'
import json,subprocess
r=json.loads(subprocess.check_output(['ip','-j','route','get','192.168.10.102']))[0]
print(r['prefsrc'])
PY
)
# This file is scoped to these child processes; no vendor or shell-global DDS configuration changes.
/usr/bin/python3 - "$runtime/observation-dds.xml" "$local_address" <<'PY'
import ipaddress,sys
from pathlib import Path
address=str(ipaddress.ip_address(sys.argv[2]))
Path(sys.argv[1]).write_text('''<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
<transport_descriptors><transport_descriptor><transport_id>transition_udp</transport_id><type>UDPv4</type>
<interfaceWhiteList><address>127.0.0.1</address><address>'''+address+'''</address></interfaceWhiteList>
</transport_descriptor></transport_descriptors><participant profile_name="transition_observation" is_default_profile="true">
<rtps><userTransports><transport_id>transition_udp</transport_id></userTransports><useBuiltinTransports>false</useBuiltinTransports></rtps>
</participant></profiles>''')
PY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$runtime/observation-dds.xml"
unset FASTDDS_BUILTIN_TRANSPORTS FASTDDS_DEFAULT_PROFILES_FILE ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_LOCALHOST_ONLY ROS_DISCOVERY_SERVER
exec /usr/bin/python3 "$repo_root/transition/deployment/k1_observation_bridge.py" --vendor-domain 0 --unity-domain 174
