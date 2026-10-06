#!/usr/bin/env bash
# K1 onboard computer only: standalone derived camera publication, no actuation.
set -eo pipefail
script_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ -f "$script_root/.runtime/k1-diagnostics-hold.json" ]]; then
    echo 'K1 camera diagnostics are held after the 2026-09-24 out-of-memory/reset incident.' >&2
    exit 78
fi
unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
mkdir -p "$script_root/.runtime"
# Bind only loopback and this robot's active private IPv4 interfaces. This is
# scoped to the observer and never edits the vendor DDS configuration.
/usr/bin/python3 - "$script_root/.runtime/camera-dds.xml" <<'PY'
import ipaddress,json,subprocess,sys
from pathlib import Path
interfaces=json.loads(subprocess.check_output(['ip','-j','-4','address','show']))
addresses={'127.0.0.1'}
for interface in interfaces:
    if 'UP' not in interface.get('flags',[]): continue
    for info in interface.get('addr_info',[]):
        address=ipaddress.ip_address(info['local'])
        if address.is_private and not address.is_link_local:
            addresses.add(str(address))
whitelist=''.join('<address>'+address+'</address>' for address in sorted(addresses))
Path(sys.argv[1]).write_text('''<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
<transport_descriptors><transport_descriptor><transport_id>transition_camera_udp</transport_id><type>UDPv4</type>
<interfaceWhiteList>'''+whitelist+'''</interfaceWhiteList></transport_descriptor></transport_descriptors>
<participant profile_name="transition_camera" is_default_profile="true"><rtps><userTransports>
<transport_id>transition_camera_udp</transport_id></userTransports><useBuiltinTransports>false</useBuiltinTransports>
</rtps></participant></profiles>''')
PY
export FASTRTPS_DEFAULT_PROFILES_FILE="$script_root/.runtime/camera-dds.xml"
unset FASTDDS_BUILTIN_TRANSPORTS FASTDDS_DEFAULT_PROFILES_FILE ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER
exec /usr/bin/python3 "$script_root/deployment/k1_camera_observer.py" "$@"
