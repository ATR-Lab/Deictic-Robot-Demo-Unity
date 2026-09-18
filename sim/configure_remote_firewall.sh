#!/usr/bin/env bash
# Run on the Isaac Linux host. Requires sudo; scope access to the Windows host IP.
set -euo pipefail
client_ip="${1:?Usage: bash sim/configure_remote_firewall.sh WINDOWS_SOURCE_IPV4}"
python3 -c 'import ipaddress,sys; ipaddress.IPv4Address(sys.argv[1])' "$client_ip"
for protocol in tcp udp; do
  for ports in 47995-48012 49000-49007; do
    rule="rule family=ipv4 source address=$client_ip/32 port port=$ports protocol=$protocol accept"
    sudo firewall-cmd --zone=public --add-rich-rule="$rule"
    sudo firewall-cmd --permanent --zone=public --add-rich-rule="$rule"
  done
done
rule="rule family=ipv4 source address=$client_ip/32 port port=49100 protocol=tcp accept"
sudo firewall-cmd --zone=public --add-rich-rule="$rule"
sudo firewall-cmd --permanent --zone=public --add-rich-rule="$rule"
sudo firewall-cmd --zone=public --list-rich-rules
# Host OUTPUT policy must also allow these ranges. Inspect rather than changing
# unrelated outbound policy; mlworkstation's OUTPUT policy is ACCEPT.
sudo nft list ruleset 2>/dev/null | grep 'hook output'
