#!/usr/bin/env bash
# Check that this machine can reach a Mid360 on the Livox subnet.
# Usage: check_mid360_net.sh [lidar_ip] [host_ip]
set -euo pipefail

LIDAR_IP="${1:-192.168.1.161}"
HOST_IP="${2:-192.168.1.50}"

echo "Interfaces:"
ip -br addr
echo
echo "Looking for host IP ${HOST_IP} ..."
if ip -4 addr show | grep -q "inet ${HOST_IP}/"; then
    echo "  found ${HOST_IP} on a local NIC"
else
    echo "  ${HOST_IP} is NOT assigned. Example:"
    echo "    sudo ip addr add ${HOST_IP}/24 dev <iface>"
    echo "    sudo ip link set <iface> up"
fi
echo
echo "Pinging lidar ${LIDAR_IP} ..."
if ping -c 3 -W 1 "${LIDAR_IP}"; then
    echo "lidar reachable"
else
    echo "lidar NOT reachable; check cable, switch, lidar power, and JSON IPs"
    exit 1
fi
