#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

if ! dpkg -l | grep -q "ethtool"; then
    echo "Error: ethtool not detected in the system."
    echo "Please install ethtool using the following command:"
    echo "sudo apt update && sudo apt install ethtool"
    exit 1
fi

if ! dpkg -l | grep -q "can-utils"; then
    echo "Error: can-utils not detected in the system."
    echo "Please install can-utils using the following command:"
    echo "sudo apt update && sudo apt install can-utils"
    exit 1
fi

echo "Both ethtool and can-utils are installed."

found=0
for iface in $(ip -br link show type can | awk '{print $1}'); do
    bus_info=$(sudo ethtool -i "$iface" | awk '/bus-info/ {print $2}')
    if [[ -z "$bus_info" ]]; then
        echo "Error: Unable to get bus-info for interface $iface."
        continue
    fi
    found=1
    echo "Interface $iface is connected to USB port $bus_info"
done

if [[ "$found" -eq 0 ]]; then
    echo "No CAN interface found. Check USB-CAN power/cable and gs_usb driver."
fi
