#!/bin/bash

# Ensure `ifconfig` is available (install `net-tools` if missing)
if ! command -v ifconfig &> /dev/null; then
    echo "Error: ifconfig command not found! Please install net-tools (sudo apt install net-tools)"
    exit 1
fi

# Extract IP address from `ifconfig eth0` (removes addr: prefix if present)
IP_ETH0=$(ifconfig eth0 2>/dev/null | awk '/inet / {print $2}' | sed 's/addr://')

# If IP is still empty, try `ip addr show` (modern alternative)
if [[ -z "$IP_ETH0" ]]; then
    IP_ETH0=$(ip -4 addr show eth0 | awk '/inet / {print $2}' | cut -d'/' -f1)
fi

# Ensure IP was retrieved
if [[ -z "$IP_ETH0" ]]; then
    echo "Error: Could not retrieve IP address for eth0."
    exit 1
fi

# Extract last byte of the IP address for BOARD_NUMBER
IFS='.' read -r -a ip_parts <<< "$IP_ETH0"
if [[ ${#ip_parts[@]} -ne 4 ]]; then
    echo "Error: Unexpected IP format: $IP_ETH0"
    exit 1
fi

BOARD_NUMBER=$(printf "%02d" "${ip_parts[3]}")  # Ensures two-digit format

# Debugging Output
echo "Detected IP_ETH0: $IP_ETH0"
echo "Calculated BOARD_NUMBER: $BOARD_NUMBER"

# Set environment variables
export BOARD_NUMBER=$BOARD_NUMBER
export IP_ETH0=$IP_ETH0
export LOAD_CONFIG='n'

# Clean up logs and old binaries
rm -rf /logdump/* 2>/dev/null
rm -rf /coredump/* 2>/dev/null
rm -rf /root/test_mac/build/*.bin 2>/dev/null

# Ensure required scripts exist before executing
if [[ -f "/etc/setup_gnb_env_v2.sh" ]]; then
    bash /etc/setup_gnb_env_v2.sh
else
    echo "Warning: /etc/setup_gnb_env_v2.sh not found, skipping..."
fi

if [[ -f "/raptor/etc/netopeer2_service.sh" ]]; then
    bash /raptor/etc/netopeer2_service.sh
else
    echo "Warning: /raptor/etc/netopeer2_service.sh not found, skipping..."
fi

# Wait for services to initialize
sleep 10s

# Reload systemd and start services
systemctl daemon-reload

# Start required services
if command -v systemctl &> /dev/null; then
    systemctl start raptor2 2>/dev/null || echo "Warning: raptor2 service failed to start."
    systemctl start timesync 2>/dev/null || echo "Warning: timesync service failed to start."
    systemctl start ems 2>/dev/null || echo "Warning: ems service failed to start."
else
    echo "Warning: systemctl command not found. Services cannot be started."
fi


# /usr/bin/rpcapd -n &

exit 0
