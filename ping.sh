#!/bin/bash

# Output file to store active IPs
output_file="active_ips.txt"
> "$output_file"  # Clear the file if it exists

# Network range
network="172.30.1"

echo "Scanning IP range ${network}.1 to ${network}.255..."

# Loop through the range of IPs
for i in {1..255}; do
    ip="${network}.${i}"
    # Ping the IP with a timeout of 1 second and send only 1 packet
    if ping -c 1 -W 1 "$ip" &> /dev/null; then
        echo "Host found: $ip"
        echo "$ip" >> "$output_file"
    fi
done

echo "Scan complete. Active IPs saved to $output_file"
