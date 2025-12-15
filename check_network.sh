#!/bin/bash

sleep 10  # 延遲 10 秒再執行

TARGET_IP="192.168.10.3"
LOG_FILE="/home/ct/EBD_agv/check_network.log"

echo "$(date): Checking connectivity to $TARGET_IP" >> "$LOG_FILE"

ping -c 3 "$TARGET_IP" > /dev/null 2>&1

if [ $? -ne 0 ]; then
    echo "$(date): Unable to reach $TARGET_IP, restarting NetworkManager" >> "$LOG_FILE"
    systemctl restart NetworkManager
else
    echo "$(date): Network OK" >> "$LOG_FILE"
fi
