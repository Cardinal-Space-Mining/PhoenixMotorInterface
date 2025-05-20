#!/bin/bash

echo "Detecting CAN USB device..."

# Find the most recently added /dev/ttyACM* device
DEVICE=$(ls -t /dev/ttyACM* 2>/dev/null | head -n 1)

if [ -z "$DEVICE" ]; then
    echo "ERROR: No /dev/ttyACM* device found."
    exit 1
fi

echo "Found device: $DEVICE"
echo "Bringing up can0..."

# Start slcand and bring up can0
sudo slcand -o -c -s0 $DEVICE can0
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 1000

echo "can0 is up and running."
