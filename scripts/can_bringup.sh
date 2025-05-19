#!/bin/bash

FLAG=false # default: run the full script

# check if "--second" is passed as argument
if [[ "$1" == "--second" ]]; then
    FLAG=true
fi

# always bring can0 up
echo "Bringing up can0"
sudo slcand -o -c -s0 /dev/ttyACM1 can0
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 1000

if [[ "FLAG" == true ]]; then
    echo "Bringing up can1..."
    sudo slcand -o -c -s0 /dev/ttyACM2 can1
    sudo ip link set can1 type can bitrate 1000000
    sudo ip link set can1 up
    sudo ip link set can1 txqueuelen 1000
fi
