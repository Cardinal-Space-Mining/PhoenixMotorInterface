#!/bin/bash

sudo ip link set can0 down
sudo pkill slcand
sudo ip link delete can0
