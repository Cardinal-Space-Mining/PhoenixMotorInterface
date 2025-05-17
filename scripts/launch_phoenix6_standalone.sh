#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
$SCRIPTPATH/can_bringup.sh

ros2 launch phoenix6_driver main.launch.py
