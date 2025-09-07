#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

$SCRIPTPATH/can_bringup.sh
ros2 launch phoenix5_driver main.launch.py
$SCRIPTPATH/can_shutdown.sh
