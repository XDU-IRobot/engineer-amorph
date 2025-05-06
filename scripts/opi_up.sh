#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ip link set can0 type can bitrate 1000000
ip link set can1 type can bitrate 1000000
ip link set can0 down
ip link set can1 down
ip link set can0 up
ip link set can1 up

source ${SCRIPT_DIR}/../install/setup.bash
ros2 launch fluorite_bringup opi.py
# chrt -f 99 ros2 launch fluorite_bringup control.launch.py
