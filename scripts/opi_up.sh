#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

${SCRIPT_DIR}/can_up.sh can0

source ${SCRIPT_DIR}/../install/setup.bash
taskset --cpu-list 4 ros2 launch fluorite_bringup opi.launch.py   # bind to CPU4 -> A76 #1, priority 99(RT)
# chrt -f 99 ros2 launch fluorite_bringup control.launch.py
