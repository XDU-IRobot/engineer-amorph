#!/bin/bash

# execute me as root

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source ${SCRIPT_DIR}/../install/setup.bash
chrt -f 99 ros2 launch fluorite_bringup control.launch.py
